/*
 * sync.cpp - Synchronization Management Implementation
 * ECU STM32F405 v8.2
 * 
 * State Machine Flow:
 * 
 *   LOST ──► SEEKING ──► CONFIRMING ──► CRANK_SYNC ──► CAM_SYNC ──► FULL_SYNC
 *     ▲          │            │              │             │            │
 *     └──────────┴────────────┴──────────────┴─────────────┴────────────┘
 *                            (on sync loss)
 * 
 * Recovery Process:
 * 1. Detect sync loss (bad teeth, timeout, invalid gap)
 * 2. Immediately cut fuel/spark (safety)
 * 3. Go to SEEKING state (not LOST - preserves some timing info)
 * 4. Look for valid gap
 * 5. Confirm with second gap
 * 6. Resume operation
 */

#include "sync.h"
#include "scheduler.h"
#include <math.h>  // v8.1.5: For isfinite()

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

volatile SyncStatus syncStatus;

// ============================================================================
// INTERNAL VARIABLES
// ============================================================================

static volatile uint32_t lastResyncAttempt = 0;
static volatile bool recoveryInProgress = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void syncInit(void) {
    memset((void*)&syncStatus, 0, sizeof(SyncStatus));
    
    syncStatus.state = SYNC_STATE_LOST;
    syncStatus.isValid = false;
    syncStatus.expectedTeeth = CRANK_TEETH - MISSING_TEETH;  // 58 for 60-2
    syncStatus.toothCount = 0;
    syncStatus.consecutiveGaps = 0;
    syncStatus.badToothCount = 0;
}

void syncReset(void) {
    // Preserve statistics
    uint32_t lostCount = syncStatus.syncLostCount;
    uint32_t resyncCount = syncStatus.resyncCount;
    uint32_t noiseCount = syncStatus.noiseRejectCount;
    
    syncInit();
    
    // Restore statistics
    syncStatus.syncLostCount = lostCount;
    syncStatus.resyncCount = resyncCount;
    syncStatus.noiseRejectCount = noiseCount;
}

// ============================================================================
// MAIN TOOTH PROCESSING
// ============================================================================

void syncProcessTooth(uint32_t toothTime) {
    uint32_t elapsed = toothTime - syncStatus.lastToothTime;
    
    // ===== NOISE REJECTION =====
    if (elapsed < MIN_TOOTH_PERIOD_US) {
        syncStatus.noiseRejectCount++;
        return;  // Too fast - electrical noise
    }
    
    // ===== STALL DETECTION =====
    if (elapsed > MAX_TOOTH_PERIOD_US && syncStatus.state >= SYNC_STATE_CRANK_SYNC) {
        // Engine stalled
        syncLost(SYNC_LOSS_TIMEOUT);
        syncStatus.lastToothTime = toothTime;
        syncStatus.toothPeriod = elapsed;
        return;
    }
    
    // ===== GAP DETECTION =====
    float gapRatio = 0;
    bool gapDetected = false;
    
    if (syncStatus.prevToothPeriod > 0) {
        gapRatio = (float)elapsed / (float)syncStatus.prevToothPeriod;
        
        // v8.1.5 FIX: Validate gapRatio is finite (not NaN or Inf)
        if (!isfinite(gapRatio)) {
            gapRatio = 0;  // Reset to safe value
            syncStatus.noiseRejectCount++;  // Track as noise
        } else {
            syncStatus.lastGapRatio = gapRatio;
            
            if (gapRatio >= GAP_RATIO_MIN && gapRatio <= GAP_RATIO_MAX) {
                gapDetected = true;
            }
        }
    }
    
    // ===== STATE MACHINE =====
    switch (syncStatus.state) {
        
        case SYNC_STATE_LOST:
            // Transition to seeking on first tooth
            syncStatus.state = SYNC_STATE_SEEKING;
            syncStatus.teethSinceGap = 0;
            syncStatus.consecutiveGaps = 0;
            break;
            
        case SYNC_STATE_SEEKING:
            // Looking for first gap
            if (gapDetected) {
                syncStatus.state = SYNC_STATE_CONFIRMING;
                syncStatus.consecutiveGaps = 1;
                syncStatus.toothCount = 1;
                syncStatus.teethSinceGap = 0;
                syncStatus.crankAngle = TRIGGER_ANGLE_BTDC;
            }
            break;
            
        case SYNC_STATE_CONFIRMING:
            // Verify sync with tooth count
            syncStatus.teethSinceGap++;
            syncStatus.toothCount++;
            syncStatus.crankAngle += DEGREES_PER_TOOTH;
            
            if (syncStatus.crankAngle >= 360) {
                syncStatus.crankAngle -= 360;
            }
            
            if (gapDetected) {
                // Found another gap
                if (syncStatus.teethSinceGap == syncStatus.expectedTeeth) {
                    // Correct tooth count - gap confirmed!
                    syncStatus.consecutiveGaps++;
                    
                    if (syncStatus.consecutiveGaps >= RESYNC_REQUIRED_GAPS) {
                        // Sync acquired!
                        syncStatus.state = SYNC_STATE_CRANK_SYNC;
                        syncStatus.isValid = true;
                        syncStatus.badToothCount = 0;
                        
                        if (recoveryInProgress) {
                            syncStatus.resyncCount++;
                            recoveryInProgress = false;
                        }
                    }
                    
                    syncStatus.toothCount = 1;
                    syncStatus.teethSinceGap = 0;
                    syncStatus.crankAngle = TRIGGER_ANGLE_BTDC;
                    
                } else {
                    // Wrong tooth count - false gap or noise
                    syncStatus.state = SYNC_STATE_SEEKING;
                    syncStatus.consecutiveGaps = 0;
                    syncStatus.teethSinceGap = 0;
                }
            }
            break;
            
        case SYNC_STATE_CRANK_SYNC:
        case SYNC_STATE_CAM_SYNC:
        case SYNC_STATE_FULL_SYNC:
            // Normal operation - track teeth and validate
            syncStatus.teethSinceGap++;
            syncStatus.toothCount++;
            syncStatus.crankAngle += DEGREES_PER_TOOTH;
            
            // Handle 720° cycle for 4-stroke
            if (syncStatus.crankAngle >= ENGINE_STROKE_DEGREES) {
                syncStatus.crankAngle -= ENGINE_STROKE_DEGREES;
                syncStatus.revolutionComplete = true;
            }
            
            if (gapDetected) {
                // Validate gap position
                if (syncStatus.teethSinceGap == syncStatus.expectedTeeth) {
                    // Good gap - reset counters
                    syncStatus.toothCount = 1;
                    syncStatus.teethSinceGap = 0;
                    
                    // Reset angle, preserving 720° phase
                    // If we have cam sync and were in second half (360-719°),
                    // stay in second half
                    if (syncStatus.camSyncAcquired && syncStatus.crankAngle >= 360) {
                        syncStatus.crankAngle = TRIGGER_ANGLE_BTDC + 360;
                    } else {
                        syncStatus.crankAngle = TRIGGER_ANGLE_BTDC;
                    }
                    
                    syncStatus.badToothCount = 0;
                    
                } else if (syncStatus.teethSinceGap < syncStatus.expectedTeeth - 2 ||
                           syncStatus.teethSinceGap > syncStatus.expectedTeeth + 2) {
                    // Gap at wrong position - possible sync loss
                    syncStatus.badToothCount++;
                    
                    if (syncStatus.badToothCount >= SYNC_LOSS_TOOTH_COUNT) {
                        syncLost(SYNC_LOSS_BAD_GAP);
                        syncAttemptRecovery();
                    }
                }
            }
            
            // Validate tooth count didn't exceed expected
            if (syncStatus.teethSinceGap > syncStatus.expectedTeeth + 3) {
                // Missed the gap - sync lost
                syncStatus.badToothCount++;
                
                if (syncStatus.badToothCount >= SYNC_LOSS_TOOTH_COUNT) {
                    syncLost(SYNC_LOSS_TOOTH_COUNT);
                    syncAttemptRecovery();
                }
            }
            break;
    }
    
    // ===== RPM CALCULATION =====
    if (elapsed > 0 && syncStatus.state >= SYNC_STATE_CRANK_SYNC) {
        // RPM = 60,000,000 / (tooth_period_us * teeth_per_rev)
        // For 60-2: teeth_per_rev = 58 actual teeth
        uint32_t newRpm = 60000000UL / (elapsed * (CRANK_TEETH - MISSING_TEETH));
        
        if (newRpm > 0 && newRpm < RPM_MAX_VALID) {
            // Validate RPM change isn't too sudden
            if (syncStatus.rpm > 0) {
                int32_t rpmDelta = (int32_t)newRpm - (int32_t)syncStatus.rpm;
                
                if (abs(rpmDelta) > SYNC_LOSS_RPM_DELTA_MAX && syncStatus.rpm > 500) {
                    // Suspicious RPM spike
                    syncStatus.badToothCount++;
                } else {
                    syncStatus.badToothCount = 0;
                    syncStatus.rpm = newRpm;
                }
            } else {
                syncStatus.rpm = newRpm;
            }
            
            // Filtered RPM (exponential moving average)
            syncStatus.rpmFiltered = (syncStatus.rpmFiltered * 7 + syncStatus.rpm) / 8;
        }
    }
    
    // ===== UPDATE TIMING =====
    syncStatus.prevToothPeriod = syncStatus.toothPeriod;
    syncStatus.toothPeriod = elapsed;
    syncStatus.lastToothTime = toothTime;
    
    // Update average tooth period (for angle calculations)
    syncStatus.avgToothPeriod = (syncStatus.avgToothPeriod * 3 + elapsed) / 4;
}

// ============================================================================
// CAM SIGNAL PROCESSING
// ============================================================================

void syncProcessCam(uint32_t camTime) {
    if (syncStatus.state < SYNC_STATE_CRANK_SYNC) {
        return;  // Need crank sync first
    }
    
    // Cam signal determines phase (which 360° half of 720° cycle)
    // Typically fires once per 720° (camshaft speed = crankshaft/2)
    
    if (!syncStatus.camSyncAcquired) {
        // First cam signal - establish phase
        syncStatus.camSyncAcquired = true;
        syncStatus.camPhase = true;  // Assume compression stroke
        
        // Correct angle based on cam phase
        if (syncStatus.crankAngle >= 360) {
            // Already in second half - correct
        } else {
            // In first half - this is actually second revolution
            syncStatus.crankAngle += 360;
        }
        
        if (syncStatus.state == SYNC_STATE_CRANK_SYNC) {
            syncStatus.state = SYNC_STATE_CAM_SYNC;
        }
    } else {
        // Subsequent cam signals - validate phase
        syncStatus.camPhase = !syncStatus.camPhase;
        syncStatus.teethSinceCam = 0;
    }
    
    // Upgrade to full sync if conditions met
    if (syncStatus.state == SYNC_STATE_CAM_SYNC && syncStatus.camSyncAcquired) {
        syncStatus.state = SYNC_STATE_FULL_SYNC;
    }
}

// ============================================================================
// SYNC LOSS HANDLING
// ============================================================================

void syncLost(uint8_t reason) {
    // Record loss
    syncStatus.syncLostCount++;
    syncStatus.lastSyncLossTime = millis();
    syncStatus.lastSyncLossReason = reason;
    
    // Invalidate sync
    syncStatus.isValid = false;
    syncStatus.camSyncAcquired = false;
    
    // Cut all outputs immediately (safety!)
    // Note: This should trigger output cuts in scheduler
    
    // Don't go all the way to LOST - go to SEEKING for faster recovery
    syncStatus.state = SYNC_STATE_SEEKING;
    syncStatus.consecutiveGaps = 0;
    syncStatus.toothCount = 0;
    syncStatus.teethSinceGap = 0;
}

bool syncAttemptRecovery(void) {
    uint32_t now = millis();
    
    // Cooldown check
    if ((now - lastResyncAttempt) < RESYNC_COOLDOWN_MS) {
        return false;
    }
    
    lastResyncAttempt = now;
    recoveryInProgress = true;
    
    // Reset to seeking state
    syncStatus.state = SYNC_STATE_SEEKING;
    syncStatus.isValid = false;
    syncStatus.consecutiveGaps = 0;
    syncStatus.badToothCount = 0;
    syncStatus.teethSinceGap = 0;
    syncStatus.camSyncAcquired = false;
    
    // Keep timing info for faster re-acquisition
    // (don't reset toothPeriod, avgToothPeriod, rpm)
    
    return true;
}

// ============================================================================
// TIMEOUT CHECK (call from main loop)
// ============================================================================

void syncCheckTimeout(void) {
    if (syncStatus.state >= SYNC_STATE_CRANK_SYNC) {
        uint32_t elapsed = micros() - syncStatus.lastToothTime;
        
        if (elapsed > SYNC_LOSS_TIMEOUT_US) {
            // No teeth for too long - engine stopped or sync lost
            syncLost(SYNC_LOSS_TIMEOUT);
            syncStatus.rpm = 0;
            syncStatus.rpmFiltered = 0;
        }
    }
}

// ============================================================================
// STATE QUERIES
// ============================================================================

bool syncIsValid(void) {
    return syncStatus.isValid;
}

bool syncIsCrankSynced(void) {
    return syncStatus.state >= SYNC_STATE_CRANK_SYNC;
}

bool syncIsCamSynced(void) {
    return syncStatus.state >= SYNC_STATE_CAM_SYNC;
}

bool syncIsFullSync(void) {
    return syncStatus.state == SYNC_STATE_FULL_SYNC;
}

SyncStateEnum syncGetState(void) {
    return syncStatus.state;
}

int16_t syncGetCrankAngle(void) {
    return syncStatus.crankAngle;
}

uint16_t syncGetRPM(void) {
    return syncStatus.rpm;
}

uint16_t syncGetFilteredRPM(void) {
    return syncStatus.rpmFiltered;
}

uint32_t syncGetLostCount(void) {
    return syncStatus.syncLostCount;
}

uint32_t syncGetResyncCount(void) {
    return syncStatus.resyncCount;
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void syncPrintStatus(void) {
    Serial.println("\n===== SYNC STATUS =====");
    
    Serial.print("State: ");
    switch (syncStatus.state) {
        case SYNC_STATE_LOST:       Serial.println("LOST"); break;
        case SYNC_STATE_SEEKING:    Serial.println("SEEKING"); break;
        case SYNC_STATE_CONFIRMING: Serial.println("CONFIRMING"); break;
        case SYNC_STATE_CRANK_SYNC: Serial.println("CRANK_SYNC"); break;
        case SYNC_STATE_CAM_SYNC:   Serial.println("CAM_SYNC"); break;
        case SYNC_STATE_FULL_SYNC:  Serial.println("FULL_SYNC"); break;
    }
    
    Serial.print("Valid: "); Serial.println(syncStatus.isValid ? "YES" : "NO");
    Serial.print("RPM: "); Serial.println(syncStatus.rpm);
    Serial.print("RPM Filtered: "); Serial.println(syncStatus.rpmFiltered);
    Serial.print("Crank Angle: "); Serial.print(syncStatus.crankAngle); Serial.println("°");
    
    Serial.println("--- Tooth Tracking ---");
    Serial.print("Tooth Count: "); Serial.println(syncStatus.toothCount);
    Serial.print("Teeth Since Gap: "); Serial.println(syncStatus.teethSinceGap);
    Serial.print("Expected Teeth: "); Serial.println(syncStatus.expectedTeeth);
    Serial.print("Tooth Period: "); Serial.print(syncStatus.toothPeriod); Serial.println(" µs");
    Serial.print("Last Gap Ratio: "); Serial.println(syncStatus.lastGapRatio, 2);
    
    Serial.println("--- Cam Sync ---");
    Serial.print("Cam Acquired: "); Serial.println(syncStatus.camSyncAcquired ? "YES" : "NO");
    Serial.print("Cam Phase: "); Serial.println(syncStatus.camPhase ? "COMPRESSION" : "EXHAUST");
    
    Serial.println("--- Statistics ---");
    Serial.print("Sync Lost Count: "); Serial.println(syncStatus.syncLostCount);
    Serial.print("Resync Count: "); Serial.println(syncStatus.resyncCount);
    Serial.print("Noise Rejects: "); Serial.println(syncStatus.noiseRejectCount);
    Serial.print("Bad Tooth Count: "); Serial.println(syncStatus.badToothCount);
    
    if (syncStatus.lastSyncLossTime > 0) {
        Serial.println("--- Last Sync Loss ---");
        Serial.print("Time: "); Serial.print((millis() - syncStatus.lastSyncLossTime) / 1000); 
        Serial.println(" sec ago");
        Serial.print("Reason: ");
        switch (syncStatus.lastSyncLossReason) {
            case SYNC_LOSS_TIMEOUT:     Serial.println("TIMEOUT"); break;
            case SYNC_LOSS_BAD_GAP:     Serial.println("BAD_GAP"); break;
            case SYNC_LOSS_TOOTH_COUNT: Serial.println("TOOTH_COUNT"); break;
            case SYNC_LOSS_RPM_SPIKE:   Serial.println("RPM_SPIKE"); break;
            case SYNC_LOSS_NOISE:       Serial.println("NOISE"); break;
            case SYNC_LOSS_MANUAL:      Serial.println("MANUAL"); break;
            default: Serial.println("UNKNOWN");
        }
    }
    
    Serial.println("=======================\n");
}
