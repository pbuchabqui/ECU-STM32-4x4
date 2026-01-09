/*
 * idle.cpp - Idle Control System Implementation
 * ECU STM32F405 v8.2
 * 
 * Idle target RPM table moved to tables.cpp
 * This module handles only the PID control logic.
 */

#include "idle.h"
#include "tables.h"  // Idle RPM table is here now
#include <math.h>

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

IdleControlState idleState;

// ============================================================================
// INITIALIZATION
// ============================================================================

void idleInit(void) {
    memset(&idleState, 0, sizeof(IdleControlState));
    idleState.state = IDLE_STATE_DISABLED;
    idleState.lastUpdateTime = millis();
}

// ============================================================================
// TARGET RPM (Now delegates to tables.h)
// ============================================================================

uint16_t idleGetTargetRpm(float clt) {
    // v8.2: Use centralized table from tables.cpp
    return getIdleTargetRpm(clt);
}

// ============================================================================
// DASHPOT DETECTION
// ============================================================================

static void updateDashpot(float tps) {
    uint32_t now = millis();
    uint32_t dt = now - idleState.lastTpsTime;
    
    if (dt > 0 && dt < 1000) {
        float tpsRate = (idleState.lastTps - tps) * 1000.0f / dt;
        
        if (tpsRate > IDLE_DASHPOT_THRESHOLD) {
            idleState.dashpotCorrection = IDLE_DASHPOT_MAX;
            idleState.state = IDLE_STATE_DASHPOT;
        }
    }
    
    if (idleState.dashpotCorrection > 0) {
        idleState.dashpotCorrection -= IDLE_DASHPOT_DECAY;
        if (idleState.dashpotCorrection < 0) {
            idleState.dashpotCorrection = 0;
        }
    }
    
    idleState.lastTps = tps;
    idleState.lastTpsTime = now;
}

// ============================================================================
// AFTERSTART HANDLING
// ============================================================================

static void updateAfterstart(void) {
    if (idleState.state == IDLE_STATE_AFTERSTART) {
        uint32_t timeSinceStart = millis() - idleState.engineStartTime;
        
        if (timeSinceStart > IDLE_AFTERSTART_TIME_MS) {
            idleState.afterstartCorrection = 0;
            idleState.state = IDLE_STATE_WARMUP;
        } else {
            idleState.afterstartCorrection -= IDLE_AFTERSTART_DECAY;
            if (idleState.afterstartCorrection < 0) {
                idleState.afterstartCorrection = 0;
            }
        }
    }
}

// ============================================================================
// PID CALCULATION
// ============================================================================

static float calculatePID(int16_t rpmError, uint32_t dtMs) {
    float dt = dtMs / 1000.0f;
    if (dt <= 0 || dt > 1.0f) dt = 0.01f;
    
    // Deadband
    if (abs(rpmError) < IDLE_RPM_DEADBAND) {
        idleState.isInDeadband = true;
        idleState.proportional = 0;
        idleState.derivative = 0;
        return idleState.integral;
    }
    
    idleState.isInDeadband = false;
    
    // Proportional
    idleState.proportional = IDLE_IGN_PID_KP * rpmError;
    
    // Integral with anti-windup
    idleState.integral += IDLE_IGN_PID_KI * rpmError * dt;
    idleState.integral = constrain(idleState.integral,
                                    -IDLE_IGN_INTEGRAL_MAX,
                                    IDLE_IGN_INTEGRAL_MAX);
    
    // Derivative
    int16_t dError = rpmError - idleState.lastRpmError;
    idleState.derivative = IDLE_IGN_PID_KD * dError / dt;
    idleState.lastRpmError = rpmError;
    
    float output = idleState.proportional + idleState.integral + idleState.derivative;
    
    // Validate
    if (!isfinite(output)) {
        output = 0;
        idleState.integral = 0;
    }
    
    return output;
}

// ============================================================================
// MAIN UPDATE
// ============================================================================

void idleUpdate(uint16_t rpm, float tps, float clt, float baseIgnition) {
    uint32_t now = millis();
    uint32_t dtMs = now - idleState.lastUpdateTime;
    idleState.lastUpdateTime = now;
    
    idleState.currentRpm = rpm;
    idleState.totalCycles++;
    
    updateDashpot(tps);
    updateAfterstart();
    
    // Cranking
    if (rpm < IDLE_RPM_MIN) {
        idleState.state = IDLE_STATE_CRANKING;
        idleState.ignitionCorrection = 10.0f - baseIgnition;  // Fixed 10° cranking
        idleState.isActive = false;
        return;
    }
    
    // TPS too high
    if (tps > IDLE_TPS_MAX) {
        if (idleState.dashpotCorrection > 0) {
            idleState.state = IDLE_STATE_DASHPOT;
            idleState.ignitionCorrection = idleState.dashpotCorrection;
            idleState.isActive = true;
        } else {
            idleState.state = IDLE_STATE_DISABLED;
            idleState.ignitionCorrection = 0;
            idleState.isActive = false;
            idleState.integral = 0;
        }
        return;
    }
    
    // RPM too high
    if (rpm > IDLE_RPM_MAX) {
        idleState.state = IDLE_STATE_DISABLED;
        idleState.ignitionCorrection = 0;
        idleState.isActive = false;
        return;
    }
    
    // IDLE CONTROL ACTIVE
    idleState.isActive = true;
    idleState.activeCycles++;
    
    // Determine state
    if (idleState.state != IDLE_STATE_AFTERSTART) {
        if (clt < 60.0f) {
            idleState.state = IDLE_STATE_WARMUP;
        } else {
            idleState.state = IDLE_STATE_RUNNING;
        }
    }
    
    // Get target from centralized table
    idleState.targetRpm = idleGetTargetRpm(clt);
    
    // Calculate error
    idleState.rpmError = (int16_t)idleState.targetRpm - (int16_t)rpm;
    
    // Run PID
    float pidCorrection = calculatePID(idleState.rpmError, dtMs);
    pidCorrection += idleState.dashpotCorrection;
    pidCorrection += idleState.afterstartCorrection;
    
    // Apply limits
    pidCorrection = constrain(pidCorrection, -IDLE_IGN_CORRECTION_MAX, IDLE_IGN_CORRECTION_MAX);
    
    float finalTiming = baseIgnition + pidCorrection;
    if (finalTiming > IDLE_IGN_ADVANCE_MAX) {
        pidCorrection = IDLE_IGN_ADVANCE_MAX - baseIgnition;
    } else if (finalTiming < IDLE_IGN_ADVANCE_MIN) {
        pidCorrection = IDLE_IGN_ADVANCE_MIN - baseIgnition;
    }
    
    idleState.ignitionCorrection = pidCorrection;
    idleState.avgCorrection = idleState.avgCorrection * 0.99f + pidCorrection * 0.01f;
}

// ============================================================================
// ACCESSORS
// ============================================================================

float idleGetIgnitionCorrection(void) {
    return idleState.ignitionCorrection;
}

bool idleIsActive(void) {
    return idleState.isActive;
}

IdleState idleGetState(void) {
    return idleState.state;
}

// ============================================================================
// NOTIFICATIONS
// ============================================================================

void idleNotifyEngineStart(void) {
    idleState.engineStartTime = millis();
    idleState.state = IDLE_STATE_AFTERSTART;
    idleState.afterstartCorrection = 5.0f;
    idleState.integral = 0;
    idleState.lastRpmError = 0;
}

void idleNotifyEngineStop(void) {
    idleState.state = IDLE_STATE_DISABLED;
    idleState.ignitionCorrection = 0;
    idleState.integral = 0;
    idleState.dashpotCorrection = 0;
    idleState.afterstartCorrection = 0;
    idleState.isActive = false;
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void idlePrintStatus(void) {
    Serial.println(F("===== IDLE CONTROL STATUS ====="));
    
    Serial.print(F("State: "));
    switch (idleState.state) {
        case IDLE_STATE_DISABLED:   Serial.println(F("DISABLED")); break;
        case IDLE_STATE_CRANKING:   Serial.println(F("CRANKING")); break;
        case IDLE_STATE_AFTERSTART: Serial.println(F("AFTERSTART")); break;
        case IDLE_STATE_WARMUP:     Serial.println(F("WARMUP")); break;
        case IDLE_STATE_RUNNING:    Serial.println(F("RUNNING")); break;
        case IDLE_STATE_DASHPOT:    Serial.println(F("DASHPOT")); break;
    }
    
    Serial.print(F("Active: ")); Serial.println(idleState.isActive ? "YES" : "NO");
    Serial.print(F("Target RPM: ")); Serial.println(idleState.targetRpm);
    Serial.print(F("Current RPM: ")); Serial.println(idleState.currentRpm);
    Serial.print(F("Error: ")); Serial.println(idleState.rpmError);
    
    Serial.println(F("--- PID ---"));
    Serial.print(F("P: ")); Serial.println(idleState.proportional, 2);
    Serial.print(F("I: ")); Serial.println(idleState.integral, 2);
    Serial.print(F("D: ")); Serial.println(idleState.derivative, 2);
    
    Serial.println(F("--- Corrections ---"));
    Serial.print(F("IGN: ")); Serial.print(idleState.ignitionCorrection, 1); Serial.println(F(" deg"));
    Serial.print(F("Dashpot: ")); Serial.print(idleState.dashpotCorrection, 1); Serial.println(F(" deg"));
    
    Serial.println(F("==============================="));
}
