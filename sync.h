/*
 * sync.h - Synchronization Management with Loss Recovery
 * ECU STM32F405 v8.2
 * 
 * Features:
 * - Multi-stage sync acquisition (crank → cam → full)
 * - Automatic sync loss detection
 * - Fast re-sync without MCU reset
 * - Noise filtering and validation
 * - Sync quality metrics
 */

#ifndef SYNC_H
#define SYNC_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// SYNC CONFIGURATION
// M1 FIX: Use constants from config.h where available, define only sync-specific ones
// ============================================================================

// Sync loss detection
#define SYNC_LOSS_TOOTH_COUNT       3       // Consecutive bad teeth to lose sync
#define SYNC_LOSS_TIMEOUT_US        500000  // 500ms without tooth = sync lost
#define SYNC_LOSS_RPM_DELTA_MAX     500     // Max RPM change per tooth

// Gap detection - M1 FIX: Use from config.h if defined, otherwise define here
#ifndef GAP_RATIO_MIN
#define GAP_RATIO_MIN               1.5f    // Minimum ratio to detect gap
#endif
#ifndef GAP_RATIO_MAX
#define GAP_RATIO_MAX               3.5f    // Maximum ratio (reject noise)
#endif
#define GAP_RATIO_IDEAL             2.0f    // Expected ratio for 60-2

// Re-sync settings
#define RESYNC_REQUIRED_GAPS        2       // Consecutive gaps to confirm re-sync
#define RESYNC_COOLDOWN_MS          100     // Min time between re-sync attempts

// Noise rejection - M1 FIX: Use from config.h if defined
#ifndef MIN_TOOTH_PERIOD_US
#define MIN_TOOTH_PERIOD_US         50      // Below this = noise
#endif
#define MAX_TOOTH_PERIOD_US         500000  // Above this = stall (120 RPM for 60-2)

// Cam sync (for sequential)
#define CAM_SYNC_TIMEOUT_TEETH      120     // Teeth to wait for cam signal (2 revs)

// ============================================================================
// SYNC STATE MACHINE
// ============================================================================

typedef enum {
    SYNC_STATE_LOST = 0,        // No synchronization
    SYNC_STATE_SEEKING,         // Looking for first gap
    SYNC_STATE_CONFIRMING,      // Found gap, confirming
    SYNC_STATE_CRANK_SYNC,      // Crank synced (gap confirmed)
    SYNC_STATE_CAM_SYNC,        // Cam synced (phase known)
    SYNC_STATE_FULL_SYNC        // Full sync, running normally
} SyncStateEnum;

// ============================================================================
// SYNC STATUS STRUCTURE
// ============================================================================

typedef struct {
    // Current state
    volatile SyncStateEnum state;
    volatile bool isValid;              // Safe to fire events
    
    // Tooth tracking
    volatile uint8_t toothCount;        // Current tooth (1-58 for 60-2)
    volatile uint8_t teethSinceGap;     // Teeth counted since last gap
    volatile uint8_t expectedTeeth;     // Expected teeth before next gap
    
    // Timing
    volatile uint32_t lastToothTime;    // micros() of last tooth
    volatile uint32_t toothPeriod;      // Last tooth-to-tooth time (µs)
    volatile uint32_t avgToothPeriod;   // Filtered average (µs)
    volatile uint32_t prevToothPeriod;  // Previous period for gap detection
    
    // RPM
    volatile uint16_t rpm;
    volatile uint16_t rpmFiltered;      // Smoothed RPM
    
    // Angle tracking
    volatile int16_t crankAngle;        // 0-719° for 4-stroke
    volatile bool revolutionComplete;   // Flag for full 720° cycle
    
    // Cam sync
    volatile bool camSyncAcquired;
    volatile bool camPhase;             // false=exhaust, true=compression
    volatile uint8_t teethSinceCam;     // Teeth since last cam signal
    
    // Gap detection
    volatile uint8_t consecutiveGaps;   // For re-sync confirmation
    volatile float lastGapRatio;        // Diagnostic
    
    // Error tracking
    volatile uint8_t badToothCount;     // Consecutive invalid teeth
    volatile uint32_t syncLostCount;    // Total sync losses
    volatile uint32_t noiseRejectCount; // Rejected noise pulses
    volatile uint32_t resyncCount;      // Successful re-syncs
    
    // Last sync loss info
    uint32_t lastSyncLossTime;
    uint8_t lastSyncLossReason;         // For diagnostics
    
} SyncStatus;

// Sync loss reasons
#define SYNC_LOSS_TIMEOUT           1
#define SYNC_LOSS_BAD_GAP           2
#define SYNC_LOSS_TOOTH_COUNT       3
#define SYNC_LOSS_RPM_SPIKE         4
#define SYNC_LOSS_NOISE             5
#define SYNC_LOSS_MANUAL            6

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern volatile SyncStatus syncStatus;

// ============================================================================
// FUNCTIONS
// ============================================================================

// Initialization
void syncInit(void);
void syncReset(void);

// Main tooth handler (call from crank ISR)
void syncProcessTooth(uint32_t toothTime);

// Cam signal handler (call from cam ISR)
void syncProcessCam(uint32_t camTime);

// Sync loss handling
void syncLost(uint8_t reason);
bool syncAttemptRecovery(void);

// State queries
bool syncIsValid(void);
bool syncIsCrankSynced(void);
bool syncIsCamSynced(void);
bool syncIsFullSync(void);
SyncStateEnum syncGetState(void);

// Angle and RPM
int16_t syncGetCrankAngle(void);
uint16_t syncGetRPM(void);
uint16_t syncGetFilteredRPM(void);

// Check for timeout (call from main loop)
void syncCheckTimeout(void);

// Diagnostics
void syncPrintStatus(void);
uint32_t syncGetLostCount(void);
uint32_t syncGetResyncCount(void);

#endif // SYNC_H
