/*
 * autotune.h - Automatic VE/AFR/MAF Tuning System
 * ECU STM32F405 v8.2
 * 
 * Implements STFT/LTFT-style fuel trim learning:
 * - STFT: Uses existing Lambda PID for immediate correction
 * - LTFT: Accumulates errors per cell and applies to base tables
 * 
 * Based on OEM concepts (Bosch/GM) and open-source (Speeduino/RusEfi)
 */

#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <Arduino.h>
#include "config.h"
#include "tables.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Autotune timing
#define AUTOTUNE_UPDATE_RATE_MS     100     // 10Hz update rate
#define AUTOTUNE_STEADY_TIME_MS     2000    // Time in steady-state before learning

// Steady-state detection thresholds
#define AUTOTUNE_DELTA_RPM_MAX      100.0f  // Max RPM change for steady-state
#define AUTOTUNE_DELTA_TPS_MAX      3.0f    // Max TPS% change for steady-state
#define AUTOTUNE_DELTA_LOAD_MAX     5.0f    // Max load change for steady-state

// Learning parameters
#define AUTOTUNE_INCORPORATION      0.02f   // How fast to accumulate (0.01-0.1)
#define AUTOTUNE_APPLY_THRESHOLD    3.0f    // Min LTFT% to trigger adjustment
#define AUTOTUNE_MAX_ADJUST_STEP    1.0f    // Max adjustment per application (%)
#define AUTOTUNE_LTFT_LIMIT         25.0f   // Max accumulated LTFT per cell (%)

// Activation conditions
#define AUTOTUNE_MIN_CLT            60.0f   // Min coolant temp (°C)
#define AUTOTUNE_MIN_RPM            1000    // Min RPM for learning
#define AUTOTUNE_MAX_RPM            6000    // Max RPM for learning
#define AUTOTUNE_MAX_TPS            85.0f   // Max TPS% (avoid WOT)
#define AUTOTUNE_MIN_LAMBDA         0.75f   // Min valid lambda
#define AUTOTUNE_MAX_LAMBDA         1.25f   // Max valid lambda

// ============================================================================
// ENUMS
// ============================================================================

typedef enum {
    AUTOTUNE_MODE_OFF = 0,
    AUTOTUNE_MODE_VE,       // Adjust VE table (Alpha-N mode)
    AUTOTUNE_MODE_AFR,      // Adjust AFR target table
    AUTOTUNE_MODE_MAF       // Adjust MAF calibration table
} AutotuneMode;

typedef enum {
    AUTOTUNE_STATE_DISABLED = 0,
    AUTOTUNE_STATE_WAITING,     // Waiting for conditions
    AUTOTUNE_STATE_STABILIZING, // Checking steady-state
    AUTOTUNE_STATE_LEARNING,    // Actively learning
    AUTOTUNE_STATE_APPLYING     // Applying adjustment
} AutotuneState;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    // Mode and state
    AutotuneMode mode;
    AutotuneState state;
    
    // LTFT grid (same dimensions as VE/AFR tables)
    float ltftGrid[TABLE_2D_SIZE][TABLE_2D_SIZE];
    
    // MAF LTFT (1D, same as MAF table)
    float ltftMaf[TABLE_1D_MAF_SIZE];
    
    // Current cell being learned
    uint8_t currentRpmIdx;
    uint8_t currentLoadIdx;
    uint8_t currentMafIdx;
    
    // Steady-state tracking
    float lastRpm;
    float lastTps;
    float lastLoad;
    uint32_t steadyStartTime;
    bool isSteady;
    
    // Statistics
    uint32_t totalAdjustments;
    uint32_t cellsLearned;
    float avgLtft;
    
    // Timing
    uint32_t lastUpdateTime;
    uint32_t learningStartTime;
    
} AutotuneData;

extern AutotuneData autotune;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Initialization
void autotuneInit(void);
void autotuneReset(void);

// Main update (call from main loop at 10-20Hz)
void autotuneUpdate(uint16_t rpm, float loadMgStroke, float mafHz, 
                    float tps, float clt, float lambda, float lambdaTarget,
                    float stftCorrection);

// Mode control
void autotuneSetMode(AutotuneMode mode);
AutotuneMode autotuneGetMode(void);
void autotuneCycleMode(void);  // Cycle through modes
const char* autotuneGetModeName(void);
const char* autotuneGetStateName(void);

// Status
bool autotuneIsActive(void);
bool autotuneIsSteadyState(void);
float autotuneGetCellLtft(uint8_t rpmIdx, uint8_t loadIdx);
float autotuneGetAvgLtft(void);

// Manual control
void autotuneApplyNow(void);   // Force apply current LTFT
void autotuneZeroCell(uint8_t rpmIdx, uint8_t loadIdx);

// Diagnostics
void autotunePrintStatus(void);
void autotunePrintLtftGrid(void);
void autotunePrintLtftMaf(void);

// Serial interface
void autotuneProcessSerial(void);

#endif // AUTOTUNE_H
