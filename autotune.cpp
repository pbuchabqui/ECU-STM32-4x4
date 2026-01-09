/*
 * autotune.cpp - Automatic VE/AFR/MAF Tuning System Implementation
 * ECU STM32F405 v8.2
 * 
 * STFT/LTFT Learning Strategy:
 * 
 * 1. STFT (Short-Term Fuel Trim):
 *    - Already implemented in lambda.cpp via PID
 *    - Provides immediate correction to fuel delivery
 *    - We use this value as input for LTFT learning
 * 
 * 2. LTFT (Long-Term Fuel Trim):
 *    - Accumulates STFT errors over time per cell
 *    - When threshold exceeded, applies adjustment to base table
 *    - Persists learning in RAM (save to EEPROM manually)
 * 
 * Learning Conditions:
 *    - Engine warm (CLT > 60°C)
 *    - Closed-loop lambda active
 *    - Steady-state (stable RPM, TPS, load for 2+ seconds)
 *    - Valid lambda reading (0.75 - 1.25)
 *    - Not at WOT (TPS < 85%)
 * 
 * Safety:
 *    - Max LTFT accumulation: ±25% per cell
 *    - Max adjustment per application: 1%
 *    - Smoothing with neighbor cells (optional)
 */

#include "autotune.h"
#include "lambda.h"
#include "calibration.h"
#include <math.h>

// ============================================================================
// GLOBAL DATA
// ============================================================================

AutotuneData autotune;

// ============================================================================
// INITIALIZATION
// ============================================================================

void autotuneInit(void) {
    autotune.mode = AUTOTUNE_MODE_OFF;
    autotune.state = AUTOTUNE_STATE_DISABLED;
    
    // Clear LTFT grids
    memset(autotune.ltftGrid, 0, sizeof(autotune.ltftGrid));
    memset(autotune.ltftMaf, 0, sizeof(autotune.ltftMaf));
    
    // Reset tracking
    autotune.currentRpmIdx = 0;
    autotune.currentLoadIdx = 0;
    autotune.currentMafIdx = 0;
    
    autotune.lastRpm = 0;
    autotune.lastTps = 0;
    autotune.lastLoad = 0;
    autotune.steadyStartTime = 0;
    autotune.isSteady = false;
    
    // Reset stats
    autotune.totalAdjustments = 0;
    autotune.cellsLearned = 0;
    autotune.avgLtft = 0;
    
    autotune.lastUpdateTime = 0;
    autotune.learningStartTime = 0;
}

void autotuneReset(void) {
    // Clear LTFT grids but keep mode
    memset(autotune.ltftGrid, 0, sizeof(autotune.ltftGrid));
    memset(autotune.ltftMaf, 0, sizeof(autotune.ltftMaf));
    
    autotune.totalAdjustments = 0;
    autotune.cellsLearned = 0;
    autotune.avgLtft = 0;
    autotune.isSteady = false;
    autotune.state = (autotune.mode != AUTOTUNE_MODE_OFF) ? 
                      AUTOTUNE_STATE_WAITING : AUTOTUNE_STATE_DISABLED;
    
    Serial.println(F("Autotune: LTFT reset"));
}

// ============================================================================
// STEADY-STATE DETECTION
// ============================================================================

static bool checkSteadyState(float rpm, float tps, float load) {
    float deltaRpm = fabsf(rpm - autotune.lastRpm);
    float deltaTps = fabsf(tps - autotune.lastTps);
    float deltaLoad = fabsf(load - autotune.lastLoad);
    
    // Update last values
    autotune.lastRpm = rpm;
    autotune.lastTps = tps;
    autotune.lastLoad = load;
    
    // Check if within steady-state thresholds
    return (deltaRpm < AUTOTUNE_DELTA_RPM_MAX &&
            deltaTps < AUTOTUNE_DELTA_TPS_MAX &&
            deltaLoad < AUTOTUNE_DELTA_LOAD_MAX);
}

// ============================================================================
// LEARNING LOGIC
// ============================================================================

static void accumulateLtft2D(uint8_t rpmIdx, uint8_t loadIdx, float stftCorrection) {
    // Accumulate STFT into LTFT grid
    // stftCorrection is in % (e.g., +5.0 means 5% more fuel)
    float increment = stftCorrection * AUTOTUNE_INCORPORATION;
    
    autotune.ltftGrid[rpmIdx][loadIdx] += increment;
    
    // Clamp to limits
    autotune.ltftGrid[rpmIdx][loadIdx] = constrain(
        autotune.ltftGrid[rpmIdx][loadIdx],
        -AUTOTUNE_LTFT_LIMIT,
        AUTOTUNE_LTFT_LIMIT
    );
    
    // Track current cell
    autotune.currentRpmIdx = rpmIdx;
    autotune.currentLoadIdx = loadIdx;
}

static void accumulateLtftMaf(uint8_t mafIdx, float stftCorrection) {
    float increment = stftCorrection * AUTOTUNE_INCORPORATION;
    
    autotune.ltftMaf[mafIdx] += increment;
    
    autotune.ltftMaf[mafIdx] = constrain(
        autotune.ltftMaf[mafIdx],
        -AUTOTUNE_LTFT_LIMIT,
        AUTOTUNE_LTFT_LIMIT
    );
    
    autotune.currentMafIdx = mafIdx;
}

static void applyLtftToTable(void) {
    uint8_t rpmIdx = autotune.currentRpmIdx;
    uint8_t loadIdx = autotune.currentLoadIdx;
    float ltft = autotune.ltftGrid[rpmIdx][loadIdx];
    
    // Check if LTFT exceeds threshold
    if (fabsf(ltft) < AUTOTUNE_APPLY_THRESHOLD) {
        return;
    }
    
    // Calculate adjustment (limited per application)
    float adjustPercent = constrain(ltft * 0.3f, -AUTOTUNE_MAX_ADJUST_STEP, AUTOTUNE_MAX_ADJUST_STEP);
    bool increase = (adjustPercent > 0);
    
    // Apply based on mode
    switch (autotune.mode) {
        case AUTOTUNE_MODE_VE:
            // Positive LTFT (lean) → increase VE (more fuel)
            adjustCellsProportional(CAL_MODE_VE, 
                                    calData.veTable.rpmBins[rpmIdx],
                                    calData.veTable.loadBins[loadIdx], 
                                    increase);
            Serial.print(F("AT VE [")); Serial.print(rpmIdx);
            Serial.print(F("][")); Serial.print(loadIdx);
            Serial.print(F("] ")); Serial.print(increase ? "+" : "-");
            Serial.print(F(" (LTFT:")); Serial.print(ltft, 1);
            Serial.println(F("%)"));
            break;
            
        case AUTOTUNE_MODE_AFR:
            // Positive LTFT (lean) → decrease AFR target (richer)
            adjustCellsProportional(CAL_MODE_AFR,
                                    calData.afrTable.rpmBins[rpmIdx],
                                    calData.afrTable.loadBins[loadIdx],
                                    !increase);  // Inverted!
            Serial.print(F("AT AFR [")); Serial.print(rpmIdx);
            Serial.print(F("][")); Serial.print(loadIdx);
            Serial.print(F("] ")); Serial.print(!increase ? "+" : "-");
            Serial.print(F(" (LTFT:")); Serial.print(ltft, 1);
            Serial.println(F("%)"));
            break;
            
        default:
            return;
    }
    
    // Decay LTFT after applying (don't zero completely for smoothing)
    autotune.ltftGrid[rpmIdx][loadIdx] *= 0.5f;
    autotune.totalAdjustments++;
}

static void applyLtftToMaf(void) {
    uint8_t mafIdx = autotune.currentMafIdx;
    float ltft = autotune.ltftMaf[mafIdx];
    
    if (fabsf(ltft) < AUTOTUNE_APPLY_THRESHOLD) {
        return;
    }
    
    // Positive LTFT (lean) → increase MAF flow value
    bool increase = (ltft > 0);
    adjustMafCell(mafIdx, increase);
    
    Serial.print(F("AT MAF [")); Serial.print(mafIdx);
    Serial.print(F("] ")); Serial.print(increase ? "+" : "-");
    Serial.print(F(" (LTFT:")); Serial.print(ltft, 1);
    Serial.println(F("%)"));
    
    autotune.ltftMaf[mafIdx] *= 0.5f;
    autotune.totalAdjustments++;
}

// ============================================================================
// MAIN UPDATE FUNCTION
// ============================================================================

void autotuneUpdate(uint16_t rpm, float loadMgStroke, float mafHz,
                    float tps, float clt, float lambda, float lambdaTarget,
                    float stftCorrection) {
    
    // Rate limiting
    uint32_t now = millis();
    if (now - autotune.lastUpdateTime < AUTOTUNE_UPDATE_RATE_MS) {
        return;
    }
    autotune.lastUpdateTime = now;
    
    // Check if autotune is enabled
    if (autotune.mode == AUTOTUNE_MODE_OFF) {
        autotune.state = AUTOTUNE_STATE_DISABLED;
        return;
    }
    
    // Check basic conditions
    bool conditionsMet = true;
    
    if (clt < AUTOTUNE_MIN_CLT) conditionsMet = false;
    if (rpm < AUTOTUNE_MIN_RPM || rpm > AUTOTUNE_MAX_RPM) conditionsMet = false;
    if (tps > AUTOTUNE_MAX_TPS) conditionsMet = false;
    if (lambda < AUTOTUNE_MIN_LAMBDA || lambda > AUTOTUNE_MAX_LAMBDA) conditionsMet = false;
    if (!lambdaIsAutoCorrectEnabled()) conditionsMet = false;
    
    if (!conditionsMet) {
        autotune.state = AUTOTUNE_STATE_WAITING;
        autotune.isSteady = false;
        autotune.steadyStartTime = 0;
        return;
    }
    
    // Check steady-state
    bool steadyNow = checkSteadyState(rpm, tps, loadMgStroke);
    
    if (!steadyNow) {
        autotune.state = AUTOTUNE_STATE_STABILIZING;
        autotune.isSteady = false;
        autotune.steadyStartTime = 0;
        return;
    }
    
    // Track steady-state duration
    if (!autotune.isSteady) {
        autotune.steadyStartTime = now;
        autotune.isSteady = true;
        autotune.state = AUTOTUNE_STATE_STABILIZING;
    }
    
    // Wait for steady-state duration
    if (now - autotune.steadyStartTime < AUTOTUNE_STEADY_TIME_MS) {
        return;
    }
    
    // Now learning!
    autotune.state = AUTOTUNE_STATE_LEARNING;
    
    // Find current cell indices
    uint8_t rpmIdx = findRpmIdx(rpm);
    uint8_t loadIdx = findLoadIdx(loadMgStroke);
    uint8_t mafIdx = findMafIdx(mafHz);
    
    // Accumulate LTFT based on mode
    switch (autotune.mode) {
        case AUTOTUNE_MODE_VE:
        case AUTOTUNE_MODE_AFR:
            accumulateLtft2D(rpmIdx, loadIdx, stftCorrection);
            
            // Check if should apply
            if (fabsf(autotune.ltftGrid[rpmIdx][loadIdx]) >= AUTOTUNE_APPLY_THRESHOLD) {
                autotune.state = AUTOTUNE_STATE_APPLYING;
                applyLtftToTable();
            }
            break;
            
        case AUTOTUNE_MODE_MAF:
            accumulateLtftMaf(mafIdx, stftCorrection);
            
            if (fabsf(autotune.ltftMaf[mafIdx]) >= AUTOTUNE_APPLY_THRESHOLD) {
                autotune.state = AUTOTUNE_STATE_APPLYING;
                applyLtftToMaf();
            }
            break;
            
        default:
            break;
    }
}

// ============================================================================
// MODE CONTROL
// ============================================================================

void autotuneSetMode(AutotuneMode mode) {
    autotune.mode = mode;
    autotune.state = (mode != AUTOTUNE_MODE_OFF) ? 
                      AUTOTUNE_STATE_WAITING : AUTOTUNE_STATE_DISABLED;
    autotune.isSteady = false;
    
    Serial.print(F("Autotune mode: "));
    Serial.println(autotuneGetModeName());
}

AutotuneMode autotuneGetMode(void) {
    return autotune.mode;
}

void autotuneCycleMode(void) {
    AutotuneMode newMode = (AutotuneMode)((autotune.mode + 1) % 4);
    autotuneSetMode(newMode);
}

const char* autotuneGetModeName(void) {
    switch (autotune.mode) {
        case AUTOTUNE_MODE_OFF: return "OFF";
        case AUTOTUNE_MODE_VE:  return "VE";
        case AUTOTUNE_MODE_AFR: return "AFR";
        case AUTOTUNE_MODE_MAF: return "MAF";
        default: return "?";
    }
}

const char* autotuneGetStateName(void) {
    switch (autotune.state) {
        case AUTOTUNE_STATE_DISABLED:    return "DISABLED";
        case AUTOTUNE_STATE_WAITING:     return "WAITING";
        case AUTOTUNE_STATE_STABILIZING: return "STABILIZING";
        case AUTOTUNE_STATE_LEARNING:    return "LEARNING";
        case AUTOTUNE_STATE_APPLYING:    return "APPLYING";
        default: return "?";
    }
}

// ============================================================================
// STATUS FUNCTIONS
// ============================================================================

bool autotuneIsActive(void) {
    return (autotune.mode != AUTOTUNE_MODE_OFF && 
            autotune.state >= AUTOTUNE_STATE_LEARNING);
}

bool autotuneIsSteadyState(void) {
    return autotune.isSteady;
}

float autotuneGetCellLtft(uint8_t rpmIdx, uint8_t loadIdx) {
    if (rpmIdx >= TABLE_2D_SIZE || loadIdx >= TABLE_2D_SIZE) return 0;
    return autotune.ltftGrid[rpmIdx][loadIdx];
}

float autotuneGetAvgLtft(void) {
    float sum = 0;
    int count = 0;
    
    for (int r = 0; r < TABLE_2D_SIZE; r++) {
        for (int l = 0; l < TABLE_2D_SIZE; l++) {
            if (fabsf(autotune.ltftGrid[r][l]) > 0.1f) {
                sum += autotune.ltftGrid[r][l];
                count++;
            }
        }
    }
    
    autotune.avgLtft = (count > 0) ? sum / count : 0;
    autotune.cellsLearned = count;
    return autotune.avgLtft;
}

// ============================================================================
// MANUAL CONTROL
// ============================================================================

void autotuneApplyNow(void) {
    if (autotune.mode == AUTOTUNE_MODE_MAF) {
        for (int i = 0; i < TABLE_1D_MAF_SIZE; i++) {
            autotune.currentMafIdx = i;
            applyLtftToMaf();
        }
    } else {
        for (int r = 0; r < TABLE_2D_SIZE; r++) {
            for (int l = 0; l < TABLE_2D_SIZE; l++) {
                autotune.currentRpmIdx = r;
                autotune.currentLoadIdx = l;
                applyLtftToTable();
            }
        }
    }
    Serial.println(F("Autotune: Force applied all LTFT"));
}

void autotuneZeroCell(uint8_t rpmIdx, uint8_t loadIdx) {
    if (rpmIdx < TABLE_2D_SIZE && loadIdx < TABLE_2D_SIZE) {
        autotune.ltftGrid[rpmIdx][loadIdx] = 0;
    }
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void autotunePrintStatus(void) {
    Serial.println(F("\n======== AUTOTUNE STATUS ========"));
    Serial.print(F("Mode:  ")); Serial.println(autotuneGetModeName());
    Serial.print(F("State: ")); Serial.println(autotuneGetStateName());
    Serial.println(F("---------------------------------"));
    
    Serial.print(F("Steady-State: ")); 
    Serial.println(autotune.isSteady ? "YES" : "NO");
    
    if (autotune.isSteady && autotune.steadyStartTime > 0) {
        Serial.print(F("Steady Time: "));
        Serial.print((millis() - autotune.steadyStartTime) / 1000.0f, 1);
        Serial.println(F(" s"));
    }
    
    Serial.println(F("---------------------------------"));
    Serial.print(F("Current Cell: ["));
    Serial.print(autotune.currentRpmIdx);
    Serial.print(F("]["));
    Serial.print(autotune.currentLoadIdx);
    Serial.println(F("]"));
    
    if (autotune.mode == AUTOTUNE_MODE_VE || autotune.mode == AUTOTUNE_MODE_AFR) {
        Serial.print(F("Cell LTFT: "));
        Serial.print(autotune.ltftGrid[autotune.currentRpmIdx][autotune.currentLoadIdx], 2);
        Serial.println(F("%"));
    } else if (autotune.mode == AUTOTUNE_MODE_MAF) {
        Serial.print(F("MAF Cell: [")); Serial.print(autotune.currentMafIdx);
        Serial.print(F("] LTFT: "));
        Serial.print(autotune.ltftMaf[autotune.currentMafIdx], 2);
        Serial.println(F("%"));
    }
    
    Serial.println(F("---------------------------------"));
    Serial.print(F("Avg LTFT: ")); Serial.print(autotuneGetAvgLtft(), 2); Serial.println(F("%"));
    Serial.print(F("Cells Learned: ")); Serial.println(autotune.cellsLearned);
    Serial.print(F("Total Adjustments: ")); Serial.println(autotune.totalAdjustments);
    Serial.println(F("=================================\n"));
}

void autotunePrintLtftGrid(void) {
    Serial.println(F("\n===== LTFT GRID (%) ====="));
    Serial.println(F("Showing cells with |LTFT| > 0.5%"));
    Serial.println(F("Format: [RPM][Load] = LTFT%"));
    Serial.println(F("-------------------------"));
    
    int printed = 0;
    for (int r = 0; r < TABLE_2D_SIZE; r++) {
        for (int l = 0; l < TABLE_2D_SIZE; l++) {
            float ltft = autotune.ltftGrid[r][l];
            if (fabsf(ltft) > 0.5f) {
                Serial.print(F("[")); Serial.print(r);
                Serial.print(F("][")); Serial.print(l);
                Serial.print(F("] "));
                Serial.print(calData.veTable.rpmBins[r], 0);
                Serial.print(F("/"));
                Serial.print(calData.veTable.loadBins[l], 0);
                Serial.print(F(": "));
                if (ltft > 0) Serial.print(F("+"));
                Serial.print(ltft, 1);
                Serial.println(F("%"));
                printed++;
            }
        }
    }
    
    if (printed == 0) {
        Serial.println(F("(no significant LTFT values)"));
    }
    
    Serial.print(F("\nTotal cells with data: ")); Serial.println(printed);
    Serial.println(F("=========================\n"));
}

void autotunePrintLtftMaf(void) {
    Serial.println(F("\n===== MAF LTFT (%) ====="));
    
    for (int i = 0; i < TABLE_1D_MAF_SIZE; i++) {
        if (fabsf(autotune.ltftMaf[i]) > 0.5f) {
            Serial.print(F("[")); Serial.print(i);
            Serial.print(F("] "));
            Serial.print(calData.mafTable.freqBins[i], 0);
            Serial.print(F(" Hz: "));
            if (autotune.ltftMaf[i] > 0) Serial.print(F("+"));
            Serial.print(autotune.ltftMaf[i], 1);
            Serial.println(F("%"));
        }
    }
    
    Serial.println(F("========================\n"));
}

// ============================================================================
// SERIAL INTERFACE
// ============================================================================

void autotuneProcessSerial(void) {
    if (!Serial.available()) return;
    
    char c = Serial.read();
    
    switch (c) {
        case 'T':
        case 't':
            autotuneCycleMode();
            break;
            
        case 'U':
        case 'u':
            autotunePrintLtftGrid();
            break;
            
        case 'Z':
        case 'z':
            autotuneReset();
            break;
            
        case 'F':
        case 'f':
            autotuneApplyNow();
            break;
            
        case 'G':
        case 'g':
            autotunePrintStatus();
            break;
            
        default:
            break;
    }
}
