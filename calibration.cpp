/*
 * calibration.cpp - Real-Time Calibration System Implementation
 * ECU STM32F405 v8.2
 * 
 * Allows tuning VE, Ignition, and AFR tables in real-time
 * using simple keyboard commands via Serial.
 * 
 * Commands:
 *   V - Select VE table (1% steps)
 *   I - Select Ignition table (1° steps)
 *   A - Select AFR table (0.1 steps)
 *   P - Toggle Lambda PID auto-correction
 *   + - Increase current cell value
 *   - - Decrease current cell value
 *   S - Save calibration to EEPROM
 *   L - Load calibration from EEPROM
 *   R - Reset to factory defaults
 *   ? - Show current cell status
 *   H - Show help
 * 
 * The system tracks which cell the ECU is currently using
 * based on RPM and Load, allowing real-time adjustment
 * while driving or on dyno.
 */

#include "calibration.h"
#include "lambda.h"
#include <math.h>

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static void printAdjustment(bool increase);

// ============================================================================
// GLOBAL STATE
// ============================================================================

CalibrationState calState;

// ============================================================================
// INITIALIZATION
// ============================================================================

void calibrationInit(void) {
    calState.mode = CAL_MODE_VE;
    calState.lastRpmIdx = 0;
    calState.lastLoadIdx = 0;
    calState.lastMafIdx = 0;
    calState.lastRpm = 0;
    calState.lastLoad = 0;
    calState.lastMafHz = 0;
    calState.cellWeight = 1.0f;
    calState.enabled = true;
    calState.lastAdjustTime = 0;
}

// ============================================================================
// POSITION TRACKING
// ============================================================================

void calibrationUpdatePosition(float rpm, float loadMgStroke) {
    calState.lastRpm = rpm;
    calState.lastLoad = loadMgStroke;
    
    // Use nearest cell for more intuitive tuning
    findNearestCell(rpm, loadMgStroke, 
                    &calState.lastRpmIdx, 
                    &calState.lastLoadIdx,
                    &calState.cellWeight);
}

void calibrationUpdateMaf(float mafHz) {
    calState.lastMafHz = mafHz;
    calState.lastMafIdx = findMafIdx(mafHz);
}

// ============================================================================
// MODE NAME
// ============================================================================

const char* calibrationGetModeName(void) {
    switch (calState.mode) {
        case CAL_MODE_VE:  return "VE (1%)";
        case CAL_MODE_IGN: return "IGN (1 deg)";
        case CAL_MODE_AFR: return "AFR (0.1)";
        case CAL_MODE_MAF: return "MAF (1%)";
        default:           return "UNKNOWN";
    }
}

// ============================================================================
// SERIAL PROCESSING
// ============================================================================

void calibrationProcessSerial(void) {
    if (!Serial.available()) return;
    
    char c = Serial.read();
    
    // Debounce rapid keypresses
    uint32_t now = millis();
    bool canAdjust = (now - calState.lastAdjustTime) > 50;
    
    switch (c) {
        // ===== MODE SELECTION =====
        case 'v':
        case 'V':
            calState.mode = CAL_MODE_VE;
            Serial.println(F("\n>> MODE: VE Table (Volumetric Efficiency)"));
            Serial.println(F("   Step: 1% per keypress"));
            calibrationPrintStatus();
            break;
            
        case 'i':
        case 'I':
            calState.mode = CAL_MODE_IGN;
            Serial.println(F("\n>> MODE: Ignition Table"));
            Serial.println(F("   Step: 1 degree per keypress"));
            calibrationPrintStatus();
            break;
            
        case 'a':
        case 'A':
            calState.mode = CAL_MODE_AFR;
            Serial.println(F("\n>> MODE: AFR Target Table"));
            Serial.println(F("   Step: 0.1 AFR per keypress"));
            calibrationPrintStatus();
            break;
        
        case 'm':  // lowercase only - 'M' prints MAF table
            calState.mode = CAL_MODE_MAF;
            Serial.println(F("\n>> MODE: MAF Calibration Table"));
            Serial.println(F("   Step: 1% per keypress"));
            Serial.println(F("   Adjust flow (g/s) for current frequency"));
            calibrationPrintStatus();
            break;
        
        // ===== LAMBDA PID TOGGLE =====
        case 'p':
        case 'P':
            lambdaSetAutoCorrect(!lambdaIsAutoCorrectEnabled());
            break;
        
        // ===== ADJUSTMENT =====
        case '+':
        case '=':  // Also accept '=' (same key without shift)
            if (canAdjust) {
                calState.lastAdjustTime = now;
                if (calState.mode == CAL_MODE_MAF) {
                    adjustMafCell(calState.lastMafIdx, true);
                } else {
                    // Adjust all 4 interpolation cells proportionally
                    adjustCellsProportional(calState.mode, calState.lastRpm, calState.lastLoad, true);
                }
                printAdjustment(true);
            }
            break;
            
        case '-':
        case '_':  // Also accept '_' (same key with shift)
            if (canAdjust) {
                calState.lastAdjustTime = now;
                if (calState.mode == CAL_MODE_MAF) {
                    adjustMafCell(calState.lastMafIdx, false);
                } else {
                    // Adjust all 4 interpolation cells proportionally
                    adjustCellsProportional(calState.mode, calState.lastRpm, calState.lastLoad, false);
                }
                printAdjustment(false);
            }
            break;
        
        // ===== STORAGE =====
        case 's':
        case 'S':
            Serial.println(F("\n>> SAVING to EEPROM..."));
            if (tablesSaveToEEPROM()) {
                Serial.println(F("   Calibration saved successfully!"));
            } else {
                Serial.println(F("   ERROR: Save failed!"));
            }
            break;
            
        case 'l':
        case 'L':
            Serial.println(F("\n>> LOADING from EEPROM..."));
            if (tablesLoadFromEEPROM()) {
                Serial.println(F("   Calibration loaded successfully!"));
            } else {
                Serial.println(F("   No valid calibration found, using current values"));
            }
            break;
            
        case 'r':
        case 'R':
            Serial.println(F("\n>> RESET to factory defaults..."));
            Serial.println(F("   Type 'Y' to confirm, any other key to cancel"));
            while (!Serial.available()) { delay(10); }
            if (Serial.read() == 'Y') {
                tablesLoadDefaults();
                Serial.println(F("   Factory defaults loaded!"));
            } else {
                Serial.println(F("   Reset cancelled."));
            }
            break;
        
        // ===== STATUS =====
        case '?':
            calibrationPrintStatus();
            break;
            
        case 'h':
        case 'H':
            calibrationPrintHelp();
            break;
        
        // ===== TABLE DISPLAY =====
        case '1':
            tablesPrintVe();
            break;
        case '2':
            tablesPrintIgnition();
            break;
        case '3':
            tablesPrintAfr();
            break;
        case '4':
            tablesPrintMaf();
            break;
        case '5':
            tablesPrintDwell();
            break;
        case '6':
            tablesPrintCranking();
            break;
            
        default:
            // Ignore unknown characters
            break;
    }
}

// ============================================================================
// PRINT FUNCTIONS
// ============================================================================

static void printAdjustment(bool increase) {
    Serial.print(increase ? F("[+] ") : F("[-] "));
    Serial.print(calibrationGetModeName());
    
    if (calState.mode == CAL_MODE_MAF) {
        // MAF is 1D table
        Serial.print(F(" ["));
        Serial.print(calState.lastMafIdx);
        Serial.print(F("] @ "));
        Serial.print(calData.mafTable.freqBins[calState.lastMafIdx], 0);
        Serial.print(F(" Hz = "));
        Serial.print(calData.mafTable.flowValues[calState.lastMafIdx], 1);
        Serial.println(F(" g/s"));
    } else {
        // 2D tables - show 4-cell adjustment info
        uint8_t rpmLo, loadLo;
        float w00, w01, w10, w11;
        getInterpolationWeights(calState.lastRpm, calState.lastLoad, 
                                &rpmLo, &loadLo, &w00, &w01, &w10, &w11);
        
        // Count how many cells were adjusted
        int cellCount = 1;  // Always at least [rpmLo][loadLo]
        if (w01 > 0.01f) cellCount++;
        if (w10 > 0.01f) cellCount++;
        if (w11 > 0.01f) cellCount++;
        
        Serial.print(F(" ")); Serial.print(cellCount); Serial.print(F("-cell"));
        
        // Show interpolated output value
        Serial.print(F(" out="));
        switch (calState.mode) {
            case CAL_MODE_VE: {
                float v00 = calData.veTable.values[rpmLo][loadLo];
                float v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.veTable.values[rpmLo][loadLo+1] : v00;
                float v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.veTable.values[rpmLo+1][loadLo] : v00;
                float v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                            calData.veTable.values[rpmLo+1][loadLo+1] : v00;
                float interp = w00*v00 + w01*v01 + w10*v10 + w11*v11;
                Serial.print(interp, 1);
                Serial.println(F("%"));
                break;
            }
            case CAL_MODE_IGN: {
                float v00 = calData.ignTable.values[rpmLo][loadLo];
                float v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.ignTable.values[rpmLo][loadLo+1] : v00;
                float v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.ignTable.values[rpmLo+1][loadLo] : v00;
                float v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                            calData.ignTable.values[rpmLo+1][loadLo+1] : v00;
                float interp = w00*v00 + w01*v01 + w10*v10 + w11*v11;
                Serial.print(interp, 1);
                Serial.println(F(" deg"));
                break;
            }
            case CAL_MODE_AFR: {
                float v00 = calData.afrTable.values[rpmLo][loadLo];
                float v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.afrTable.values[rpmLo][loadLo+1] : v00;
                float v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.afrTable.values[rpmLo+1][loadLo] : v00;
                float v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                            calData.afrTable.values[rpmLo+1][loadLo+1] : v00;
                float interp = w00*v00 + w01*v01 + w10*v10 + w11*v11;
                Serial.println(interp, 2);
                break;
            }
            default:
                Serial.println();
                break;
        }
    }
}

void calibrationPrintStatus(void) {
    Serial.println(F("\n======== CALIBRATION STATUS ========"));
    Serial.print(F("Mode: ")); Serial.println(calibrationGetModeName());
    Serial.println(F("------------------------------------"));
    
    if (calState.mode == CAL_MODE_MAF) {
        // MAF mode - show frequency and flow
        Serial.print(F("MAF:  ")); Serial.print(calState.lastMafHz, 0);
        Serial.print(F(" Hz -> Bin[")); Serial.print(calState.lastMafIdx);
        Serial.print(F("] (")); Serial.print(calData.mafTable.freqBins[calState.lastMafIdx], 0);
        Serial.print(F("-")); 
        if (calState.lastMafIdx < TABLE_1D_MAF_SIZE - 1) {
            Serial.print(calData.mafTable.freqBins[calState.lastMafIdx + 1], 0);
        } else {
            Serial.print(F("max"));
        }
        Serial.println(F(" Hz)"));
        
        Serial.println(F("------------------------------------"));
        Serial.print(F("Current Flow: ")); 
        Serial.print(calData.mafTable.flowValues[calState.lastMafIdx], 1);
        Serial.println(F(" g/s"));
        
        // Show neighboring cells for context
        Serial.println(F("Neighbors:"));
        if (calState.lastMafIdx > 0) {
            Serial.print(F("  [")); Serial.print(calState.lastMafIdx - 1);
            Serial.print(F("] ")); Serial.print(calData.mafTable.freqBins[calState.lastMafIdx - 1], 0);
            Serial.print(F(" Hz = ")); Serial.print(calData.mafTable.flowValues[calState.lastMafIdx - 1], 1);
            Serial.println(F(" g/s"));
        }
        Serial.print(F("  [")); Serial.print(calState.lastMafIdx);
        Serial.print(F("] ")); Serial.print(calData.mafTable.freqBins[calState.lastMafIdx], 0);
        Serial.print(F(" Hz = ")); Serial.print(calData.mafTable.flowValues[calState.lastMafIdx], 1);
        Serial.println(F(" g/s  <-- CURRENT"));
        if (calState.lastMafIdx < TABLE_1D_MAF_SIZE - 1) {
            Serial.print(F("  [")); Serial.print(calState.lastMafIdx + 1);
            Serial.print(F("] ")); Serial.print(calData.mafTable.freqBins[calState.lastMafIdx + 1], 0);
            Serial.print(F(" Hz = ")); Serial.print(calData.mafTable.flowValues[calState.lastMafIdx + 1], 1);
            Serial.println(F(" g/s"));
        }
    } else {
        // 2D table modes - show RPM and Load with interpolation info
        uint8_t rpmLo, loadLo;
        float w00, w01, w10, w11;
        getInterpolationWeights(calState.lastRpm, calState.lastLoad,
                                &rpmLo, &loadLo, &w00, &w01, &w10, &w11);
        
        Serial.print(F("Position: RPM=")); Serial.print(calState.lastRpm, 0);
        Serial.print(F(", Load=")); Serial.print(calState.lastLoad, 1);
        Serial.println(F(" mg/stk"));
        
        Serial.println(F("------------------------------------"));
        Serial.println(F("Interpolation Grid (all 4 adjusted):"));
        
        // Get table pointer based on mode
        float v00, v01, v10, v11;
        const char* unit = "";
        switch (calState.mode) {
            case CAL_MODE_VE:
                v00 = calData.veTable.values[rpmLo][loadLo];
                v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.veTable.values[rpmLo][loadLo+1] : v00;
                v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.veTable.values[rpmLo+1][loadLo] : v00;
                v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                      calData.veTable.values[rpmLo+1][loadLo+1] : v00;
                unit = "%";
                break;
            case CAL_MODE_IGN:
                v00 = calData.ignTable.values[rpmLo][loadLo];
                v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.ignTable.values[rpmLo][loadLo+1] : v00;
                v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.ignTable.values[rpmLo+1][loadLo] : v00;
                v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                      calData.ignTable.values[rpmLo+1][loadLo+1] : v00;
                unit = "deg";
                break;
            case CAL_MODE_AFR:
            default:
                v00 = calData.afrTable.values[rpmLo][loadLo];
                v01 = (loadLo+1 < TABLE_2D_SIZE) ? calData.afrTable.values[rpmLo][loadLo+1] : v00;
                v10 = (rpmLo+1 < TABLE_2D_SIZE) ? calData.afrTable.values[rpmLo+1][loadLo] : v00;
                v11 = (rpmLo+1 < TABLE_2D_SIZE && loadLo+1 < TABLE_2D_SIZE) ? 
                      calData.afrTable.values[rpmLo+1][loadLo+1] : v00;
                unit = "";
                break;
        }
        
        // Row 1: [rpmLo][loadLo] and [rpmLo][loadLo+1]
        Serial.print(F("  [")); Serial.print(rpmLo); Serial.print(F("][")); Serial.print(loadLo);
        Serial.print(F("] ")); Serial.print(v00, 1); Serial.print(unit);
        Serial.print(F(" (")); Serial.print((int)(w00 * 100)); Serial.print(F("%)"));
        if (loadLo + 1 < TABLE_2D_SIZE) {
            Serial.print(F("   [")); Serial.print(rpmLo); Serial.print(F("][")); Serial.print(loadLo + 1);
            Serial.print(F("] ")); Serial.print(v01, 1); Serial.print(unit);
            Serial.print(F(" (")); Serial.print((int)(w01 * 100)); Serial.print(F("%)"));
        }
        Serial.println();
        
        // Row 2: [rpmLo+1][loadLo] and [rpmLo+1][loadLo+1]
        if (rpmLo + 1 < TABLE_2D_SIZE) {
            Serial.print(F("  [")); Serial.print(rpmLo + 1); Serial.print(F("][")); Serial.print(loadLo);
            Serial.print(F("] ")); Serial.print(v10, 1); Serial.print(unit);
            Serial.print(F(" (")); Serial.print((int)(w10 * 100)); Serial.print(F("%)"));
            if (loadLo + 1 < TABLE_2D_SIZE) {
                Serial.print(F("   [")); Serial.print(rpmLo + 1); Serial.print(F("][")); Serial.print(loadLo + 1);
                Serial.print(F("] ")); Serial.print(v11, 1); Serial.print(unit);
                Serial.print(F(" (")); Serial.print((int)(w11 * 100)); Serial.print(F("%)"));
            }
            Serial.println();
        }
        
        // Interpolated output
        float interp = w00*v00 + w01*v01 + w10*v10 + w11*v11;
        Serial.println(F("------------------------------------"));
        Serial.print(F("Interpolated OUTPUT: ")); Serial.print(interp, 2); Serial.println(unit);
        Serial.println(F("+/- adjusts ALL 4 cells by same step"));
        
        // Show bin ranges
        Serial.println(F("------------------------------------"));
        Serial.print(F("RPM bins: ")); 
        Serial.print(calData.veTable.rpmBins[rpmLo], 0); Serial.print(F("-"));
        Serial.println(calData.veTable.rpmBins[rpmLo + 1], 0);
        Serial.print(F("Load bins: ")); 
        Serial.print(calData.veTable.loadBins[loadLo], 0); Serial.print(F("-"));
        Serial.print(calData.veTable.loadBins[loadLo + 1], 0); Serial.println(F(" mg/stk"));
    }
    
    Serial.println(F("====================================\n"));
}

void calibrationPrintHelp(void) {
    Serial.println(F("\n"));
    Serial.println(F("========================================"));
    Serial.println(F("    ECU v8.2 CALIBRATION SYSTEM"));
    Serial.println(F("========================================"));
    Serial.println(F(""));
    Serial.println(F("FUEL MODEL:"));
    Serial.println(F("  MAF Mode:    Tune AFR + MAF calibration"));
    Serial.println(F("  Alpha-N:     Tune VE (air) + AFR (mix)"));
    Serial.println(F(""));
    Serial.println(F("TABLE SELECTION:"));
    Serial.println(F("  V - VE Table       (Alpha-N air mass)"));
    Serial.println(F("  I - Ignition Table (timing)"));
    Serial.println(F("  A - AFR Table      (fuel mixture)"));
    Serial.println(F("  m - MAF Table      (sensor calibration)"));
    Serial.println(F(""));
    Serial.println(F("LAMBDA CONTROL:"));
    Serial.println(F("  P - Toggle Lambda PID auto-correct"));
    Serial.println(F("      ON:  PID corrects to hit AFR target"));
    Serial.println(F("      OFF: Correction frozen (raw tune)"));
    Serial.println(F(""));
    Serial.println(F("ADJUSTMENT:"));
    Serial.println(F("  + - Increase current cell value"));
    Serial.println(F("  - - Decrease current cell value"));
    Serial.println(F("  VE:  1% per step (multiplicative)"));
    Serial.println(F("  IGN: 1 deg per step (additive)"));
    Serial.println(F("  AFR: 0.1 per step (additive)"));
    Serial.println(F("  MAF: 1% per step (min 0.5 g/s)"));
    Serial.println(F(""));
    Serial.println(F("STORAGE:"));
    Serial.println(F("  S - Save to EEPROM (permanent)"));
    Serial.println(F("  L - Load from EEPROM"));
    Serial.println(F("  R - Reset to factory defaults"));
    Serial.println(F(""));
    Serial.println(F("DISPLAY:"));
    Serial.println(F("  ? - Show current cell status"));
    Serial.println(F("  H - Show this help"));
    Serial.println(F("  1 - Print VE table"));
    Serial.println(F("  2 - Print IGN table"));
    Serial.println(F("  3 - Print AFR table"));
    Serial.println(F("  4 - Print MAF calibration"));
    Serial.println(F("  5 - Print Dwell table"));
    Serial.println(F("  6 - Print Cranking table"));
    Serial.println(F(""));
    Serial.println(F("MAF CALIBRATION WORKFLOW:"));
    Serial.println(F("  1. Press m to select MAF table"));
    Serial.println(F("  2. Rev engine to target frequency"));
    Serial.println(F("  3. Compare AFR to target (wideband)"));
    Serial.println(F("  4. If lean: + (increase flow)"));
    Serial.println(F("  5. If rich: - (decrease flow)"));
    Serial.println(F("  6. Save when done (S)"));
    Serial.println(F(""));
    Serial.println(F("Note: M (uppercase) prints MAF table"));
    Serial.println(F("========================================\n"));
}
