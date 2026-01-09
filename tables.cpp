/*
 * tables.cpp - Consolidated Lookup Tables Implementation
 * ECU STM32F405 v8.2
 * 
 * All calibration tables centralized with:
 * - Linear interpolation for 1D tables
 * - Bilinear interpolation for 2D tables (16x16)
 * - Real-time calibration via +/- adjustment
 * - EEPROM persistence for tuned values
 * 
 * Units preserved from v8.1:
 * - Dwell/DeadTime: µs (microseconds)
 * - Temperatures: °C
 * - Enrichments: % (100 = 1.0x multiplier)
 */

#include "tables.h"
#include <EEPROM.h>
#include <math.h>

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================

// 1D Tables (Factory defaults)
DwellTable dwellTable;
DeadTimeTable deadTimeTable;
CrankingTable crankingTable;
WarmupTable warmupTable;
AfterstartTable afterstartTable;
AccelEnrichTable accelTable;
IdleRpmTable idleRpmTable;

// Calibration Data (Tunable)
CalibrationData calData;

// Current calibration mode
CalibrationMode currentCalMode = CAL_MODE_VE;

// ============================================================================
// STATIC STATE VARIABLES
// ============================================================================

static float currentAfterstartEnrich = 0;
static bool afterstartActive = false;
static uint32_t afterstartCycles = 0;
static float currentAccelEnrich = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static inline float safeDivide(float num, float denom) {
    return (fabsf(denom) > 0.001f) ? (num / denom) : 0.0f;
}

// 1D interpolation for float bins and uint16_t values
static float interpolate1D_uint16(float x, const float* xBins, const uint16_t* yValues, uint8_t size) {
    if (x <= xBins[0]) return (float)yValues[0];
    if (x >= xBins[size - 1]) return (float)yValues[size - 1];
    
    for (uint8_t i = 0; i < size - 1; i++) {
        if (x >= xBins[i] && x < xBins[i + 1]) {
            float frac = safeDivide(x - xBins[i], xBins[i + 1] - xBins[i]);
            return yValues[i] + frac * (yValues[i + 1] - yValues[i]);
        }
    }
    return (float)yValues[size - 1];
}

// 1D interpolation for int16_t bins and uint16_t values
static float interpolate1D_int16(float x, const int16_t* xBins, const uint16_t* yValues, uint8_t size) {
    if (x <= (float)xBins[0]) return (float)yValues[0];
    if (x >= (float)xBins[size - 1]) return (float)yValues[size - 1];
    
    for (uint8_t i = 0; i < size - 1; i++) {
        if (x >= (float)xBins[i] && x < (float)xBins[i + 1]) {
            float frac = safeDivide(x - xBins[i], (float)(xBins[i + 1] - xBins[i]));
            return yValues[i] + frac * (yValues[i + 1] - yValues[i]);
        }
    }
    return (float)yValues[size - 1];
}

// 1D interpolation for float bins and float values
static float interpolate1D_float(float x, const float* xBins, const float* yValues, uint8_t size) {
    if (x <= xBins[0]) return yValues[0];
    if (x >= xBins[size - 1]) return yValues[size - 1];
    
    for (uint8_t i = 0; i < size - 1; i++) {
        if (x >= xBins[i] && x < xBins[i + 1]) {
            float frac = safeDivide(x - xBins[i], xBins[i + 1] - xBins[i]);
            return yValues[i] + frac * (yValues[i + 1] - yValues[i]);
        }
    }
    return yValues[size - 1];
}

// 2D bilinear interpolation for 16x16 tables
static float interpolate2D_16x16(float rpm, float load, 
                                  const float* rpmBins, const float* loadBins,
                                  const float values[TABLE_2D_SIZE][TABLE_2D_SIZE]) {
    // Find RPM index
    uint8_t rpmIdx = 0;
    if (rpm <= rpmBins[0]) rpmIdx = 0;
    else if (rpm >= rpmBins[TABLE_2D_SIZE - 1]) rpmIdx = TABLE_2D_SIZE - 2;
    else {
        for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
            if (rpm >= rpmBins[i] && rpm < rpmBins[i + 1]) {
                rpmIdx = i;
                break;
            }
        }
    }
    
    // Find Load index
    uint8_t loadIdx = 0;
    if (load <= loadBins[0]) loadIdx = 0;
    else if (load >= loadBins[TABLE_2D_SIZE - 1]) loadIdx = TABLE_2D_SIZE - 2;
    else {
        for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
            if (load >= loadBins[i] && load < loadBins[i + 1]) {
                loadIdx = i;
                break;
            }
        }
    }
    
    // Calculate interpolation fractions
    float rpmFrac = safeDivide(rpm - rpmBins[rpmIdx], rpmBins[rpmIdx + 1] - rpmBins[rpmIdx]);
    float loadFrac = safeDivide(load - loadBins[loadIdx], loadBins[loadIdx + 1] - loadBins[loadIdx]);
    
    rpmFrac = constrain(rpmFrac, 0.0f, 1.0f);
    loadFrac = constrain(loadFrac, 0.0f, 1.0f);
    
    // Get 4 corner values
    float v00 = values[rpmIdx][loadIdx];
    float v01 = values[rpmIdx][loadIdx + 1];
    float v10 = values[rpmIdx + 1][loadIdx];
    float v11 = values[rpmIdx + 1][loadIdx + 1];
    
    // Bilinear interpolation
    float v0 = v00 + loadFrac * (v01 - v00);
    float v1 = v10 + loadFrac * (v11 - v10);
    
    return v0 + rpmFrac * (v1 - v0);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void tablesInit(void) {
    // Try to load from EEPROM first
    if (!tablesLoadFromEEPROM()) {
        Serial.println(F("EEPROM: No valid data, loading factory defaults"));
        tablesLoadDefaults();
    } else {
        Serial.println(F("EEPROM: Calibration loaded successfully"));
    }
}

void tablesLoadDefaults(void) {
    // ========================================================================
    // DWELL TABLE (Battery Voltage -> Dwell µs) - 8 points
    // ========================================================================
    float dwellVolts[8] = {6.0f, 8.0f, 10.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f};
    uint16_t dwellUs[8] = {6500, 5200, 4300, 3600, 3300, 3000, 2800, 2600};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        dwellTable.voltageBins[i] = dwellVolts[i];
        dwellTable.dwellValues[i] = dwellUs[i];
    }
    
    // ========================================================================
    // DEAD TIME TABLE (Battery Voltage -> Dead Time µs) - 8 points
    // ========================================================================
    float dtVolts[8] = {6.0f, 8.0f, 10.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f};
    uint16_t dtValues[8] = {2100, 1550, 1200, 980, 890, 800, 740, 680};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        deadTimeTable.voltageBins[i] = dtVolts[i];
        deadTimeTable.deadTimeValues[i] = dtValues[i];
    }
    
    // ========================================================================
    // CRANKING ENRICHMENT TABLE (CLT -> Enrichment %) - 8 points
    // ========================================================================
    int16_t crankTemps[8] = {-40, -20, 0, 20, 40, 60, 80, 100};
    uint16_t crankEnrich[8] = {450, 350, 250, 160, 125, 110, 100, 100};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        crankingTable.tempBins[i] = crankTemps[i];
        crankingTable.enrichValues[i] = crankEnrich[i];
    }
    crankingTable.crankingRpm = 400;
    crankingTable.primeMs = 100;
    crankingTable.floodClearTps = 80;
    
    // ========================================================================
    // WARMUP ENRICHMENT TABLE (CLT -> Added %) - 8 points
    // ========================================================================
    int16_t warmTemps[8] = {-40, -20, 0, 20, 40, 60, 80, 100};
    uint16_t warmEnrich[8] = {50, 40, 28, 18, 10, 4, 0, 0};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        warmupTable.tempBins[i] = warmTemps[i];
        warmupTable.enrichValues[i] = warmEnrich[i];
    }
    
    // ========================================================================
    // AFTERSTART ENRICHMENT TABLE - 8 points
    // ========================================================================
    int16_t asTemps[8] = {-20, 0, 20, 40, 60, 80, 90, 100};
    uint16_t asEnrich[8] = {50, 40, 30, 20, 15, 10, 5, 0};
    uint16_t asDecay[8] = {2, 2, 3, 4, 5, 6, 8, 10};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        afterstartTable.tempBins[i] = asTemps[i];
        afterstartTable.enrichValues[i] = asEnrich[i];
        afterstartTable.decayRates[i] = asDecay[i];
    }
    afterstartTable.holdCycles = 10;
    
    // ========================================================================
    // ACCELERATION ENRICHMENT TABLE - 8 points
    // ========================================================================
    float tpsDots[8] = {5.0f, 15.0f, 30.0f, 50.0f, 80.0f, 120.0f, 180.0f, 300.0f};
    uint16_t aeEnrich[8] = {2, 5, 10, 15, 22, 30, 40, 50};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        accelTable.tpsDotBins[i] = tpsDots[i];
        accelTable.enrichValues[i] = aeEnrich[i];
    }
    accelTable.decayRate = 10;
    accelTable.coldMultiplier = 150;
    
    // ========================================================================
    // IDLE TARGET RPM TABLE - 8 points
    // ========================================================================
    int16_t idleTemps[8] = {-20, 0, 20, 40, 60, 80, 90, 100};
    uint16_t idleRpms[8] = {1400, 1300, 1150, 1000, 900, 850, 850, 850};
    
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        idleRpmTable.tempBins[i] = idleTemps[i];
        idleRpmTable.rpmTargets[i] = idleRpms[i];
    }
    
    // ========================================================================
    // MAF CALIBRATION TABLE (Moved from .ino)
    // ========================================================================
    float mafHz[24] = {
        0, 500, 1000, 1500, 2000, 2500, 3000, 3500,
        4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500,
        8000, 8500, 9000, 9500, 10000, 11000, 12000, 14000
    };
    float mafFlow[24] = {
        0.0f, 2.5f, 5.5f, 10.0f, 16.0f, 24.0f, 34.0f, 46.0f,
        60.0f, 76.0f, 94.0f, 114.0f, 136.0f, 160.0f, 186.0f, 214.0f,
        244.0f, 276.0f, 310.0f, 346.0f, 384.0f, 466.0f, 556.0f, 750.0f
    };
    
    for (int i = 0; i < TABLE_1D_MAF_SIZE; i++) {
        calData.mafTable.freqBins[i] = mafHz[i];
        calData.mafTable.flowValues[i] = mafFlow[i];
    }
    
    // ========================================================================
    // VE TABLE 16x16 (Upgraded from 8x8 in .ino)
    // RPM bins: 500-8000 RPM
    // Load bins: 10-250 mg/stroke
    // ========================================================================
    float veRpmBins[16] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                           4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000};
    float veLoadBins[16] = {10, 20, 30, 45, 60, 75, 90, 105,
                            120, 140, 160, 180, 200, 220, 240, 260};
    
    for (int i = 0; i < TABLE_2D_SIZE; i++) {
        calData.veTable.rpmBins[i] = veRpmBins[i];
        calData.veTable.loadBins[i] = veLoadBins[i];
    }
    
    // VE values - interpolated from original 8x8
    // Base pattern: higher VE at mid-RPM, drops at extremes
    float veBase[16][16] = {
        //  10   20   30   45   60   75   90  105  120  140  160  180  200  220  240  260
        {25, 28, 32, 38, 45, 52, 58, 62, 65, 68, 70, 72, 73, 74, 75, 75},  // 500
        {28, 32, 38, 46, 54, 62, 68, 72, 75, 78, 80, 81, 82, 83, 84, 84},  // 1000
        {30, 36, 44, 54, 62, 70, 76, 80, 83, 85, 87, 88, 89, 89, 90, 90},  // 1500
        {32, 40, 50, 60, 68, 76, 82, 86, 88, 90, 91, 92, 92, 93, 93, 93},  // 2000
        {34, 44, 54, 64, 72, 80, 86, 89, 91, 92, 93, 94, 94, 94, 94, 94},  // 2500
        {36, 46, 56, 66, 74, 82, 88, 91, 93, 94, 95, 95, 95, 95, 95, 95},  // 3000
        {38, 48, 58, 68, 76, 84, 89, 92, 94, 95, 95, 96, 96, 96, 96, 96},  // 3500
        {40, 50, 60, 70, 78, 85, 90, 93, 94, 95, 96, 96, 96, 96, 96, 96},  // 4000
        {40, 50, 60, 70, 78, 85, 90, 92, 94, 95, 95, 95, 95, 95, 95, 95},  // 4500
        {40, 50, 60, 70, 78, 85, 89, 91, 93, 94, 94, 94, 94, 94, 94, 94},  // 5000
        {40, 50, 58, 68, 76, 83, 88, 90, 92, 93, 93, 93, 93, 93, 93, 93},  // 5500
        {40, 48, 56, 66, 74, 81, 86, 89, 91, 92, 92, 92, 92, 92, 92, 92},  // 6000
        {38, 46, 54, 64, 72, 79, 84, 87, 89, 90, 91, 91, 91, 91, 91, 91},  // 6500
        {36, 44, 52, 62, 70, 77, 82, 85, 87, 88, 89, 89, 89, 89, 89, 89},  // 7000
        {34, 42, 50, 60, 68, 75, 80, 83, 85, 86, 87, 87, 87, 87, 87, 87},  // 7500
        {32, 40, 48, 58, 66, 73, 78, 81, 83, 84, 85, 85, 85, 85, 85, 85}   // 8000
    };
    
    for (int r = 0; r < TABLE_2D_SIZE; r++) {
        for (int l = 0; l < TABLE_2D_SIZE; l++) {
            calData.veTable.values[r][l] = veBase[r][l];
        }
    }
    
    // ========================================================================
    // IGNITION TABLE 16x16 (Upgraded from 8x8 in .ino)
    // ========================================================================
    float ignRpmBins[16] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                            4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000};
    float ignLoadBins[16] = {10, 20, 30, 45, 60, 75, 90, 105,
                             120, 140, 160, 180, 200, 220, 240, 260};
    
    for (int i = 0; i < TABLE_2D_SIZE; i++) {
        calData.ignTable.rpmBins[i] = ignRpmBins[i];
        calData.ignTable.loadBins[i] = ignLoadBins[i];
    }
    
    // Ignition values - conservative base map
    // More advance at light load, less at high load
    float ignBase[16][16] = {
        //  10   20   30   45   60   75   90  105  120  140  160  180  200  220  240  260
        {10, 12, 14, 16, 18, 18, 16, 14, 12, 10,  8,  6,  5,  4,  3,  2},  // 500
        {12, 16, 20, 24, 26, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6},  // 1000
        {14, 18, 24, 28, 30, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10},  // 1500
        {16, 22, 28, 32, 34, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14},  // 2000
        {18, 24, 30, 34, 36, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16},  // 2500
        {20, 26, 32, 36, 38, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18},  // 3000
        {22, 28, 34, 38, 40, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20},  // 3500
        {24, 30, 36, 40, 42, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22},  // 4000
        {24, 30, 36, 40, 42, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22},  // 4500
        {24, 30, 36, 40, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20},  // 5000
        {24, 30, 36, 40, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18},  // 5500
        {24, 30, 36, 38, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16},  // 6000
        {22, 28, 34, 36, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14},  // 6500
        {20, 26, 32, 34, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12},  // 7000
        {18, 24, 30, 32, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10},  // 7500
        {16, 22, 28, 30, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8}   // 8000
    };
    
    for (int r = 0; r < TABLE_2D_SIZE; r++) {
        for (int l = 0; l < TABLE_2D_SIZE; l++) {
            calData.ignTable.values[r][l] = ignBase[r][l];
        }
    }
    
    // ========================================================================
    // AFR TARGET TABLE 16x16 (Upgraded from 8x8 in lambda.cpp)
    // E27 Brazilian gasoline - Stoich = 13.0
    // ========================================================================
    float afrRpmBins[16] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                            4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000};
    float afrLoadBins[16] = {10, 20, 30, 45, 60, 75, 90, 105,
                             120, 140, 160, 180, 200, 220, 240, 260};
    
    for (int i = 0; i < TABLE_2D_SIZE; i++) {
        calData.afrTable.rpmBins[i] = afrRpmBins[i];
        calData.afrTable.loadBins[i] = afrLoadBins[i];
    }
    
    // AFR values - stoich at cruise, rich at high load
    float afrBase[16][16] = {
        // 10    20    30    45    60    75    90   105   120   140   160   180   200   220   240   260
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 12.0, 11.8, 11.5, 11.2, 11.0, 10.8, 10.5, 10.5},
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 12.0, 11.8, 11.5, 11.2, 11.0, 10.8, 10.5, 10.5},
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 12.0, 11.8, 11.5, 11.2, 11.0, 10.8, 10.5, 10.5},
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 12.0, 11.8, 11.5, 11.0, 10.8, 10.5, 10.3, 10.3},
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 12.0, 11.5, 11.2, 10.8, 10.5, 10.3, 10.2, 10.2},
        {13.0, 13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.0, 11.8, 11.3, 11.0, 10.5, 10.3, 10.2, 10.0, 10.0},
        {13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 11.8, 11.5, 11.0, 10.8, 10.5, 10.2, 10.0, 10.0, 10.0},
        {13.0, 13.0, 13.0, 13.0, 12.8, 12.5, 12.0, 11.5, 11.2, 10.8, 10.5, 10.3, 10.0, 10.0, 10.0, 10.0},
        {13.0, 13.0, 13.0, 12.8, 12.5, 12.2, 11.8, 11.3, 11.0, 10.5, 10.3, 10.2, 10.0, 10.0, 10.0, 10.0},
        {13.0, 13.0, 13.0, 12.8, 12.5, 12.0, 11.5, 11.0, 10.8, 10.5, 10.2, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 13.0, 12.8, 12.5, 12.2, 11.8, 11.3, 10.8, 10.5, 10.3, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 13.0, 12.8, 12.5, 12.0, 11.5, 11.0, 10.5, 10.3, 10.2, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 12.8, 12.5, 12.2, 11.8, 11.3, 10.8, 10.5, 10.2, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 12.8, 12.5, 12.0, 11.5, 11.0, 10.5, 10.3, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 12.5, 12.2, 11.8, 11.3, 10.8, 10.5, 10.2, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
        {13.0, 12.5, 12.0, 11.5, 11.0, 10.5, 10.3, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}
    };
    
    for (int r = 0; r < TABLE_2D_SIZE; r++) {
        for (int l = 0; l < TABLE_2D_SIZE; l++) {
            calData.afrTable.values[r][l] = afrBase[r][l];
        }
    }
    
    // Set magic and version
    calData.magic = CALIBRATION_MAGIC;
    calData.version = 820;  // v8.2.0
    calData.checksum = calculateChecksum(&calData);
}

// ============================================================================
// STATE RESET
// ============================================================================

void tablesResetState(void) {
    currentAfterstartEnrich = 0;
    afterstartActive = false;
    afterstartCycles = 0;
    currentAccelEnrich = 0;
}

// ============================================================================
// 1D TABLE ACCESSORS
// ============================================================================

uint16_t getDwellTime(float voltage) {
    voltage = constrain(voltage, 6.0f, 16.0f);
    return (uint16_t)interpolate1D_uint16(voltage, dwellTable.voltageBins, 
                                           dwellTable.dwellValues, TABLE_1D_SIZE);
}

uint16_t getDeadTime(float voltage) {
    voltage = constrain(voltage, 6.0f, 16.0f);
    return (uint16_t)interpolate1D_uint16(voltage, deadTimeTable.voltageBins,
                                           deadTimeTable.deadTimeValues, TABLE_1D_SIZE);
}

float getCrankingEnrichment(float clt) {
    clt = constrain(clt, -40.0f, 100.0f);
    float enrich = interpolate1D_int16(clt, crankingTable.tempBins,
                                        crankingTable.enrichValues, TABLE_1D_SIZE);
    return enrich / 100.0f;  // Convert % to multiplier
}

float getWarmupEnrichment(float clt) {
    clt = constrain(clt, -40.0f, 100.0f);
    float enrich = interpolate1D_int16(clt, warmupTable.tempBins,
                                        warmupTable.enrichValues, TABLE_1D_SIZE);
    return enrich / 100.0f;  // Convert % to adder (0.45 = +45%)
}

float getAfterstartEnrichment(float clt, uint32_t cyclesSinceStart) {
    clt = constrain(clt, -20.0f, 100.0f);
    
    float initialEnrich = interpolate1D_int16(clt, afterstartTable.tempBins,
                                               afterstartTable.enrichValues, TABLE_1D_SIZE);
    float decayRate = interpolate1D_int16(clt, afterstartTable.tempBins,
                                           afterstartTable.decayRates, TABLE_1D_SIZE);
    
    if (!afterstartActive || cyclesSinceStart == 0) {
        afterstartActive = true;
        currentAfterstartEnrich = initialEnrich;
        afterstartCycles = 0;
    }
    
    if (cyclesSinceStart < afterstartTable.holdCycles) {
        return currentAfterstartEnrich / 100.0f;
    }
    
    if (currentAfterstartEnrich > 0) {
        float decayPerCycle = decayRate / 100.0f;
        currentAfterstartEnrich -= decayPerCycle;
        if (currentAfterstartEnrich < 0) {
            currentAfterstartEnrich = 0;
            afterstartActive = false;
        }
    }
    
    return currentAfterstartEnrich / 100.0f;
}

float getAccelEnrichment(float tpsDot, float clt) {
    if (tpsDot > accelTable.tpsDotBins[0]) {
        float baseEnrich = interpolate1D_float(tpsDot, accelTable.tpsDotBins,
                                                (float*)accelTable.enrichValues, TABLE_1D_SIZE);
        
        if (clt < 60.0f) {
            float coldFactor = 1.0f + (60.0f - clt) / 60.0f * 
                               (accelTable.coldMultiplier / 100.0f - 1.0f);
            baseEnrich *= coldFactor;
        }
        
        currentAccelEnrich += baseEnrich;
        if (currentAccelEnrich > 100.0f) currentAccelEnrich = 100.0f;
    } else {
        currentAccelEnrich *= (100.0f - accelTable.decayRate) / 100.0f;
        if (currentAccelEnrich < 0.5f) currentAccelEnrich = 0;
    }
    
    return currentAccelEnrich / 100.0f;
}

uint16_t getIdleTargetRpm(float clt) {
    clt = constrain(clt, -20.0f, 100.0f);
    return (uint16_t)interpolate1D_int16(clt, idleRpmTable.tempBins,
                                          idleRpmTable.rpmTargets, TABLE_1D_SIZE);
}

float getMafFlow(float freqHz) {
    if (freqHz <= 0) return 0.0f;
    freqHz = constrain(freqHz, 0.0f, 14000.0f);
    return interpolate1D_float(freqHz, calData.mafTable.freqBins,
                                calData.mafTable.flowValues, TABLE_1D_MAF_SIZE);
}

// ============================================================================
// 2D TABLE ACCESSORS
// ============================================================================

float getVe(float rpm, float loadMgStroke) {
    return interpolate2D_16x16(rpm, loadMgStroke,
                                calData.veTable.rpmBins,
                                calData.veTable.loadBins,
                                calData.veTable.values);
}

float getIgnition(float rpm, float loadMgStroke) {
    return interpolate2D_16x16(rpm, loadMgStroke,
                                calData.ignTable.rpmBins,
                                calData.ignTable.loadBins,
                                calData.ignTable.values);
}

float getAfrTarget(float rpm, float loadMgStroke) {
    return interpolate2D_16x16(rpm, loadMgStroke,
                                calData.afrTable.rpmBins,
                                calData.afrTable.loadBins,
                                calData.afrTable.values);
}

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

uint8_t findRpmIdx(float rpm) {
    const float* bins = calData.veTable.rpmBins;  // All tables share same RPM bins
    for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
        if (rpm < bins[i + 1]) return i;
    }
    return TABLE_2D_SIZE - 1;
}

uint8_t findLoadIdx(float loadMgStroke) {
    const float* bins = calData.veTable.loadBins;  // All tables share same load bins
    for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
        if (loadMgStroke < bins[i + 1]) return i;
    }
    return TABLE_2D_SIZE - 1;
}

// Find nearest cell (for more intuitive tuning)
// Returns the cell index that has the most influence on current value
void findNearestCell(float rpm, float load, uint8_t* rpmIdx, uint8_t* loadIdx, float* weight) {
    const float* rpmBins = calData.veTable.rpmBins;
    const float* loadBins = calData.veTable.loadBins;
    
    // Find base indices
    uint8_t rpmLo = 0, loadLo = 0;
    for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
        if (rpm >= rpmBins[i] && rpm < rpmBins[i + 1]) { rpmLo = i; break; }
        if (i == TABLE_2D_SIZE - 2) rpmLo = i;
    }
    for (uint8_t i = 0; i < TABLE_2D_SIZE - 1; i++) {
        if (load >= loadBins[i] && load < loadBins[i + 1]) { loadLo = i; break; }
        if (i == TABLE_2D_SIZE - 2) loadLo = i;
    }
    
    // Calculate fractions (0.0 = at lower bin, 1.0 = at upper bin)
    float rpmRange = rpmBins[rpmLo + 1] - rpmBins[rpmLo];
    float loadRange = loadBins[loadLo + 1] - loadBins[loadLo];
    
    float rpmFrac = (rpmRange > 0) ? (rpm - rpmBins[rpmLo]) / rpmRange : 0;
    float loadFrac = (loadRange > 0) ? (load - loadBins[loadLo]) / loadRange : 0;
    
    rpmFrac = constrain(rpmFrac, 0.0f, 1.0f);
    loadFrac = constrain(loadFrac, 0.0f, 1.0f);
    
    // Determine nearest cell (snap to closest)
    // If fraction > 0.5, use upper cell
    if (rpmFrac > 0.5f && rpmLo < TABLE_2D_SIZE - 1) {
        *rpmIdx = rpmLo + 1;
        rpmFrac = 1.0f - rpmFrac;  // Invert for weight calc
    } else {
        *rpmIdx = rpmLo;
    }
    
    if (loadFrac > 0.5f && loadLo < TABLE_2D_SIZE - 1) {
        *loadIdx = loadLo + 1;
        loadFrac = 1.0f - loadFrac;
    } else {
        *loadIdx = loadLo;
    }
    
    // Weight = how close we are to this cell (1.0 = exactly on cell)
    // Weight = (1 - rpmDistance) * (1 - loadDistance)
    *weight = (1.0f - rpmFrac) * (1.0f - loadFrac);
}

// Get interpolation weights for all 4 cells (for display)
void getInterpolationWeights(float rpm, float load, 
                              uint8_t* rpmLo, uint8_t* loadLo,
                              float* w00, float* w01, float* w10, float* w11) {
    const float* rpmBins = calData.veTable.rpmBins;
    const float* loadBins = calData.veTable.loadBins;
    
    *rpmLo = findRpmIdx(rpm);
    *loadLo = findLoadIdx(load);
    
    float rpmRange = rpmBins[*rpmLo + 1] - rpmBins[*rpmLo];
    float loadRange = loadBins[*loadLo + 1] - loadBins[*loadLo];
    
    float rpmFrac = (rpmRange > 0) ? (rpm - rpmBins[*rpmLo]) / rpmRange : 0;
    float loadFrac = (loadRange > 0) ? (load - loadBins[*loadLo]) / loadRange : 0;
    
    rpmFrac = constrain(rpmFrac, 0.0f, 1.0f);
    loadFrac = constrain(loadFrac, 0.0f, 1.0f);
    
    // Bilinear weights
    *w00 = (1.0f - rpmFrac) * (1.0f - loadFrac);  // [rpmLo][loadLo]
    *w01 = (1.0f - rpmFrac) * loadFrac;           // [rpmLo][loadLo+1]
    *w10 = rpmFrac * (1.0f - loadFrac);           // [rpmLo+1][loadLo]
    *w11 = rpmFrac * loadFrac;                    // [rpmLo+1][loadLo+1]
}

uint8_t findMafIdx(float freqHz) {
    const float* bins = calData.mafTable.freqBins;
    for (uint8_t i = 0; i < TABLE_1D_MAF_SIZE - 1; i++) {
        if (freqHz < bins[i + 1]) return i;
    }
    return TABLE_1D_MAF_SIZE - 1;
}

void adjustMafCell(uint8_t idx, bool increase) {
    idx = constrain(idx, 0, TABLE_1D_MAF_SIZE - 1);
    
    // MAF: +/- 1% (multiplicative) - minimum 0.5 g/s step for low values
    float currentVal = calData.mafTable.flowValues[idx];
    float step = currentVal * 0.01f;  // 1%
    if (step < 0.5f) step = 0.5f;     // Minimum step
    
    if (increase) {
        calData.mafTable.flowValues[idx] += step;
    } else {
        calData.mafTable.flowValues[idx] -= step;
    }
    
    // Constrain to valid range
    calData.mafTable.flowValues[idx] = constrain(
        calData.mafTable.flowValues[idx], 0.0f, 1000.0f);
}

void adjustCurrentCell(CalibrationMode mode, bool increase) {
    // This function should be called with current RPM and Load
    // The actual indices are found by the caller using findRpmIdx/findLoadIdx
}

// Adjust cell at specific indices (basic - no weight compensation)
void adjustCell(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase) {
    adjustSingleCell(mode, rpmIdx, loadIdx, increase, 1.0f);
}

// Internal: adjust a single cell with a multiplier
static void adjustSingleCell(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase, float multiplier) {
    if (rpmIdx >= TABLE_2D_SIZE || loadIdx >= TABLE_2D_SIZE) return;
    if (multiplier < 0.01f) return;  // Skip negligible adjustments
    
    switch (mode) {
        case CAL_MODE_VE: {
            // VE: +/- 1% base (multiplicative)
            float stepPercent = 1.0f * multiplier;
            float factor = increase ? (1.0f + stepPercent/100.0f) : (1.0f - stepPercent/100.0f);
            calData.veTable.values[rpmIdx][loadIdx] *= factor;
            calData.veTable.values[rpmIdx][loadIdx] = constrain(
                calData.veTable.values[rpmIdx][loadIdx], 10.0f, 150.0f);
            break;
        }
        case CAL_MODE_IGN: {
            // IGN: +/- 1 degree base (additive)
            float step = (increase ? 1.0f : -1.0f) * multiplier;
            calData.ignTable.values[rpmIdx][loadIdx] += step;
            calData.ignTable.values[rpmIdx][loadIdx] = constrain(
                calData.ignTable.values[rpmIdx][loadIdx], -10.0f, 50.0f);
            break;
        }
        case CAL_MODE_AFR: {
            // AFR: +/- 0.1 base (additive)
            float step = (increase ? 0.1f : -0.1f) * multiplier;
            calData.afrTable.values[rpmIdx][loadIdx] += step;
            calData.afrTable.values[rpmIdx][loadIdx] = constrain(
                calData.afrTable.values[rpmIdx][loadIdx], 9.0f, 16.0f);
            break;
        }
        default:
            break;
    }
}

// Adjust all 4 interpolation cells by the same amount
// This ensures the OUTPUT changes by exactly the nominal step
// Math: output = sum(wi*vi), delta = sum(wi*step) = step*sum(wi) = step*1.0 = step
void adjustCellsProportional(CalibrationMode mode, float rpm, float load, bool increase) {
    uint8_t rpmLo, loadLo;
    float w00, w01, w10, w11;
    
    getInterpolationWeights(rpm, load, &rpmLo, &loadLo, &w00, &w01, &w10, &w11);
    
    // Adjust ALL 4 cells by the SAME step (not weighted!)
    // This makes the interpolated output change by exactly the nominal step
    
    // Cell [rpmLo][loadLo]
    adjustSingleCell(mode, rpmLo, loadLo, increase, 1.0f);
    
    // Cell [rpmLo][loadLo+1]
    if (loadLo + 1 < TABLE_2D_SIZE && w01 > 0.01f) {
        adjustSingleCell(mode, rpmLo, loadLo + 1, increase, 1.0f);
    }
    
    // Cell [rpmLo+1][loadLo]
    if (rpmLo + 1 < TABLE_2D_SIZE && w10 > 0.01f) {
        adjustSingleCell(mode, rpmLo + 1, loadLo, increase, 1.0f);
    }
    
    // Cell [rpmLo+1][loadLo+1]
    if (rpmLo + 1 < TABLE_2D_SIZE && loadLo + 1 < TABLE_2D_SIZE && w11 > 0.01f) {
        adjustSingleCell(mode, rpmLo + 1, loadLo + 1, increase, 1.0f);
    }
}

// Legacy function for compatibility
void adjustCellWeighted(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase, float weight) {
    // Now redirects to proportional adjustment using stored RPM/Load
    // This is called from calibration.cpp which has the actual RPM/Load values
    // So we use the single cell method with compensation as fallback
    float clampedWeight = constrain(weight, 0.25f, 1.0f);
    float compensation = 1.0f / clampedWeight;
    adjustSingleCell(mode, rpmIdx, loadIdx, increase, compensation);
}

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================

uint32_t calculateChecksum(const CalibrationData* data) {
    uint32_t sum = 0;
    const uint8_t* ptr = (const uint8_t*)data;
    size_t len = sizeof(CalibrationData) - sizeof(uint32_t);  // Exclude checksum field
    
    for (size_t i = 0; i < len; i++) {
        sum += ptr[i];
        sum = (sum << 1) | (sum >> 31);  // Rotate
    }
    return sum;
}

bool tablesSaveToEEPROM(void) {
    calData.magic = CALIBRATION_MAGIC;
    calData.checksum = calculateChecksum(&calData);
    
    EEPROM.put(0, calData);
    
    // Verify write
    CalibrationData verify;
    EEPROM.get(0, verify);
    
    if (verify.magic == CALIBRATION_MAGIC && 
        verify.checksum == calData.checksum) {
        Serial.println(F("EEPROM: Save successful"));
        return true;
    }
    
    Serial.println(F("EEPROM: Save FAILED - verification error"));
    return false;
}

bool tablesLoadFromEEPROM(void) {
    CalibrationData stored;
    EEPROM.get(0, stored);
    
    // Check magic number
    if (stored.magic != CALIBRATION_MAGIC) {
        return false;
    }
    
    // Verify checksum
    uint32_t expectedChecksum = calculateChecksum(&stored);
    if (stored.checksum != expectedChecksum) {
        Serial.println(F("EEPROM: Checksum mismatch"));
        return false;
    }
    
    // Copy to active calibration
    memcpy(&calData, &stored, sizeof(CalibrationData));
    return true;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

uint16_t getPrimePulseMs(float clt) {
    float multiplier = getCrankingEnrichment(clt);
    return (uint16_t)(crankingTable.primeMs * multiplier);
}

bool isFloodClearActive(float tps, uint16_t rpm) {
    return (tps > crankingTable.floodClearTps) && (rpm < crankingTable.crankingRpm);
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void tablesPrintCurrentCell(float rpm, float load) {
    uint8_t rIdx = findRpmIdx(rpm);
    uint8_t lIdx = findLoadIdx(load);
    
    Serial.println(F("\n===== CURRENT CELL ====="));
    Serial.print(F("RPM: ")); Serial.print(rpm, 0);
    Serial.print(F(" -> Bin[")); Serial.print(rIdx); Serial.println(F("]"));
    Serial.print(F("Load: ")); Serial.print(load, 1);
    Serial.print(F(" mg/stk -> Bin[")); Serial.print(lIdx); Serial.println(F("]"));
    Serial.println(F("------------------------"));
    Serial.print(F("VE: ")); Serial.print(calData.veTable.values[rIdx][lIdx], 1); Serial.println(F("%"));
    Serial.print(F("IGN: ")); Serial.print(calData.ignTable.values[rIdx][lIdx], 1); Serial.println(F(" deg"));
    Serial.print(F("AFR: ")); Serial.println(calData.afrTable.values[rIdx][lIdx], 2);
    Serial.println(F("========================\n"));
}

void tablesPrintDwell(void) {
    Serial.println(F("\n===== DWELL TABLE (us) ====="));
    Serial.println(F("Voltage | Dwell"));
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        Serial.print(F("  ")); Serial.print(dwellTable.voltageBins[i], 1);
        Serial.print(F("V  |  ")); Serial.println(dwellTable.dwellValues[i]);
    }
    Serial.println(F("============================\n"));
}

void tablesPrintCranking(void) {
    Serial.println(F("\n===== CRANKING ENRICHMENT ====="));
    Serial.println(F("CLT (C) | Enrich (%)"));
    for (int i = 0; i < TABLE_1D_SIZE; i++) {
        Serial.print(F("  ")); 
        if (crankingTable.tempBins[i] >= 0) Serial.print(F(" "));
        Serial.print(crankingTable.tempBins[i]);
        Serial.print(F("    |    ")); Serial.println(crankingTable.enrichValues[i]);
    }
    Serial.println(F("===============================\n"));
}

void tablesPrintMaf(void) {
    Serial.println(F("\n===== MAF CALIBRATION ====="));
    Serial.println(F("Freq (Hz) | Flow (g/s)"));
    for (int i = 0; i < TABLE_1D_MAF_SIZE; i++) {
        Serial.print(F("  ")); Serial.print(calData.mafTable.freqBins[i], 0);
        Serial.print(F("    |  ")); Serial.println(calData.mafTable.flowValues[i], 1);
    }
    Serial.println(F("===========================\n"));
}

void tablesPrintVe(void) {
    Serial.println(F("\n===== VE TABLE 16x16 ====="));
    Serial.print(F("Load->  "));
    for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
        Serial.print(calData.veTable.loadBins[l], 0); Serial.print(F("  "));
    }
    Serial.println();
    
    for (int r = 0; r < TABLE_2D_SIZE; r += 2) {
        Serial.print(calData.veTable.rpmBins[r], 0); Serial.print(F(": "));
        for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
            Serial.print(calData.veTable.values[r][l], 0); Serial.print(F(" "));
        }
        Serial.println();
    }
    Serial.println(F("==========================\n"));
}

void tablesPrintIgnition(void) {
    Serial.println(F("\n===== IGN TABLE 16x16 ====="));
    Serial.print(F("Load->  "));
    for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
        Serial.print(calData.ignTable.loadBins[l], 0); Serial.print(F("  "));
    }
    Serial.println();
    
    for (int r = 0; r < TABLE_2D_SIZE; r += 2) {
        Serial.print(calData.ignTable.rpmBins[r], 0); Serial.print(F(": "));
        for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
            Serial.print(calData.ignTable.values[r][l], 0); Serial.print(F(" "));
        }
        Serial.println();
    }
    Serial.println(F("===========================\n"));
}

void tablesPrintAfr(void) {
    Serial.println(F("\n===== AFR TABLE 16x16 ====="));
    Serial.print(F("Load->  "));
    for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
        Serial.print(calData.afrTable.loadBins[l], 0); Serial.print(F(" "));
    }
    Serial.println();
    
    for (int r = 0; r < TABLE_2D_SIZE; r += 2) {
        Serial.print(calData.afrTable.rpmBins[r], 0); Serial.print(F(": "));
        for (int l = 0; l < TABLE_2D_SIZE; l += 2) {
            Serial.print(calData.afrTable.values[r][l], 1); Serial.print(F(" "));
        }
        Serial.println();
    }
    Serial.println(F("===========================\n"));
}
