/*
 * tables.h - Consolidated Lookup Tables
 * ECU STM32F405 v8.2
 * 
 * ALL calibration tables centralized here:
 * - 1D Tables: Dwell, DeadTime, Cranking, Warmup, Afterstart, Accel, IdleRpm, MAF
 * - 2D Tables: VE (16x16), Ignition (16x16), AFR Target (16x16)
 * 
 * Units:
 * - Dwell/DeadTime: microseconds (µs)
 * - Temperatures: Celsius (°C)
 * - Enrichments: percentage (%)
 * - Ignition: degrees BTDC (°)
 * - MAF: g/s
 * - Load: mg/stroke
 */

#ifndef TABLES_H
#define TABLES_H

#include <Arduino.h>

// ============================================================================
// TABLE DIMENSIONS
// ============================================================================

#define TABLE_1D_SIZE           8       // All 1D tables (standardized)
#define TABLE_1D_MAF_SIZE       24      // MAF calibration (high resolution)
#define TABLE_2D_SIZE           16      // All 2D tables (16x16 = 256 cells)

// ============================================================================
// 1D TABLE STRUCTURES
// ============================================================================

// Dwell Table (Battery Voltage -> Dwell Time in µs)
typedef struct {
    float voltageBins[TABLE_1D_SIZE];
    uint16_t dwellValues[TABLE_1D_SIZE];     // µs
} DwellTable;

// Injector Dead Time Table (Battery Voltage -> Dead Time in µs)
typedef struct {
    float voltageBins[TABLE_1D_SIZE];
    uint16_t deadTimeValues[TABLE_1D_SIZE];  // µs
} DeadTimeTable;

// Cranking Enrichment Table (CLT -> Enrichment %)
typedef struct {
    int16_t tempBins[TABLE_1D_SIZE];
    uint16_t enrichValues[TABLE_1D_SIZE];    // % (100 = no enrichment)
    uint16_t crankingRpm;                    // Below this = cranking
    uint16_t primeMs;                        // Prime pulse duration
    uint8_t floodClearTps;                   // TPS% for flood clear
} CrankingTable;

// Warmup Enrichment Table (CLT -> Added %)
typedef struct {
    int16_t tempBins[TABLE_1D_SIZE];
    uint16_t enrichValues[TABLE_1D_SIZE];    // % to add
} WarmupTable;

// Afterstart Enrichment Table (CLT-based initial + decay)
typedef struct {
    int16_t tempBins[TABLE_1D_SIZE];
    uint16_t enrichValues[TABLE_1D_SIZE];    // Initial % 
    uint16_t decayRates[TABLE_1D_SIZE];      // % per 100 cycles
    uint16_t holdCycles;                     // Cycles before decay
} AfterstartTable;

// Acceleration Enrichment Table (TPS Rate -> Added %)
typedef struct {
    float tpsDotBins[TABLE_1D_SIZE];         // %/second
    uint16_t enrichValues[TABLE_1D_SIZE];    // % to add
    uint8_t decayRate;                       // % decay per cycle
    uint8_t coldMultiplier;                  // % multiplier when cold
} AccelEnrichTable;

// Idle Target RPM Table (CLT -> Target RPM)
typedef struct {
    int16_t tempBins[TABLE_1D_SIZE];
    uint16_t rpmTargets[TABLE_1D_SIZE];
} IdleRpmTable;

// MAF Calibration Table (Frequency Hz -> Flow g/s) - Moved from .ino
typedef struct {
    float freqBins[TABLE_1D_MAF_SIZE];           // Hz
    float flowValues[TABLE_1D_MAF_SIZE];         // g/s
} MafCalTable;

// ============================================================================
// 2D TABLE STRUCTURES (16x16)
// ============================================================================

// VE Table (RPM x Load -> Volumetric Efficiency %) - Moved from .ino
typedef struct {
    float rpmBins[TABLE_2D_SIZE];
    float loadBins[TABLE_2D_SIZE];               // mg/stroke
    float values[TABLE_2D_SIZE][TABLE_2D_SIZE];  // VE %
} VeTable;

// Ignition Table (RPM x Load -> Advance °BTDC) - Moved from .ino
typedef struct {
    float rpmBins[TABLE_2D_SIZE];
    float loadBins[TABLE_2D_SIZE];               // mg/stroke
    float values[TABLE_2D_SIZE][TABLE_2D_SIZE];  // degrees
} IgnitionTable;

// AFR Target Table (RPM x Load -> Target AFR) - Moved from lambda.cpp
typedef struct {
    float rpmBins[TABLE_2D_SIZE];
    float loadBins[TABLE_2D_SIZE];               // mg/stroke
    float values[TABLE_2D_SIZE][TABLE_2D_SIZE];  // AFR
} AfrTargetTable;

// ============================================================================
// CALIBRATION DATA (For EEPROM Storage)
// ============================================================================

#define CALIBRATION_MAGIC       0x45435538      // "ECU8" signature

typedef struct {
    uint32_t magic;                             // Validation signature
    uint32_t version;                           // Calibration version
    
    // 2D Tables (These are the ones most commonly tuned)
    VeTable veTable;
    IgnitionTable ignTable;
    AfrTargetTable afrTable;
    
    // MAF Calibration
    MafCalTable mafTable;
    
    // Checksum for data integrity
    uint32_t checksum;
} CalibrationData;

// ============================================================================
// GLOBAL TABLE INSTANCES
// ============================================================================

// 1D Tables (Factory defaults, rarely changed at runtime)
extern DwellTable dwellTable;
extern DeadTimeTable deadTimeTable;
extern CrankingTable crankingTable;
extern WarmupTable warmupTable;
extern AfterstartTable afterstartTable;
extern AccelEnrichTable accelTable;
extern IdleRpmTable idleRpmTable;

// Calibration Data (Tunable, stored in EEPROM)
extern CalibrationData calData;

// ============================================================================
// CALIBRATION MODE
// ============================================================================

typedef enum {
    CAL_MODE_VE = 0,
    CAL_MODE_IGN,
    CAL_MODE_AFR,
    CAL_MODE_MAF
} CalibrationMode;

// ============================================================================
// INITIALIZATION
// ============================================================================

void tablesInit(void);
void tablesLoadDefaults(void);
void tablesResetState(void);

// ============================================================================
// 1D TABLE ACCESSORS
// ============================================================================

uint16_t getDwellTime(float voltage);
uint16_t getDeadTime(float voltage);
float getCrankingEnrichment(float clt);
float getWarmupEnrichment(float clt);
float getAfterstartEnrichment(float clt, uint32_t cyclesSinceStart);
float getAccelEnrichment(float tpsDot, float clt);
uint16_t getIdleTargetRpm(float clt);
float getMafFlow(float freqHz);

// ============================================================================
// 2D TABLE ACCESSORS (16x16)
// ============================================================================

float getVe(float rpm, float loadMgStroke);
float getIgnition(float rpm, float loadMgStroke);
float getAfrTarget(float rpm, float loadMgStroke);

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

// Find current cell indices (lower bound)
uint8_t findRpmIdx(float rpm);
uint8_t findLoadIdx(float loadMgStroke);
uint8_t findMafIdx(float freqHz);

// Find nearest cell for tuning (snaps to closest cell)
void findNearestCell(float rpm, float load, uint8_t* rpmIdx, uint8_t* loadIdx, float* weight);

// Get interpolation weights for all 4 cells
void getInterpolationWeights(float rpm, float load, 
                              uint8_t* rpmLo, uint8_t* loadLo,
                              float* w00, float* w01, float* w10, float* w11);

// Real-time adjustment
void adjustCurrentCell(CalibrationMode mode, bool increase);
void adjustCell(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase);
void adjustCellWeighted(CalibrationMode mode, uint8_t rpmIdx, uint8_t loadIdx, bool increase, float weight);
void adjustCellsProportional(CalibrationMode mode, float rpm, float load, bool increase);
void adjustMafCell(uint8_t idx, bool increase);

// EEPROM Storage
bool tablesSaveToEEPROM(void);
bool tablesLoadFromEEPROM(void);
uint32_t calculateChecksum(const CalibrationData* data);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

uint16_t getPrimePulseMs(float clt);
bool isFloodClearActive(float tps, uint16_t rpm);

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void tablesPrintDwell(void);
void tablesPrintCranking(void);
void tablesPrintVe(void);
void tablesPrintIgnition(void);
void tablesPrintAfr(void);
void tablesPrintMaf(void);
void tablesPrintCurrentCell(float rpm, float load);

#endif // TABLES_H
