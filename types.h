/*
 * types.h - Data Types and Structures
 * ECU STM32F405 v8.2
 */

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// ============================================================================

// ============================================================================
// SENSOR DATA
// ============================================================================

typedef struct {
    // Analog sensors
    volatile float tpsPercent;          // Throttle position (0-100%)
    volatile float mapKpa;              // Manifold pressure (kPa)
    volatile float cltCelsius;          // Coolant temperature (°C)
    volatile float iatCelsius;          // Intake air temperature (°C)
    volatile float lambda;              // Lambda (1.0 = stoich)
    volatile float batteryVoltage;      // Battery voltage
    
    // MAF sensor
    volatile float mafGramsSec;         // Mass air flow (g/s)
    volatile float mafFrequencyHz;      // MAF frequency
    
    // Calculated values
    volatile float loadMgStroke;        // Air mass per stroke (mg)
    volatile float afrTarget;           // Target AFR
    
    // Status flags
    bool tpsValid;
    bool cltValid;
    bool iatValid;
    bool mapValid;
    bool mafValid;
} SensorData;

// ============================================================================
// ENGINE STATUS
// ============================================================================

typedef struct {
    // Running state
    bool isCranking;
    bool isRunning;
    bool isWarmedUp;
    bool isIdling;
    
    // Fuel state
    float fuelPulseWidth;               // Current PW (µs)
    float fuelCorrection;               // Total correction multiplier
    
    // Ignition state
    float ignitionAdvance;              // Current advance (°BTDC)
    float dwellTime;                    // Current dwell (µs)
    
    // Lambda control
    float lambdaCorrection;             // Lambda PID output
    float lambdaIntegral;               // Lambda PID integral
    
    // Rev limiter
    bool revLimitActive;
    bool softLimitActive;
    
    // Idle control
    float idleCorrection;               // Idle timing correction
    bool idleActive;
    
    // Error flags
    uint32_t errorFlags;
} EngineStatus;

// ============================================================================
// ERROR FLAGS
// ============================================================================

#define ERR_SYNC_LOST           (1 << 0)
#define ERR_TPS_FAULT           (1 << 1)
#define ERR_CLT_FAULT           (1 << 2)
#define ERR_IAT_FAULT           (1 << 3)
#define ERR_MAP_FAULT           (1 << 4)
#define ERR_MAF_FAULT           (1 << 5)
#define ERR_LAMBDA_FAULT        (1 << 6)
#define ERR_VBAT_LOW            (1 << 7)
#define ERR_VBAT_HIGH           (1 << 8)
#define ERR_TLE_FAULT           (1 << 9)
#define ERR_TLE_OVERTEMP        (1 << 10)
#define ERR_OVERDWELL           (1 << 11)
#define ERR_EMERGENCY_SHUTDOWN  (1 << 15)

// ============================================================================
// CALIBRATION TABLE TYPES
// ============================================================================

// 2D table for lookups (e.g., VE, AFR, IGN)
#define TABLE_SIZE_X    16
#define TABLE_SIZE_Y    16

typedef struct {
    float values[TABLE_SIZE_X][TABLE_SIZE_Y];
    float xBins[TABLE_SIZE_X];          // e.g., RPM
    float yBins[TABLE_SIZE_Y];          // e.g., Load
} Table2D;

// 1D curve for simple lookups
#define CURVE_SIZE      16

typedef struct {
    float values[CURVE_SIZE];
    float xBins[CURVE_SIZE];
} Curve1D;

// ============================================================================
// ANGLE TYPES (Fixed-point for precision)
// ============================================================================

// Q16.16 fixed-point for angles (0.000015° resolution)
typedef int32_t angleQ16;

#define ANGLE_TO_Q16(deg)   ((angleQ16)((deg) * 65536.0f))
#define Q16_TO_ANGLE(q)     ((float)(q) / 65536.0f)

// ============================================================================
// TIME-SAFE MACROS
// ============================================================================

// Safe comparison for millis() rollover
#define TIME_ELAPSED(start, duration) \
    ((uint32_t)(millis() - (start)) >= (duration))

#define TIME_ELAPSED_US(start, duration) \
    ((uint32_t)(micros() - (start)) >= (duration))

// ============================================================================
// CRITICAL SECTION MACROS
// ============================================================================

#define ENTER_CRITICAL() \
    uint32_t __primask = __get_PRIMASK(); \
    __disable_irq()

#define EXIT_CRITICAL() \
    if (!__primask) __enable_irq()

#endif // TYPES_H
