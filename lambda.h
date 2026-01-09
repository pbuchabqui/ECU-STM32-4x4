/*
 * lambda.h - Lambda (O2) Closed-Loop Control
 * ECU STM32F405 v8.2
 * 
 * AFR target table moved to tables.h/cpp
 * This module handles only the PID control logic.
 * 
 * Supports:
 * - Wideband lambda sensors (0-5V linear output)
 * - Narrowband lambda sensors (switching type)
 * - CAN-based wideband controllers
 * - PID-based fuel correction
 */

#ifndef LAMBDA_H
#define LAMBDA_H

#include <Arduino.h>

// ============================================================================
// LAMBDA CONFIGURATION
// ============================================================================

// Sensor type
#define LAMBDA_SENSOR_WIDEBAND      1
#define LAMBDA_SENSOR_NARROWBAND    2
#define LAMBDA_SENSOR_TYPE          LAMBDA_SENSOR_WIDEBAND

// Wideband calibration (LSU 4.9 with Spartan/LC-2 controller)
#define WB_VOLTAGE_MIN              0.0f
#define WB_VOLTAGE_MAX              5.0f
#define WB_AFR_MIN                  7.35f
#define WB_AFR_MAX                  22.39f

// Narrowband thresholds
#define NB_RICH_THRESHOLD           0.65f
#define NB_LEAN_THRESHOLD           0.35f

// PID gains
#define LAMBDA_PID_KP               5.0f
#define LAMBDA_PID_KI               0.5f
#define LAMBDA_PID_KD               0.5f

// Correction limits
#define LAMBDA_CORRECTION_MAX       25.0f
#define LAMBDA_CORRECTION_MIN       -20.0f
#define LAMBDA_INTEGRAL_MAX         15.0f

// Activation conditions
#define LAMBDA_ENABLE_CLT_MIN       60.0f
#define LAMBDA_ENABLE_RPM_MIN       800
#define LAMBDA_ENABLE_RPM_MAX       6500
#define LAMBDA_ENABLE_TPS_MAX       90.0f

// Sensor warmup
#define LAMBDA_WARMUP_TIME_MS       15000
#define LAMBDA_WARMUP_VOLTAGE_MIN   0.1f

// Deadband
#define LAMBDA_DEADBAND             0.02f

// Update rate
#define LAMBDA_UPDATE_INTERVAL_MS   20

// ============================================================================
// AFR CONSTANTS
// ============================================================================

#define AFR_STOICH_GASOLINE         14.7f
#define AFR_STOICH_E85              9.8f
#define AFR_STOICH_E27              13.0f
#define AFR_STOICH_DEFAULT          AFR_STOICH_E27

#define AFR_TO_LAMBDA(afr)          ((afr) / AFR_STOICH_DEFAULT)
#define LAMBDA_TO_AFR(lambda)       ((lambda) * AFR_STOICH_DEFAULT)

// ============================================================================
// LAMBDA STATE
// ============================================================================

typedef enum {
    LAMBDA_STATE_DISABLED = 0,
    LAMBDA_STATE_WARMUP,
    LAMBDA_STATE_OPEN_LOOP,
    LAMBDA_STATE_CLOSED_LOOP,
    LAMBDA_STATE_ERROR
} LambdaState;

// ============================================================================
// LAMBDA CONTROL STRUCTURE
// ============================================================================

typedef struct {
    LambdaState state;
    
    // Sensor readings
    float sensorVoltage;
    float afr;
    float lambda;
    
    // Target (from tables.h)
    float targetAfr;
    float targetLambda;
    
    // Error
    float lambdaError;
    
    // PID state
    float proportional;
    float integral;
    float derivative;
    float lastError;
    
    // Output
    float correction;
    
    // Statistics
    uint32_t closedLoopTime;
    float avgCorrection;
    float avgLambda;
    
    // Timing
    uint32_t warmupStartTime;
    uint32_t lastUpdateTime;
    
    // Status
    bool sensorValid;
    bool isWarmedUp;
    bool isEnabled;
    bool autoCorrectEnabled;        // When false, correction is calculated but not applied
    
} LambdaControl;

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern LambdaControl lambdaControl;

// ============================================================================
// FUNCTIONS
// ============================================================================

void lambdaInit(void);

// Main update
void lambdaUpdate(float sensorVoltage, uint16_t rpm, float tps, float clt, float loadMgStroke);

// Get fuel correction
float lambdaGetCorrection(void);

// Get AFR target (now uses tables.h internally)
float lambdaGetTargetAfr(uint16_t rpm, float loadMgStroke);

// Control
void lambdaEnable(void);
void lambdaDisable(void);
void lambdaReset(void);

// Auto-correction toggle (for tuning)
void lambdaSetAutoCorrect(bool enabled);
bool lambdaIsAutoCorrectEnabled(void);

// State queries
LambdaState lambdaGetState(void);
bool lambdaIsClosedLoop(void);
float lambdaGetAfr(void);
float lambdaGetLambda(void);

// Diagnostics
void lambdaPrintStatus(void);

#endif // LAMBDA_H
