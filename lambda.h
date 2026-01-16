/*
 * lambda.h - Wideband Lambda Closed-Loop Control
 * ECU STM32F405 v8.2
 * 
 * Supports WIDEBAND sensors only:
 * - 0-5V linear output (LSU 4.9 with controller, Spartan, LC-2, etc.)
 * - CAN-based controllers (AEM X-Series, Innovate, etc.)
 * 
 * AFR target table is in tables.h/cpp
 * This module handles only the PID control logic.
 */

#ifndef LAMBDA_H
#define LAMBDA_H

#include <Arduino.h>

// ============================================================================
// WIDEBAND SENSOR CONFIGURATION
// ============================================================================

// Wideband 0-5V calibration (LSU 4.9 with Spartan/LC-2 controller)
// Most controllers output 0V = 7.35 AFR, 5V = 22.39 AFR (linear)
#define WB_VOLTAGE_MIN          0.0f
#define WB_VOLTAGE_MAX          5.0f
#define WB_AFR_MIN              7.35f   // AFR at 0V
#define WB_AFR_MAX              22.39f  // AFR at 5V

// Alternative calibrations (adjust for your controller):
// AEM UEGO: AFR_MIN=8.5, AFR_MAX=18.0
// Innovate LC-2: AFR_MIN=7.35, AFR_MAX=22.39

// ============================================================================
// PID CONFIGURATION
// ============================================================================

#define LAMBDA_PID_KP           5.0f    // Proportional gain
#define LAMBDA_PID_KI           0.5f    // Integral gain
#define LAMBDA_PID_KD           0.5f    // Derivative gain

// Correction limits (% fuel adjustment)
#define LAMBDA_CORRECTION_MAX   25.0f   // Max enrichment
#define LAMBDA_CORRECTION_MIN   -20.0f  // Max enleanment
#define LAMBDA_INTEGRAL_MAX     15.0f   // Anti-windup limit

// Deadband (lambda units)
#define LAMBDA_DEADBAND         0.02f   // ~0.3 AFR

// ============================================================================
// ACTIVATION CONDITIONS
// ============================================================================

#define LAMBDA_ENABLE_CLT_MIN   60.0f   // Min coolant temp (°C)
#define LAMBDA_ENABLE_RPM_MIN   800     // Min RPM
#define LAMBDA_ENABLE_RPM_MAX   6500    // Max RPM (open loop above)
#define LAMBDA_ENABLE_TPS_MAX   90.0f   // Max TPS (%) - open loop at WOT

// Sensor warmup
#define LAMBDA_WARMUP_TIME_MS   15000   // 15 seconds
#define LAMBDA_WARMUP_VOLTAGE_MIN 0.1f  // Below this = sensor not ready

// Update rate
#define LAMBDA_UPDATE_INTERVAL_MS 20    // 50 Hz

// ============================================================================
// AFR CONSTANTS
// ============================================================================

#define AFR_STOICH_GASOLINE     14.7f
#define AFR_STOICH_E85          9.8f
#define AFR_STOICH_E27          13.0f   // Brazilian E27 gasoline
#define AFR_STOICH_DEFAULT      AFR_STOICH_E27

#define AFR_TO_LAMBDA(afr)      ((afr) / AFR_STOICH_DEFAULT)
#define LAMBDA_TO_AFR(lambda)   ((lambda) * AFR_STOICH_DEFAULT)

// ============================================================================
// LAMBDA STATE
// ============================================================================

typedef enum {
    LAMBDA_STATE_DISABLED = 0,  // System disabled
    LAMBDA_STATE_WARMUP,        // Sensor warming up
    LAMBDA_STATE_OPEN_LOOP,     // Conditions not met for closed loop
    LAMBDA_STATE_CLOSED_LOOP,   // Actively correcting
    LAMBDA_STATE_ERROR          // Sensor fault detected
} LambdaState;

// ============================================================================
// LAMBDA CONTROL STRUCTURE
// ============================================================================

typedef struct {
    LambdaState state;
    
    // Sensor readings
    float sensorVoltage;        // Raw voltage (0-5V) or converted from CAN
    float afr;                  // Calculated AFR
    float lambda;               // Calculated lambda (AFR / stoich)
    
    // Target (from tables.h)
    float targetAfr;
    float targetLambda;
    
    // Error
    float lambdaError;          // targetLambda - lambda
    
    // PID state
    float proportional;
    float integral;
    float derivative;
    float lastError;
    
    // Output
    float correction;           // Fuel correction (%)
    
    // Statistics
    uint32_t closedLoopTime;    // Total time in closed loop (ms)
    float avgCorrection;        // Running average correction
    float avgLambda;            // Running average lambda
    
    // Timing
    uint32_t warmupStartTime;
    uint32_t lastUpdateTime;
    
    // Status flags
    bool sensorValid;
    bool isWarmedUp;
    bool isEnabled;
    bool autoCorrectEnabled;    // When false, correction calculated but not applied
    
} LambdaControl;

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern LambdaControl lambdaControl;

// ============================================================================
// FUNCTIONS
// ============================================================================

// Initialization
void lambdaInit(void);

// Main update from 0-5V analog input
void lambdaUpdate(float sensorVoltage, uint16_t rpm, float tps, float clt, float loadMgStroke);

// Update with direct lambda value (for CAN input)
void lambdaUpdateDirect(float lambdaValue, uint16_t rpm, float tps, float clt, float loadMgStroke);

// Get fuel correction (%)
float lambdaGetCorrection(void);

// Get AFR target from table
float lambdaGetTargetAfr(uint16_t rpm, float loadMgStroke);

// Control
void lambdaReset(void);

// Auto-correction toggle (for tuning - freeze correction)
void lambdaSetAutoCorrect(bool enabled);
bool lambdaIsAutoCorrectEnabled(void);

// State queries
LambdaState lambdaGetState(void);
bool lambdaIsClosedLoop(void);

// Diagnostics
void lambdaPrintStatus(void);

#endif // LAMBDA_H
