/*
 * lambda.cpp - Wideband Lambda Closed-Loop Control
 * ECU STM32F405 v8.2
 * 
 * Supports WIDEBAND sensors only:
 * - 0-5V linear output
 * - CAN-based controllers
 * 
 * Theory of Operation:
 * 1. Read wideband O2 sensor (0-5V analog or CAN)
 * 2. Convert to AFR/Lambda
 * 3. Compare to target from AFR table
 * 4. PID calculates fuel correction
 * 5. Correction applied to fuel pulse width
 */

#include "lambda.h"
#include "tables.h"
#include <math.h>

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

LambdaControl lambdaControl;

// ============================================================================
// INITIALIZATION
// ============================================================================

void lambdaInit(void) {
    memset(&lambdaControl, 0, sizeof(LambdaControl));
    
    lambdaControl.state = LAMBDA_STATE_DISABLED;
    lambdaControl.isEnabled = true;
    lambdaControl.autoCorrectEnabled = true;
    lambdaControl.warmupStartTime = millis();
    lambdaControl.lastUpdateTime = millis();
}

// ============================================================================
// WIDEBAND SENSOR CONVERSION
// ============================================================================

// Convert 0-5V to AFR (linear interpolation)
static float voltageToAfr(float voltage) {
    // Clamp voltage to valid range
    voltage = constrain(voltage, WB_VOLTAGE_MIN, WB_VOLTAGE_MAX);
    
    // Linear interpolation: AFR = AFR_MIN + (V / V_MAX) * (AFR_MAX - AFR_MIN)
    float afr = WB_AFR_MIN + (voltage / WB_VOLTAGE_MAX) * (WB_AFR_MAX - WB_AFR_MIN);
    
    return afr;
}

// Convert lambda directly to AFR
static float lambdaToAfr(float lambda) {
    return lambda * AFR_STOICH_DEFAULT;
}

// ============================================================================
// AFR TARGET (delegates to tables.h)
// ============================================================================

float lambdaGetTargetAfr(uint16_t rpm, float loadMgStroke) {
    return getAfrTarget((float)rpm, loadMgStroke);
}

// ============================================================================
// PID CALCULATION
// ============================================================================

static float calculatePID(float error, uint32_t dtMs) {
    float dt = dtMs / 1000.0f;
    if (dt <= 0 || dt > 1.0f) dt = 0.02f;
    
    // Deadband - no correction for small errors
    if (fabsf(error) < LAMBDA_DEADBAND) {
        lambdaControl.proportional = 0;
        lambdaControl.derivative = 0;
        return lambdaControl.integral;
    }
    
    // Scale error for better PID response
    float scaledError = error * 10.0f;
    
    // Proportional
    lambdaControl.proportional = LAMBDA_PID_KP * scaledError;
    
    // Integral with anti-windup
    lambdaControl.integral += LAMBDA_PID_KI * scaledError * dt;
    lambdaControl.integral = constrain(lambdaControl.integral,
                                        -LAMBDA_INTEGRAL_MAX,
                                        LAMBDA_INTEGRAL_MAX);
    
    // Derivative
    float dError = (error - lambdaControl.lastError) / dt;
    lambdaControl.derivative = LAMBDA_PID_KD * dError * 10.0f;
    lambdaControl.lastError = error;
    
    // Sum PID terms
    float output = lambdaControl.proportional + 
                   lambdaControl.integral + 
                   lambdaControl.derivative;
    
    // Validate output
    if (!isfinite(output)) {
        output = 0;
        lambdaControl.integral = 0;
    }
    
    return output;
}

// ============================================================================
// CORE UPDATE LOGIC
// ============================================================================

static void lambdaUpdateInternal(float afr, uint16_t rpm, float tps, float clt, float loadMgStroke) {
    uint32_t now = millis();
    uint32_t dtMs = now - lambdaControl.lastUpdateTime;
    
    // Rate limit updates
    if (dtMs < LAMBDA_UPDATE_INTERVAL_MS) return;
    lambdaControl.lastUpdateTime = now;
    
    // Store AFR and calculate lambda
    lambdaControl.afr = afr;
    lambdaControl.lambda = afr / AFR_STOICH_DEFAULT;
    
    // Check if enabled
    if (!lambdaControl.isEnabled) {
        lambdaControl.state = LAMBDA_STATE_DISABLED;
        lambdaControl.correction = 0;
        return;
    }
    
    // Validate sensor (check for reasonable AFR range)
    if (afr < WB_AFR_MIN || afr > WB_AFR_MAX) {
        lambdaControl.sensorValid = false;
        if (lambdaControl.state == LAMBDA_STATE_CLOSED_LOOP) {
            lambdaControl.state = LAMBDA_STATE_ERROR;
        }
    } else {
        lambdaControl.sensorValid = true;
    }
    
    // Sensor warmup period
    uint32_t warmupElapsed = now - lambdaControl.warmupStartTime;
    if (warmupElapsed < LAMBDA_WARMUP_TIME_MS) {
        lambdaControl.state = LAMBDA_STATE_WARMUP;
        lambdaControl.isWarmedUp = false;
        lambdaControl.correction = 0;
        return;
    }
    lambdaControl.isWarmedUp = true;
    
    // Running average lambda
    lambdaControl.avgLambda = lambdaControl.avgLambda * 0.95f +
                               lambdaControl.lambda * 0.05f;
    
    // Check closed loop conditions
    bool conditionsMet = true;
    if (clt < LAMBDA_ENABLE_CLT_MIN) conditionsMet = false;
    if (rpm < LAMBDA_ENABLE_RPM_MIN) conditionsMet = false;
    if (rpm > LAMBDA_ENABLE_RPM_MAX) conditionsMet = false;
    if (tps > LAMBDA_ENABLE_TPS_MAX) conditionsMet = false;
    if (!lambdaControl.sensorValid) conditionsMet = false;
    
    if (!conditionsMet) {
        lambdaControl.state = LAMBDA_STATE_OPEN_LOOP;
        // Decay correction and integral slowly
        lambdaControl.correction *= 0.95f;
        lambdaControl.integral *= 0.98f;
        return;
    }
    
    // CLOSED LOOP OPERATION
    lambdaControl.state = LAMBDA_STATE_CLOSED_LOOP;
    lambdaControl.closedLoopTime += dtMs;
    
    // Get target from AFR table
    lambdaControl.targetAfr = lambdaGetTargetAfr(rpm, loadMgStroke);
    lambdaControl.targetLambda = lambdaControl.targetAfr / AFR_STOICH_DEFAULT;
    
    // Calculate error (positive = too lean, negative = too rich)
    lambdaControl.lambdaError = lambdaControl.targetLambda - lambdaControl.lambda;
    
    // Only update correction if auto-correct is enabled
    if (lambdaControl.autoCorrectEnabled) {
        // Run PID
        float pidOutput = calculatePID(lambdaControl.lambdaError, dtMs);
        
        // Apply limits
        lambdaControl.correction = constrain(pidOutput,
                                              LAMBDA_CORRECTION_MIN,
                                              LAMBDA_CORRECTION_MAX);
        
        // Update running average
        lambdaControl.avgCorrection = lambdaControl.avgCorrection * 0.99f +
                                       lambdaControl.correction * 0.01f;
    }
    // When autoCorrectEnabled is false, correction stays frozen
}

// ============================================================================
// PUBLIC UPDATE FUNCTIONS
// ============================================================================

void lambdaUpdate(float sensorVoltage, uint16_t rpm, float tps, float clt, float loadMgStroke) {
    lambdaControl.sensorVoltage = sensorVoltage;
    float afr = voltageToAfr(sensorVoltage);
    lambdaUpdateInternal(afr, rpm, tps, clt, loadMgStroke);
}

void lambdaUpdateDirect(float lambdaValue, uint16_t rpm, float tps, float clt, float loadMgStroke) {
    // Convert lambda to pseudo-voltage for diagnostics
    lambdaControl.sensorVoltage = (lambdaValue - 0.5f) * (5.0f / 1.0f);
    float afr = lambdaToAfr(lambdaValue);
    lambdaUpdateInternal(afr, rpm, tps, clt, loadMgStroke);
}

// ============================================================================
// ACCESSORS
// ============================================================================

float lambdaGetCorrection(void) {
    return lambdaControl.correction;
}

LambdaState lambdaGetState(void) {
    return lambdaControl.state;
}

bool lambdaIsClosedLoop(void) {
    return lambdaControl.state == LAMBDA_STATE_CLOSED_LOOP;
}

// ============================================================================
// CONTROL
// ============================================================================

void lambdaReset(void) {
    lambdaControl.integral = 0;
    lambdaControl.correction = 0;
    lambdaControl.lastError = 0;
    lambdaControl.warmupStartTime = millis();
    lambdaControl.closedLoopTime = 0;
    lambdaControl.avgCorrection = 0;
}

void lambdaSetAutoCorrect(bool enabled) {
    lambdaControl.autoCorrectEnabled = enabled;
    if (!enabled) {
        Serial.print(F("Lambda Auto-Correct: OFF (frozen at "));
        Serial.print(lambdaControl.correction, 1);
        Serial.println(F("%)"));
    } else {
        Serial.println(F("Lambda Auto-Correct: ON"));
    }
}

bool lambdaIsAutoCorrectEnabled(void) {
    return lambdaControl.autoCorrectEnabled;
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

void lambdaPrintStatus(void) {
    Serial.println(F("\n===== LAMBDA STATUS ====="));
    
    Serial.print(F("State: "));
    switch (lambdaControl.state) {
        case LAMBDA_STATE_DISABLED:     Serial.println(F("DISABLED")); break;
        case LAMBDA_STATE_WARMUP:       Serial.println(F("WARMUP")); break;
        case LAMBDA_STATE_OPEN_LOOP:    Serial.println(F("OPEN_LOOP")); break;
        case LAMBDA_STATE_CLOSED_LOOP:  Serial.println(F("CLOSED_LOOP")); break;
        case LAMBDA_STATE_ERROR:        Serial.println(F("ERROR")); break;
    }
    
    Serial.print(F("Enabled: ")); Serial.println(lambdaControl.isEnabled ? "YES" : "NO");
    Serial.print(F("Auto-Correct: ")); Serial.println(lambdaControl.autoCorrectEnabled ? "ON" : "OFF");
    Serial.print(F("Sensor Valid: ")); Serial.println(lambdaControl.sensorValid ? "YES" : "NO");
    
    Serial.println(F("--- Readings ---"));
    Serial.print(F("Voltage: ")); Serial.print(lambdaControl.sensorVoltage, 2); Serial.println(F(" V"));
    Serial.print(F("AFR: ")); Serial.println(lambdaControl.afr, 1);
    Serial.print(F("Lambda: ")); Serial.println(lambdaControl.lambda, 3);
    Serial.print(F("Avg Lambda: ")); Serial.println(lambdaControl.avgLambda, 3);
    
    Serial.println(F("--- Target ---"));
    Serial.print(F("Target AFR: ")); Serial.println(lambdaControl.targetAfr, 1);
    Serial.print(F("Target Lambda: ")); Serial.println(lambdaControl.targetLambda, 3);
    Serial.print(F("Error: ")); Serial.println(lambdaControl.lambdaError, 3);
    
    Serial.println(F("--- PID ---"));
    Serial.print(F("P: ")); Serial.print(lambdaControl.proportional, 2); Serial.println(F("%"));
    Serial.print(F("I: ")); Serial.print(lambdaControl.integral, 2); Serial.println(F("%"));
    Serial.print(F("D: ")); Serial.print(lambdaControl.derivative, 2); Serial.println(F("%"));
    
    Serial.println(F("--- Output ---"));
    Serial.print(F("Correction: ")); Serial.print(lambdaControl.correction, 1); Serial.println(F("%"));
    Serial.print(F("Avg Correction: ")); Serial.print(lambdaControl.avgCorrection, 1); Serial.println(F("%"));
    
    Serial.print(F("Closed Loop Time: ")); Serial.print(lambdaControl.closedLoopTime / 1000); Serial.println(F(" s"));
    
    Serial.println(F("=========================\n"));
}
