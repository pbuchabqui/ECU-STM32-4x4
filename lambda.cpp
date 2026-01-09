/*
 * lambda.cpp - Lambda (O2) Closed-Loop Control Implementation
 * ECU STM32F405 v8.2
 * 
 * AFR target table moved to tables.cpp
 * This module handles only the PID control logic.
 * 
 * Theory of Operation:
 * 1. Read wideband O2 sensor voltage (or CAN)
 * 2. Convert to AFR/Lambda
 * 3. Compare to target from AFR table (via tables.h)
 * 4. PID calculates fuel correction
 * 5. Correction applied to fuel pulse width
 */

#include "lambda.h"
#include "tables.h"  // AFR target table is here now
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
    lambdaControl.autoCorrectEnabled = true;  // Auto-correction ON by default
    lambdaControl.warmupStartTime = millis();
    lambdaControl.lastUpdateTime = millis();
}

// ============================================================================
// SENSOR READING
// ============================================================================

static float voltageToAfr(float voltage) {
#if LAMBDA_SENSOR_TYPE == LAMBDA_SENSOR_WIDEBAND
    float afr = WB_AFR_MIN + (voltage / WB_VOLTAGE_MAX) * (WB_AFR_MAX - WB_AFR_MIN);
    return constrain(afr, WB_AFR_MIN, WB_AFR_MAX);
#else
    if (voltage > NB_RICH_THRESHOLD) {
        return AFR_STOICH_DEFAULT * 0.95f;
    } else if (voltage < NB_LEAN_THRESHOLD) {
        return AFR_STOICH_DEFAULT * 1.05f;
    } else {
        return AFR_STOICH_DEFAULT;
    }
#endif
}

// ============================================================================
// AFR TARGET (Now delegates to tables.h)
// ============================================================================

float lambdaGetTargetAfr(uint16_t rpm, float loadMgStroke) {
    // v8.2: Use centralized table from tables.cpp
    return getAfrTarget((float)rpm, loadMgStroke);
}

// ============================================================================
// PID CALCULATION
// ============================================================================

static float calculatePID(float error, uint32_t dtMs) {
    float dt = dtMs / 1000.0f;
    if (dt <= 0 || dt > 1.0f) dt = 0.02f;
    
    // Deadband
    if (fabsf(error) < LAMBDA_DEADBAND) {
        lambdaControl.proportional = 0;
        lambdaControl.derivative = 0;
        return lambdaControl.integral;
    }
    
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
// MAIN UPDATE
// ============================================================================

void lambdaUpdate(float sensorVoltage, uint16_t rpm, float tps, float clt, float loadMgStroke) {
    uint32_t now = millis();
    uint32_t dtMs = now - lambdaControl.lastUpdateTime;
    
    if (dtMs < LAMBDA_UPDATE_INTERVAL_MS) return;
    lambdaControl.lastUpdateTime = now;
    
    lambdaControl.sensorVoltage = sensorVoltage;
    
    // Check if enabled
    if (!lambdaControl.isEnabled) {
        lambdaControl.state = LAMBDA_STATE_DISABLED;
        lambdaControl.correction = 0;
        return;
    }
    
    // Validate sensor
    if (sensorVoltage < LAMBDA_WARMUP_VOLTAGE_MIN || sensorVoltage > 5.1f) {
        lambdaControl.sensorValid = false;
        if (lambdaControl.state == LAMBDA_STATE_CLOSED_LOOP) {
            lambdaControl.state = LAMBDA_STATE_ERROR;
        }
    } else {
        lambdaControl.sensorValid = true;
    }
    
    // Sensor warmup
    uint32_t warmupElapsed = now - lambdaControl.warmupStartTime;
    if (warmupElapsed < LAMBDA_WARMUP_TIME_MS) {
        lambdaControl.state = LAMBDA_STATE_WARMUP;
        lambdaControl.isWarmedUp = false;
        lambdaControl.correction = 0;
        return;
    }
    lambdaControl.isWarmedUp = true;
    
    // Convert to AFR/Lambda
    lambdaControl.afr = voltageToAfr(sensorVoltage);
    lambdaControl.lambda = lambdaControl.afr / AFR_STOICH_DEFAULT;
    
    // Running average
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
        lambdaControl.correction *= 0.95f;
        lambdaControl.integral *= 0.98f;
        return;
    }
    
    // CLOSED LOOP
    lambdaControl.state = LAMBDA_STATE_CLOSED_LOOP;
    lambdaControl.closedLoopTime += dtMs;
    
    // Get target from centralized table
    lambdaControl.targetAfr = lambdaGetTargetAfr(rpm, loadMgStroke);
    lambdaControl.targetLambda = lambdaControl.targetAfr / AFR_STOICH_DEFAULT;
    
    // Calculate error (always, for diagnostics)
    lambdaControl.lambdaError = lambdaControl.targetLambda - lambdaControl.lambda;
    
    // Only update correction if auto-correct is enabled
    if (lambdaControl.autoCorrectEnabled) {
        // Run PID
        float pidOutput = calculatePID(lambdaControl.lambdaError, dtMs);
        
        // Apply limits
        lambdaControl.correction = constrain(pidOutput,
                                              LAMBDA_CORRECTION_MIN,
                                              LAMBDA_CORRECTION_MAX);
        
        // Update average
        lambdaControl.avgCorrection = lambdaControl.avgCorrection * 0.99f +
                                       lambdaControl.correction * 0.01f;
    }
    // When autoCorrectEnabled is false, correction stays frozen at last value
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

float lambdaGetAfr(void) {
    return lambdaControl.afr;
}

float lambdaGetLambda(void) {
    return lambdaControl.lambda;
}

// ============================================================================
// CONTROL
// ============================================================================

void lambdaEnable(void) {
    lambdaControl.isEnabled = true;
}

void lambdaDisable(void) {
    lambdaControl.isEnabled = false;
    lambdaControl.state = LAMBDA_STATE_DISABLED;
    lambdaControl.correction = 0;
}

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
        // When disabling, keep correction at current value (freeze)
        // This allows seeing the "learned" correction
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
    Serial.print(F("Auto-Correct: ")); Serial.println(lambdaControl.autoCorrectEnabled ? "ON" : "OFF (frozen)");
    Serial.print(F("Sensor Valid: ")); Serial.println(lambdaControl.sensorValid ? "YES" : "NO");
    
    Serial.println(F("--- Readings ---"));
    Serial.print(F("Voltage: ")); Serial.print(lambdaControl.sensorVoltage, 2); Serial.println(F(" V"));
    Serial.print(F("AFR: ")); Serial.println(lambdaControl.afr, 1);
    Serial.print(F("Lambda: ")); Serial.println(lambdaControl.lambda, 3);
    
    Serial.println(F("--- Target ---"));
    Serial.print(F("Target AFR: ")); Serial.println(lambdaControl.targetAfr, 1);
    Serial.print(F("Error: ")); Serial.println(lambdaControl.lambdaError, 3);
    
    Serial.println(F("--- PID ---"));
    Serial.print(F("P: ")); Serial.print(lambdaControl.proportional, 2); Serial.println(F("%"));
    Serial.print(F("I: ")); Serial.print(lambdaControl.integral, 2); Serial.println(F("%"));
    Serial.print(F("D: ")); Serial.print(lambdaControl.derivative, 2); Serial.println(F("%"));
    
    Serial.println(F("--- Output ---"));
    Serial.print(F("Correction: ")); Serial.print(lambdaControl.correction, 1); Serial.println(F("%"));
    
    Serial.println(F("=========================\n"));
}
