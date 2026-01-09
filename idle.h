/*
 * idle.h - Idle Control System
 * ECU STM32F405 v8.2
 * 
 * Idle target RPM table moved to tables.h/cpp
 * This module handles only the PID control logic.
 * 
 * Method: Adjusts ignition timing based on RPM error from target.
 */

#ifndef IDLE_H
#define IDLE_H

#include <Arduino.h>

// ============================================================================
// IDLE CONTROL CONFIGURATION
// ============================================================================

#define IDLE_RPM_DEADBAND       25
#define IDLE_IGN_ADVANCE_MAX    25.0f
#define IDLE_IGN_ADVANCE_MIN    -5.0f
#define IDLE_IGN_CORRECTION_MAX 15.0f

#define IDLE_IGN_PID_KP         0.05f
#define IDLE_IGN_PID_KI         0.001f
#define IDLE_IGN_PID_KD         0.01f
#define IDLE_IGN_INTEGRAL_MAX   10.0f

#define IDLE_TPS_MAX            3.0f
#define IDLE_RPM_MIN            400
#define IDLE_RPM_MAX            2000

#define IDLE_DASHPOT_THRESHOLD  20.0f
#define IDLE_DASHPOT_DECAY      0.5f
#define IDLE_DASHPOT_MAX        10.0f

#define IDLE_AFTERSTART_TIME_MS 5000
#define IDLE_AFTERSTART_DECAY   0.5f

// ============================================================================
// IDLE STATE
// ============================================================================

typedef enum {
    IDLE_STATE_DISABLED = 0,
    IDLE_STATE_CRANKING,
    IDLE_STATE_AFTERSTART,
    IDLE_STATE_WARMUP,
    IDLE_STATE_RUNNING,
    IDLE_STATE_DASHPOT
} IdleState;

// ============================================================================
// IDLE CONTROL STRUCTURE
// ============================================================================

typedef struct {
    IdleState state;
    
    uint16_t targetRpm;
    uint16_t currentRpm;
    int16_t rpmError;
    
    float proportional;
    float integral;
    float derivative;
    int16_t lastRpmError;
    uint32_t lastUpdateTime;
    
    float ignitionCorrection;
    
    float dashpotCorrection;
    float lastTps;
    uint32_t lastTpsTime;
    
    float afterstartCorrection;
    uint32_t engineStartTime;
    
    bool isActive;
    bool isInDeadband;
    
    uint32_t totalCycles;
    uint32_t activeCycles;
    float avgCorrection;
    
} IdleControlState;

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern IdleControlState idleState;

// ============================================================================
// FUNCTIONS
// ============================================================================

void idleInit(void);

// Main update
void idleUpdate(uint16_t rpm, float tps, float clt, float baseIgnition);

// Get ignition correction
float idleGetIgnitionCorrection(void);

// Get target RPM (now uses tables.h internally)
uint16_t idleGetTargetRpm(float clt);

// State queries
bool idleIsActive(void);
IdleState idleGetState(void);

// Notifications
void idleNotifyEngineStart(void);
void idleNotifyEngineStop(void);

// Diagnostics
void idlePrintStatus(void);

#endif // IDLE_H
