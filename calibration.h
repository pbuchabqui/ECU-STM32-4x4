/*
 * calibration.h - Real-Time Calibration System
 * ECU STM32F405 v8.2
 * 
 * Serial-based tuning interface:
 * - V/I/A: Select VE/IGN/AFR table
 * - +/-: Adjust current cell by 1% (VE) or 1° (IGN) or 0.1 (AFR)
 * - S: Save to EEPROM
 * - ?: Show current cell status
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "tables.h"

// ============================================================================
// CALIBRATION STATE
// ============================================================================

typedef struct {
    CalibrationMode mode;           // Current editing mode
    uint8_t lastRpmIdx;             // Last active RPM bin (nearest)
    uint8_t lastLoadIdx;            // Last active Load bin (nearest)
    uint8_t lastMafIdx;             // Last active MAF bin
    float lastRpm;                  // Last RPM value
    float lastLoad;                 // Last Load value
    float lastMafHz;                // Last MAF frequency
    float cellWeight;               // How close we are to selected cell (0-1)
    bool enabled;                   // Calibration mode active
    uint32_t lastAdjustTime;        // Debounce
} CalibrationState;

extern CalibrationState calState;

// ============================================================================
// FUNCTIONS
// ============================================================================

// Initialize calibration system
void calibrationInit(void);

// Process serial input (call from loop)
void calibrationProcessSerial(void);

// Update current position (call with real sensor data)
void calibrationUpdatePosition(float rpm, float loadMgStroke);
void calibrationUpdateMaf(float mafHz);

// Get mode name string
const char* calibrationGetModeName(void);

// Print help
void calibrationPrintHelp(void);

// Print current status
void calibrationPrintStatus(void);

#endif // CALIBRATION_H
