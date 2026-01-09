/*
 * tle8888.h - TLE8888-1QK Driver (Simplified)
 * ECU STM32F405 v8.2
 * 
 * SIMPLIFIED VERSION - Just make it work first!
 * 
 * TLE8888 SPI Protocol:
 * - 16-bit frames (8-bit command + 8-bit data)
 * - SPI Mode 1 (CPOL=0, CPHA=1)
 * - MSB first
 * - Max 5 MHz
 */

#ifndef TLE8888_H
#define TLE8888_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// REGISTER ADDRESSES
// ============================================================================

// Control registers
#define TLE_REG_CONT        0x30    // OUT1-8 control
#define TLE_REG_CONT1       0x31    // OUT9-16 control
#define TLE_REG_IGNCONT     0x32    // IGN1-4 control

// Diagnostic registers
#define TLE_REG_OPSTAT      0x00    // Operating status
#define TLE_REG_DIAG0       0x20    // Main diagnostics

// ============================================================================
// TLE8888 CLASS
// ============================================================================

class TLE8888 {
public:
    TLE8888(uint8_t csPin);
    
    // Initialize - returns true if communication OK
    bool begin(void);
    
    // Output control (channels 0-15)
    void setOutput(uint8_t channel, bool state);
    void setAllOutputs(uint16_t state);
    
    // Injector control (alias for setOutput, channels 0-7)
    inline void setInjector(uint8_t channel, bool state) {
        setOutput(channel, state);
    }
    
    // Ignition control (channels 0-3)
    void setIgnition(uint8_t channel, bool state);
    void setAllIgnition(uint8_t state);
    
    // Diagnostics
    bool isConnected(void);
    bool hasCriticalFault(void);
    void serviceWatchdog(void);
    
    // Status
    uint16_t getOutputState(void) { return _outputState; }
    uint8_t getIgnitionState(void) { return _ignitionState; }
    
    // Debug
    void printStatus(void);
    
private:
    uint8_t _csPin;
    uint16_t _outputState;
    uint8_t _ignitionState;
    bool _connected;
    
    // SPI transfer - sends 16-bit frame, returns response
    uint16_t transfer16(uint8_t cmd, uint8_t data);
    
    // Register access
    void writeReg(uint8_t reg, uint8_t data);
    uint8_t readReg(uint8_t reg);
};

extern TLE8888 tle8888;

#endif // TLE8888_H
