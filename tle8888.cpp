/*
 * tle8888.cpp - TLE8888-1QK Driver Implementation (Simplified)
 * ECU STM32F405 v8.2
 * 
 * Uses proper 16-bit SPI frames as per TLE8888 datasheet.
 */

#include "tle8888.h"
#include "pinout.h"

// Global instance
TLE8888 tle8888(PIN_TLE_CS);

// SPI settings for TLE8888
// Mode 1: CPOL=0, CPHA=1
// Max 5 MHz, using 4 MHz for safety margin
SPISettings tleSettings(4000000, MSBFIRST, SPI_MODE1);

TLE8888::TLE8888(uint8_t csPin) : _csPin(csPin) {
    _outputState = 0;
    _ignitionState = 0;
    _connected = false;
}

// ============================================================================
// SPI TRANSFER - Core 16-bit transaction
// ============================================================================

uint16_t TLE8888::transfer16(uint8_t cmd, uint8_t data) {
    uint16_t response;
    
    SPI.beginTransaction(tleSettings);
    digitalWrite(_csPin, LOW);
    
    // Send command byte, receive status high byte
    uint8_t respHi = SPI.transfer(cmd);
    
    // Send data byte, receive status low byte
    uint8_t respLo = SPI.transfer(data);
    
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    
    response = (respHi << 8) | respLo;
    return response;
}

// ============================================================================
// REGISTER ACCESS
// ============================================================================

void TLE8888::writeReg(uint8_t reg, uint8_t data) {
    // Write command: bit 7 = 1, bits 6:0 = register address
    transfer16(0x80 | (reg & 0x3F), data);
}

uint8_t TLE8888::readReg(uint8_t reg) {
    // Read is a two-step process:
    // 1. Send read command (bit 7 = 0)
    // 2. Send dummy to clock out data
    
    // First transaction: send address
    transfer16(reg & 0x3F, 0x00);
    
    // Small delay between transactions
    delayMicroseconds(1);
    
    // Second transaction: read data (send dummy)
    uint16_t resp = transfer16(0x00, 0x00);
    
    // Data is in lower byte of response
    return resp & 0xFF;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool TLE8888::begin(void) {
    // Configure CS pin
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    
    // Wait for TLE8888 to be ready after power-up
    delay(10);
    
    // Try to read operating status register
    uint8_t opstat = readReg(TLE_REG_OPSTAT);
    
    // Check if we got a valid response
    // 0xFF or 0x00 typically means no communication
    if (opstat == 0xFF || opstat == 0x00) {
        _connected = false;
        Serial.println(F("TLE8888: NOT CONNECTED"));
        return false;
    }
    
    _connected = true;
    Serial.print(F("TLE8888: Connected (OPSTAT=0x"));
    Serial.print(opstat, HEX);
    Serial.println(F(")"));
    
    // Clear any faults by reading diagnostic registers
    readReg(TLE_REG_DIAG0);
    
    // Initialize all outputs OFF
    _outputState = 0;
    _ignitionState = 0;
    setAllOutputs(0);
    setAllIgnition(0);
    
    return true;
}

// ============================================================================
// OUTPUT CONTROL
// ============================================================================

void TLE8888::setOutput(uint8_t channel, bool state) {
    if (channel > 15) return;
    
    // Update state tracking
    if (state) {
        _outputState |= (1 << channel);
    } else {
        _outputState &= ~(1 << channel);
    }
    
    // Write to appropriate register
    if (channel < 8) {
        writeReg(TLE_REG_CONT, _outputState & 0xFF);
    } else {
        writeReg(TLE_REG_CONT1, (_outputState >> 8) & 0xFF);
    }
}

void TLE8888::setAllOutputs(uint16_t state) {
    _outputState = state;
    writeReg(TLE_REG_CONT, state & 0xFF);
    writeReg(TLE_REG_CONT1, (state >> 8) & 0xFF);
}

// ============================================================================
// IGNITION CONTROL
// ============================================================================

void TLE8888::setIgnition(uint8_t channel, bool state) {
    if (channel > 3) return;
    
    if (state) {
        _ignitionState |= (1 << channel);
    } else {
        _ignitionState &= ~(1 << channel);
    }
    
    writeReg(TLE_REG_IGNCONT, _ignitionState);
}

void TLE8888::setAllIgnition(uint8_t state) {
    _ignitionState = state & 0x0F;
    writeReg(TLE_REG_IGNCONT, _ignitionState);
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================

bool TLE8888::isConnected(void) {
    if (!_connected) return false;
    
    // Verify by reading OPSTAT
    uint8_t opstat = readReg(TLE_REG_OPSTAT);
    return (opstat != 0xFF && opstat != 0x00);
}

bool TLE8888::hasCriticalFault(void) {
    if (!_connected) return true;
    
    uint8_t diag0 = readReg(TLE_REG_DIAG0);
    
    // Check critical bits:
    // Bit 0: COT (chip over temperature)
    // Bit 2: VS_UV (supply undervoltage)
    // Bit 3: VS_OV (supply overvoltage)
    // Bit 6: TSD (thermal shutdown)
    const uint8_t criticalMask = 0x4D;  // bits 0,2,3,6
    
    return (diag0 & criticalMask) != 0;
}

void TLE8888::serviceWatchdog(void) {
    if (!_connected) return;
    
    // TLE8888 has a window watchdog
    // In default mode, writing 0x0F to register 0x15 services it
    writeReg(0x15, 0x0F);
}

// ============================================================================
// DEBUG
// ============================================================================

void TLE8888::printStatus(void) {
    Serial.println(F("\n===== TLE8888 STATUS ====="));
    Serial.print(F("Connected: ")); Serial.println(_connected ? "YES" : "NO");
    
    if (_connected) {
        uint8_t opstat = readReg(TLE_REG_OPSTAT);
        uint8_t diag0 = readReg(TLE_REG_DIAG0);
        
        Serial.print(F("OPSTAT: 0x")); Serial.println(opstat, HEX);
        Serial.print(F("DIAG0:  0x")); Serial.println(diag0, HEX);
        
        // Decode DIAG0
        if (diag0 & 0x01) Serial.println(F("  - COT: Chip Over Temp"));
        if (diag0 & 0x02) Serial.println(F("  - CF: Comm Fault"));
        if (diag0 & 0x04) Serial.println(F("  - VS_UV: Undervoltage"));
        if (diag0 & 0x08) Serial.println(F("  - VS_OV: Overvoltage"));
        if (diag0 & 0x40) Serial.println(F("  - TSD: Thermal Shutdown"));
        if (diag0 == 0x00) Serial.println(F("  (no faults)"));
    }
    
    Serial.print(F("Outputs:  0x")); Serial.println(_outputState, HEX);
    Serial.print(F("Ignition: 0x")); Serial.println(_ignitionState, HEX);
    Serial.println(F("==========================\n"));
}
