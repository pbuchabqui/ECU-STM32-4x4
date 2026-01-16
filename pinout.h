/*
 * pinout.h - Pin Definitions
 * ECU STM32F405 v8.2
 * 
 * STM32F405RGT6 + TLE8888-1QK
 */

#ifndef PINOUT_H
#define PINOUT_H

// ============================================================================
// ANALOG INPUTS (ADC1)
// ============================================================================

#define PIN_TPS         PA0     // Throttle Position Sensor
#define PIN_CLT         PA1     // Coolant Temperature (NTC)
#define PIN_IAT         PA2     // Intake Air Temperature (NTC)
#define PIN_MAP         PA3     // Manifold Absolute Pressure (optional)
#define PIN_VBAT        PA4     // Battery Voltage (via divider)
#define PIN_WBO2        PA6     // Wideband O2 analog 0-5V (optional, if not using CAN)

// ============================================================================
// FREQUENCY INPUTS
// ============================================================================

#define PIN_CRANK       PB6     // Crankshaft Position (36-1 trigger)
#define PIN_CAM         PB7     // Camshaft Position (single tooth)
#define PIN_MAF_FREQ    PB8     // MAF Frequency Output
#define PIN_VSS         PB9     // Vehicle Speed Sensor (optional)

// ============================================================================
// SPI - TLE8888
// ============================================================================

#define PIN_SPI_SCK     PA5     // SPI1 Clock
#define PIN_SPI_MISO    PA6     // SPI1 MISO
#define PIN_SPI_MOSI    PA7     // SPI1 MOSI
#define PIN_TLE_CS      PA8     // TLE8888 Chip Select
#define PIN_TLE_RST     PA9     // TLE8888 Reset
#define PIN_TLE_EN      PA10    // TLE8888 Enable

// ============================================================================
// CAN BUS - Wideband O2
// ============================================================================

#define PIN_CAN_RX      PB8     // CAN1 RX (shared with MAF_FREQ via remap)
#define PIN_CAN_TX      PB9     // CAN1 TX

// ============================================================================
// DIRECT OUTPUTS (For testing/backup)
// ============================================================================

#define PIN_INJ1_DIRECT PC0     // Injector 1 direct drive
#define PIN_INJ2_DIRECT PC1     // Injector 2 direct drive
#define PIN_INJ3_DIRECT PC2     // Injector 3 direct drive
#define PIN_INJ4_DIRECT PC3     // Injector 4 direct drive

#define PIN_IGN1_DIRECT PC4     // Ignition 1 direct drive
#define PIN_IGN2_DIRECT PC5     // Ignition 2 direct drive
#define PIN_IGN3_DIRECT PC6     // Ignition 3 direct drive
#define PIN_IGN4_DIRECT PC7     // Ignition 4 direct drive

// ============================================================================
// AUXILIARY OUTPUTS
// ============================================================================

#define PIN_FUEL_PUMP   PC8     // Fuel Pump Relay
#define PIN_FAN         PC9     // Radiator Fan Relay
#define PIN_IDLE_VALVE  PC10    // Idle Air Control Valve
#define PIN_VVT         PC11    // Variable Valve Timing (optional)
#define PIN_TACHO       PC12    // Tachometer Output

// ============================================================================
// STATUS LEDs
// ============================================================================

#define PIN_LED_STATUS  PD2     // Green - Heartbeat
#define PIN_LED_SYNC    PD3     // Blue - Sync Valid
#define PIN_LED_ERROR   PD4     // Red - Error/Fault

// ============================================================================
// DEBUG/SERIAL
// ============================================================================

#define PIN_UART_TX     PA2     // USART2 TX (shared with IAT, use USART1 instead)
#define PIN_UART_RX     PA3     // USART2 RX

// Actually using USB CDC for Serial
// PA11 = USB_DM
// PA12 = USB_DP

// Bluetooth Serial (HC-05/HC-06) via USART1
#define PIN_BT_TX       PA9     // USART1 TX -> HC-05 RX
#define PIN_BT_RX       PA10    // USART1 RX -> HC-05 TX

// ============================================================================
// TIMER ASSIGNMENTS (For Reference)
// ============================================================================

// TIM2 (32-bit): Ignition scheduling (CH1-CH4)
// TIM3 (16-bit): Injection scheduling (CH1-CH4)
// TIM4: General purpose / MAF frequency capture
// TIM5: System timing / RPM calculation

#endif // PINOUT_H
