/*
 * config.h - Engine Configuration Constants
 * ECU STM32F405 v8.2
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// ENGINE CONFIGURATION
// ============================================================================

#define ENGINE_CYLINDERS        4
#define ENGINE_STROKE_DEGREES   720     // 4-stroke = 720°
#define CRANK_DEGREES_PER_REV   360

// Firing order: 1-3-4-2 (common for inline 4 cylinder)
// This defines TDC compression angle for each cylinder
// Cylinder 1: TDC at 0° (and 720°)
// Cylinder 3: TDC at 180°
// Cylinder 4: TDC at 360°
// Cylinder 2: TDC at 540°
#define FIRING_ORDER_CYL1       0       // Cylinder 1 fires at 0°
#define FIRING_ORDER_CYL3       180     // Cylinder 3 fires at 180°
#define FIRING_ORDER_CYL4       360     // Cylinder 4 fires at 360°
#define FIRING_ORDER_CYL2       540     // Cylinder 2 fires at 540°

// ============================================================================
// TRIGGER WHEEL CONFIGURATION
// ============================================================================

#define CRANK_TEETH             60
#define MISSING_TEETH           2
#define DEGREES_PER_TOOTH       (360 / CRANK_TEETH)  // 6°

// Trigger angle (degrees BTDC where tooth 1 occurs after gap)
// CRITICAL: Measure this on your actual engine!
#define TRIGGER_ANGLE_BTDC      90

// Gap detection ratios
#define GAP_RATIO_MIN           1.5f    // Min ratio to detect gap
#define GAP_RATIO_MAX           3.5f    // Max ratio (reject noise)

// Minimum tooth period for noise rejection (µs)
#define MIN_TOOTH_PERIOD_US     50

// ============================================================================
// SENSOR CALIBRATION
// ============================================================================

// ADC configuration
#define ADC_RESOLUTION          4096.0f // 12-bit ADC
#define ADC_VREF                3.3f    // Reference voltage
#define ADC_OVERSAMPLE_COUNT    4       // Oversampling for noise reduction

// TPS calibration (voltage)
#define TPS_MIN_VOLTAGE         0.5f    // Voltage at 0% throttle
#define TPS_MAX_VOLTAGE         4.5f    // Voltage at 100% throttle

// Battery voltage divider (e.g., 47k/10k gives 5.7:1 ratio)
#define VBAT_DIVIDER_RATIO      5.7f

// NTC thermistor (10k nominal, B=3950)
#define NTC_NOMINAL_RESISTANCE  10000.0f
#define NTC_NOMINAL_TEMP        25.0f   // °C
#define NTC_BETA                3950.0f
#define NTC_SERIES_RESISTOR     2200.0f // Pull-up resistor

// ============================================================================
// FUEL SYSTEM
// ============================================================================

// Injector specifications
#define INJECTOR_FLOW_CC_MIN    440.0f  // cc/min at 3 bar
#define INJECTOR_DEAD_TIME_US   800     // Dead time at 14V
#define INJECTOR_DEAD_TIME_VOLT 14.0f   // Reference voltage for dead time

// Fuel properties (E27 Brazilian gasoline)
// Note: Stoichiometric AFR is defined in lambda.h as AFR_STOICH_E27 (13.0)
#define FUEL_DENSITY_G_CC       0.755f  // Fuel density

// Pulse width limits
#define MIN_INJECTOR_PW_US      500     // Below this, don't inject
#define MAX_INJECTOR_PW_US      25000   // Above this, cap (prevent hydrolock)

// Injection timing (sequential mode)
#define INJ_TIMING_BTDC         330     // Degrees BTDC to start injection (330° = during intake)

// ============================================================================
// IGNITION SYSTEM
// ============================================================================

// Dwell configuration
#define DWELL_TIME_TARGET_US    3500    // Target dwell at 14V
#define DWELL_TIME_MAX_US       10000   // Maximum dwell (overdwell protection)
#define DWELL_VOLTAGE_REF       14.0f   // Reference voltage for dwell

// Ignition limits
#define IGN_ADVANCE_MAX         45.0f   // Maximum advance (degrees BTDC)
#define IGN_ADVANCE_MIN         -10.0f  // Maximum retard
#define IGN_ADVANCE_CRANKING    10.0f   // Fixed advance during cranking

// ============================================================================
// RPM THRESHOLDS
// ============================================================================

#define RPM_CRANKING_THRESHOLD  400     // Below this = cranking
#define RPM_IDLE_MIN            500     // Minimum idle RPM
#define RPM_IDLE_MAX            1500    // Maximum idle RPM
#define RPM_REV_LIMIT_SOFT      7500    // Soft rev limit (fuel cut starts)
#define RPM_REV_LIMIT_HARD      8000    // Hard rev limit (full cut)
#define RPM_MAX_VALID           12000   // Above this = noise

// ============================================================================
// SAFETY
// ============================================================================

// Note: Sync loss timeout is defined in sync.h as SYNC_LOSS_TIMEOUT_US

// TLE8888 watchdog
#define TLE_WATCHDOG_PERIOD_MS  10      // Service watchdog every 10ms

// Battery voltage limits (for diagnostics)
#define VBAT_MIN                9.0f    // Below this = warning
#define VBAT_MAX                16.0f   // Above this = warning

// Temperature limits (for diagnostics)
#define CLT_OVERHEAT            110.0f  // Coolant overheat warning (°C)

// ============================================================================
// COMMUNICATION
// ============================================================================

#define SERIAL_BAUD             115200

// Bluetooth Serial (HC-05/HC-06 module)
// Uncomment to enable Bluetooth tuning via USART1
#define BLUETOOTH_ENABLED       1
#define BLUETOOTH_BAUD          9600    // HC-05 default (pode ser 38400 ou 115200)

// Bluetooth module pins (USART1)
// PA9  = TX (connect to HC-05 RX)
// PA10 = RX (connect to HC-05 TX)
// VCC  = 3.3V or 5V (check module)
// GND  = GND

// ============================================================================
// CAN BUS (Not yet implemented - reserved for future use)
// ============================================================================
// #define CAN_BAUD                500000  // 500 kbps
// #define CAN_ID_ECU_STATUS       0x100
// #define CAN_ID_SENSOR_DATA      0x101
// #define CAN_ID_FUEL_IGN         0x102
// #define CAN_ID_LAMBDA           0x103

#endif // CONFIG_H
