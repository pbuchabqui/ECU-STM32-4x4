/*
 * ECU STM32F405 + TLE8888 - v8.2.1
 * Hardware Output Compare Scheduler + Full Feature Set
 * 
 * Hardware: STM32F405RGT6 + TLE8888-1QK
 * Platform: Arduino IDE with STM32duino
 * 
 * v8.2.1 FEATURES:
 * - All tables consolidated in tables.h/cpp
 * - 16x16 VE, IGN, AFR tables with mg/stroke load axis
 * - Real-time calibration via Serial (+/- keys)
 * - EEPROM persistence for tuned values
 * - MAF primary fuel model with Alpha-N fallback
 * - Autotune: STFT/LTFT automatic table learning
 * - Bluetooth serial support (HC-05/HC-06)
 * 
 * Timer Assignment:
 *   TIM2 (32-bit): IGN1-IGN4 scheduling
 *   TIM3 (16-bit): INJ1-INJ4 scheduling
 */

#include "types.h"
#include "config.h"
#include "pinout.h"
#include "tables.h"
#include "calibration.h"
#include "autotune.h"
#include "tle8888.h"
#include "scheduler.h"
#include "idle.h"
#include "sync.h"
#include "lambda.h"
#include <STM32_CAN.h>

// ============================================================================
// GLOBAL STATE
// ============================================================================

volatile SensorData sensorData;
EngineStatus engineStatus;

// Angle tracking
volatile bool needsCalculation = false;

// Engine state
bool wasRunning = false;
uint32_t engineStartTime = 0;
volatile uint32_t cyclesSinceStart = 0;
bool isCranking = false;

// Emergency shutdown
bool emergencyShutdown = false;
uint32_t emergencyTime = 0;

// TPS tracking for accel enrichment
float lastTps = 0;
uint32_t lastTpsReadTime = 0;
float tpsDot = 0;

// ============================================================================
// FUEL MODEL SELECTION
// ============================================================================

typedef enum {
    FUEL_MODEL_MAF = 0,     // Primary: Mass Air Flow sensor
    FUEL_MODEL_ALPHA_N      // Fallback: Speed-Density with VE table
} FuelModel;

FuelModel activeFuelModel = FUEL_MODEL_MAF;
bool mafFailed = false;

// MAF variables
volatile uint32_t mafPulseCount = 0;
volatile uint32_t mafLastPulseTime = 0;
float mafFrequencyHz = 0;
uint32_t mafRecoveryTime = 0;

#define MAF_UPDATE_INTERVAL_MS      50
#define MAF_RECOVERY_TIMEOUT_MS     1000
#define MAF_MIN_VALID_HZ            100.0f
#define MAF_MAX_VALID_HZ            12000.0f

// ============================================================================
// WIDEBAND O2 CONFIGURATION
// ============================================================================

// Input source: 0 = CAN, 1 = Analog 0-5V
#define WB_INPUT_SOURCE         0

// CAN configuration (AEM X-Series, Innovate, etc.)
#define WB_CAN_ID               0x180   // AEM X-Series default
#define WB_CAN_TIMEOUT_MS       500

// Analog configuration (for 0-5V wideband controllers)
// Uses PIN_WBO2 (PA6) defined in pinout.h
// Calibration is in lambda.h (WB_AFR_MIN, WB_AFR_MAX)

volatile float canLambda = 1.0f;
volatile uint32_t canLambdaTime = 0;
volatile bool canLambdaValid = false;

// CAN instance
STM32_CAN Can1(CAN1, DEF, RX_SIZE_64, TX_SIZE_16);

// ============================================================================
// ISR: MAF PULSE COUNTER
// ============================================================================

void mafPulseISR(void) {
    mafPulseCount++;
    mafLastPulseTime = micros();
}

// ============================================================================
// ISR: CRANK POSITION
// ============================================================================

void crankPositionISR(void) {
    uint32_t now = micros();
    syncProcessTooth(now);
    
    if (syncIsValid()) {
        needsCalculation = true;
    }
}

// ============================================================================
// ISR: CAM POSITION
// ============================================================================

void camPositionISR(void) {
    uint32_t now = micros();
    syncProcessCam(now);
}

// ============================================================================
// SENSOR READING
// ============================================================================

float readTPS(void) {
    int raw = 0;
    for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        raw += analogRead(PIN_TPS);
    }
    raw /= ADC_OVERSAMPLE_COUNT;
    
    float voltage = (raw / ADC_RESOLUTION) * ADC_VREF;
    float tps = ((voltage - TPS_MIN_VOLTAGE) / (TPS_MAX_VOLTAGE - TPS_MIN_VOLTAGE)) * 100.0f;
    return constrain(tps, 0.0f, 100.0f);
}

float readNTC(uint8_t pin) {
    int raw = 0;
    for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
        raw += analogRead(pin);
    }
    raw /= ADC_OVERSAMPLE_COUNT;
    raw = constrain(raw, 10, 4085);
    
    float resistance = NTC_SERIES_RESISTOR / ((ADC_RESOLUTION / raw) - 1.0f);
    resistance = constrain(resistance, 100.0f, 100000.0f);
    
    float steinhart = log(resistance / NTC_NOMINAL_RESISTANCE);
    steinhart /= NTC_BETA;
    steinhart += 1.0f / (NTC_NOMINAL_TEMP + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;
    
    return constrain(steinhart, -40.0f, 150.0f);
}

float readBatteryVoltage(void) {
    int raw = analogRead(PIN_VBAT);
    float voltage = (raw / ADC_RESOLUTION) * ADC_VREF * VBAT_DIVIDER_RATIO;
    return constrain(voltage, 6.0f, 18.0f);
}

// ============================================================================
// MAF FREQUENCY MEASUREMENT
// ============================================================================

void updateMafFrequency(void) {
    static uint32_t lastMafUpdate = 0;
    static uint32_t lastMafPulseCount = 0;
    
    uint32_t now = millis();
    if (now - lastMafUpdate < MAF_UPDATE_INTERVAL_MS) return;
    
    noInterrupts();
    uint32_t pulses = mafPulseCount - lastMafPulseCount;
    lastMafPulseCount = mafPulseCount;
    uint32_t elapsed = now - lastMafUpdate;
    interrupts();
    
    lastMafUpdate = now;
    
    if (elapsed > 0) {
        mafFrequencyHz = (pulses * 1000.0f) / elapsed;
    }
    
    // Validate MAF signal
    uint16_t rpm = syncGetRPM();
    if (rpm > 300) {
        if (mafFrequencyHz < MAF_MIN_VALID_HZ || mafFrequencyHz > MAF_MAX_VALID_HZ) {
            if (!mafFailed) {
                mafFailed = true;
                mafRecoveryTime = now;
                activeFuelModel = FUEL_MODEL_ALPHA_N;
                Serial.println(F("MAF FAIL -> Alpha-N fallback"));
            }
        } else if (mafFailed) {
            // Try to recover after timeout
            if (now - mafRecoveryTime > MAF_RECOVERY_TIMEOUT_MS) {
                mafFailed = false;
                activeFuelModel = FUEL_MODEL_MAF;
                Serial.println(F("MAF RECOVERED"));
            }
        }
    }
}

// ============================================================================
// WIDEBAND CAN PROCESSING
// ============================================================================

void processWidebandCAN(uint32_t id, uint8_t* data, uint8_t len) {
    if (data == NULL || len < 2) return;
    
    if (id == WB_CAN_ID) {
        // AEM X-Series format: Lambda = (data[0] * 256 + data[1]) / 10000
        uint16_t rawLambda = ((uint16_t)data[0] << 8) | data[1];
        
        noInterrupts();
        canLambda = rawLambda / 10000.0f;
        canLambdaTime = millis();
        canLambdaValid = true;
        interrupts();
    }
}

// Read wideband lambda value
// Returns lambda value (1.0 = stoichiometric)
float readLambdaSensor(void) {
#if WB_INPUT_SOURCE == 0
    // CAN input
    noInterrupts();
    float lambda = canLambda;
    uint32_t age = millis() - canLambdaTime;
    bool valid = canLambdaValid;
    interrupts();
    
    if (!valid || age > WB_CAN_TIMEOUT_MS) {
        return 1.0f;  // Default to stoich if no valid CAN data
    }
    return lambda;
#else
    // Analog 0-5V input
    // Read ADC and convert to lambda
    float voltage = analogRead(PIN_WBO2) * (3.3f / 4096.0f) * (5.0f / 3.3f);  // Assuming 5V signal scaled to 3.3V
    // Convert voltage to AFR using calibration from lambda.h
    float afr = WB_AFR_MIN + (voltage / WB_VOLTAGE_MAX) * (WB_AFR_MAX - WB_AFR_MIN);
    return afr / AFR_STOICH_DEFAULT;
#endif
}

bool isLambdaSensorValid(void) {
#if WB_INPUT_SOURCE == 0
    // CAN input
    noInterrupts();
    uint32_t age = millis() - canLambdaTime;
    bool valid = canLambdaValid;
    interrupts();
    return valid && (age < WB_CAN_TIMEOUT_MS);
#else
    // Analog input - check for valid voltage range
    float voltage = analogRead(PIN_WBO2) * (3.3f / 4096.0f) * (5.0f / 3.3f);
    return (voltage > 0.1f && voltage < 4.9f);
#endif
}

// ============================================================================
// SENSOR UPDATE
// ============================================================================

void updateSensors(void) {
    // TPS with rate calculation
    uint32_t now = millis();
    float currentTps = readTPS();
    
    if (lastTpsReadTime > 0) {
        uint32_t dt = now - lastTpsReadTime;
        if (dt > 0 && dt < 1000) {
            tpsDot = (currentTps - lastTps) * 1000.0f / dt;
        }
    }
    lastTps = currentTps;
    lastTpsReadTime = now;
    
    sensorData.tpsPercent = currentTps;
    sensorData.cltCelsius = readNTC(PIN_CLT);
    sensorData.iatCelsius = readNTC(PIN_IAT);
    sensorData.batteryVoltage = readBatteryVoltage();
    
    // MAF frequency and flow
    updateMafFrequency();
    sensorData.mafFrequencyHz = mafFrequencyHz;
    sensorData.mafGramsSec = getMafFlow(mafFrequencyHz);
    
    // Calculate load (mg/stroke) based on active fuel model
    uint16_t rpm = syncGetRPM();
    if (rpm > 0) {
        if (activeFuelModel == FUEL_MODEL_MAF && !mafFailed) {
            // MAF MODE: Direct air mass measurement
            // load = (g/s * 1000 * 120) / (rpm * cylinders) = mg/stroke
            // Tuning: AFR table only (VE not used)
            sensorData.loadMgStroke = (sensorData.mafGramsSec * 120000.0f) / 
                                       (rpm * ENGINE_CYLINDERS);
        } else {
            // ALPHA-N FALLBACK: VE-based air mass estimate
            // Tuning: VE table (air mass) + AFR table (mixture)
            float ve = getVe(rpm, sensorData.tpsPercent);
            float displacement_cc = 2000.0f / ENGINE_CYLINDERS;  // 500cc per cylinder
            float airDensity = 1.2f;  // g/L at sea level
            sensorData.loadMgStroke = (displacement_cc * (ve / 100.0f) * airDensity);
        }
    }
    
    // Lambda
    sensorData.lambda = readLambdaSensor();
}

// ============================================================================
// FUEL PULSE WIDTH CALCULATION
// ============================================================================

float calculateFuelPW(void) {
    uint16_t rpm = syncGetRPM();
    
    // Validation
    if (rpm < 100 || rpm > RPM_MAX_VALID) return 0;
    if (!isfinite(sensorData.loadMgStroke) || sensorData.loadMgStroke < 1.0f) return 0;
    
    float load = constrain(sensorData.loadMgStroke, 10.0f, 300.0f);
    
    // =======================================================================
    // FUEL MODEL:
    // - MAF Mode: air mass comes directly from MAF sensor (updateSensors)
    // - Alpha-N Mode: air mass calculated via VE table (updateSensors)
    // 
    // In BOTH modes, tuning is done via AFR table only.
    // VE table is ONLY used for Alpha-N air mass estimation.
    // Lambda closed-loop auto-corrects to hit AFR target.
    // =======================================================================
    
    // Get target AFR from table (this is the tuning parameter)
    float targetAfr = getAfrTarget((float)rpm, load);
    if (targetAfr < 9.0f) targetAfr = 9.0f;
    
    // Air mass already calculated in updateSensors() based on fuel model
    float airMass = load;  // mg/stroke
    
    // Fuel mass needed: fuel_mg = air_mg / AFR
    float fuelMass = airMass / targetAfr;
    
    // Injector flow: cc/min -> mg/ms (assuming 0.75 g/cc fuel density)
    float injectorFlow_mg_ms = (INJECTOR_FLOW_CC_MIN * FUEL_DENSITY_G_CC) / 60000.0f;
    
    // Base pulse width (ms)
    float basePW = fuelMass / injectorFlow_mg_ms;
    
    // Apply corrections
    float totalCorrection = 1.0f;
    
    // Cranking enrichment
    if (rpm < RPM_CRANKING_THRESHOLD) {
        totalCorrection *= getCrankingEnrichment(sensorData.cltCelsius);
    } else {
        // Running enrichments
        totalCorrection *= (1.0f + getWarmupEnrichment(sensorData.cltCelsius));
        totalCorrection *= (1.0f + getAfterstartEnrichment(sensorData.cltCelsius, cyclesSinceStart));
        totalCorrection *= (1.0f + getAccelEnrichment(tpsDot, sensorData.cltCelsius));
        
        // Lambda closed-loop correction (auto-tunes to AFR target)
        if (lambdaIsClosedLoop()) {
            totalCorrection *= (1.0f + lambdaGetCorrection() / 100.0f);
        }
    }
    
    // Validation
    totalCorrection = constrain(totalCorrection, 0.5f, 5.0f);
    
    // Apply correction
    float finalPW = basePW * totalCorrection;
    
    // Add dead time (voltage compensated)
    float deadTime = getDeadTime(sensorData.batteryVoltage) / 1000.0f;  // µs -> ms
    finalPW += deadTime;
    
    // Convert to µs and apply limits
    float pwUs = finalPW * 1000.0f;
    pwUs = constrain(pwUs, MIN_INJECTOR_PW_US, MAX_INJECTOR_PW_US);
    
    // Flood clear
    if (isFloodClearActive(sensorData.tpsPercent, rpm)) {
        pwUs = 0;
    }
    
    return pwUs;
}

// ============================================================================
// IGNITION ADVANCE CALCULATION
// ============================================================================

float calculateIgnitionAdvance(void) {
    uint16_t rpm = syncGetRPM();
    
    if (rpm < 100) return 0;
    
    float load = constrain(sensorData.loadMgStroke, 10.0f, 300.0f);
    
    // Cranking: fixed advance
    if (rpm < RPM_CRANKING_THRESHOLD) {
        return IGN_ADVANCE_CRANKING;
    }
    
    // Get base advance from table
    float baseAdvance = getIgnition((float)rpm, load);
    
    // Apply idle correction
    float idleCorr = 0;
    if (idleIsActive()) {
        idleCorr = idleGetIgnitionCorrection();
    }
    
    float finalAdvance = baseAdvance + idleCorr;
    
    // Apply limits
    finalAdvance = constrain(finalAdvance, IGN_ADVANCE_MIN, IGN_ADVANCE_MAX);
    
    return finalAdvance;
}

// ============================================================================
// DWELL CALCULATION
// ============================================================================

uint32_t calculateDwell(void) {
    uint32_t dwell = getDwellTime(sensorData.batteryVoltage);
    return constrain(dwell, 1000, DWELL_TIME_MAX_US);
}

// ============================================================================
// EVENT SCHEDULING - SEQUENTIAL INJECTION AND IGNITION
// ============================================================================

// Firing order configuration (1-3-4-2 for most inline 4)
// Maps schedule number to firing angle (TDC compression)
// Schedule 1 = Cylinder 1 = fires at 0°
// Schedule 2 = Cylinder 2 = fires at 540°  
// Schedule 3 = Cylinder 3 = fires at 180°
// Schedule 4 = Cylinder 4 = fires at 360°

static const uint16_t cylinderFireAngle[4] = {
    FIRING_ORDER_CYL1,  // Cylinder 1: TDC compression at 0°
    FIRING_ORDER_CYL2,  // Cylinder 2: TDC compression at 540°
    FIRING_ORDER_CYL3,  // Cylinder 3: TDC compression at 180°
    FIRING_ORDER_CYL4   // Cylinder 4: TDC compression at 360°
};

// Calculate degrees until target angle
static int16_t degreesUntil(int16_t currentAngle, int16_t targetAngle) {
    int16_t delta = targetAngle - currentAngle;
    if (delta < 0) delta += 720;
    if (delta > 720) delta -= 720;
    return delta;
}

void scheduleEvents(void) {
    if (!syncIsValid()) return;
    if (!needsCalculation) return;
    
    needsCalculation = false;
    
    uint16_t rpm = syncGetRPM();
    if (rpm < 100) return;
    
    // Calculate values
    float fuelPW = calculateFuelPW();
    float ignAdvance = calculateIgnitionAdvance();
    uint32_t dwellTime = calculateDwell();
    
    // Store for diagnostics
    engineStatus.fuelPulseWidth = fuelPW;
    engineStatus.ignitionAdvance = ignAdvance;
    engineStatus.dwellTime = dwellTime;
    
    // Rev limiter
    engineStatus.revLimitActive = false;
    engineStatus.softLimitActive = false;
    
    if (rpm >= RPM_REV_LIMIT_HARD) {
        engineStatus.revLimitActive = true;
        fuelPW = 0;
    } else if (rpm >= RPM_REV_LIMIT_SOFT) {
        engineStatus.softLimitActive = true;
        // Soft limit: cut some cylinders
        static uint8_t softLimitCounter = 0;
        softLimitCounter++;
        if (softLimitCounter % 2 == 0) {
            fuelPW = 0;
        }
    }
    
    // Get current angle and timing
    int16_t crankAngle = syncGetCrankAngle();
    uint32_t toothPeriod = syncStatus.avgToothPeriod;
    
    if (toothPeriod == 0) return;
    
    // Time per degree (µs/deg)
    float usPerDeg = toothPeriod / (float)DEGREES_PER_TOOTH;
    
    // =========================================================================
    // SEQUENTIAL INJECTION SCHEDULING
    // =========================================================================
    // Inject at INJ_ANGLE_BTDC before each cylinder's TDC compression
    
    if (fuelPW > MIN_INJECTOR_PW_US) {
        // Calculate injection start angle for each cylinder
        // Injection starts INJ_TIMING_BTDC before TDC
        
        for (int cyl = 0; cyl < 4; cyl++) {
            // Calculate target angle for injection start
            int16_t injStartAngle = cylinderFireAngle[cyl] - INJ_TIMING_BTDC;
            if (injStartAngle < 0) injStartAngle += 720;
            
            // Degrees until injection start
            int16_t degsToInj = degreesUntil(crankAngle, injStartAngle);
            
            // Only schedule if event is upcoming (within next 720°)
            // and not too close (> 10°)
            if (degsToInj > 10 && degsToInj < 700) {
                uint32_t injTimeout = (uint32_t)(degsToInj * usPerDeg);
                
                // Limit timeout to reasonable value
                if (injTimeout < 50000) {  // Max 50ms
                    noInterrupts();
                    switch (cyl) {
                        case 0: setFuelSchedule1(injTimeout, (uint32_t)fuelPW); break;
                        case 1: setFuelSchedule2(injTimeout, (uint32_t)fuelPW); break;
                        case 2: setFuelSchedule3(injTimeout, (uint32_t)fuelPW); break;
                        case 3: setFuelSchedule4(injTimeout, (uint32_t)fuelPW); break;
                    }
                    interrupts();
                }
            }
        }
    }
    
    // =========================================================================
    // SEQUENTIAL IGNITION SCHEDULING
    // =========================================================================
    // Dwell starts at (fireAngle - advance - dwellDegrees)
    // Spark fires at (fireAngle - advance)
    
    if (dwellTime > 0) {
        // Convert dwell time to degrees
        float dwellDegrees = dwellTime / usPerDeg;
        
        for (int cyl = 0; cyl < 4; cyl++) {
            // Spark fires at TDC minus advance
            int16_t sparkAngle = cylinderFireAngle[cyl] - (int16_t)ignAdvance;
            if (sparkAngle < 0) sparkAngle += 720;
            
            // Dwell starts before spark
            int16_t dwellStartAngle = sparkAngle - (int16_t)dwellDegrees;
            if (dwellStartAngle < 0) dwellStartAngle += 720;
            
            // Degrees until dwell start
            int16_t degsToDwell = degreesUntil(crankAngle, dwellStartAngle);
            
            // Only schedule if event is upcoming
            if (degsToDwell > 5 && degsToDwell < 700) {
                uint32_t ignTimeout = (uint32_t)(degsToDwell * usPerDeg);
                
                // Limit timeout
                if (ignTimeout < 100000) {  // Max 100ms
                    noInterrupts();
                    switch (cyl) {
                        case 0: setIgnitionSchedule1(ignTimeout, dwellTime); break;
                        case 1: setIgnitionSchedule2(ignTimeout, dwellTime); break;
                        case 2: setIgnitionSchedule3(ignTimeout, dwellTime); break;
                        case 3: setIgnitionSchedule4(ignTimeout, dwellTime); break;
                    }
                    interrupts();
                }
            }
        }
    }
}

// ============================================================================
// EMERGENCY SHUTDOWN
// ============================================================================

void doEmergencyShutdown(const char* reason) {
    emergencyShutdown = true;
    emergencyTime = millis();
    
    disableAllSchedules();
    tle8888.setAllOutputs(0);
    tle8888.setAllIgnition(0);
    digitalWrite(PIN_FUEL_PUMP, LOW);
    
    engineStatus.errorFlags |= ERR_EMERGENCY_SHUTDOWN;
    
    Serial.print(F("EMERGENCY SHUTDOWN: "));
    Serial.println(reason);
}

// ============================================================================
// SERIAL PROCESSING (Extended with Calibration)
// ============================================================================

// Active serial port for responses (USB or Bluetooth)
Stream* activeSerial = &Serial;

// Process command from any serial port
void processSerialCommand(char c) {
    // Calibration commands are single characters handled by calibration module
    // Note: 'M' (uppercase) prints MAF table, 'm' (lowercase) enters MAF calibration mode
    if (c == 'V' || c == 'v' || c == 'I' || c == 'i' || c == 'A' || c == 'a' ||
        c == 'P' || c == 'p' ||  // Lambda PID toggle
        c == 'm' ||              // MAF calibration mode (lowercase only)
        c == '+' || c == '=' || c == '-' || c == '_' ||
        c == 'H' || c == 'h' || c == '1' || c == '2' || c == '3' || 
        c == '4' || c == '5' || c == '6') {
        // Calibration module handles these
        if (activeSerial == &Serial) {
            calibrationProcessSerial();
        } else {
            activeSerial->println(F("Calibration: use USB or send 'S' for status"));
        }
        return;
    }
    
    switch (c) {
        case 'S':
            activeSerial->println(F("\n===== ECU STATUS v8.2.1 ====="));
            activeSerial->print(F("Fuel Model: "));
            activeSerial->println(activeFuelModel == FUEL_MODEL_MAF ? "MAF" : "ALPHA-N");
            activeSerial->print(F("RPM: ")); activeSerial->println(syncGetRPM());
            activeSerial->print(F("MAF Hz: ")); activeSerial->println(mafFrequencyHz, 0);
            activeSerial->print(F("MAF g/s: ")); activeSerial->println(sensorData.mafGramsSec, 1);
            activeSerial->print(F("Load: ")); activeSerial->print(sensorData.loadMgStroke, 1); 
            activeSerial->println(F(" mg/stk"));
            activeSerial->print(F("TPS: ")); activeSerial->print(sensorData.tpsPercent, 1); 
            activeSerial->println(F("%"));
            activeSerial->print(F("CLT: ")); activeSerial->print(sensorData.cltCelsius, 1); 
            activeSerial->println(F("C"));
            activeSerial->print(F("IAT: ")); activeSerial->print(sensorData.iatCelsius, 1); 
            activeSerial->println(F("C"));
            activeSerial->print(F("VBAT: ")); activeSerial->print(sensorData.batteryVoltage, 1); 
            activeSerial->println(F("V"));
            activeSerial->print(F("Lambda: ")); activeSerial->print(sensorData.lambda, 3);
            activeSerial->print(F(" (PID: ")); 
            activeSerial->print(lambdaIsAutoCorrectEnabled() ? "ON" : "OFF");
            activeSerial->print(F(", Corr: ")); 
            activeSerial->print(lambdaGetCorrection(), 1); 
            activeSerial->println(F("%)"));
            activeSerial->print(F("Fuel PW: ")); 
            activeSerial->print(engineStatus.fuelPulseWidth, 0); 
            activeSerial->println(F(" us"));
            activeSerial->print(F("Advance: ")); 
            activeSerial->print(engineStatus.ignitionAdvance, 1); 
            activeSerial->println(F(" deg"));
            activeSerial->print(F("Dwell: ")); 
            activeSerial->print(engineStatus.dwellTime, 0); 
            activeSerial->println(F(" us"));
            activeSerial->print(F("Errors: 0x")); 
            activeSerial->println(engineStatus.errorFlags, HEX);
            // Autotune status
            activeSerial->print(F("Autotune: ")); 
            activeSerial->print(autotuneGetModeName());
            if (autotuneGetMode() != AUTOTUNE_MODE_OFF) {
                activeSerial->print(F(" (")); 
                activeSerial->print(autotuneGetStateName());
                activeSerial->print(F(", AvgLTFT: "));
                activeSerial->print(autotuneGetAvgLtft(), 1);
                activeSerial->print(F("%)"));
            }
            activeSerial->println();
            activeSerial->println(F("===========================\n"));
            break;
            
        case 'Y': syncPrintStatus(); break;
        case 'O': idlePrintStatus(); break;
        case 'L': lambdaPrintStatus(); break;
        case 'D': tablesPrintDwell(); break;
        case 'C': tablesPrintCranking(); break;
        case 'M': tablesPrintMaf(); break;
        case 'X': tle8888.printStatus(); break;  // TLE8888 status
        
        case 's':
            activeSerial->println(F("\nSaving calibration to EEPROM..."));
            if (tablesSaveToEEPROM()) {
                activeSerial->println(F("Calibration saved successfully!"));
            } else {
                activeSerial->println(F("Save FAILED!"));
            }
            break;
        
        case 'R':
            if (emergencyShutdown) {
                emergencyShutdown = false;
                activeSerial->println(F("Emergency shutdown CLEARED"));
            }
            break;
            
        case 'B':
            #if BLUETOOTH_ENABLED
            activeSerial->println(F("Bluetooth: ENABLED"));
            activeSerial->print(F("Baud: ")); 
            activeSerial->println(BLUETOOTH_BAUD);
            #else
            activeSerial->println(F("Bluetooth: DISABLED"));
            #endif
            break;
        
        // ===== AUTOTUNE COMMANDS =====
        case 'T':  // Toggle autotune mode (OFF -> VE -> AFR -> MAF -> OFF)
            autotuneCycleMode();
            break;
            
        case 'G':  // Autotune status
            autotunePrintStatus();
            break;
            
        case 'U':  // Print LTFT grid
            autotunePrintLtftGrid();
            break;
            
        case 'J':  // Print MAF LTFT
            autotunePrintLtftMaf();
            break;
            
        case 'Z':  // Zero/reset LTFT
            autotuneReset();
            break;
            
        case 'F':  // Force apply all LTFT now
            autotuneApplyNow();
            break;
            
        case '?':
            activeSerial->println(F("\n=== COMMANDS ==="));
            activeSerial->println(F("S - ECU Status"));
            activeSerial->println(F("Y - Sync Status"));
            activeSerial->println(F("O - Idle Status"));
            activeSerial->println(F("L - Lambda Status"));
            activeSerial->println(F("G - Autotune Status"));
            activeSerial->println(F("X - TLE8888 Diagnostics"));
            activeSerial->println(F("D - Dwell Table"));
            activeSerial->println(F("C - Cranking Table"));
            activeSerial->println(F("M - MAF Table"));
            activeSerial->println(F("B - Bluetooth Info"));
            activeSerial->println(F("--- AUTOTUNE ---"));
            activeSerial->println(F("T - Toggle Mode (OFF/VE/AFR/MAF)"));
            activeSerial->println(F("U - Print LTFT Grid"));
            activeSerial->println(F("J - Print MAF LTFT"));
            activeSerial->println(F("Z - Reset LTFT"));
            activeSerial->println(F("F - Force Apply LTFT"));
            activeSerial->println(F("--- CALIBRATION ---"));
            activeSerial->println(F("s - Save EEPROM"));
            activeSerial->println(F("R - Reset Emergency"));
            activeSerial->println(F("H - Calibration Help"));
            activeSerial->println(F("================\n"));
            break;
    }
}

void processSerial(void) {
    // Check USB Serial
    if (Serial.available()) {
        activeSerial = &Serial;
        char c = Serial.read();
        processSerialCommand(c);
    }
    
    #if BLUETOOTH_ENABLED
    // Check Bluetooth Serial
    if (Serial1.available()) {
        activeSerial = &Serial1;
        char c = Serial1.read();
        processSerialCommand(c);
    }
    #endif
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    
    #if BLUETOOTH_ENABLED
    Serial1.begin(BLUETOOTH_BAUD);  // USART1: PA9=TX, PA10=RX
    #endif
    
    delay(100);
    
    Serial.println(F("\n========================================"));
    Serial.println(F("ECU STM32F405 + TLE8888 v8.2.1"));
    Serial.println(F("MAF Primary + Alpha-N Fallback"));
    Serial.println(F("16x16 VE/IGN/AFR Tables"));
    Serial.println(F("Real-Time Calibration + Autotune"));
    #if BLUETOOTH_ENABLED
    Serial.println(F("Bluetooth: ENABLED (Serial1)"));
    #endif
    Serial.println(F("========================================\n"));
    
    #if BLUETOOTH_ENABLED
    Serial1.println(F("\n== ECU v8.2.1 Bluetooth Ready =="));
    Serial1.println(F("Press H for help"));
    #endif
    
    // GPIO setup
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_SYNC, OUTPUT);
    pinMode(PIN_LED_ERROR, OUTPUT);
    pinMode(PIN_FUEL_PUMP, OUTPUT);
    
    digitalWrite(PIN_LED_ERROR, HIGH);
    
    // Initialize tables (loads from EEPROM if valid)
    Serial.print(F("Tables... "));
    tablesInit();
    Serial.println(F("OK"));
    
    // Initialize calibration system
    Serial.print(F("Calibration... "));
    calibrationInit();
    Serial.println(F("OK"));
    
    // CAN bus for wideband O2
    Serial.print(F("CAN Bus... "));
    Can1.begin();
    Can1.setBaudRate(500000);
    CAN_filter_t filter;
    filter.id = WB_CAN_ID;
    filter.mask = 0x7FF;
    filter.extended = false;
    Can1.setFilter(0, filter);
    Serial.println(F("OK (500kbps)"));
    
    // TLE8888 driver
    Serial.print(F("TLE8888... "));
    if (!tle8888.begin()) {
        Serial.println(F("FAIL"));
        while(1) { 
            digitalWrite(PIN_LED_ERROR, !digitalRead(PIN_LED_ERROR)); 
            delay(100); 
        }
    }
    Serial.println(F("OK"));
    
    // Other modules
    Serial.print(F("Scheduler... ")); schedulerInit(); Serial.println(F("OK"));
    Serial.print(F("Sync... ")); syncInit(); Serial.println(F("OK"));
    Serial.print(F("Idle... ")); idleInit(); Serial.println(F("OK"));
    Serial.print(F("Lambda... ")); lambdaInit(); Serial.println(F("OK"));
    Serial.print(F("Autotune... ")); autotuneInit(); Serial.println(F("OK"));
    
    // Interrupts
    pinMode(PIN_CRANK, INPUT_PULLUP);
    pinMode(PIN_CAM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_CRANK), crankPositionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_CAM), camPositionISR, RISING);
    
    // MAF frequency input
    pinMode(PIN_MAF_FREQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_MAF_FREQ), mafPulseISR, RISING);
    Serial.println(F("MAF input configured"));
    
    // Initial sensor read
    updateSensors();
    
    // Fuel pump prime
    Serial.println(F("Fuel pump prime..."));
    digitalWrite(PIN_FUEL_PUMP, HIGH);
    delay(2000);
    digitalWrite(PIN_FUEL_PUMP, LOW);
    
    digitalWrite(PIN_LED_ERROR, LOW);
    Serial.println(F("\nReady! Type ? for help\n"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static uint32_t lastSensor = 0, lastIdle = 0, lastLambda = 0;
    static uint32_t lastTle = 0, lastBlink = 0, lastCalUpdate = 0;
    
    // Process CAN messages (wideband O2)
    CAN_message_t rxMsg;
    while (Can1.read(rxMsg)) {
        processWidebandCAN(rxMsg.id, rxMsg.buf, rxMsg.len);
    }
    
    // Emergency shutdown handling
    if (emergencyShutdown) {
        processSerial();
        if (TIME_ELAPSED(lastBlink, 200)) {
            lastBlink = millis();
            digitalWrite(PIN_LED_ERROR, !digitalRead(PIN_LED_ERROR));
        }
        return;
    }
    
    // Serial commands (including calibration)
    processSerial();
    
    // Sync timeout check
    syncCheckTimeout();
    
    // Sync recovery attempt
    if (!syncIsValid() && wasRunning) {
        syncAttemptRecovery();
    }
    
    // Event scheduling
    scheduleEvents();
    
    // Overdwell protection
    checkOverdwell();
    
    // Sensors 10Hz
    if (TIME_ELAPSED(lastSensor, 100)) {
        lastSensor = millis();
        updateSensors();
        
        uint16_t rpm = syncGetRPM();
        isCranking = (rpm > 0 && rpm < RPM_CRANKING_THRESHOLD);
    }
    
    // Calibration position update 20Hz
    if (TIME_ELAPSED(lastCalUpdate, 50)) {
        lastCalUpdate = millis();
        calibrationUpdatePosition(syncGetRPM(), sensorData.loadMgStroke);
        calibrationUpdateMaf(sensorData.mafFrequencyHz);
    }
    
    // Idle control 50Hz
    if (TIME_ELAPSED(lastIdle, 20)) {
        lastIdle = millis();
        
        uint16_t rpm = syncGetRPM();
        float baseIgn = getIgnition(rpm, sensorData.loadMgStroke);
        idleUpdate(rpm, sensorData.tpsPercent, sensorData.cltCelsius, baseIgn);
        
        // Engine start/stop detection
        if (rpm > RPM_CRANKING_THRESHOLD && !wasRunning) {
            wasRunning = true;
            engineStartTime = millis();
            cyclesSinceStart = 0;
            idleNotifyEngineStart();
            digitalWrite(PIN_FUEL_PUMP, HIGH);
            Serial.println(F("Engine Started!"));
        } else if (rpm < 100 && wasRunning) {
            wasRunning = false;
            idleNotifyEngineStop();
            lambdaReset();
            tablesResetState();
            digitalWrite(PIN_FUEL_PUMP, LOW);
            Serial.println(F("Engine Stopped"));
        }
        
        // Cycle counter
        if (wasRunning && syncStatus.revolutionComplete) {
            cyclesSinceStart++;
            syncStatus.revolutionComplete = false;
        }
    }
    
    // Lambda control 20Hz
    if (TIME_ELAPSED(lastLambda, 50)) {
        lastLambda = millis();
        float lambdaValue = readLambdaSensor();  // Returns lambda (1.0 = stoich)
        uint16_t rpm = syncGetRPM();
        
        // Use lambdaUpdateDirect for CAN input (receives lambda directly)
        lambdaUpdateDirect(lambdaValue,
                           rpm, 
                           sensorData.tpsPercent,
                           sensorData.cltCelsius, 
                           sensorData.loadMgStroke);
        
        // Autotune learning (10Hz, runs every other lambda update)
        static bool autotuneToggle = false;
        autotuneToggle = !autotuneToggle;
        if (autotuneToggle) {
            float targetAfr = lambdaGetTargetAfr(rpm, sensorData.loadMgStroke);
            autotuneUpdate(
                rpm,
                sensorData.loadMgStroke,
                sensorData.mafFrequencyHz,
                sensorData.tpsPercent,
                sensorData.cltCelsius,
                sensorData.lambda,
                targetAfr / 14.7f,      // Convert AFR to lambda
                lambdaGetCorrection()   // STFT from PID
            );
        }
    }
    
    // TLE8888 watchdog 100Hz
    if (TIME_ELAPSED(lastTle, 10)) {
        lastTle = millis();
        tle8888.serviceWatchdog();
        if (tle8888.hasCriticalFault()) {
            doEmergencyShutdown("TLE FAULT");
        }
    }
    
    // Status LED
    if (TIME_ELAPSED(lastBlink, 500)) {
        lastBlink = millis();
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
    }
    digitalWrite(PIN_LED_SYNC, syncIsValid());
}
