/*
 * scheduler.cpp - Hardware Output Compare Scheduler Implementation
 * ECU STM32F405 v8.2
 * 
 * Uses STM32duino HardwareTimer API for precise timing.
 * 
 * Timer Assignment:
 *   TIM2 (32-bit): IGN1-IGN4 scheduling
 *   TIM3 (16-bit): INJ1-INJ4 scheduling
 * 
 * Prescaler: 84 → 1µs per tick @ 84MHz APB1
 */

#include "scheduler.h"
#include "tle8888.h"
#include "tables.h"
#include <HardwareTimer.h>

// ============================================================================
// HARDWARE TIMER INSTANCES
// ============================================================================

HardwareTimer *ignTimer = NULL;
HardwareTimer *injTimer = NULL;

// ============================================================================
// SCHEDULE INSTANCES
// ============================================================================

FuelSchedule fuelSchedule1;
FuelSchedule fuelSchedule2;
FuelSchedule fuelSchedule3;
FuelSchedule fuelSchedule4;

IgnitionSchedule ignitionSchedule1;
IgnitionSchedule ignitionSchedule2;
IgnitionSchedule ignitionSchedule3;
IgnitionSchedule ignitionSchedule4;

// ============================================================================
// INTERNAL STATE
// ============================================================================

static volatile bool schedulerInitialized = false;

// ============================================================================
// FUEL ISR HANDLERS
// ============================================================================

void fuelStartISR1(void) {
    if (fuelSchedule1.status == SCHED_PENDING) {
        tle8888.setInjector(fuelSchedule1.channel, true);
        fuelSchedule1.status = SCHED_RUNNING;
        fuelSchedule1.actualStartTime = micros();
        
        // Set end compare
        injTimer->setCaptureCompare(1, fuelSchedule1.endCompare, TICK_COMPARE_FORMAT);
    } else if (fuelSchedule1.status == SCHED_RUNNING) {
        tle8888.setInjector(fuelSchedule1.channel, false);
        fuelSchedule1.actualEndTime = micros();
        
        if (fuelSchedule1.hasNextSchedule) {
            fuelSchedule1.startCompare = fuelSchedule1.nextStartCompare;
            fuelSchedule1.endCompare = fuelSchedule1.nextEndCompare;
            fuelSchedule1.hasNextSchedule = false;
            fuelSchedule1.status = SCHED_PENDING;
            injTimer->setCaptureCompare(1, fuelSchedule1.startCompare, TICK_COMPARE_FORMAT);
        } else {
            fuelSchedule1.status = SCHED_OFF;
        }
    }
}

void fuelStartISR2(void) {
    if (fuelSchedule2.status == SCHED_PENDING) {
        tle8888.setInjector(fuelSchedule2.channel, true);
        fuelSchedule2.status = SCHED_RUNNING;
        fuelSchedule2.actualStartTime = micros();
        injTimer->setCaptureCompare(2, fuelSchedule2.endCompare, TICK_COMPARE_FORMAT);
    } else if (fuelSchedule2.status == SCHED_RUNNING) {
        tle8888.setInjector(fuelSchedule2.channel, false);
        fuelSchedule2.actualEndTime = micros();
        
        if (fuelSchedule2.hasNextSchedule) {
            fuelSchedule2.startCompare = fuelSchedule2.nextStartCompare;
            fuelSchedule2.endCompare = fuelSchedule2.nextEndCompare;
            fuelSchedule2.hasNextSchedule = false;
            fuelSchedule2.status = SCHED_PENDING;
            injTimer->setCaptureCompare(2, fuelSchedule2.startCompare, TICK_COMPARE_FORMAT);
        } else {
            fuelSchedule2.status = SCHED_OFF;
        }
    }
}

void fuelStartISR3(void) {
    if (fuelSchedule3.status == SCHED_PENDING) {
        tle8888.setInjector(fuelSchedule3.channel, true);
        fuelSchedule3.status = SCHED_RUNNING;
        fuelSchedule3.actualStartTime = micros();
        injTimer->setCaptureCompare(3, fuelSchedule3.endCompare, TICK_COMPARE_FORMAT);
    } else if (fuelSchedule3.status == SCHED_RUNNING) {
        tle8888.setInjector(fuelSchedule3.channel, false);
        fuelSchedule3.actualEndTime = micros();
        
        if (fuelSchedule3.hasNextSchedule) {
            fuelSchedule3.startCompare = fuelSchedule3.nextStartCompare;
            fuelSchedule3.endCompare = fuelSchedule3.nextEndCompare;
            fuelSchedule3.hasNextSchedule = false;
            fuelSchedule3.status = SCHED_PENDING;
            injTimer->setCaptureCompare(3, fuelSchedule3.startCompare, TICK_COMPARE_FORMAT);
        } else {
            fuelSchedule3.status = SCHED_OFF;
        }
    }
}

void fuelStartISR4(void) {
    if (fuelSchedule4.status == SCHED_PENDING) {
        tle8888.setInjector(fuelSchedule4.channel, true);
        fuelSchedule4.status = SCHED_RUNNING;
        fuelSchedule4.actualStartTime = micros();
        injTimer->setCaptureCompare(4, fuelSchedule4.endCompare, TICK_COMPARE_FORMAT);
    } else if (fuelSchedule4.status == SCHED_RUNNING) {
        tle8888.setInjector(fuelSchedule4.channel, false);
        fuelSchedule4.actualEndTime = micros();
        
        if (fuelSchedule4.hasNextSchedule) {
            fuelSchedule4.startCompare = fuelSchedule4.nextStartCompare;
            fuelSchedule4.endCompare = fuelSchedule4.nextEndCompare;
            fuelSchedule4.hasNextSchedule = false;
            fuelSchedule4.status = SCHED_PENDING;
            injTimer->setCaptureCompare(4, fuelSchedule4.startCompare, TICK_COMPARE_FORMAT);
        } else {
            fuelSchedule4.status = SCHED_OFF;
        }
    }
}

// ============================================================================
// IGNITION ISR HANDLERS
// ============================================================================

void ignStartISR1(void) {
    if (ignitionSchedule1.status == SCHED_PENDING) {
        tle8888.setIgnition(ignitionSchedule1.channel, true);
        ignitionSchedule1.status = SCHED_RUNNING;
        ignitionSchedule1.dwellStartTime = micros();
        ignTimer->setCaptureCompare(1, ignitionSchedule1.endCompare, TICK_COMPARE_FORMAT);
    } else if (ignitionSchedule1.status == SCHED_RUNNING) {
        tle8888.setIgnition(ignitionSchedule1.channel, false);
        ignitionSchedule1.actualDwell = micros() - ignitionSchedule1.dwellStartTime;
        
        if (ignitionSchedule1.hasNextSchedule) {
            ignitionSchedule1.startCompare = ignitionSchedule1.nextStartCompare;
            ignitionSchedule1.endCompare = ignitionSchedule1.nextEndCompare;
            ignitionSchedule1.hasNextSchedule = false;
            ignitionSchedule1.status = SCHED_PENDING;
            ignTimer->setCaptureCompare(1, ignitionSchedule1.startCompare, TICK_COMPARE_FORMAT);
        } else {
            ignitionSchedule1.status = SCHED_OFF;
        }
    }
}

void ignStartISR2(void) {
    if (ignitionSchedule2.status == SCHED_PENDING) {
        tle8888.setIgnition(ignitionSchedule2.channel, true);
        ignitionSchedule2.status = SCHED_RUNNING;
        ignitionSchedule2.dwellStartTime = micros();
        ignTimer->setCaptureCompare(2, ignitionSchedule2.endCompare, TICK_COMPARE_FORMAT);
    } else if (ignitionSchedule2.status == SCHED_RUNNING) {
        tle8888.setIgnition(ignitionSchedule2.channel, false);
        ignitionSchedule2.actualDwell = micros() - ignitionSchedule2.dwellStartTime;
        
        if (ignitionSchedule2.hasNextSchedule) {
            ignitionSchedule2.startCompare = ignitionSchedule2.nextStartCompare;
            ignitionSchedule2.endCompare = ignitionSchedule2.nextEndCompare;
            ignitionSchedule2.hasNextSchedule = false;
            ignitionSchedule2.status = SCHED_PENDING;
            ignTimer->setCaptureCompare(2, ignitionSchedule2.startCompare, TICK_COMPARE_FORMAT);
        } else {
            ignitionSchedule2.status = SCHED_OFF;
        }
    }
}

void ignStartISR3(void) {
    if (ignitionSchedule3.status == SCHED_PENDING) {
        tle8888.setIgnition(ignitionSchedule3.channel, true);
        ignitionSchedule3.status = SCHED_RUNNING;
        ignitionSchedule3.dwellStartTime = micros();
        ignTimer->setCaptureCompare(3, ignitionSchedule3.endCompare, TICK_COMPARE_FORMAT);
    } else if (ignitionSchedule3.status == SCHED_RUNNING) {
        tle8888.setIgnition(ignitionSchedule3.channel, false);
        ignitionSchedule3.actualDwell = micros() - ignitionSchedule3.dwellStartTime;
        
        if (ignitionSchedule3.hasNextSchedule) {
            ignitionSchedule3.startCompare = ignitionSchedule3.nextStartCompare;
            ignitionSchedule3.endCompare = ignitionSchedule3.nextEndCompare;
            ignitionSchedule3.hasNextSchedule = false;
            ignitionSchedule3.status = SCHED_PENDING;
            ignTimer->setCaptureCompare(3, ignitionSchedule3.startCompare, TICK_COMPARE_FORMAT);
        } else {
            ignitionSchedule3.status = SCHED_OFF;
        }
    }
}

void ignStartISR4(void) {
    if (ignitionSchedule4.status == SCHED_PENDING) {
        tle8888.setIgnition(ignitionSchedule4.channel, true);
        ignitionSchedule4.status = SCHED_RUNNING;
        ignitionSchedule4.dwellStartTime = micros();
        ignTimer->setCaptureCompare(4, ignitionSchedule4.endCompare, TICK_COMPARE_FORMAT);
    } else if (ignitionSchedule4.status == SCHED_RUNNING) {
        tle8888.setIgnition(ignitionSchedule4.channel, false);
        ignitionSchedule4.actualDwell = micros() - ignitionSchedule4.dwellStartTime;
        
        if (ignitionSchedule4.hasNextSchedule) {
            ignitionSchedule4.startCompare = ignitionSchedule4.nextStartCompare;
            ignitionSchedule4.endCompare = ignitionSchedule4.nextEndCompare;
            ignitionSchedule4.hasNextSchedule = false;
            ignitionSchedule4.status = SCHED_PENDING;
            ignTimer->setCaptureCompare(4, ignitionSchedule4.startCompare, TICK_COMPARE_FORMAT);
        } else {
            ignitionSchedule4.status = SCHED_OFF;
        }
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void schedulerInit(void) {
    // Initialize schedule structures
    memset(&fuelSchedule1, 0, sizeof(FuelSchedule));
    memset(&fuelSchedule2, 0, sizeof(FuelSchedule));
    memset(&fuelSchedule3, 0, sizeof(FuelSchedule));
    memset(&fuelSchedule4, 0, sizeof(FuelSchedule));
    
    memset(&ignitionSchedule1, 0, sizeof(IgnitionSchedule));
    memset(&ignitionSchedule2, 0, sizeof(IgnitionSchedule));
    memset(&ignitionSchedule3, 0, sizeof(IgnitionSchedule));
    memset(&ignitionSchedule4, 0, sizeof(IgnitionSchedule));
    
    // Assign TLE8888 channels
    fuelSchedule1.channel = 0;  // INJ1
    fuelSchedule2.channel = 1;  // INJ2
    fuelSchedule3.channel = 2;  // INJ3
    fuelSchedule4.channel = 3;  // INJ4
    
    ignitionSchedule1.channel = 0;  // IGN1
    ignitionSchedule2.channel = 1;  // IGN2
    ignitionSchedule3.channel = 2;  // IGN3
    ignitionSchedule4.channel = 3;  // IGN4
    
    // Create timer instances
    ignTimer = new HardwareTimer(TIM2);
    injTimer = new HardwareTimer(TIM3);
    
    // Configure ignition timer (TIM2 - 32-bit)
    ignTimer->setPrescaleFactor(TIMER_PRESCALER);
    ignTimer->setOverflow(0xFFFFFFFF, TICK_FORMAT);
    
    ignTimer->attachInterrupt(1, ignStartISR1);
    ignTimer->attachInterrupt(2, ignStartISR2);
    ignTimer->attachInterrupt(3, ignStartISR3);
    ignTimer->attachInterrupt(4, ignStartISR4);
    
    // Configure injection timer (TIM3 - 16-bit)
    injTimer->setPrescaleFactor(TIMER_PRESCALER);
    injTimer->setOverflow(0xFFFF, TICK_FORMAT);
    
    injTimer->attachInterrupt(1, fuelStartISR1);
    injTimer->attachInterrupt(2, fuelStartISR2);
    injTimer->attachInterrupt(3, fuelStartISR3);
    injTimer->attachInterrupt(4, fuelStartISR4);
    
    // Start timers
    ignTimer->resume();
    injTimer->resume();
    
    schedulerInitialized = true;
}

// ============================================================================
// FUEL SCHEDULING FUNCTIONS
// ============================================================================

static void setFuelSchedule(FuelSchedule &sched, uint8_t channel, uint32_t timeout, uint32_t duration) {
    if (!schedulerInitialized) return;
    if (timeout < MIN_SCHEDULE_US) timeout = MIN_SCHEDULE_US;
    if (duration < MIN_SCHEDULE_US) return;
    if (timeout > MAX_INJ_TIMEOUT) timeout = MAX_INJ_TIMEOUT;
    
    uint16_t currentTick = injTimer->getCount();
    uint16_t startTick = currentTick + (uint16_t)timeout;
    uint16_t endTick = startTick + (uint16_t)duration;
    
    noInterrupts();
    
    if (sched.status == SCHED_RUNNING) {
        // Queue for next cycle
        sched.nextStartCompare = startTick;
        sched.nextEndCompare = endTick;
        sched.hasNextSchedule = true;
    } else {
        sched.startCompare = startTick;
        sched.endCompare = endTick;
        sched.duration = duration;
        sched.status = SCHED_PENDING;
        sched.hasNextSchedule = false;
        
        injTimer->setCaptureCompare(channel, startTick, TICK_COMPARE_FORMAT);
    }
    
    interrupts();
}

void setFuelSchedule1(uint32_t timeout, uint32_t duration) {
    setFuelSchedule(fuelSchedule1, 1, timeout, duration);
}

void setFuelSchedule2(uint32_t timeout, uint32_t duration) {
    setFuelSchedule(fuelSchedule2, 2, timeout, duration);
}

void setFuelSchedule3(uint32_t timeout, uint32_t duration) {
    setFuelSchedule(fuelSchedule3, 3, timeout, duration);
}

void setFuelSchedule4(uint32_t timeout, uint32_t duration) {
    setFuelSchedule(fuelSchedule4, 4, timeout, duration);
}

// ============================================================================
// IGNITION SCHEDULING FUNCTIONS
// ============================================================================

static void setIgnitionSchedule(IgnitionSchedule &sched, uint8_t channel, uint32_t timeout, uint32_t dwell) {
    if (!schedulerInitialized) return;
    if (timeout < MIN_SCHEDULE_US) timeout = MIN_SCHEDULE_US;
    if (dwell < MIN_SCHEDULE_US) return;
    if (dwell > MAX_DWELL_TIME_US) dwell = MAX_DWELL_TIME_US;
    
    uint32_t currentTick = ignTimer->getCount();
    uint32_t startTick = currentTick + timeout;
    uint32_t endTick = startTick + dwell;
    
    noInterrupts();
    
    if (sched.status == SCHED_RUNNING) {
        // Queue for next cycle
        sched.nextStartCompare = startTick;
        sched.nextEndCompare = endTick;
        sched.hasNextSchedule = true;
    } else {
        sched.startCompare = startTick;
        sched.endCompare = endTick;
        sched.scheduledDwell = dwell;
        sched.status = SCHED_PENDING;
        sched.hasNextSchedule = false;
        
        ignTimer->setCaptureCompare(channel, startTick, TICK_COMPARE_FORMAT);
    }
    
    interrupts();
}

void setIgnitionSchedule1(uint32_t timeout, uint32_t dwell) {
    setIgnitionSchedule(ignitionSchedule1, 1, timeout, dwell);
}

void setIgnitionSchedule2(uint32_t timeout, uint32_t dwell) {
    setIgnitionSchedule(ignitionSchedule2, 2, timeout, dwell);
}

void setIgnitionSchedule3(uint32_t timeout, uint32_t dwell) {
    setIgnitionSchedule(ignitionSchedule3, 3, timeout, dwell);
}

void setIgnitionSchedule4(uint32_t timeout, uint32_t dwell) {
    setIgnitionSchedule(ignitionSchedule4, 4, timeout, dwell);
}

// ============================================================================
// OVERDWELL PROTECTION
// ============================================================================

static void checkIgnitionOverdwell(IgnitionSchedule &sched, uint8_t channel) {
    if (sched.status == SCHED_RUNNING) {
        uint32_t dwellDuration = micros() - sched.dwellStartTime;
        if (dwellDuration > MAX_DWELL_TIME_US) {
            tle8888.setIgnition(channel, false);
            sched.status = SCHED_OFF;
            sched.hasNextSchedule = false;
        }
    }
}

void checkOverdwell(void) {
    checkIgnitionOverdwell(ignitionSchedule1, ignitionSchedule1.channel);
    checkIgnitionOverdwell(ignitionSchedule2, ignitionSchedule2.channel);
    checkIgnitionOverdwell(ignitionSchedule3, ignitionSchedule3.channel);
    checkIgnitionOverdwell(ignitionSchedule4, ignitionSchedule4.channel);
}

// ============================================================================
// EMERGENCY DISABLE
// ============================================================================

void disableAllSchedules(void) {
    noInterrupts();
    
    // Disable fuel schedules
    fuelSchedule1.status = SCHED_OFF;
    fuelSchedule2.status = SCHED_OFF;
    fuelSchedule3.status = SCHED_OFF;
    fuelSchedule4.status = SCHED_OFF;
    
    fuelSchedule1.hasNextSchedule = false;
    fuelSchedule2.hasNextSchedule = false;
    fuelSchedule3.hasNextSchedule = false;
    fuelSchedule4.hasNextSchedule = false;
    
    // Disable ignition schedules
    ignitionSchedule1.status = SCHED_OFF;
    ignitionSchedule2.status = SCHED_OFF;
    ignitionSchedule3.status = SCHED_OFF;
    ignitionSchedule4.status = SCHED_OFF;
    
    ignitionSchedule1.hasNextSchedule = false;
    ignitionSchedule2.hasNextSchedule = false;
    ignitionSchedule3.hasNextSchedule = false;
    ignitionSchedule4.hasNextSchedule = false;
    
    // Turn off all outputs
    tle8888.setAllOutputs(0);
    tle8888.setAllIgnition(0);
    
    interrupts();
}
