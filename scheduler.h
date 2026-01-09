/*
 * scheduler.h - Hardware Output Compare Scheduler
 * ECU STM32F405 v8.2
 * 
 * Uses STM32duino HardwareTimer API for timer management.
 * 
 * Timer Assignment (STM32F405 @ 168MHz, APB1 @ 84MHz):
 *   TIM2 (32-bit): IGN1-IGN4 (CCR1-CCR4) - Ignition timing
 *   TIM3 (16-bit): INJ1-INJ4 (CCR1-CCR4) - Fuel injection
 * 
 * Prescaler: 84 → 1µs per tick
 * Ignition precision: <1µs jitter
 * Injection precision: <1µs jitter
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>

// ============================================================================
// TIMER CONFIGURATION
// ============================================================================

#define TIMER_CLOCK_FREQ    84000000UL  // APB1 = 84MHz
#define TIMER_PRESCALER     84          // 1µs per tick
#define TIMER_TICK_US       1

// Timer assignments
#define IGN_TIMER           TIM2        // 32-bit for ignition
#define INJ_TIMER           TIM3        // 16-bit for injection

// Compare register types
typedef uint32_t ign_compare_t;         // TIM2 is 32-bit
typedef uint16_t inj_compare_t;         // TIM3 is 16-bit

// Maximum timeout values (µs)
#define MAX_INJ_TIMEOUT     65000UL     // ~65ms for 16-bit (A3: accounts for overflow)
#define MAX_IGN_TIMEOUT     4000000UL   // 4 seconds for 32-bit

// Scheduling limits
#define MIN_SCHEDULE_US     4           // Minimum schedulable time
#define MAX_DWELL_TIME_US   10000       // Maximum dwell (overdwell protection)

// ============================================================================
// SCHEDULE STATUS (M8 FIX: Removed unused SCHED_STAGING)
// ============================================================================

typedef enum {
    SCHED_OFF = 0,            // Schedule disabled
    SCHED_PENDING,            // Waiting to start
    SCHED_RUNNING             // Currently active (output ON)
} ScheduleStatus;

// ============================================================================
// FUEL SCHEDULE STRUCTURE (M4 FIX: Removed unused callbacks)
// ============================================================================

typedef struct {
    volatile ScheduleStatus status;
    
    volatile inj_compare_t startCompare;    // When to open injector
    volatile inj_compare_t endCompare;      // When to close injector
    volatile uint32_t duration;             // Pulse width in µs
    
    // Queue for next pulse while current is running
    inj_compare_t nextStartCompare;
    inj_compare_t nextEndCompare;
    volatile bool hasNextSchedule;
    
    // Timestamps for diagnostics
    volatile uint32_t actualStartTime;
    volatile uint32_t actualEndTime;
    
    // Output channel (TLE8888)
    uint8_t channel;
    
} FuelSchedule;

// ============================================================================
// IGNITION SCHEDULE STRUCTURE (M4 FIX: Removed unused callbacks)
// ============================================================================

typedef struct {
    volatile ScheduleStatus status;
    
    volatile ign_compare_t startCompare;    // Dwell start (coil charge)
    volatile ign_compare_t endCompare;      // Spark fire (coil release)
    
    // Queue
    ign_compare_t nextStartCompare;
    ign_compare_t nextEndCompare;
    volatile bool hasNextSchedule;
    
    // Overdwell protection
    volatile uint32_t dwellStartTime;       // micros() when dwell started
    
    // Diagnostics
    volatile uint32_t actualDwell;          // Measured dwell time
    volatile uint32_t scheduledDwell;
    
    // Output channel (TLE8888)
    uint8_t channel;
    
} IgnitionSchedule;

// ============================================================================
// SCHEDULE INSTANCES
// ============================================================================

extern FuelSchedule fuelSchedule1;
extern FuelSchedule fuelSchedule2;
extern FuelSchedule fuelSchedule3;
extern FuelSchedule fuelSchedule4;

extern IgnitionSchedule ignitionSchedule1;
extern IgnitionSchedule ignitionSchedule2;
extern IgnitionSchedule ignitionSchedule3;
extern IgnitionSchedule ignitionSchedule4;

// ============================================================================
// SCHEDULER API
// ============================================================================

// Initialize timers and interrupts
void schedulerInit(void);

// Fuel: timeout (µs until start), duration (pulse width µs)
void setFuelSchedule1(uint32_t timeout, uint32_t duration);
void setFuelSchedule2(uint32_t timeout, uint32_t duration);
void setFuelSchedule3(uint32_t timeout, uint32_t duration);
void setFuelSchedule4(uint32_t timeout, uint32_t duration);

// Ignition: timeout (µs until dwell start), dwell (µs)
void setIgnitionSchedule1(uint32_t timeout, uint32_t dwell);
void setIgnitionSchedule2(uint32_t timeout, uint32_t dwell);
void setIgnitionSchedule3(uint32_t timeout, uint32_t dwell);
void setIgnitionSchedule4(uint32_t timeout, uint32_t dwell);

// Status checks
inline bool fuelScheduleIsRunning(const FuelSchedule &sched) {
    return sched.status == SCHED_RUNNING;
}
inline bool ignitionScheduleIsRunning(const IgnitionSchedule &sched) {
    return sched.status == SCHED_RUNNING;
}

// Overdwell protection (call from main loop)
void checkOverdwell(void);

// Disable all outputs (emergency)
void disableAllSchedules(void);

// ============================================================================
// A4 NOTE: Legacy timer macros removed
// The HardwareTimer API manages timer enable/disable internally.
// Direct register access macros were removed as they are incompatible
// with the STM32duino HardwareTimer approach used in scheduler.cpp.
// ============================================================================

#endif // SCHEDULER_H
