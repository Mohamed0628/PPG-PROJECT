#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    POWER_OFF,
    POWER_WAIT,
    ANALOG_SETTLE,
    SERVO_INIT,
    LED_ENABLE,
    FAST_ACQUIRE,
    RUN,
    SETTLING,
    RECOVERY,
    FAULT
} supervisor_state_t;

typedef enum
{
    FAULT_NONE,
    FAULT_POWER_TIMEOUT,
    FAULT_ANALOG_SETTLE_TIMEOUT,
    FAULT_SERVO_INIT_TIMEOUT,
    FAULT_LED_ENABLE_TIMEOUT,
    FAULT_FAST_ACQUIRE_TIMEOUT,
    FAULT_SETTLING_TIMEOUT,
    FAULT_RECOVERY_TIMEOUT,
    FAULT_TOO_MANY_RECOVERY_ATTEMPTS,
    FAULT_UNRECOVERABLE,
    FAULT_RECOVERY_FAILED,
    FAULT_INVALID_STATE
} supervisor_fault_t;

typedef struct
{
    bool power_requested;
    bool power_good;
    bool analog_reference_stable;
    bool cancellation_neutral;
    bool led_confirmed_on;
    bool servo_converged;
    bool agc_step_occurred;
    bool recoverable_condition;
    bool unrecoverable_fault;
    bool recovery_succeeded;
    bool recovery_failed;
    uint32_t now_ms;
} supervisor_inputs_t;

void supervisor_reset(uint32_t now_ms);
void supervisor_tick(const supervisor_inputs_t *inputs);

supervisor_state_t supervisor_get_state(void);
supervisor_fault_t supervisor_get_fault(void);
uint8_t supervisor_get_recovery_attempts(void);

bool supervisor_led_enabled(void);
bool supervisor_nlms_adapt_enabled(void);
bool supervisor_ppg_valid(void);
bool supervisor_hr_valid(void);
bool supervisor_ppg_acquisition_active(void);
bool supervisor_servo_fast_mode(void);

#endif
