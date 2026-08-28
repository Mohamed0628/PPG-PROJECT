#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>




#define POWER_WAIT_TIMEOUT_MS        1000
#define ANALOG_SETTLE_TIMEOUT_MS     2000
#define SERVO_INIT_TIMEOUT_MS        2000
#define LED_ENABLE_TIMEOUT_MS        1000
#define FAST_ACQUIRE_TIMEOUT_MS     30000
#define SETTLING_TIMEOUT_MS         30000
#define RECOVERY_TIMEOUT_MS         10000


// amount of recovery attempts allowed before giving up on the current run
#define MAX_RECOVERY_ATTEMPTS           3

// how long RUN must remain healthy before old recovery attempts are forgotten
#define RECOVERY_RESET_RUN_MS       10000


// persistence counters
// placeholder values until supervisor tick rate is finalized
#define RECOVERABLE_PERSIST_TICKS        5
#define UNRECOVERABLE_PERSIST_TICKS      3
#define RECOVERY_FAILED_PERSIST_TICKS    3




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

    // servo reports that the baseline has converged
    // supervisor then decides when the system can enter RUN
    bool servo_converged;

    bool agc_step_occurred;

    bool recoverable_condition;
    bool unrecoverable_fault;

    bool recovery_succeeded;
    bool recovery_failed;

    uint32_t now_ms; // timer

} supervisor_inputs_t;


// essentially this is the information being reported to the state machine
// the supervisor looks at these conditions and determines whether the system
// should remain in its current state or move into another state
//
// analog_reference_stable for instance means that the analog reference
// has become stable enough for the next stage of startup
//
// startup readiness signals such as power_good, analog_reference_stable,
// and led_confirmed_on establish startup dependencies
// if one of those critical dependencies is lost after startup, the lower-level
// health monitor must report it through recoverable_condition or
// unrecoverable_fault so the supervisor can react




static supervisor_state_t current_state = POWER_OFF;

static supervisor_fault_t current_fault = FAULT_NONE;

static uint32_t state_entry_ms = 0;


// tracks how many recovery cycles have happened during the current bad period
static uint8_t recovery_attempts = 0;


// persistence counters stop one noisy tick from causing a major state change
static uint16_t recoverable_counter = 0;
static uint16_t unrecoverable_counter = 0;
static uint16_t recovery_failed_counter = 0;


// prevents an old servo_converged = true from a previous operating point
// from immediately skipping a new FAST_ACQUIRE or SETTLING period
static bool servo_convergence_armed = false;




static void enter_state(supervisor_state_t new_state, uint32_t now_ms)
{
    current_state = new_state;
    state_entry_ms = now_ms;

    if ((new_state == FAST_ACQUIRE) || (new_state == SETTLING))
    {
        servo_convergence_armed = false;
    }
}

// state tracking updater




static void enter_fault(supervisor_fault_t fault, uint32_t now_ms)
{
    current_fault = fault;
    current_state = FAULT;
    state_entry_ms = now_ms;
}

// fault tracking updater




static bool persistence_counter_update(
    bool condition,
    uint16_t *counter,
    uint16_t threshold)
{
    if (condition)
    {
        if (*counter < threshold)
        {
            (*counter)++;
        }
    }
    else
    {
        *counter = 0;
    }

    return (*counter >= threshold);
}

// persistence helper
// if something abnormal happens for only one tick we ignore it
// the condition has to remain true for multiple ticks before the
// supervisor acts on it




static void reset_persistence_counters(void)
{
    recoverable_counter = 0;
    unrecoverable_counter = 0;
    recovery_failed_counter = 0;
}

// clears temporary condition counters




static bool servo_new_convergence_detected(bool servo_converged)
{
    if (!servo_convergence_armed)
    {
        if (!servo_converged)
        {
            servo_convergence_armed = true;
        }

        return false;
    }

    return servo_converged;
}

// requires the servo to first report not converged after a new acquisition
// period begins, then report converged again before RUN is allowed
// this prevents a stale true value from the previous operating point
// from immediately skipping FAST_ACQUIRE or SETTLING




void supervisor_reset(uint32_t now_ms)
{
    current_state = POWER_OFF;
    current_fault = FAULT_NONE;
    state_entry_ms = now_ms;

    recovery_attempts = 0;
    servo_convergence_armed = false;

    reset_persistence_counters();
}

// resets the supervisor back to the beginning
// clears the current fault and starts a new run




supervisor_state_t supervisor_get_state(void)
{
    return current_state;
}

// returns the current system state




supervisor_fault_t supervisor_get_fault(void)
{
    return current_fault;
}

// returns the current fault for debugging




uint8_t supervisor_get_recovery_attempts(void)
{
    return recovery_attempts;
}

// returns how many recovery attempts have happened
// useful later for debugging and diagnostics




bool supervisor_led_enabled(void)
{
    switch (current_state)
    {
        case LED_ENABLE:
        case FAST_ACQUIRE:
        case RUN:
        case SETTLING:

            return true;


        case POWER_OFF:
        case POWER_WAIT:
        case ANALOG_SETTLE:
        case SERVO_INIT:
        case RECOVERY:
        case FAULT:
        default:

            return false;
    }
}

// tells the LED driver whether illumination should be enabled
//
// important:
// we do not use something like:
//
// current_state >= LED_ENABLE
//
// because RECOVERY and FAULT appear later numerically but require
// the LED to be turned off




bool supervisor_nlms_adapt_enabled(void)
{
    return (current_state == RUN);
}

// NLMS is only allowed to change its adaptive coefficients during RUN
//
// during fast servo movement or settling the NLMS coefficients are frozen
// so the adaptive filter does not learn servo transients as motion




bool supervisor_ppg_valid(void)
{
    return (current_state == RUN);
}

// tells the physiological processing path whether the PPG data
// can currently be trusted
//
// SETTLING is intentionally false because the ADC may still be running
// but the optical / servo operating point is moving and therefore the
// samples should not yet be treated as valid physiological PPG




bool supervisor_hr_valid(void)
{
    return (current_state == RUN);
}

// heart rate can only be reported while the system is fully acquired
// and operating normally




bool supervisor_ppg_acquisition_active(void)
{
    switch (current_state)
    {
        case FAST_ACQUIRE:
        case RUN:
        case SETTLING:

            return true;


        default:

            return false;
    }
}

// this is different from supervisor_ppg_valid()
//
// acquisition can continue during FAST_ACQUIRE and SETTLING so that
// the servo and signal processing can observe what is happening
//
// however the physiological data is only valid during RUN




bool supervisor_servo_fast_mode(void)
{
    switch (current_state)
    {
        case FAST_ACQUIRE:
        case SETTLING:
        case RECOVERY:

            return true;


        default:

            return false;
    }
}

// tells the servo whether it should use the fast acquisition behavior
//
// RUN uses the normal slow tracking behavior




// supervisor tick basically is the main monitor
// monitors the state of the state machine and moves it onto either
// the next state in line, refer to state structure for states,
// or moves into the fault protocol and shows the fault
void supervisor_tick(const supervisor_inputs_t *inputs)
{
    if (inputs == NULL)
    {
        return;
    }


    // unrecoverable faults matter throughout the operating sequence
    // so this counter is allowed to persist across state transitions
    bool persistent_unrecoverable =
        persistence_counter_update(
            inputs->unrecoverable_fault,
            &unrecoverable_counter,
            UNRECOVERABLE_PERSIST_TICKS);


    // recoverable conditions only have meaning while the system is in RUN
    bool persistent_recoverable = false;

    if (current_state == RUN)
    {
        persistent_recoverable =
            persistence_counter_update(
                inputs->recoverable_condition,
                &recoverable_counter,
                RECOVERABLE_PERSIST_TICKS);
    }
    else
    {
        recoverable_counter = 0;
    }


    // recovery failure only has meaning while actually attempting recovery
    bool persistent_recovery_failure = false;

    if (current_state == RECOVERY)
    {
        persistent_recovery_failure =
            persistence_counter_update(
                inputs->recovery_failed,
                &recovery_failed_counter,
                RECOVERY_FAILED_PERSIST_TICKS);
    }
    else
    {
        recovery_failed_counter = 0;
    }




    switch (current_state)
    {
        case POWER_OFF:

            if (inputs->power_requested)
            {
                enter_state(POWER_WAIT, inputs->now_ms);
            }

            break;




        case POWER_WAIT:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (inputs->power_good)
            {
                enter_state(ANALOG_SETTLE, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= POWER_WAIT_TIMEOUT_MS)
            {
                enter_fault(FAULT_POWER_TIMEOUT, inputs->now_ms);
            }

            break;




        case ANALOG_SETTLE:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (inputs->analog_reference_stable)
            {
                enter_state(SERVO_INIT, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= ANALOG_SETTLE_TIMEOUT_MS)
            {
                enter_fault(FAULT_ANALOG_SETTLE_TIMEOUT, inputs->now_ms);
            }

            break;




        case SERVO_INIT:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (inputs->cancellation_neutral)
            {
                enter_state(LED_ENABLE, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= SERVO_INIT_TIMEOUT_MS)
            {
                enter_fault(FAULT_SERVO_INIT_TIMEOUT, inputs->now_ms);
            }

            break;




        case LED_ENABLE:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (inputs->led_confirmed_on)
            {
                enter_state(FAST_ACQUIRE, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= LED_ENABLE_TIMEOUT_MS)
            {
                enter_fault(FAULT_LED_ENABLE_TIMEOUT, inputs->now_ms);
            }

            break;




        case FAST_ACQUIRE:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (servo_new_convergence_detected(inputs->servo_converged))
            {
                enter_state(RUN, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= FAST_ACQUIRE_TIMEOUT_MS)
            {
                enter_fault(FAULT_FAST_ACQUIRE_TIMEOUT, inputs->now_ms);
            }

            break;




        case RUN:

            // independent housekeeping
            // if we stay healthy in RUN long enough then previous
            // recovery attempts are forgotten
            if ((inputs->now_ms - state_entry_ms) >= RECOVERY_RESET_RUN_MS)
            {
                recovery_attempts = 0;
            }


            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }

            else if (persistent_recoverable)
            {
                if (recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    enter_fault(
                        FAULT_TOO_MANY_RECOVERY_ATTEMPTS,
                        inputs->now_ms);
                }
                else
                {
                    recovery_attempts++;

                    recoverable_counter = 0;
                    recovery_failed_counter = 0;

                    enter_state(RECOVERY, inputs->now_ms);
                }
            }

            else if (inputs->agc_step_occurred)
            {
                enter_state(SETTLING, inputs->now_ms);
            }

            break;




        case SETTLING:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }
            else if (servo_new_convergence_detected(inputs->servo_converged))
            {
                enter_state(RUN, inputs->now_ms);
            }
            else if ((inputs->now_ms - state_entry_ms) >= SETTLING_TIMEOUT_MS)
            {
                enter_fault(FAULT_SETTLING_TIMEOUT, inputs->now_ms);
            }

            break;




        case RECOVERY:

            if (persistent_unrecoverable)
            {
                enter_fault(FAULT_UNRECOVERABLE, inputs->now_ms);
            }

            else if (persistent_recovery_failure)
            {
                enter_fault(FAULT_RECOVERY_FAILED, inputs->now_ms);
            }

            else if (inputs->recovery_succeeded)
            {
                recoverable_counter = 0;
                recovery_failed_counter = 0;

                // recovery deliberately destroyed the old operating point
                // so we must rebuild the startup dependencies instead of
                // jumping straight back into RUN
                enter_state(SERVO_INIT, inputs->now_ms);
            }

            else if ((inputs->now_ms - state_entry_ms) >= RECOVERY_TIMEOUT_MS)
            {
                enter_fault(FAULT_RECOVERY_TIMEOUT, inputs->now_ms);
            }

            break;




        case FAULT:

            // fault is the end of the road for the current run
            // supervisor remains here until reset or system reboot

            break;




        default:

            enter_fault(FAULT_INVALID_STATE, inputs->now_ms);

            break;
    }
}