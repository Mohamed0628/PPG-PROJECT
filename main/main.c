#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ads131m02.h"
#include "ppg_acq.h"
#include "servo.h"
#include "state_machine.h"
#include "driver/gpio.h"
#define POWER_GOOD_GPIO          GPIO_NUM_4
#define POWER_GOOD_ACTIVE_LEVEL  1

/* Supervisor loop period.  All persistence intervals below are expressed
 * in milliseconds and compared with rollover-safe unsigned subtraction,
 * so they stay correct if this period is retuned. */
#define SUPERVISOR_PERIOD_MS        10u

/* PWRGD must read valid continuously for this long before power_good is
 * reported true.  40 ms = 4 supervisor loops: long enough to ride out
 * comparator chatter as the rail crosses its threshold, short enough to
 * sit well inside the supervisor's 1000 ms POWER_WAIT timeout. */
#define PG_STARTUP_PERSIST_MS       40u

/* Once the system is past POWER_WAIT the analog rail is expected to be
 * valid.  PWRGD must read invalid continuously for this long before the
 * analog domain is declared untrustworthy.  50 ms ignores a single
 * transient without letting a genuinely collapsed rail keep feeding the
 * signal chain. */
#define PG_RUNTIME_LOSS_PERSIST_MS  50u

/* Provisional analog reference settling time.  TODO VALIDATION: replace
 * with a measured value once the analog board exists. */
#define ANALOG_SETTLE_MS            100u

/*
 * How long an acquisition error is presented to the supervisor as
 * recoverable_condition.
 *
 * state_machine.c requires recoverable_condition to be true for
 * RECOVERABLE_PERSIST_TICKS consecutive ticks (currently 5) before RUN
 * transitions to RECOVERY, and it zeroes the counter on any false tick.
 * A one-loop pulse would therefore never trigger recovery at all.
 *
 * The acquisition layer reports an error as a single event, so main.c
 * bridges event -> sustained condition here.  300 ms is 30 loops, six
 * times the required threshold, which leaves margin if the supervisor
 * period or the threshold is retuned.  The hold is released as soon as
 * the supervisor acts (state leaves RUN) so the condition never latches.
 */
#define ACQ_FAULT_HOLD_MS           300u

/* Minimum spacing between ppg_acq_start() attempts after a failure, so a
 * persistent start error cannot spin at the loop rate or spam the log. */
#define ACQ_START_RETRY_MS          500u

/* Consecutive start failures tolerated before the acquisition hardware is
 * treated as unusable rather than merely unlucky. */
#define ACQ_START_MAX_FAILURES      3u

static const char *TAG = "MAIN";

/* ------------------------------------------------------------------ */
/* Servo state mutex.                                                  */
/*                                                                     */
/* servo_process_sample() runs from raw_sample_cb() in the acquisition */
/* task; servo_init() runs from the supervisor loop during recovery.   */
/* Both touch the same servo state, so both take this mutex.           */
/*                                                                     */
/* ppg_acq_stop() must still be called before recovery attempts        */
/* servo_init(): stopping guarantees no NEW callback is created, and   */
/* the mutex then waits out any callback already in flight.  Neither   */
/* mechanism alone is sufficient.                                      */
/*                                                                     */
/* A mutex rather than a binary semaphore, for priority inheritance:   */
/* the acquisition task runs near the top of the priority range and    */
/* the supervisor loop is far below it.                                */
/* ------------------------------------------------------------------ */
static SemaphoreHandle_t servo_mux;

/* ------------------------------------------------------------------ */
/* Acquisition error handoff: acquisition task -> supervisor task.     */
/*                                                                     */
/* The callback runs in the ppg_acq task, not an ISR, but it is still  */
/* a different task from the supervisor loop that consumes the event.  */
/* A plain bool with a separate test and clear can lose an event if    */
/* preemption lands between the two, so both sides run under a         */
/* portMUX spinlock.  volatile alone would give visibility but not     */
/* atomicity; the critical section is what makes the read-and-clear    */
/* indivisible.  Both sections are a handful of instructions.          */
/* ------------------------------------------------------------------ */
static portMUX_TYPE      acq_err_mux = portMUX_INITIALIZER_UNLOCKED;
static bool              acq_err_pending;
static ppg_acq_error_t   acq_err_reason;

static void acq_error_cb(ppg_acq_error_t reason, void *ctx)
{
    (void)ctx;

    /* Nonblocking, no allocation, no SPI, no supervisor call, no
     * start/stop, no logging.  First error wins until consumed: the
     * acquisition layer has already halted itself, so any later reason
     * describes the same outage. */
    portENTER_CRITICAL(&acq_err_mux);

    if (!acq_err_pending) {
        acq_err_pending = true;
        acq_err_reason  = reason;
    }

    portEXIT_CRITICAL(&acq_err_mux);
}

/* Atomic take-and-clear.  Returns true exactly once per posted event. */
static bool acq_error_take(ppg_acq_error_t *reason)
{
    bool had;

    portENTER_CRITICAL(&acq_err_mux);

    had = acq_err_pending;

    if (had) {
        *reason = acq_err_reason;
        acq_err_pending = false;
    }

    portEXIT_CRITICAL(&acq_err_mux);

    return had;
}

static void raw_sample_cb(int32_t sample, int64_t t_drdy_us, void *ctx)
{
    (void)t_drdy_us;// not needed for servo
    (void)ctx; // similarly not needed for servo

    /* Bounded and nonblocking in practice: the only other holder is the
     * supervisor loop during recovery entry, which holds it for the
     * duration of servo_init() alone. */
    if (xSemaphoreTake(servo_mux, portMAX_DELAY) == pdTRUE) {
        servo_process_sample(sample);
        xSemaphoreGive(servo_mux);
    }
}

void app_main(void) {
    ESP_LOGI("MAIN", "PPG firmware starting");

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    supervisor_reset(now_ms);

    // adc initilization here
    static ads131m02_t adc; // keeps track of running state of the adc after running

    ads131m02_config_t adc_cfg = {
        .spi_host = SPI2_HOST,
        .pin_sclk = 12,
        .pin_mosi = 11,
        .pin_miso = 13,
        .pin_cs = 10,
        .pin_drdy = 14,
        .pin_sync_reset = 21,
        .spi_clock_hz = 8000000,
    }; //tells the driver the pinout of the adc

    // Initialize the ADS131M02 using the hardware config and store the success/error result.
    esp_err_t err = ads131m02_init(&adc, &adc_cfg);

    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "ADS131M02 init failed: %s",
                 esp_err_to_name(err));
        return;
    } // error check

    /* Created before servo_init() and before the acquisition layer
     * exists, so no callback can ever run without it. */
    servo_mux = xSemaphoreCreateMutex();

    if (servo_mux == NULL) {
        ESP_LOGE("MAIN", "servo mutex creation failed");
        return;
    }

    servo_init(); // servo intilization

    ppg_acq_config_t acq_cfg = {
        .adc = &adc,
        .raw_sample_cb = raw_sample_cb,
        .output_cb = NULL,
        .error_cb = acq_error_cb,
        .cb_ctx = NULL,
        .task_priority = configMAX_PRIORITIES - 2,
        .output_queue_len = 16,
    };

    err = ppg_acq_init(&acq_cfg);

    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "PPG acquisition init failed: %s",
                 esp_err_to_name(err));
        return;
    }

    bool acq_running = false;

    supervisor_inputs_t inputs = {0}; // intaizle states to false
    inputs.power_requested = true; // power the ppg

    gpio_config_t pg_cfg = {
        .pin_bit_mask = (1ULL << POWER_GOOD_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&pg_cfg);

    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Power-good GPIO config failed: %s",
                 esp_err_to_name(err));
        return;
    }

    uint32_t analog_settle_start_ms = 0;
    bool analog_settle_started = false; // for analog settle

    /* PWRGD debounce state.  pg_high_since_ms / pg_low_since_ms are only
     * meaningful while the corresponding _valid flag is set. */
    uint32_t pg_high_since_ms = 0;
    bool     pg_high_timing   = false;
    bool     pg_stable_good   = false;

    uint32_t pg_low_since_ms  = 0;
    bool     pg_low_timing    = false;

    /* Event -> sustained condition bridge for acquisition errors. */
    bool     acq_fault_condition   = false;
    uint32_t acq_fault_start_ms    = 0;

    /* Bounded ppg_acq_start() failure handling. */
    uint32_t acq_start_last_try_ms = 0;
    bool     acq_start_tried       = false;
    uint32_t acq_start_failures    = 0;
    bool     acq_start_unusable    = false;

    /* State-entry detection.  supervisor_reset() left us in POWER_OFF. */
    supervisor_state_t previous_state = supervisor_get_state();
    bool fault_handled = false;

    ////////////////
    while (1) {
        inputs.now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        supervisor_state_t state = supervisor_get_state();
        bool state_changed = (state != previous_state);

        /* ---------------------------------------------------------- */
        /* Consume any acquisition error posted by the acquisition     */
        /* task.  ppg_acq has already halted itself (running = false,  */
        /* DRDY disabled), so main's shadow flag must follow or the    */
        /* start/stop edge logic below would never restart it.         */
        /* ---------------------------------------------------------- */
        ppg_acq_error_t acq_reason;

        if (acq_error_take(&acq_reason)) {
            acq_running = false;

            ESP_LOGE(TAG, "acquisition error %d in state %d",
                     (int)acq_reason, (int)state);

            /* Present it as a sustained condition; see ACQ_FAULT_HOLD_MS. */
            acq_fault_condition = true;
            acq_fault_start_ms  = inputs.now_ms;
        }

        /* Release the hold once the supervisor has acted on it, or once
         * the window expires.  recoverable_condition is only evaluated in
         * RUN, so outside RUN the hold is pointless and is dropped
         * immediately rather than left asserted. */
        if (acq_fault_condition) {
            if (state != RUN) {
                acq_fault_condition = false;
            }
            else if ((uint32_t)(inputs.now_ms - acq_fault_start_ms)
                     >= ACQ_FAULT_HOLD_MS) {
                acq_fault_condition = false;
            }
        }

        /* ---------------------------------------------------------- */
        /* PWRGD sampling and persistence.                             */
        /* ---------------------------------------------------------- */
        bool pg_raw =
            gpio_get_level(POWER_GOOD_GPIO) == POWER_GOOD_ACTIVE_LEVEL;

        if (pg_raw) {
            if (!pg_high_timing) {
                pg_high_since_ms = inputs.now_ms;
                pg_high_timing   = true;
            }

            if ((uint32_t)(inputs.now_ms - pg_high_since_ms)
                >= PG_STARTUP_PERSIST_MS) {
                pg_stable_good = true;
            }

            pg_low_timing = false;
        }
        else {
            pg_high_timing = false;
            pg_stable_good = false;

            if (!pg_low_timing) {
                pg_low_since_ms = inputs.now_ms;
                pg_low_timing   = true;
            }
        }

        inputs.power_good = pg_stable_good;

        /* Runtime loss.  POWER_OFF and POWER_WAIT are the states where a
         * low rail is expected and is handled by the supervisor's own
         * POWER_WAIT timeout, so they are excluded.  FAULT is excluded
         * because it is already terminal. */
        bool analog_expected_valid =
            (state != POWER_OFF) &&
            (state != POWER_WAIT) &&
            (state != FAULT);

        bool pg_runtime_lost =
            analog_expected_valid &&
            pg_low_timing &&
            ((uint32_t)(inputs.now_ms - pg_low_since_ms)
             >= PG_RUNTIME_LOSS_PERSIST_MS);

        /* ---------------------------------------------------------- */
        /* Servo-derived level inputs.                                 */
        /* ---------------------------------------------------------- */
        inputs.servo_converged = servo_is_converged();

        /*
         * TEMPORARY SOFTWARE PROXY.
         *
         * This only reports that the servo's internal quantized command
         * is zero.  It does NOT prove that the cancellation PWM output is
         * at mid-scale, that the reconstruction filter has settled, or
         * that the physical cancellation current is zero.  It is kept
         * because the cancellation PWM driver does not exist yet and
         * startup cannot progress without it.
         *
         * TODO: replace or augment with a real actuator/settling check
         * once the PWM driver exists.
         */
        inputs.cancellation_neutral =
            (servo_get_actual_current_q16() == 0);

        if (state == ANALOG_SETTLE) {
            if (!analog_settle_started) {
                analog_settle_start_ms = inputs.now_ms;
                analog_settle_started = true;
            }

            inputs.analog_reference_stable =
                (uint32_t)(inputs.now_ms - analog_settle_start_ms)
                    >= ANALOG_SETTLE_MS;
        }
        else {
            analog_settle_started = false;
            inputs.analog_reference_stable = false;
        }

        /* ---------------------------------------------------------- */
        /* State-entry actions.  Each runs once per entry.             */
        /* ---------------------------------------------------------- */
        if (state_changed && (state == RECOVERY)) {
                /*
                * ORDERING IS LOAD-BEARING.
                *
                * servo_process_sample() runs from raw_sample_cb() in the
                * acquisition task; servo_init() runs here in the supervisor
                * task. Acquisition must be stopped before resetting the servo.
                *
                * ppg_acq_stop() disabling DRDY and draining the queue stops NEW work
                * but does not by itself join an iteration already inside
                * raw_sample_cb(). The servo_mux taken below covers that: stopping
                * prevents new callbacks, and the mutex waits out the in-flight one.
                * Neither mechanism alone is sufficient.
                */
            if (acq_running) {
                err = ppg_acq_stop();

                if (err == ESP_OK) {
                    acq_running = false;
                }
                else {
                    ESP_LOGE(TAG, "recovery: ppg_acq_stop failed: %s",
                             esp_err_to_name(err));
                }
            }

            if (!acq_running) {
                /* Acquisition is stopped, so no NEW callback can start.
                 * Taking the mutex waits out any callback that was
                 * already inside servo_process_sample() when stop was
                 * issued. */
                xSemaphoreTake(servo_mux, portMAX_DELAY);

                /* Software state only.  This resets the servo's
                 * integrator and quantized command so that the following
                 * SERVO_INIT can observe cancellation_neutral.  It does
                 * NOT drive the physical actuator to neutral.  When the
                 * cancellation PWM driver exists, recovery must also
                 * command the actuator to mid-scale and wait for the
                 * reconstruction filter to settle before SERVO_INIT is
                 * allowed to succeed. */
                servo_init();

                xSemaphoreGive(servo_mux);

                /* A fresh acquisition epoch may start cleanly. */
                acq_start_failures = 0;
                acq_start_unusable = false;
                acq_start_tried    = false;
            }
            else {
                ESP_LOGE(TAG, "recovery: servo not reset, acquisition "
                              "still running");
            }
        }

        if (state_changed && (state == FAULT)) {
            fault_handled = false;
        }

        if ((state == FAULT) && !fault_handled) {
            fault_handled = true;

            ESP_LOGE(TAG, "FAULT entered: code %d (terminal until reset)",
                     (int)supervisor_get_fault());

            if (acq_running) {
                err = ppg_acq_stop();

                if (err == ESP_OK) {
                    acq_running = false;
                }
                else {
                    ESP_LOGE(TAG, "FAULT: ppg_acq_stop failed: %s",
                             esp_err_to_name(err));
                }
            }
        }

        /* ---------------------------------------------------------- */
        /* Event-style inputs.  Set deliberately every tick so none of  */
        /* them can latch true across iterations.                      */
        /* ---------------------------------------------------------- */
        inputs.recoverable_condition = acq_fault_condition;

        /* Persistent PWRGD loss, or acquisition hardware that will not
         * start, both mean the signal chain cannot be trusted.  Both are
         * level conditions while they persist, which is what the
         * supervisor's UNRECOVERABLE_PERSIST_TICKS filter expects. */
        inputs.unrecoverable_fault = pg_runtime_lost || acq_start_unusable;

        /* Not implemented yet.  Deliberately left false: the LED driver
         * does not exist, so the supervisor is expected to wait in
         * LED_ENABLE and eventually time out during bring-up. */
        inputs.led_confirmed_on = false;

        /* No AGC yet. */
        inputs.agc_step_occurred = false;

        /* No valid producer exists for either of these.  See the notes
         * after this file: RECOVERY currently ends in its own timeout. */
        inputs.recovery_succeeded = false;
        inputs.recovery_failed    = false;

        supervisor_tick(&inputs); // getting input structure data for structure tick function

        /* ---------------------------------------------------------- */
        /* Acquisition start/stop, edge triggered.                     */
        /* ---------------------------------------------------------- */
        bool acq_should_run = supervisor_ppg_acquisition_active();

        /* While a recoverable acquisition error is still being presented
         * to the supervisor in RUN, restarting acquisition would resume
         * sampling under the very condition the supervisor is about to
         * act on, and would clear the fault from underneath it.  Hold off
         * until the supervisor transitions out of RUN.
         *
         * FAST_ACQUIRE and SETTLING are deliberately not held off: the
         * state machine does not consume recoverable_condition outside
         * RUN, so an immediate clean restart (resync + FIR reset + settle
         * discard) is the correct response there, bounded by those
         * states' own 30 s timeouts. */
        bool restart_blocked =
            (state == RUN) && acq_fault_condition;

        if (acq_should_run && !acq_running && !restart_blocked) { // if function should run and not running start it
            bool may_try =
                !acq_start_unusable &&
                (!acq_start_tried ||
                 ((uint32_t)(inputs.now_ms - acq_start_last_try_ms)
                  >= ACQ_START_RETRY_MS));

            if (may_try) {
                acq_start_tried       = true;
                acq_start_last_try_ms = inputs.now_ms;

                err = ppg_acq_start();

                if (err == ESP_OK) {
                    acq_running        = true;
                    acq_start_failures = 0;
                    acq_start_tried    = false;
                }
                else {
                    acq_start_failures++;

                    ESP_LOGE("MAIN", "PPG acquisition start failed: %s (%u)",
                             esp_err_to_name(err),
                             (unsigned)acq_start_failures);

                    if (acq_start_failures >= ACQ_START_MAX_FAILURES) {
                        /* Escalate rather than retry forever. */
                        acq_start_unusable = true;
                        ESP_LOGE("MAIN", "acquisition unusable after %u "
                                 "attempts", (unsigned)acq_start_failures);
                    }
                }
            }
        }
        else if (!acq_should_run && acq_running) { // if function shouldnt run and is running stop it
            err = ppg_acq_stop();

            if (err == ESP_OK) {
                acq_running = false;
            }
            else {
                /* Do not pretend it stopped.  acq_running stays true so
                 * this is retried on the next loop rather than leaving
                 * main's view of the acquisition layer wrong. */
                ESP_LOGE("MAIN", "PPG acquisition stop failed: %s",
                         esp_err_to_name(err));
            }
        }

        previous_state = state;

        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
    }
}