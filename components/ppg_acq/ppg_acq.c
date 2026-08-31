/*
 * PPG acquisition layer.  See ppg_acq.h for architecture, supervisor
 * hook points, timestamp policy and error policy.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "driver/gpio.h"

#include "ppg_acq.h"

static const char *TAG = "ppg_acq";

typedef struct {
    int64_t t_drdy_us;
} drdy_evt_t;

static struct {
    ppg_acq_config_t cfg;
    QueueHandle_t    q;
    QueueHandle_t    out_q;             /* optional 120-SPS output queue */
    TaskHandle_t     task;
    volatile bool    running;
    uint32_t         settle_discard_remaining;
    ppg_acq_stats_t  stats;
    volatile uint32_t isr_overruns;     /* incremented in ISR only       */
} s;

/* ------------------------------------------------------------------ */
/* DRDY ISR: timestamp capture + queue post only.  No SPI, no FIR.    */
/* esp_timer_get_time() is documented ISR-safe on ESP-IDF.            */
/* ------------------------------------------------------------------ */
static void IRAM_ATTR drdy_isr(void *arg)
{
    (void)arg;
    drdy_evt_t evt = { .t_drdy_us = esp_timer_get_time() };
    BaseType_t hpw = pdFALSE;

    if (xQueueSendFromISR(s.q, &evt, &hpw) != pdTRUE) {
        /* Queue full: DRDY overrun.  Count it; the task escalates. */
        s.isr_overruns++;
    }
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ------------------------------------------------------------------ */

/*
 * CASE B: a physical 1200-SPS conversion is unrecoverable.  The stream
 * is discontinuous: stop accepting samples immediately (nothing further
 * may reach the FIR), then hand control to the EXISTING supervisor
 * recovery, which re-enters through ppg_acq_start() (ADC resync +
 * ppg_stream_reset()/fir_init() + settling discard = clean uniform
 * restart).  See ppg_acq_policy.h.
 */
static void lost_conversion(ppg_acq_error_t reason)
{
    s.stats.lost_conversions++;
    s.running = false;
    ppg_stream_halt();
    (void)gpio_intr_disable(s.cfg.adc->cfg.pin_drdy);
    if (s.cfg.error_cb) {
        s.cfg.error_cb(reason, s.cfg.cb_ctx);
    }
}

/* DRDY asserted again? MODE.DRDY_FMT = 0: active-low level, returns
 * high when data are retrieved, drives low when NEW data are ready
 * (SBAS853A 8.5.1.5).  Low here after a completed read = the next
 * conversion result is (about to be) loaded: the failed conversion can
 * no longer be re-read as itself. */
static bool drdy_reasserted(void)
{
    return gpio_get_level(s.cfg.adc->cfg.pin_drdy) == 0;
}

static void acq_task(void *arg)
{
    (void)arg;
    drdy_evt_t evt;
    ads131m02_frame_t frame;
    uint32_t reported_overruns = 0;

    for (;;) {
        if (xQueueReceive(s.q, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!s.running) {
            continue;   /* drained event from a stopped session */
        }

        /* Overrun = a DRDY edge (a physical conversion) was dropped:
         * that conversion is unrecoverable => case B, halt. */
        uint32_t ov = s.isr_overruns;
        if (ov != reported_overruns) {
            s.stats.drdy_overruns += (ov - reported_overruns);
            reported_overruns = ov;
            lost_conversion(PPG_ACQ_ERR_OVERRUN);
            continue;
        }

        /* Backlog AFTER this dequeue.  Exactly 1 pending event is
         * covered by the device holding the current AND previous
         * conversion results (SBAS853A 8.5.1.9.1); more than that
         * breaks the conversion<->timestamp pairing. */
        uint32_t backlog = (uint32_t)uxQueueMessagesWaiting(s.q);

        /* SPI frame read happens here, in task context, not in the
         * ISR.  The sample time is evt.t_drdy_us (captured at DRDY),
         * NOT the SPI completion time.
         *
         * CASE A retry loop: an SPI/CRC-failed read of a conversion is
         * re-attempted for the SAME conversion, bounded and guarded
         * (ppg_acq_policy.h).  A retried sample keeps its original
         * DRDY timestamp because it IS the same conversion. */
        uint32_t retries = 0;
        ppg_acq_read_decision_t decision;
        esp_err_t err;

        for (;;) {
            err = ads131m02_read_frame(s.cfg.adc, &frame);

            bool frame_ok = (err == ESP_OK);
            /* STATUS fault screening (datasheet Table 8-15):
             *  - RESET set unexpectedly => the device rebooted
             *    underneath us (default, wrong, configuration).
             *  - F_RESYNC set => conversions resynchronized; timing
             *    continuity is broken at this instant.
             * Retrying cannot repair either => treated as lost. */
            bool status_fault = frame_ok &&
                (frame.status & (ADS131M02_STATUS_RESET |
                                 ADS131M02_STATUS_F_RESYNC));

            decision = ppg_acq_classify_read(frame_ok, status_fault,
                                             retries, drdy_reasserted(),
                                             backlog,
                                             esp_timer_get_time(),
                                             evt.t_drdy_us);
            if (decision != PPG_ACQ_READ_RETRY) {
                if (!frame_ok) {
                    if (err == ESP_ERR_INVALID_CRC) s.stats.crc_errors++;
                    else                            s.stats.spi_errors++;
                } else if (status_fault) {
                    s.stats.status_errors++;
                }
                break;
            }
            if (err == ESP_ERR_INVALID_CRC) s.stats.crc_errors++;
            else                            s.stats.spi_errors++;
            retries++;
            s.stats.frame_retries++;
        }

        if (decision == PPG_ACQ_READ_LOST) {
            ppg_acq_error_t reason =
                (backlog > PPG_ACQ_MAX_TOLERATED_BACKLOG)
                    ? PPG_ACQ_ERR_LOST_CONVERSION
                    : (err == ESP_ERR_INVALID_CRC) ? PPG_ACQ_ERR_FRAME_CRC
                    : (err != ESP_OK)              ? PPG_ACQ_ERR_SPI
                                                   : PPG_ACQ_ERR_STATUS;
            lost_conversion(reason);
            continue;   /* halted; queue drains until supervisor acts */
        }

        if (retries > 0) {
            s.stats.retry_recoveries++;
        }
        s.stats.frames_ok++;

        /* ADC digital-filter settling discard after (re)sync. */
        if (s.settle_discard_remaining > 0) {
            s.settle_discard_remaining--;
            s.stats.settle_discards++;
            continue;
        }

        /* Valid 1200-SPS PPG sample (CH0 = TIA vs analog reference,
         * measured differentially by the ADC — no digital pedestal
         * subtraction).  The existing baseline servo is designed around
         * a 1200-SPS update rate, so it receives this valid raw stream
         * BEFORE decimation.  The callback must remain bounded/nonblocking. */
        s.stats.raw_samples++;
        if (s.cfg.raw_sample_cb) {
            s.cfg.raw_sample_cb(frame.ch[0], evt.t_drdy_us, s.cfg.cb_ctx);
        }

        /* Feed that same valid physical conversion to the validated FIR
         * exactly once. */
        ppg_output_t out;
        if (ppg_stream_push(frame.ch[0], evt.t_drdy_us, &out)) {
            /* Exactly one 120-SPS output per 10 valid inputs.  This is
             * the only stream that goes downstream.  Hand-off must stay
             * inside the real-time budget: enqueue and/or a short,
             * nonblocking callback only (see OUTPUT REAL-TIME CONTRACT
             * in ppg_acq.h). */
            s.stats.outputs++;
            if (s.out_q) {
                if (xQueueSend(s.out_q, &out, 0) != pdTRUE) {
                    s.stats.output_drops++;   /* consumer too slow */
                }
            }
            if (s.cfg.output_cb) {
                s.cfg.output_cb(&out, s.cfg.cb_ctx);
            }
        }
    }
}

/* ------------------------------------------------------------------ */

esp_err_t ppg_acq_init(const ppg_acq_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->adc, ESP_ERR_INVALID_ARG, TAG, "args");

    memset(&s, 0, sizeof(s));
    s.cfg = *cfg;

    s.q = xQueueCreate(PPG_ACQ_QUEUE_DEPTH, sizeof(drdy_evt_t));
    ESP_RETURN_ON_FALSE(s.q, ESP_ERR_NO_MEM, TAG, "queue");

    if (cfg->output_queue_len > 0) {
        s.out_q = xQueueCreate(cfg->output_queue_len, sizeof(ppg_output_t));
        ESP_RETURN_ON_FALSE(s.out_q, ESP_ERR_NO_MEM, TAG, "out queue");
    }

    ESP_RETURN_ON_FALSE(
        xTaskCreate(acq_task, "ppg_acq", 3072, NULL,
                    cfg->task_priority, &s.task) == pdPASS,
        ESP_ERR_NO_MEM, TAG, "task");

    /* DRDY: falling edge marks new data (MODE.DRDY_FMT=0, active-low
     * level; datasheet 8.5.1.5).  ISR service may already be installed
     * by the application. */
    esp_err_t err = gpio_install_isr_service(0);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(s.cfg.adc->cfg.pin_drdy,
                                           GPIO_INTR_NEGEDGE),
                        TAG, "intr type");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s.cfg.adc->cfg.pin_drdy,
                                             drdy_isr, NULL),
                        TAG, "isr add");
    ESP_RETURN_ON_ERROR(gpio_intr_disable(s.cfg.adc->cfg.pin_drdy),
                        TAG, "intr off");
    return ESP_OK;
}

esp_err_t ppg_acq_start(void)
{
    ESP_RETURN_ON_FALSE(s.q, ESP_ERR_INVALID_STATE, TAG, "not init");

    s.running = false;
    (void)gpio_intr_disable(s.cfg.adc->cfg.pin_drdy);

    /* 1. Fresh FIR delay line, halt cleared: no pre-fault/pre-recovery
     *    samples may survive an acquisition restart.  This is the ONLY
     *    place a case-B discontinuity is cleared. */
    ppg_stream_reset();
    s.settle_discard_remaining = PPG_ACQ_SETTLE_DISCARD;

    /* 2. Realign the ADC output FIFO / DRDY after the pause
     *    (datasheet 8.5.1.9.1): SYNC pulse when the pin is wired,
     *    otherwise the double-read method. */
    if (ads131m02_sync_pulse(s.cfg.adc) == ESP_ERR_NOT_SUPPORTED) {
        ESP_RETURN_ON_ERROR(ads131m02_flush_fifo(s.cfg.adc), TAG, "flush");
    }

    /* 3. Drain stale acquisition and downstream outputs.  No pre-fault
     *    120-SPS output may survive into a new acquisition epoch. */
    xQueueReset(s.q);
    if (s.out_q) {
        xQueueReset(s.out_q);
    }
    s.running = true;
    ESP_RETURN_ON_ERROR(gpio_intr_enable(s.cfg.adc->cfg.pin_drdy),
                        TAG, "intr on");
    return ESP_OK;
}

esp_err_t ppg_acq_stop(void)
{
    ESP_RETURN_ON_FALSE(s.q, ESP_ERR_INVALID_STATE, TAG, "not init");
    s.running = false;
    (void)gpio_intr_disable(s.cfg.adc->cfg.pin_drdy);
    xQueueReset(s.q);
    if (s.out_q) {
        xQueueReset(s.out_q);
    }
    return ESP_OK;
}

void ppg_acq_get_stats(ppg_acq_stats_t *out)
{
    if (out) {
        *out = s.stats;
        /* The ISR-side counter is the authoritative overrun total
         * (the task folds it into stats lazily). */
        out->drdy_overruns = s.isr_overruns;
    }
}

QueueHandle_t ppg_acq_output_queue(void)
{
    return s.out_q;
}
