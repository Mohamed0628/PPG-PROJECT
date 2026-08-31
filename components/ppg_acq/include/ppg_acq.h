#ifndef PPG_ACQ_H
#define PPG_ACQ_H

/*
 * PPG acquisition layer: ADS131M02 -> DRDY ISR -> timestamp -> queue ->
 * SPI frame read (task context) -> decode -> ppg_stream (validated FIR)
 * -> 120-SPS callback.
 *
 * Architecture (per project requirement 4):
 *
 *   DRDY falling edge (defines the PPG sampling instant)
 *       |  GPIO ISR: esp_timer_get_time() ONLY + queue post   (~ few us)
 *       v
 *   acquisition task (high priority)
 *       |  SPI frame read (polling, ~12 us on the wire)
 *       |  CRC + STATUS validation
 *       |  CH0 -> int32 (already sign-extended by the frame layer)
 *       v
 *   raw_sample_cb(sample, t_drdy)         <- existing baseline servo
 *       |                                     consumes the VALID raw
 *       |                                     1200-SPS stream here
 *       v
 *   ppg_stream_push(sample, t_drdy)        <- fir_process_sample() here,
 *       |                                     exactly once per valid
 *       v                                     1200-SPS sample
 *   output_cb(ppg_output_t) / output queue  <- 120-SPS filtered stream
 *                                               for downstream PPG DSP,
 *                                               NLMS and HR processing
 *
 * The SPI transaction completion time is never used as the sample time;
 * the timestamp is captured in the DRDY ISR itself.
 *
 * SUPERVISOR INTEGRATION (state_machine.c was not available in this
 * patch; these are the exact hook points):
 *
 *   - LED_ENABLE/FAST_ACQUIRE entry (and every RECOVERY -> reacquire
 *     transition): call ppg_acq_start().  It performs, in order:
 *     ppg_stream_reset() (fir_init(); empties the delay line of any
 *     pre-fault samples), SYNC/RESET pulse (or double-read FIFO flush),
 *     settling discard arming, then enables the DRDY interrupt.
 *   - RUN: nothing to do; do NOT reset the FIR.
 *   - SETTLING: the supervisor's existing sample-valid gating applies
 *     downstream of output_cb, unchanged.  Independently of that, this
 *     layer discards the first PPG_ACQ_SETTLE_DISCARD conversions after
 *     each (re)sync because the ADC itself outputs unsettled data then
 *     (fast-settling filter for 2 samples, sinc3 settled at the 3rd:
 *     datasheet 8.4.2 / Table 8-3).
 *   - FAULT/RECOVERY entry: call ppg_acq_stop() (disables the DRDY
 *     interrupt, drains the queue).
 *   - error_cb fires from the acquisition task when the error policy
 *     (below) escalates; the supervisor maps it onto its existing
 *     RECOVERY/FAULT transitions.  No second state machine exists here:
 *     this layer only counts and reports.
 *
 * ERROR POLICY (see ppg_acq_policy.h for the full derivation):
 *   The FIR assumes a UNIFORMLY sampled 1200-SPS sequence on the ADC
 *   conversion grid.  A physical conversion is therefore never silently
 *   skipped: either the very same conversion is recovered, or the
 *   stream is treated as discontinuous.
 *
 *   - CASE A (SPI retry, same conversion): on an invalid frame
 *     (SPI error / output-CRC mismatch) the SAME conversion is re-read,
 *     bounded by PPG_ACQ_MAX_FRAME_RETRIES, only while DRDY has not
 *     re-asserted, within PPG_ACQ_RETRY_DEADLINE_US of the DRDY edge,
 *     and with no backlog.  The ADS131M02 holds conversion results in
 *     its output buffer until the next conversion result is loaded
 *     (SBAS853A 8.5.1.9 / 8.5.1.9.1), so the retried frame carries the
 *     same conversion and keeps its original DRDY timestamp: the grid
 *     stays intact and the sample is pushed to the FIR exactly once.
 *   - CASE B (lost conversion): retries exhausted / window expired /
 *     DRDY re-asserted first / DRDY queue overrun / backlog >= 2 /
 *     STATUS.RESET or STATUS.F_RESYNC set.  Acquisition then:
 *       1. stops accepting samples (ppg_stream_halt() + DRDY interrupt
 *          disabled) — nothing further reaches the FIR,
 *       2. notifies the EXISTING supervisor via error_cb(reason),
 *       3. the supervisor's existing recovery re-enters through
 *          ppg_acq_start(): ADC resync/FIFO realign, ppg_stream_reset()
 *          (fir_init()) at the restart boundary, ADC settling discard,
 *          then a clean uniform 1200-SPS stream restarts.
 *   - All error paths are counted (ppg_acq_stats_t), never logged
 *     per-sample.
 *
 * RAW + OUTPUT REAL-TIME CONTRACT:
 *   raw_sample_cb runs once for every valid 1200-SPS conversion, after
 *   ADC settling discard and before the FIR.  In this project it is the
 *   hook for servo_process_sample(); it must remain bounded/nonblocking.
 *
 *   output_cb runs in the HIGH-PRIORITY acquisition task, inside the
 *   833 us sample budget.  It MUST be nonblocking and short (target a
 *   few microseconds: copy the sample, set a flag, or enqueue).  It
 *   MUST NOT perform BLE operations, logging, NLMS or other heavy DSP,
 *   or any blocking I/O — put those in a consumer task.  If the
 *   consumer needs decoupling, set output_queue_len > 0: the
 *   acquisition task then only enqueues each 120-SPS ppg_output_t
 *   (zero-timeout xQueueSend) and returns; the consumer drains via
 *   ppg_acq_output_queue().  A full output queue increments
 *   stats.output_drops and never blocks or halts acquisition — sizing
 *   the queue and draining it in time is the consumer's contract.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ads131m02.h"
#include "ppg_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC digital-filter settling after reset/sync: fast-settling filter
 * covers the first two DRDY samples, the third is the first fully
 * settled sinc3 result (datasheet 8.4.2, Table 8-3: 2648 t_CLKIN for
 * OSR 1024).  Discard 4 for margin (3.3 ms @ 1200 SPS). */
#define PPG_ACQ_SETTLE_DISCARD      4u

#define PPG_ACQ_QUEUE_DEPTH         32u     /* ISR->task event queue     */

#include "ppg_acq_policy.h"          /* retry/lost decision constants    */

typedef enum {
    PPG_ACQ_ERR_SPI,                /* SPI transaction failure           */
    PPG_ACQ_ERR_FRAME_CRC,          /* output CRC mismatch               */
    PPG_ACQ_ERR_STATUS,             /* STATUS fault bits set             */
    PPG_ACQ_ERR_OVERRUN,            /* DRDY queue overflow (edge lost)   */
    PPG_ACQ_ERR_CONFIG,             /* register verification mismatch    */
    PPG_ACQ_ERR_LOST_CONVERSION,    /* timestamp<->conversion pairing
                                       broken (backlog beyond buffering) */
} ppg_acq_error_t;

typedef struct {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t spi_errors;
    uint32_t status_errors;
    uint32_t drdy_overruns;
    uint32_t settle_discards;
    uint32_t frame_retries;         /* case-A same-conversion re-reads   */
    uint32_t retry_recoveries;      /* conversions recovered by retry    */
    uint32_t lost_conversions;      /* case-B discontinuities (halts)    */
    uint32_t raw_samples;           /* valid 1200-SPS samples delivered  */
    uint32_t outputs;               /* 120-SPS samples delivered         */
    uint32_t output_drops;          /* output queue full (consumer late) */
} ppg_acq_stats_t;

/* Valid raw 1200-SPS sample sink.  Called after ADC settling discard and
 * before fir_process_sample().  This is the correct integration point for
 * the existing baseline servo, whose coefficients/timing are defined for
 * 1200 SPS.  Keep the callback bounded and nonblocking. */
typedef void (*ppg_acq_raw_sample_cb_t)(int32_t sample, int64_t t_drdy_us,
                                        void *ctx);

/* 120-SPS output sink.  Called from the HIGH-PRIORITY acquisition task:
 * MUST be nonblocking and short (see OUTPUT REAL-TIME CONTRACT above).
 * This is the hand-off point to downstream PPG DSP/NLMS/HR processing.
 * Do NOT feed the baseline servo here; the existing servo is a 1200-SPS
 * control loop and belongs on raw_sample_cb above. */
typedef void (*ppg_acq_output_cb_t)(const ppg_output_t *out, void *ctx);

/* Escalated-error sink -> existing supervisor RECOVERY/FAULT paths. */
typedef void (*ppg_acq_error_cb_t)(ppg_acq_error_t reason, void *ctx);

typedef struct {
    ads131m02_t           *adc;     /* initialized by ads131m02_init()   */
    ppg_acq_raw_sample_cb_t raw_sample_cb;
    ppg_acq_output_cb_t     output_cb;
    ppg_acq_error_cb_t  error_cb;
    void               *cb_ctx;
    int                 task_priority;   /* e.g. configMAX_PRIORITIES-2  */
    unsigned            output_queue_len;/* 0 = callback only; >0 = also
                                            enqueue each 120-SPS output
                                            into a queue the consumer
                                            drains (recommended when the
                                            consumer does BLE/NLMS/IO)  */
} ppg_acq_config_t;

/* One-time setup: task + queue + DRDY ISR registration (interrupt left
 * disabled until ppg_acq_start()). */
esp_err_t ppg_acq_init(const ppg_acq_config_t *cfg);

/* (Re)start acquisition: FIR reset, ADC resync/FIFO flush, settle
 * discard arm, DRDY interrupt enable.  Call from supervisor on startup
 * and on every recovery/reacquisition — never during RUN. */
esp_err_t ppg_acq_start(void);

/* Stop acquisition: DRDY interrupt disable + queue drain. */
esp_err_t ppg_acq_stop(void);

void ppg_acq_get_stats(ppg_acq_stats_t *out);

/* Downstream 120-SPS queue of ppg_output_t (NULL when
 * output_queue_len == 0).  Consumer task blocks on this instead of
 * doing work inside output_cb. */
QueueHandle_t ppg_acq_output_queue(void);

#ifdef __cplusplus
}
#endif

#endif /* PPG_ACQ_H */
