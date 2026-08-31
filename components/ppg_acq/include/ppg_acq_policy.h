#ifndef PPG_ACQ_POLICY_H
#define PPG_ACQ_POLICY_H

/*
 * Acquisition read-decision policy (pure, host-testable: no ESP-IDF).
 *
 * The FIR (fir.c) assumes a UNIFORMLY sampled 1200-SPS sequence: the
 * decimation phase is defined on the physical ADC conversion grid.  A
 * conversion that cannot be delivered to fir_process_sample() therefore
 * makes the stream DISCONTINUOUS; pushing the next conversion as if it
 * were adjacent in time would compress the time axis and silently shift
 * the decimation phase off the ADC grid.  That is never allowed.
 *
 * Two cases are distinguished for a failed frame read:
 *
 *  CASE A - SPI-transaction failure, SAME conversion still recoverable.
 *    The ADS131M02 output data are not consumed destructively by a
 *    read: conversion results are held in the output buffer and are
 *    replaced only when the next conversion result is loaded (the
 *    device buffers the current and previous conversion results;
 *    datasheet SBAS853A Section 8.5.1.9 / 8.5.1.9.1 - this same
 *    buffering is what makes the Figure 8-21 double-read realignment
 *    possible).  So if
 *      (a) DRDY has NOT re-asserted (with MODE.DRDY_FMT = 0 the pin
 *          returns high when data are retrieved and drives low again
 *          only when NEW data become ready, Section 8.5.1.5), and
 *      (b) we are still safely inside the current conversion period,
 *          and
 *      (c) we are not already processing a backlog,
 *    then re-reading the frame returns the SAME conversion, and the
 *    retried sample keeps its original DRDY timestamp: the uniform grid
 *    is intact.  Bounded retries are allowed in this window.
 *    NOTE: the STATUS.DRDY0 flag will read 0 on the retry frame (data
 *    already retrieved once) - that is expected and is not an error.
 *
 *  CASE B - genuinely lost conversion.  Any of:
 *      - retries exhausted or the retry window expired,
 *      - DRDY re-asserted before the retry (the buffer now pairs with
 *        the NEXT conversion; the failed one cannot be recovered with
 *        its own timestamp),
 *      - DRDY queue overrun (an edge was dropped),
 *      - backlog of >= 2 pending DRDY events (beyond the device's
 *        current+previous buffering, sample<->timestamp pairing is no
 *        longer guaranteed),
 *      - STATUS.RESET or STATUS.F_RESYNC set (the device itself broke
 *        conversion continuity; no retry can fix timing).
 *    The stream is then DISCONTINUOUS: acquisition halts immediately
 *    (ppg_stream_halt(); no further samples reach the FIR), error_cb
 *    notifies the EXISTING supervisor, and the existing recovery path
 *    re-enters through ppg_acq_start(): ADC resync, ppg_stream_reset()
 *    (fir_init()), ADC settling discard, clean uniform restart.
 *    A physical sample is never silently skipped.
 *
 * A backlog of exactly 1 pending event is tolerated: the device holds
 * the current AND previous conversion results (Section 8.5.1.9.1), so a
 * read that lags by one period still returns the conversion matching
 * the dequeued DRDY timestamp.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bounded same-conversion retries (case A).  Each retry is ~25 us of
 * bus time; 2 retries stay far inside the period. */
#define PPG_ACQ_MAX_FRAME_RETRIES   2u

/* Latest time after the DRDY edge at which a retry may still be issued.
 * Chosen well short of the 833.33 us conversion period so the retry
 * completes before the next conversion result can replace the buffer
 * contents (retry itself ~25 us, plus margin for scheduling). */
#define PPG_ACQ_RETRY_DEADLINE_US   600

/* Backlog (events still pending AFTER dequeuing the current one) that
 * is covered by the device's current+previous conversion buffering. */
#define PPG_ACQ_MAX_TOLERATED_BACKLOG 1u

typedef enum {
    PPG_ACQ_READ_ACCEPT = 0,    /* valid sample, push to FIR            */
    PPG_ACQ_READ_RETRY,         /* case A: re-read the SAME conversion  */
    PPG_ACQ_READ_LOST,          /* case B: discontinuity -> halt +
                                   supervisor recovery                  */
} ppg_acq_read_decision_t;

/*
 * Classify one frame-read attempt.
 *   frame_ok         SPI transfer succeeded AND output CRC matched
 *   status_fault     STATUS.RESET or STATUS.F_RESYNC set (frame_ok only)
 *   retries_done     same-conversion retries already performed
 *   drdy_reasserted  DRDY pin is asserted (low) again = new data ready
 *   backlog_events   DRDY events still pending after this dequeue
 *   now_us           current monotonic time
 *   t_drdy_us        DRDY timestamp of the conversion being read
 */
static inline ppg_acq_read_decision_t ppg_acq_classify_read(
        bool frame_ok, bool status_fault, uint32_t retries_done,
        bool drdy_reasserted, uint32_t backlog_events,
        int64_t now_us, int64_t t_drdy_us)
{
    /* Beyond current+previous buffering: pairing of conversion data
     * with the dequeued timestamp is no longer guaranteed, whatever the
     * frame contents say. */
    if (backlog_events > PPG_ACQ_MAX_TOLERATED_BACKLOG) {
        return PPG_ACQ_READ_LOST;
    }
    if (frame_ok) {
        /* Device-level continuity break: not recoverable by retry. */
        return status_fault ? PPG_ACQ_READ_LOST : PPG_ACQ_READ_ACCEPT;
    }
    /* Invalid frame: case A only inside all three guards. */
    if (retries_done >= PPG_ACQ_MAX_FRAME_RETRIES) {
        return PPG_ACQ_READ_LOST;
    }
    if (drdy_reasserted) {
        return PPG_ACQ_READ_LOST;
    }
    if ((now_us - t_drdy_us) >= PPG_ACQ_RETRY_DEADLINE_US) {
        return PPG_ACQ_READ_LOST;
    }
    return PPG_ACQ_READ_RETRY;
}

#ifdef __cplusplus
}
#endif

#endif /* PPG_ACQ_POLICY_H */
