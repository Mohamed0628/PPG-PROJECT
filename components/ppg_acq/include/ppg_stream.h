#ifndef PPG_STREAM_H
#define PPG_STREAM_H

/*
 * Timestamped 1200 SPS -> 120 SPS PPG stream.
 *
 * Thin, hardware-independent wrapper around the VALIDATED fir.c module.
 * It adds exactly one thing: a defined timestamp for every decimated
 * output.  It does not filter, does not decimate on its own (decimation
 * belongs to fir_process_sample()), and never modifies FIR behavior.
 *
 * ========================= TIMESTAMP POLICY =========================
 *
 * Policy A ("acquisition-time"): the timestamp attached to each 120-SPS
 * output is the DRDY timestamp of the 1200-SPS input sample that
 * TRIGGERED that output (i.e. the newest sample in the FIR window, the
 * 10th input of the decimation phase).  Raw ADC timing is therefore
 * preserved exactly — nothing is subtracted.
 *
 * The FIR is linear phase with a group delay of exactly
 *
 *     50 input samples @ 1200 SPS = 41666.667 us = 5 output samples,
 *
 * so the physical-signal time an output value represents is
 *
 *     t_signal = t_out - PPG_STREAM_GROUP_DELAY_US.
 *
 * Downstream IMU/NLMS alignment must apply that constant explicitly
 * (it is exported below) rather than this module silently shifting
 * timestamps.  Rationale: the IMU path will have its own (different)
 * filter delays; keeping every stream stamped at acquisition time with
 * published per-stream group delays lets the fusion stage compensate
 * each path once, correctly, in one place.
 * ====================================================================
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exact value is 41666.667 us; integer microseconds are kept and the
 * 1/3 us truncation (0.04% of one output period) is negligible against
 * ISR jitter.  50 samples / 1200 Hz. */
#define PPG_STREAM_GROUP_DELAY_US   41667ll

typedef struct {
    int32_t value;          /* filtered 120-SPS PPG sample                */
    int64_t t_acq_us;       /* DRDY time of the triggering input sample   */
} ppg_output_t;

/*
 * Reset stream state AND the FIR delay line (calls fir_init()), and
 * clear any halt.  Must be called at every acquisition (re)start —
 * startup, RECOVERY, reacquisition — so the delay line never carries
 * pre-fault samples.  Must NOT be called during normal RUN operation.
 */
void ppg_stream_reset(void);

/*
 * Mark the stream DISCONTINUOUS (a physical 1200-SPS conversion was
 * lost — see ppg_acq_policy.h case B).  From this point every
 * ppg_stream_push() is rejected without touching the FIR: the FIR
 * requires a uniformly sampled input sequence, so continuing across a
 * missing conversion would compress the time axis and shift the
 * decimation phase off the ADC sample grid.  Only ppg_stream_reset()
 * (at the acquisition restart boundary) clears the halt.
 */
void ppg_stream_halt(void);
bool ppg_stream_is_halted(void);

/*
 * Push one valid 1200-SPS sample with its DRDY timestamp.
 * Returns true when a 120-SPS output was produced (exactly once per
 * 10 inputs, driven by fir_process_sample()).  Returns false and does
 * NOT invoke the FIR while the stream is halted.
 *
 * Every sample pushed here must sit on the uniform 1200-SPS ADC
 * conversion grid.  A retried SPI read of the SAME conversion (case A
 * in ppg_acq_policy.h) is pushed once, with its original DRDY
 * timestamp.  A lost conversion (case B) must halt the stream instead
 * of being skipped.
 */
bool ppg_stream_push(int32_t sample, int64_t t_drdy_us, ppg_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PPG_STREAM_H */
