/*
 * Timestamped 1200 -> 120 SPS PPG stream.  See ppg_stream.h for the
 * timestamp policy and the discontinuity (halt) semantics.  Contains no
 * filtering of its own: all filtering and decimation is the validated
 * fir.c module.
 */

#include "ppg_stream.h"
#include "fir.h"

static bool s_halted;

void ppg_stream_reset(void)
{
    s_halted = false;
    fir_init();
}

void ppg_stream_halt(void)
{
    s_halted = true;
}

bool ppg_stream_is_halted(void)
{
    return s_halted;
}

bool ppg_stream_push(int32_t sample, int64_t t_drdy_us, ppg_output_t *out)
{
    int32_t filtered;

    if (s_halted) {
        /* Discontinuous stream: the FIR must not be fed until the
         * acquisition restart boundary calls ppg_stream_reset(). */
        return false;
    }

    if (!fir_process_sample(sample, &filtered)) {
        return false;
    }

    /* Policy A: stamp with the DRDY time of the triggering input.
     * No group-delay subtraction here - see ppg_stream.h. */
    out->value    = filtered;
    out->t_acq_us = t_drdy_us;
    return true;
}
