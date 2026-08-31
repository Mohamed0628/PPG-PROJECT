#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "fir.h"

#define FIR_NUM_TAPS 101
#define FIR_Q 23
#define FIR_DECIMATION_FACTOR 10 

static int32_t sample_buffer[FIR_NUM_TAPS];
static uint32_t write_index;
static uint32_t decimation_counter;
static const int32_t fir_coeffs[FIR_NUM_TAPS] = {
       448,     441,     622,     816,    1003,    1158,    1247,    1228,
      1053,     672,      30,    -926,   -2244,   -3962,   -6102,   -8669,
    -11642,  -14971,  -18572,  -22326,  -26078,  -29633,  -32762,  -35206,
    -36679,  -36880,  -35501,  -32238,  -26804,  -18944,   -8447,    4841,
     21003,   40044,   61882,   86338,  113144,  141936,  172264,  203601,
    235352,  266877,  297502,  326546,  353339,  377242,  397673,  414127,
    426189,  433552,  436028,  433552,  426189,  414127,  397673,  377242,
    353339,  326546,  297502,  266877,  235352,  203601,  172264,  141936,
    113144,   86338,   61882,   40044,   21003,    4841,   -8447,  -18944,
    -26804,  -32238,  -35501,  -36880,  -36679,  -35206,  -32762,  -29633,
    -26078,  -22326,  -18572,  -14971,  -11642,   -8669,   -6102,   -3962,
     -2244,    -926,      30,     672,    1053,    1228,    1247,    1158,
      1003,     816,     622,     441,     448
};
void fir_init(void)
{
    write_index = 0;
    decimation_counter = 0; 

    for (int i = 0; i < FIR_NUM_TAPS; i++) {
        sample_buffer[i] = 0;
    }
}



bool fir_process_sample(int32_t adc_sample, int32_t *out)
{
    sample_buffer[write_index] = adc_sample;

    write_index++;

    if (write_index == FIR_NUM_TAPS) {
        write_index = 0;
    }

    decimation_counter++;

    if (decimation_counter != FIR_DECIMATION_FACTOR) {
        return false;
    }

    decimation_counter = 0;

    int32_t sample_index = (int32_t)write_index - 1;

    if (sample_index < 0) {
        sample_index = FIR_NUM_TAPS - 1;
    }

    int64_t accumulator = 0;

    for (int k = 0; k < FIR_NUM_TAPS; k++) {

        accumulator +=
            (int64_t)sample_buffer[sample_index] * fir_coeffs[k];

        sample_index--;

        if (sample_index < 0) {
            sample_index = FIR_NUM_TAPS - 1;
        }
    }

    const int64_t half = (1LL << (FIR_Q - 1));
    const int64_t scale = (1LL << FIR_Q);

    if (accumulator >= 0) {
    *out = (int32_t)((accumulator + half) / scale);
    } 
    else {
    *out = (int32_t)((accumulator - half) / scale);
    }
    return true;

}

uint32_t fir_group_delay_samples(void)
{
    return ((FIR_NUM_TAPS - 1) / 2) / FIR_DECIMATION_FACTOR;
}
