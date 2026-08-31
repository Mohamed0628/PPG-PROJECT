#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "fir.h"

#define INPUT_FS       1200
#define OUTPUT_FS       120
#define DECIMATION       10

#define TEST_AMPLITUDE 1000000
#define ADC_MAX        8388607

#define PI 3.14159265358979323846

static int tests_run = 0;
static int tests_passed = 0;


/* ----------------------------------------------------------
 * PASS / FAIL helper
 * ---------------------------------------------------------- */

static void check(bool condition, const char *name)
{
    tests_run++;

    if (condition) {
        tests_passed++;
        printf("[PASS] %s\n", name);
    }
    else {
        printf("[FAIL] %s\n", name);
    }
}


/* ----------------------------------------------------------
 * Measure AC amplitude at FIR output for a sine-wave input.
 *
 * Returns gain in dB:
 *
 *          output RMS
 * 20 log10(------------)
 *          input RMS
 *
 * We throw away startup samples so FIR filling does not
 * contaminate the measurement.
 * ---------------------------------------------------------- */

static double measure_tone_gain_db(double frequency_hz)
{
    const int total_input_samples = 26400;
    const int warmup_input_samples = 2400;

    double sum = 0.0;
    double sum_sq = 0.0;
    int output_count = 0;

    int32_t out = 0;

    fir_init();

    for (int n = 0; n < total_input_samples; n++) {

        /*
         * Non-zero starting phase avoids accidentally sampling
         * a sine exactly at a zero crossing during decimation.
         */
        double phase =
            2.0 * PI * frequency_hz * (double)n / INPUT_FS
            + 0.371;

        int32_t input =
            (int32_t)llround(TEST_AMPLITUDE * sin(phase));

        if (fir_process_sample(input, &out)) {

            if (n >= warmup_input_samples) {
                sum += (double)out;
                sum_sq += (double)out * (double)out;
                output_count++;
            }
        }
    }

    double mean = sum / output_count;

    /*
     * Remove any DC offset before calculating AC RMS.
     */
    double variance =
        (sum_sq / output_count) - (mean * mean);

    if (variance < 0.0) {
        variance = 0.0;
    }

    double output_rms = sqrt(variance);
    double input_rms = TEST_AMPLITUDE / sqrt(2.0);

    if (output_rms == 0.0) {
        return -300.0;
    }

    return 20.0 * log10(output_rms / input_rms);
}


/* ----------------------------------------------------------
 * Test 1
 *
 * 1200 input samples should produce exactly 120 outputs.
 * ---------------------------------------------------------- */

static void test_decimation_rate(void)
{
    fir_init();

    int32_t out;
    int output_count = 0;

    for (int n = 0; n < 1200; n++) {

        if (fir_process_sample(0, &out)) {
            output_count++;
        }
    }

    printf("       outputs = %d, expected = 120\n",
           output_count);

    check(output_count == 120,
          "1200 SPS -> 120 SPS decimation");
}


/* ----------------------------------------------------------
 * Test 2
 *
 * 101 taps:
 *
 * group delay = (101 - 1) / 2 = 50 input samples
 *
 * 50 / 10 = 5 output samples
 * ---------------------------------------------------------- */

static void test_group_delay(void)
{
    uint32_t delay = fir_group_delay_samples();

    printf("       group delay = %u output samples\n",
           delay);

    check(delay == 5,
          "FIR group delay is 5 output samples");
}


/* ----------------------------------------------------------
 * Test 3
 *
 * FIR means FINITE impulse response.
 *
 * After the impulse has completely passed through the
 * 101-sample buffer, the output must return to exactly zero.
 * ---------------------------------------------------------- */

static void test_impulse_finite(void)
{
    fir_init();

    int32_t out = 0;
    bool tail_is_zero = true;

    for (int n = 0; n < 1000; n++) {

        int32_t input =
            (n == 0) ? TEST_AMPLITUDE : 0;

        if (fir_process_sample(input, &out)) {

            /*
             * By n = 200 the 101-tap impulse response
             * absolutely must be gone.
             */
            if ((n > 200) && (out != 0)) {
                tail_is_zero = false;
            }
        }
    }

    check(tail_is_zero,
          "Impulse response returns to zero");
}


/* ----------------------------------------------------------
 * Test 4
 *
 * DC gain test.
 *
 * From our Q23 coefficient set:
 *
 * 1,000,000 input -> approximately 999,355 output
 * ---------------------------------------------------------- */

static void test_dc_gain(void)
{
    fir_init();

    int32_t out = 0;

    for (int n = 0; n < 2000; n++) {

        fir_process_sample(TEST_AMPLITUDE, &out);
    }

    printf("       settled output = %d\n", out);
    printf("       expected       ~= 999355\n");

    check(llabs((long long)out - 999355LL) <= 2,
          "DC gain / Q23 scaling");
}


/* ----------------------------------------------------------
 * Test 5
 *
 * PPG heart-rate region.
 *
 * 1.2 Hz should pass almost unchanged.
 * ---------------------------------------------------------- */

static void test_ppg_passband(void)
{
    double gain_db = measure_tone_gain_db(1.2);

    printf("       1.2 Hz gain = %.4f dB\n",
           gain_db);

    check(fabs(gain_db) < 0.02,
          "1.2 Hz PPG signal preserved");
}


/* ----------------------------------------------------------
 * Test 6
 *
 * High edge of intended PPG passband.
 * ---------------------------------------------------------- */

static void test_passband_edge(void)
{
    double gain_db = measure_tone_gain_db(8.0);

    printf("       8 Hz gain = %.4f dB\n",
           gain_db);

    check(fabs(gain_db) < 0.02,
          "8 Hz passband edge preserved");
}


/* ----------------------------------------------------------
 * Test 7
 *
 * Stopband begins at approximately 60 Hz.
 *
 * Use 61 Hz rather than exactly 60 Hz to avoid the special
 * Nyquist sampling case at the 120 SPS output.
 * ---------------------------------------------------------- */

static void test_stopband(void)
{
    double gain_db = measure_tone_gain_db(61.0);

    printf("       61 Hz gain = %.2f dB\n",
           gain_db);

    check(gain_db <= -80.0,
          "Stopband attenuation exceeds 80 dB");
}


/* ----------------------------------------------------------
 * Test 8
 *
 * Critical alias test.
 *
 * After downsampling:
 *
 * 115 Hz -> |120 - 115| = 5 Hz
 *
 * Without anti-alias filtering this would look like a
 * legitimate 5 Hz signal.
 * ---------------------------------------------------------- */

static void test_115_hz_alias(void)
{
    double gain_db = measure_tone_gain_db(115.0);

    printf("       115 Hz gain = %.2f dB\n",
           gain_db);
    printf("       would alias -> 5 Hz\n");

    check(gain_db <= -80.0,
          "115 Hz prevented from aliasing into 5 Hz");
}


/* ----------------------------------------------------------
 * Test 9
 *
 * 119 Hz -> 1 Hz after decimation.
 *
 * This one is especially nasty because 1 Hz sits directly
 * inside the heart-rate band.
 * ---------------------------------------------------------- */

static void test_119_hz_alias(void)
{
    double gain_db = measure_tone_gain_db(119.0);

    printf("       119 Hz gain = %.2f dB\n",
           gain_db);
    printf("       would alias -> 1 Hz\n");

    check(gain_db <= -80.0,
          "119 Hz prevented from aliasing into 1 Hz");
}


/* ----------------------------------------------------------
 * Test 10
 *
 * Exact 120 Hz is special.
 *
 * 120 Hz becomes DC when sampled at 120 SPS, so ordinary
 * AC RMS measurement is not appropriate.
 *
 * Instead check the residual settled magnitude directly.
 * ---------------------------------------------------------- */

static void test_120_hz_flicker(void)
{
    fir_init();

    int32_t out = 0;
    int32_t max_residual = 0;

    for (int n = 0; n < 12000; n++) {

        double phase =
            2.0 * PI * 120.0 * (double)n / INPUT_FS
            + 0.371;

        int32_t input =
            (int32_t)llround(TEST_AMPLITUDE * sin(phase));

        if (fir_process_sample(input, &out)) {

            if (n > 1000) {

                int32_t magnitude =
                    (out < 0) ? -out : out;

                if (magnitude > max_residual) {
                    max_residual = magnitude;
                }
            }
        }
    }

    printf("       120 Hz max residual = %d counts\n",
           max_residual);

    /*
     * 100 counts relative to a 1,000,000-count input
     * already represents 80 dB suppression.
     */
    check(max_residual <= 100,
          "120 Hz optical flicker suppressed >80 dB");
}


/* ----------------------------------------------------------
 * Test 11
 *
 * Exercise the MAC with full-scale 24-bit ADC values.
 *
 * Compile with UndefinedBehaviorSanitizer to catch signed
 * overflow or other undefined behavior.
 * ---------------------------------------------------------- */

static void test_full_scale_stress(void)
{
    fir_init();

    int32_t out = 0;
    bool output_in_range = true;

    for (int n = 0; n < 5000; n++) {

        int32_t input =
            (n & 1) ? ADC_MAX : -ADC_MAX;

        if (fir_process_sample(input, &out)) {

            if ((out > ADC_MAX) ||
                (out < -ADC_MAX)) {

                output_in_range = false;
            }
        }
    }

    check(output_in_range,
          "Full-scale ADC stress test");
}


/* ----------------------------------------------------------
 * MAIN
 * ---------------------------------------------------------- */

int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf(" PPG FIR HOST VALIDATION\n");
    printf("========================================\n\n");

    test_decimation_rate();
    printf("\n");

    test_group_delay();
    printf("\n");

    test_impulse_finite();
    printf("\n");

    test_dc_gain();
    printf("\n");

    test_ppg_passband();
    printf("\n");

    test_passband_edge();
    printf("\n");

    test_stopband();
    printf("\n");

    test_115_hz_alias();
    printf("\n");

    test_119_hz_alias();
    printf("\n");

    test_120_hz_flicker();
    printf("\n");

    test_full_scale_stress();
    printf("\n");

    printf("========================================\n");
    printf(" RESULT: %d / %d tests passed\n",
           tests_passed,
           tests_run);
    printf("========================================\n");

    if (tests_passed == tests_run) {
        printf("FIR VALIDATION PASSED\n\n");
        return 0;
    }

    printf("FIR VALIDATION FAILED\n\n");
    return 1;
}
