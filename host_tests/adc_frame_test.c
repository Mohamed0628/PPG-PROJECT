/*
 * Host tests for the ADS131M02 integration (hardware-independent parts).
 *
 * Covers (project requirement 11):
 *   - 24-bit sign extension: zero, +1, +FS, -FS, -1 boundary coding
 *     (datasheet Table 8-10)
 *   - conversion frame parsing (word placement, byte order, CRC)
 *   - CRC-16/CCITT-FALSE known-answer test
 *   - invalid (corrupted) frame behavior: parser rejects, FIR untouched
 *   - decimation count through the real pipeline wrapper
 *   - timestamp propagation: each 120-SPS output carries the DRDY time
 *     of its triggering (10th) input, raw timing preserved
 *   - FIR reset behavior after a simulated recovery
 *
 * Builds against the real ads131m02_frame.c, ppg_stream.c and the
 * validated fir.c — no mocks of the code under test.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ads131m02_frame.h"
#include "ppg_acq_policy.h"
#include "ppg_stream.h"
#include "fir.h"

static int tests_run = 0;
static int tests_passed = 0;

static void check(bool condition, const char *name)
{
    tests_run++;
    if (condition) { tests_passed++; printf("[PASS] %s\n", name); }
    else           {                 printf("[FAIL] %s\n", name); }
}

/* ------------------------------------------------------------------ */

static void test_sign_extension(void)
{
    check(ads131m02_sign_extend24(0x000000u) == 0,
          "sign-extend 0x000000 -> 0");
    check(ads131m02_sign_extend24(0x000001u) == 1,
          "sign-extend 0x000001 -> +1");
    check(ads131m02_sign_extend24(0x7FFFFFu) == 8388607,
          "sign-extend 0x7FFFFF -> +8388607 (positive full scale)");
    check(ads131m02_sign_extend24(0x800000u) == -8388608,
          "sign-extend 0x800000 -> -8388608 (negative full scale)");
    check(ads131m02_sign_extend24(0xFFFFFFu) == -1,
          "sign-extend 0xFFFFFF -> -1");
    /* Upper garbage bits must be ignored (defensive masking). */
    check(ads131m02_sign_extend24(0xAB800000u) == -8388608,
          "sign-extend masks bits above 23");
}

static void test_crc_known_answer(void)
{
    /* CRC-16/CCITT-FALSE standard check value: "123456789" -> 0x29B1. */
    const uint8_t msg[] = "123456789";
    uint16_t crc = ads131m02_crc16_ccitt(msg, 9);
    printf("       CRC(\"123456789\") = 0x%04X, expected 0x29B1\n", crc);
    check(crc == 0x29B1, "CRC-16/CCITT-FALSE known-answer");
}

/* Build a synthetic 12-byte DOUT frame with a valid device CRC. */
static void make_frame(uint16_t status, uint32_t ch0, uint32_t ch1,
                       uint8_t out[ADS131M02_FRAME_BYTES])
{
    out[0] = (uint8_t)(status >> 8);
    out[1] = (uint8_t)(status & 0xFFu);
    out[2] = 0x00;                              /* 16-bit word, LSB pad */
    out[3] = (uint8_t)(ch0 >> 16);
    out[4] = (uint8_t)(ch0 >> 8);
    out[5] = (uint8_t)(ch0);
    out[6] = (uint8_t)(ch1 >> 16);
    out[7] = (uint8_t)(ch1 >> 8);
    out[8] = (uint8_t)(ch1);
    uint16_t crc = ads131m02_crc16_ccitt(out, 9);
    out[9]  = (uint8_t)(crc >> 8);
    out[10] = (uint8_t)(crc & 0xFFu);
    out[11] = 0x00;                             /* LSB pad             */
}

static void test_frame_parse(void)
{
    uint8_t rx[ADS131M02_FRAME_BYTES];
    ads131m02_frame_t f;

    /* STATUS 0x0500 (defaults, no faults), CH0 = -FS, CH1 = +1. */
    make_frame(0x0500u, 0x800000u, 0x000001u, rx);

    ads131m02_frame_result_t r =
        ads131m02_parse_data_frame(rx, sizeof(rx), &f);

    check(r == ADS131M02_FRAME_OK,          "frame parse: CRC accepted");
    check(f.status == 0x0500u,              "frame parse: STATUS word");
    check(f.ch[0] == -8388608,              "frame parse: CH0 -FS decoded");
    check(f.ch[1] == 1,                     "frame parse: CH1 +1 decoded");
}

static void test_invalid_frame(void)
{
    uint8_t rx[ADS131M02_FRAME_BYTES];
    ads131m02_frame_t f;

    make_frame(0x0500u, 0x123456u, 0x000000u, rx);
    rx[4] ^= 0x01;      /* corrupt one CH0 data bit; CRC must catch it */

    ads131m02_frame_result_t r =
        ads131m02_parse_data_frame(rx, sizeof(rx), &f);
    check(r == ADS131M02_FRAME_ERR_CRC,
          "corrupted frame rejected by output CRC");

    check(ads131m02_parse_data_frame(rx, 5, &f) == ADS131M02_FRAME_ERR_ARG,
          "short buffer rejected");

    /*
     * Case-A recovery semantics: an SPI-failed read that is retried
     * successfully recovers the SAME conversion, which is pushed once
     * with its ORIGINAL DRDY timestamp — the uniform grid and the
     * decimation phase are untouched.  Emulate: samples n=0..9 on the
     * grid; the read of n=9 fails once, the retry succeeds; n=9 is
     * pushed once at its own timestamp.
     */
    ppg_stream_reset();
    ppg_output_t out;
    int outputs = 0;
    for (int n = 0; n < 9; n++) {
        if (ppg_stream_push(1000, (int64_t)n * 833, &out)) outputs++;
    }
    check(outputs == 0, "no premature output before 10 samples");
    /* n=9: first read invalid (NOT pushed), retry recovers the same
     * conversion -> exactly one push, original timestamp. */
    if (ppg_stream_push(1000, (int64_t)9 * 833, &out)) outputs++;
    check(outputs == 1 && out.t_acq_us == 9 * 833,
          "retried (case A) conversion pushed once, original timestamp");
}

static void test_read_policy_classification(void)
{
    /*
     * ppg_acq_classify_read(): the pure decision function used by the
     * acquisition task (case A vs case B, ppg_acq_policy.h).
     */
    const int64_t t0 = 1000000;         /* DRDY timestamp */

    check(ppg_acq_classify_read(true, false, 0, false, 0, t0 + 50, t0)
              == PPG_ACQ_READ_ACCEPT,
          "policy: valid frame -> ACCEPT");
    check(ppg_acq_classify_read(true, false, 0, false, 1, t0 + 50, t0)
              == PPG_ACQ_READ_ACCEPT,
          "policy: backlog of 1 covered by 2-conversion buffer -> ACCEPT");
    check(ppg_acq_classify_read(false, false, 0, false, 0, t0 + 50, t0)
              == PPG_ACQ_READ_RETRY,
          "policy: invalid frame inside all guards -> RETRY same sample");
    check(ppg_acq_classify_read(false, false,
                                PPG_ACQ_MAX_FRAME_RETRIES, false, 0,
                                t0 + 50, t0) == PPG_ACQ_READ_LOST,
          "policy: retries exhausted -> LOST");
    check(ppg_acq_classify_read(false, false, 0, true, 0, t0 + 50, t0)
              == PPG_ACQ_READ_LOST,
          "policy: DRDY re-asserted (next conversion loaded) -> LOST");
    check(ppg_acq_classify_read(false, false, 0, false, 0,
                                t0 + PPG_ACQ_RETRY_DEADLINE_US, t0)
              == PPG_ACQ_READ_LOST,
          "policy: retry window expired -> LOST");
    check(ppg_acq_classify_read(true, true, 0, false, 0, t0 + 50, t0)
              == PPG_ACQ_READ_LOST,
          "policy: STATUS RESET/F_RESYNC -> LOST (no retry can fix timing)");
    check(ppg_acq_classify_read(true, false, 0, false, 2, t0 + 50, t0)
              == PPG_ACQ_READ_LOST,
          "policy: backlog >= 2 breaks timestamp pairing -> LOST");
}

static void test_lost_conversion_halts_stream(void)
{
    /*
     * Case B: a genuinely missing conversion must NOT silently shift
     * the FIR/decimation phase.  Emulate the acquisition behavior:
     * 5 valid samples, one conversion lost -> ppg_stream_halt(); the
     * next physical conversions must be REJECTED (no FIR call, no
     * output ever from the mixed pre/post-gap grid), until the
     * supervisor restart boundary calls ppg_stream_reset(); then the
     * phase restarts cleanly on the new grid (output on the 10th
     * sample of the NEW stream).
     */
    ppg_output_t out;

    ppg_stream_reset();
    for (int n = 0; n < 5; n++) {
        ppg_stream_push(1234, (int64_t)n * 833, &out);
    }

    /* conversion n=5 lost -> acquisition halts the stream */
    ppg_stream_halt();
    check(ppg_stream_is_halted(), "lost conversion: stream halted");

    /* would-be samples n=6.. arrive: all rejected, FIR untouched */
    int outputs = 0;
    for (int n = 6; n < 40; n++) {
        if (ppg_stream_push(1234, (int64_t)n * 833, &out)) outputs++;
    }
    check(outputs == 0,
          "halted stream rejects samples: no output across the gap");

    /* supervisor recovery boundary: reset, clean uniform restart */
    ppg_stream_reset();
    check(!ppg_stream_is_halted(), "reset clears halt");
    int first_output_at = -1;
    for (int n = 0; n < 10; n++) {
        if (ppg_stream_push(0, (int64_t)(100000 + n * 833), &out)) {
            first_output_at = n;
        }
    }
    check(first_output_at == 9,
          "post-recovery phase restarts on new grid (no silent shift)");
}

static void test_decimation_and_timestamps(void)
{
    /*
     * 1200 inputs at t = n * 833 us must give exactly 120 outputs, and
     * output k must carry the timestamp of input n = 10k + 9 (the
     * triggering / 10th sample of each decimation group).  Raw DRDY
     * timing must be preserved bit-exactly (no group-delay shift).
     */
    ppg_stream_reset();

    ppg_output_t out;
    int outputs = 0;
    bool ts_ok = true;

    for (int n = 0; n < 1200; n++) {
        int64_t t = (int64_t)n * 833;
        if (ppg_stream_push(0, t, &out)) {
            int64_t expected = (int64_t)(10 * outputs + 9) * 833;
            if (out.t_acq_us != expected) ts_ok = false;
            outputs++;
        }
    }
    printf("       outputs = %d (expected 120)\n", outputs);
    check(outputs == 120, "pipeline decimation count 1200 -> 120");
    check(ts_ok, "timestamp = DRDY time of triggering input (policy A)");
    check(PPG_STREAM_GROUP_DELAY_US == 41667,
          "exported group delay constant = 41667 us (50/1200 s)");
}

static void test_fir_reset_after_recovery(void)
{
    /*
     * Simulated recovery: run large DC through the stream (delay line
     * saturated with pre-fault data), then ppg_stream_reset() as
     * ppg_acq_start() does, then feed zeros.  Every post-reset output
     * must be exactly 0 — no pre-fault sample may leak through.
     * Also verify the decimation phase restarts: first post-reset
     * output arrives on the 10th input.
     */
    ppg_output_t out;

    ppg_stream_reset();
    for (int n = 0; n < 500; n++) {
        ppg_stream_push(7000000, (int64_t)n * 833, &out);
    }

    ppg_stream_reset();     /* <- recovery boundary */

    bool clean = true;
    int first_output_at = -1;
    int outputs = 0;
    for (int n = 0; n < 300; n++) {
        if (ppg_stream_push(0, (int64_t)n * 833, &out)) {
            if (first_output_at < 0) first_output_at = n;
            if (out.value != 0) clean = false;
            outputs++;
        }
    }
    printf("       first post-reset output at input n = %d "
           "(expected 9)\n", first_output_at);
    check(clean, "no stale pre-recovery samples after reset");
    check(first_output_at == 9, "decimation phase restarts after reset");
    check(outputs == 30, "post-reset output cadence intact");
}

int main(void)
{
    printf("\n========================================\n");
    printf(" ADS131M02 INTEGRATION HOST VALIDATION\n");
    printf("========================================\n\n");

    test_sign_extension();      printf("\n");
    test_crc_known_answer();    printf("\n");
    test_frame_parse();         printf("\n");
    test_invalid_frame();       printf("\n");
    test_read_policy_classification(); printf("\n");
    test_lost_conversion_halts_stream(); printf("\n");
    test_decimation_and_timestamps(); printf("\n");
    test_fir_reset_after_recovery();  printf("\n");

    printf("========================================\n");
    printf(" RESULT: %d / %d tests passed\n", tests_passed, tests_run);
    printf("========================================\n");
    if (tests_passed == tests_run) {
        printf("ADC INTEGRATION VALIDATION PASSED\n\n");
        return 0;
    }
    printf("ADC INTEGRATION VALIDATION FAILED\n\n");
    return 1;
}
