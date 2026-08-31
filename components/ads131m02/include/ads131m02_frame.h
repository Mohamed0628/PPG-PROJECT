#ifndef ADS131M02_FRAME_H
#define ADS131M02_FRAME_H

/*
 * ADS131M02 wire-format layer.
 *
 * This file is deliberately free of any ESP-IDF / hardware dependency so
 * that frame parsing, sign extension and CRC checking can be validated in
 * the host test suite (see host_tests/adc_frame_test.c).
 *
 * All wire-format facts below are derived from the ADS131M02 datasheet
 * SBAS853A (Jan 2020, rev Apr 2021):
 *
 *  - SPI frame structure:            Section 8.5.1.7, Figure 8-18.
 *    With the default 24-bit word length (MODE.WLENGTH = 01b, Section
 *    8.6.3) a full frame on the two-channel ADS131M02 is FOUR 24-bit
 *    words: [response/STATUS] [CH0 data] [CH1 data] [output CRC].
 *    "Four words are required to complete a frame on the ADS131M02."
 *    (Section 8.4.1.3.)
 *  - Conversion data coding:         Section 8.5.1.9, Table 8-10.
 *    24-bit binary two's complement, MSB first on DOUT (Figure 6-1),
 *    +FS = 7FFFFFh, -FS = 800000h, 0 = 000000h.
 *  - Output CRC:                     Section 8.3.12, Table 8-7.
 *    Always present as the last output word; 16-bit, MSB-aligned in the
 *    24-bit word; default type CCITT (x^16+x^12+x^5+1, poly 0x1021),
 *    seed FFFFh, computed over all preceding words of the frame
 *    including pad bits.
 *  - Commands:                       Section 8.5.1.10, Table 8-11.
 *  - STATUS register bits:           Section 8.6.2, Table 8-15.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frame geometry (24-bit word length, 2-channel device) ---------- */

#define ADS131M02_WORD_BYTES        3u              /* WLENGTH = 24 bit  */
#define ADS131M02_FRAME_WORDS       4u              /* resp,ch0,ch1,crc  */
#define ADS131M02_FRAME_BYTES       (ADS131M02_FRAME_WORDS * \
                                     ADS131M02_WORD_BYTES)   /* = 12    */

/* ---- SPI commands (datasheet Table 8-11) ---------------------------- */

#define ADS131M02_CMD_NULL          0x0000u
#define ADS131M02_CMD_RESET         0x0011u
#define ADS131M02_CMD_STANDBY       0x0022u
#define ADS131M02_CMD_WAKEUP        0x0033u
#define ADS131M02_RSP_RESET_OK      0xFF22u         /* Section 8.5.1.10.2 */

/* RREG = 101a aaaa annn nnnn, WREG = 011a aaaa annn nnnn (Table 8-11). */
#define ADS131M02_CMD_RREG(addr, n) \
    (uint16_t)(0xA000u | (((uint16_t)(addr) & 0x3Fu) << 7) | ((n) & 0x7Fu))
#define ADS131M02_CMD_WREG(addr, n) \
    (uint16_t)(0x6000u | (((uint16_t)(addr) & 0x3Fu) << 7) | ((n) & 0x7Fu))
/* WREG acknowledgement: 010a aaaa ammm mmmm (Section 8.5.1.10.8). */
#define ADS131M02_ACK_WREG(addr, m) \
    (uint16_t)(0x4000u | (((uint16_t)(addr) & 0x3Fu) << 7) | ((m) & 0x7Fu))

/* ---- Register addresses (datasheet Table 8-12) ---------------------- */

#define ADS131M02_REG_ID            0x00u
#define ADS131M02_REG_STATUS        0x01u
#define ADS131M02_REG_MODE          0x02u
#define ADS131M02_REG_CLOCK         0x03u
#define ADS131M02_REG_GAIN1         0x04u
#define ADS131M02_REG_CFG           0x06u
#define ADS131M02_REG_THRSHLD_LSB   0x08u
#define ADS131M02_REG_CH0_CFG       0x09u
#define ADS131M02_REG_CH1_CFG       0x0Eu

/* STATUS register bits used for error detection (Table 8-15). */
#define ADS131M02_STATUS_LOCK       (1u << 15)
#define ADS131M02_STATUS_F_RESYNC   (1u << 14)
#define ADS131M02_STATUS_REG_MAP    (1u << 13)
#define ADS131M02_STATUS_CRC_ERR    (1u << 12)
#define ADS131M02_STATUS_RESET      (1u << 10)
#define ADS131M02_STATUS_DRDY1      (1u << 1)
#define ADS131M02_STATUS_DRDY0      (1u << 0)

/* ---- Decoded data frame --------------------------------------------- */

typedef enum {
    ADS131M02_FRAME_OK = 0,
    ADS131M02_FRAME_ERR_CRC,        /* output CRC mismatch               */
    ADS131M02_FRAME_ERR_ARG,        /* NULL pointer / bad length         */
} ads131m02_frame_result_t;

typedef struct {
    uint16_t status;                /* first output word (STATUS after a
                                       NULL command, Section 8.5.1.10.1) */
    int32_t  ch[2];                 /* sign-extended conversion results  */
    uint16_t crc_rx;                /* CRC received in the frame         */
    uint16_t crc_calc;              /* CRC recomputed by the host        */
} ads131m02_frame_t;

/*
 * Sign-extend a 24-bit two's-complement conversion word into int32_t.
 * Coding per datasheet Section 8.5.1.9 / Table 8-10:
 *   0x000000 ->  0
 *   0x000001 -> +1
 *   0x7FFFFF -> +8388607  (positive full scale)
 *   0x800000 -> -8388608  (negative full scale)
 *   0xFFFFFF -> -1
 * Implemented with the portable xor/subtract idiom (no implementation-
 * defined right shift of negative values).
 */
static inline int32_t ads131m02_sign_extend24(uint32_t raw24)
{
    raw24 &= 0x00FFFFFFu;
    return (int32_t)(raw24 ^ 0x00800000u) - (int32_t)0x00800000;
}

/*
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final
 * XOR.  Matches the device default output CRC (Section 8.3.12, Table 8-7,
 * "The seed value of the CRC calculation is FFFFh").
 */
uint16_t ads131m02_crc16_ccitt(const uint8_t *data, size_t len);

/*
 * Extract one MSB-first 24-bit word starting at 'bytes'.
 */
static inline uint32_t ads131m02_word24(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 16) |
           ((uint32_t)bytes[1] << 8)  |
            (uint32_t)bytes[2];
}

/*
 * Parse and validate one 12-byte DOUT frame (NULL-command response):
 * word 0 = STATUS, word 1 = CH0, word 2 = CH1, word 3 = CRC (MSB-aligned
 * 16 bits, LSB zero-padded to 24 bits, Section 8.5.1.8).
 *
 * The output CRC covers the first three words (9 bytes) including pad
 * bits (Section 8.3.12).  On CRC mismatch the frame contents are still
 * filled in (for diagnostics) but the caller MUST NOT use ch[].
 */
ads131m02_frame_result_t ads131m02_parse_data_frame(
    const uint8_t *rx,
    size_t         len,
    ads131m02_frame_t *frame);

/*
 * Build a 12-byte DIN frame containing a single 16-bit command word,
 * MSB-aligned into the first 24-bit word, remaining words zero
 * (Sections 8.5.1.7 / 8.5.1.8).  'extra_word' is written into word 1
 * when nonzero (used for single-register WREG data).
 */
void ads131m02_build_cmd_frame(uint16_t cmd,
                               uint16_t extra_word,
                               bool     has_extra,
                               uint8_t  tx[ADS131M02_FRAME_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* ADS131M02_FRAME_H */
