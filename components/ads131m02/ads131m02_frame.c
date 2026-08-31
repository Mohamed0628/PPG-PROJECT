/*
 * ADS131M02 wire-format layer (hardware independent).
 * See ads131m02_frame.h for datasheet references.
 */

#include <string.h>
#include "ads131m02_frame.h"

uint16_t ads131m02_crc16_ccitt(const uint8_t *data, size_t len)
{
    /* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF (datasheet 8.3.12). */
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

ads131m02_frame_result_t ads131m02_parse_data_frame(
    const uint8_t *rx,
    size_t         len,
    ads131m02_frame_t *frame)
{
    if ((rx == NULL) || (frame == NULL) ||
        (len != ADS131M02_FRAME_BYTES)) {
        return ADS131M02_FRAME_ERR_ARG;
    }

    /* Word 0: response/STATUS.  Commands/responses are 16 bits of real
     * data, MSB-aligned in the 24-bit word (datasheet 8.5.1.8). */
    frame->status = (uint16_t)(ads131m02_word24(&rx[0]) >> 8);

    /* Words 1..2: CH0 / CH1 conversion data, 24-bit two's complement. */
    frame->ch[0] = ads131m02_sign_extend24(ads131m02_word24(&rx[3]));
    frame->ch[1] = ads131m02_sign_extend24(ads131m02_word24(&rx[6]));

    /* Word 3: output CRC, 16 bits MSB-aligned.  Covers words 0..2
     * including pad bits (datasheet 8.3.12). */
    frame->crc_rx   = (uint16_t)(ads131m02_word24(&rx[9]) >> 8);
    frame->crc_calc = ads131m02_crc16_ccitt(rx, 9);

    if (frame->crc_rx != frame->crc_calc) {
        return ADS131M02_FRAME_ERR_CRC;
    }

    return ADS131M02_FRAME_OK;
}

void ads131m02_build_cmd_frame(uint16_t cmd,
                               uint16_t extra_word,
                               bool     has_extra,
                               uint8_t  tx[ADS131M02_FRAME_BYTES])
{
    memset(tx, 0, ADS131M02_FRAME_BYTES);

    /* 16-bit command MSB-aligned into 24-bit word 0 (datasheet 8.5.1.8). */
    tx[0] = (uint8_t)(cmd >> 8);
    tx[1] = (uint8_t)(cmd & 0xFFu);
    tx[2] = 0x00;

    if (has_extra) {
        /* Register data for WREG, MSB-aligned into word 1
         * (datasheet 8.5.1.10.8: "Write the intended contents of each
         * register into individual words, MSB aligned"). */
        tx[3] = (uint8_t)(extra_word >> 8);
        tx[4] = (uint8_t)(extra_word & 0xFFu);
        tx[5] = 0x00;
    }
}
