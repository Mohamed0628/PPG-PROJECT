/*
 * ESP-IDF driver for the TI ADS131M02.  Datasheet references: SBAS853A.
 * See ads131m02.h for the operating point and scope notes.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include "ads131m02.h"

static const char *TAG = "ads131m02";

/* ---- Low-level full-duplex frame exchange ---------------------------- */

/*
 * Every SPI transaction is one full fixed-length frame
 * (ADS131M02_FRAME_BYTES = 12 bytes at the 24-bit word length): the
 * device requires complete frames — "Terminating the frame early causes
 * the RESET command to be ignored.  Four words are required to complete
 * a frame on the ADS131M02." (Section 8.4.1.3) — and reading every
 * output word each period keeps DRDY behavior predictable
 * (Section 8.5.1.11).
 *
 * Polling transactions are used deliberately: a 12-byte frame at 8 MHz
 * is ~12 us on the wire; interrupt-driven transactions would add more
 * scheduling latency than they save (see report section D).
 */
static esp_err_t xfer_frame(ads131m02_t *dev,
                            const uint8_t tx[ADS131M02_FRAME_BYTES],
                            uint8_t rx[ADS131M02_FRAME_BYTES])
{
    spi_transaction_t t = {
        .length    = ADS131M02_FRAME_BYTES * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(dev->spi, &t);
}

/* Send a command frame; the response arrives in the NEXT frame
 * (Section 8.5.1.7: the first output word "always begins with the
 * response to the command that was written on the previous input
 * frame").  Returns the raw rx of THIS frame in 'rx' if non-NULL. */
static esp_err_t send_cmd(ads131m02_t *dev, uint16_t cmd,
                          uint16_t extra, bool has_extra,
                          uint8_t rx[ADS131M02_FRAME_BYTES])
{
    uint8_t tx[ADS131M02_FRAME_BYTES];
    uint8_t rx_local[ADS131M02_FRAME_BYTES];

    ads131m02_build_cmd_frame(cmd, extra, has_extra, tx);
    return xfer_frame(dev, tx, rx ? rx : rx_local);
}

/* ---- Register access -------------------------------------------------- */

esp_err_t ads131m02_read_reg(ads131m02_t *dev, uint8_t addr, uint16_t *val)
{
    uint8_t rx[ADS131M02_FRAME_BYTES];

    ESP_RETURN_ON_FALSE(dev && val, ESP_ERR_INVALID_ARG, TAG, "args");

    /* Frame N: RREG command.  Frame N+1: first output word = register
     * contents (Section 8.5.1.10.7.1, Figure 8-23). */
    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_RREG(addr, 0),
                                 0, false, NULL), TAG, "rreg tx");
    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_NULL,
                                 0, false, rx), TAG, "rreg rx");

    *val = (uint16_t)(ads131m02_word24(&rx[0]) >> 8);
    return ESP_OK;
}

esp_err_t ads131m02_write_reg(ads131m02_t *dev, uint8_t addr, uint16_t val)
{
    uint8_t rx[ADS131M02_FRAME_BYTES];
    uint16_t ack;

    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "args");

    /* Frame N: WREG command + register data (Section 8.5.1.10.8).
     * Frame N+1: acknowledgement 010a aaaa a000 0000. */
    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_WREG(addr, 0),
                                 val, true, NULL), TAG, "wreg tx");
    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_NULL,
                                 0, false, rx), TAG, "wreg ack");

    ack = (uint16_t)(ads131m02_word24(&rx[0]) >> 8);
    if (ack != ADS131M02_ACK_WREG(addr, 0)) {
        ESP_LOGE(TAG, "WREG ack mismatch addr=0x%02x ack=0x%04x", addr, ack);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t ads131m02_write_reg_verified(ads131m02_t *dev, uint8_t addr,
                                       uint16_t val)
{
    uint16_t rb = 0;

    ESP_RETURN_ON_ERROR(ads131m02_write_reg(dev, addr, val), TAG, "write");
    ESP_RETURN_ON_ERROR(ads131m02_read_reg(dev, addr, &rb), TAG, "readback");

    if (rb != val) {
        ESP_LOGE(TAG, "readback mismatch addr=0x%02x wrote=0x%04x "
                 "read=0x%04x", addr, val, rb);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

/* ---- Reset / startup -------------------------------------------------- */

static esp_err_t reset_device(ads131m02_t *dev)
{
    if (dev->cfg.pin_sync_reset >= 0) {
        /* Hardware reset: hold SYNC/RESET low >= t_w(RSL) = 2048 t_CLKIN
         * (Section 6.6).  At 2.4576 MHz that is 833 us; use 2 ms margin.
         * Then wait >= t_REGACQ = 5 us before communicating
         * (Section 8.4.1.2). */
        gpio_set_level(dev->cfg.pin_sync_reset, 0);
        vTaskDelay(pdMS_TO_TICKS(3));
        gpio_set_level(dev->cfg.pin_sync_reset, 1);
        esp_rom_delay_us(10);
        return ESP_OK;
    }

    /* SPI RESET command (0x0011).  A full frame is required; the reset
     * latches at frame end; wait t_REGACQ afterwards; the NEXT frame's
     * response is 0xFF22 on success (Sections 8.4.1.3, 8.5.1.10.2). */
    uint8_t rx[ADS131M02_FRAME_BYTES];
    uint16_t rsp;

    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_RESET, 0, false, NULL),
                        TAG, "reset tx");
    esp_rom_delay_us(10);
    ESP_RETURN_ON_ERROR(send_cmd(dev, ADS131M02_CMD_NULL, 0, false, rx),
                        TAG, "reset rsp");

    rsp = (uint16_t)(ads131m02_word24(&rx[0]) >> 8);
    if (rsp != ADS131M02_RSP_RESET_OK) {
        ESP_LOGE(TAG, "RESET response 0x%04x (expected 0xFF22)", rsp);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t ads131m02_sync_pulse(ads131m02_t *dev)
{
    if (dev->cfg.pin_sync_reset < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* Synchronization pulse: 1 <= width < 2048 t_CLKIN (Section 6.6,
     * t_w(SYL)).  At 2.4576 MHz, 2048 t_CLKIN = 833 us; 10 us is safely
     * inside the window and > 1 t_CLKIN (407 ns). */
    gpio_set_level(dev->cfg.pin_sync_reset, 0);
    esp_rom_delay_us(10);
    gpio_set_level(dev->cfg.pin_sync_reset, 1);
    return ESP_OK;
}

esp_err_t ads131m02_flush_fifo(ads131m02_t *dev)
{
    /* Read two frames in quick succession to drain the 2-deep FIFO
     * (Section 8.5.1.9.1, Figure 8-21). */
    ads131m02_frame_t f;
    esp_err_t e1 = ads131m02_read_frame(dev, &f);
    esp_err_t e2 = ads131m02_read_frame(dev, &f);
    return (e1 != ESP_OK) ? e1 : e2;
}

esp_err_t ads131m02_read_frame(ads131m02_t *dev, ads131m02_frame_t *frame)
{
    uint8_t tx[ADS131M02_FRAME_BYTES];
    uint8_t rx[ADS131M02_FRAME_BYTES];

    ESP_RETURN_ON_FALSE(dev && frame, ESP_ERR_INVALID_ARG, TAG, "args");

    ads131m02_build_cmd_frame(ADS131M02_CMD_NULL, 0, false, tx);
    ESP_RETURN_ON_ERROR(xfer_frame(dev, tx, rx), TAG, "spi");

    switch (ads131m02_parse_data_frame(rx, sizeof(rx), frame)) {
    case ADS131M02_FRAME_OK:
        return ESP_OK;
    case ADS131M02_FRAME_ERR_CRC:
        return ESP_ERR_INVALID_CRC;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t ads131m02_verify_config(ads131m02_t *dev)
{
    static const struct { uint8_t addr; uint16_t val; } expected[] = {
        { ADS131M02_REG_MODE,    ADS131M02_MODE_VAL    },
        { ADS131M02_REG_CLOCK,   ADS131M02_CLOCK_VAL   },
        { ADS131M02_REG_GAIN1,   ADS131M02_GAIN1_VAL   },
        { ADS131M02_REG_CH0_CFG, ADS131M02_CH0_CFG_VAL },
    };

    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        uint16_t rb = 0;
        ESP_RETURN_ON_ERROR(ads131m02_read_reg(dev, expected[i].addr, &rb),
                            TAG, "verify read");
        if (rb != expected[i].val) {
            ESP_LOGE(TAG, "config drift addr=0x%02x expect=0x%04x "
                     "read=0x%04x", expected[i].addr, expected[i].val, rb);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    return ESP_OK;
}

esp_err_t ads131m02_init(ads131m02_t *dev, const ads131m02_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(dev && cfg, ESP_ERR_INVALID_ARG, TAG, "args");

    memset(dev, 0, sizeof(*dev));
    dev->cfg = *cfg;

    /* SYNC/RESET GPIO (if wired): idle high. */
    if (cfg->pin_sync_reset >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pin_sync_reset,
            .mode         = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "sync gpio");
        gpio_set_level(cfg->pin_sync_reset, 1);
    }

    /* DRDY as plain input here; the acquisition layer attaches the ISR. */
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pin_drdy,
            .mode         = GPIO_MODE_INPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "drdy gpio");
    }

    /* SPI bus + device.  Mode 1 (CPOL=0, CPHA=1) per Figure 6-1. */
    {
        spi_bus_config_t bus = {
            .sclk_io_num = cfg->pin_sclk,
            .mosi_io_num = cfg->pin_mosi,
            .miso_io_num = cfg->pin_miso,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = ADS131M02_FRAME_BYTES,
        };
        esp_err_t err = spi_bus_initialize(cfg->spi_host, &bus,
                                           SPI_DMA_DISABLED);
        if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
            /* INVALID_STATE = bus already initialized by the app: fine. */
            return err;
        }

        spi_device_interface_config_t devcfg = {
            .clock_speed_hz = cfg->spi_clock_hz,
            .mode           = 1,                 /* CPOL=0, CPHA=1 */
            .spics_io_num   = cfg->pin_cs,
            .queue_size     = 2,
            /* CS must frame the whole transaction and transition while
             * SCLK is low (Figure 6-1); the IDF driver guarantees this
             * for a hardware-managed CS with mode 1. */
        };
        ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg->spi_host, &devcfg,
                                               &dev->spi), TAG, "add dev");
    }

    /*
     * Startup (Section 8.4.1 / 8.4.2):
     *  - After POR the DRDY low->high transition (t_POR, typ 250 us from
     *    supplies at 90%) marks SPI readiness; SPI before that is
     *    ignored.  A conservative fixed delay is used since supply-ramp
     *    timing relative to boot is board-dependent.
     *  - CLKIN (external 2.4576 MHz oscillator) is assumed running;
     *    conversions free-run once it toggles.
     */
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(reset_device(dev), TAG, "reset");

    /* ID check: 22xxh, CHANCNT=2 (Section 8.6.1). */
    {
        uint16_t id = 0;
        ESP_RETURN_ON_ERROR(ads131m02_read_reg(dev, ADS131M02_REG_ID, &id),
                            TAG, "id read");
        if ((id >> 8) != ADS131M02_ID_MSB) {
            ESP_LOGE(TAG, "bad ID 0x%04x (ADC not responding?)", id);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    /* Operating-point configuration, each write readback-verified. */
    ESP_RETURN_ON_ERROR(ads131m02_write_reg_verified(dev,
                        ADS131M02_REG_MODE,    ADS131M02_MODE_VAL),
                        TAG, "MODE");
    ESP_RETURN_ON_ERROR(ads131m02_write_reg_verified(dev,
                        ADS131M02_REG_CLOCK,   ADS131M02_CLOCK_VAL),
                        TAG, "CLOCK");
    ESP_RETURN_ON_ERROR(ads131m02_write_reg_verified(dev,
                        ADS131M02_REG_GAIN1,   ADS131M02_GAIN1_VAL),
                        TAG, "GAIN1");
    ESP_RETURN_ON_ERROR(ads131m02_write_reg_verified(dev,
                        ADS131M02_REG_CH0_CFG, ADS131M02_CH0_CFG_VAL),
                        TAG, "CH0_CFG");

    dev->configured = true;
    ESP_LOGI(TAG, "configured: CLKIN %u Hz, OSR 1024, HR mode -> %u SPS",
             (unsigned)ADS131M02_CLKIN_HZ, (unsigned)ADS131M02_FDATA_SPS);
    return ESP_OK;
}
