#ifndef ADS131M02_H
#define ADS131M02_H

/*
 * Minimal ESP-IDF driver for the TI ADS131M02 (SBAS853A).
 *
 * Scope (deliberately narrow, per project instructions):
 *   - SPI bus/device setup (SPI mode 1: CPOL = 0, CPHA = 1, Figure 6-1:
 *     "SPI settings are CPOL = 0 and CPHA = 1")
 *   - device reset + startup sequencing (Section 8.4.1)
 *   - register read/write with readback verification
 *   - configuration for the PPG operating point:
 *       CLKIN = 2.4576 MHz (external oscillator, see note below)
 *       fMOD  = CLKIN / 2 = 1.2288 MHz          (Section 8.3.6)
 *       OSR   = 1024  -> pure sinc3 path        (Section 8.3.7.1.2)
 *       fDATA = fMOD / OSR = 1200.000 SPS       (Table 8-2 relationship)
 *       CH0 enabled (PPG: AIN0P = TIA out, AIN0N = analog ref node),
 *       CH1 disabled, PGA gain 1, DC block filter DISABLED (baseline
 *       drift is handled by the existing baseline servo, not the ADC).
 *   - conversion frame read + decode (via ads131m02_frame.h)
 *
 * NOT in scope here: DRDY ISR / timestamping / FIR — those live in the
 * acquisition layer (ppg_acq.c) so that no signal processing sits inside
 * the SPI driver.
 *
 * CLKIN NOTE (architecturally important — see INTEGRATION_NOTES.md,
 * "CLKIN generation" for the full analysis):
 * exactly 1200 SPS requires fCLKIN = 2 * OSR * 1200 = 2.4576 MHz at
 * OSR = 1024.  Two viable sources:
 *   1. dedicated external 2.4576 MHz oscillator (recommended default:
 *      no dither jitter, no peripheral cost), or
 *   2. ESP32-C3 I2S MCLK output: PLL_F160M / (65 + 5/48) =
 *      2,457,600.000 Hz exactly on average (fractional divider fits the
 *      hardware fields; verified against the ESP-IDF C3 register
 *      definitions).  Caveat: the fractional divider dithers between
 *      /65 and /66, giving ~6.25 ns deterministic edge jitter on the
 *      modulator clock — noise impact must be bench-verified A/B
 *      against a clean oscillator before shipping.
 * ESP32-C3 LEDC can NOT generate the frequency exactly (8 fractional
 * divider bits; closest is −80/+160 ppm) and is not used.
 * This driver intentionally does NOT configure any approximate clock:
 * whichever source is chosen must be exactly 2.4576 MHz nominal.
 */

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "ads131m02_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Operating-point register values (see report section C) --------- */

/*
 * MODE (02h), datasheet 8.6.3:
 *   REG_CRC_EN=0, RX_CRC_EN=0, CRC_TYPE=0 (CCITT), RESET=0 (clear the
 *   power-up reset flag), WLENGTH=01b (24-bit words), TIMEOUT=1,
 *   DRDY_SEL=00b (most lagging enabled channel), DRDY_HIZ=0 (push-pull),
 *   DRDY_FMT=0 (level, active low).
 */
#define ADS131M02_MODE_VAL      0x0110u

/*
 * CLOCK (03h), datasheet 8.6.4:
 *   CH1_EN=0, CH0_EN=1, TBM=0, OSR[2:0]=011b (1024), PWR[1:0]=10b (HR).
 *   With CLKIN = 2.4576 MHz: fDATA = 2.4576e6 / (2*1024) = 1200.000 SPS.
 *   (2.4576 MHz is within the HR-mode CLKIN range 0.3–8.4 MHz,
 *   Recommended Operating Conditions, Section 6.3.)
 */
#define ADS131M02_CLOCK_VAL     0x010Eu

/*
 * GAIN1 (04h), datasheet 8.6.5: PGA gain 1 on both channels.
 * FSR = ±1.2 V / gain = ±1.2 V (Table 8-1); the TIA output vs the
 * analog reference node must stay inside this differential range.
 */
#define ADS131M02_GAIN1_VAL     0x0000u

/*
 * CH0_CFG (09h), datasheet 8.6.10: PHASE0=0, DCBLK0_DIS0=0, MUX0=00b
 * (AIN0P/AIN0N).  The global DC block filter stays disabled
 * (THRSHLD_LSB.DCBLOCK = 0h default, Section 8.3.8) — the external
 * baseline servo owns drift removal; the FIR intentionally passes DC.
 */
#define ADS131M02_CH0_CFG_VAL   0x0000u

/* Expected ID register contents: 22xxh, CHANCNT = 0010b (8.6.1). */
#define ADS131M02_ID_MSB        0x22u

/* Nominal CLKIN for this design (external oscillator). */
#define ADS131M02_CLKIN_HZ      2457600u
#define ADS131M02_FDATA_SPS     1200u

/* ---- Driver types ---------------------------------------------------- */

typedef struct {
    spi_host_device_t spi_host;     /* e.g. SPI2_HOST                    */
    int  pin_sclk;
    int  pin_mosi;                  /* -> ADC DIN                        */
    int  pin_miso;                  /* <- ADC DOUT                       */
    int  pin_cs;
    int  pin_drdy;                  /* input; ISR installed by ppg_acq   */
    int  pin_sync_reset;            /* -1 if SYNC/RESET tied to DVDD     */
    int  spi_clock_hz;              /* <= 25 MHz per t_c(SC) 40 ns min at
                                       DVDD >= 2.7 V (Section 6.6);
                                       8–10 MHz recommended here.        */
} ads131m02_config_t;

typedef struct {
    spi_device_handle_t spi;
    ads131m02_config_t  cfg;
    bool                configured;
} ads131m02_t;

/* ---- API -------------------------------------------------------------- */

/*
 * Full bring-up: SPI init, device reset, wait for readiness, write and
 * READBACK-VERIFY the operating-point registers above.  Returns
 * ESP_ERR_INVALID_RESPONSE on ID/readback mismatch (supervisor should
 * treat that as a FAULT/RECOVERY trigger).
 *
 * Conversions free-run as soon as CLKIN is present (Section 8.4.1:
 * "the device begins generating conversion data as soon as a valid
 * MCLK is provided") — there is no separate start-conversion command
 * in continuous-conversion mode.
 */
esp_err_t ads131m02_init(ads131m02_t *dev, const ads131m02_config_t *cfg);

/*
 * Strobe SYNC/RESET for synchronization (pulse >= 1 t_CLKIN and
 * < t_w(RSL) = 2048 t_CLKIN, Section 6.6 / 8.5.2).  Used on
 * (re)acquisition start to clear the 2-deep output FIFO and realign
 * DRDY (Section 8.5.1.9.1 / Figure 8-20).  No-op (returns
 * ESP_ERR_NOT_SUPPORTED) if pin_sync_reset < 0; the caller then falls
 * back to the double-read method (Figure 8-21) via
 * ads131m02_flush_fifo().
 */
esp_err_t ads131m02_sync_pulse(ads131m02_t *dev);

/* Double-read FIFO flush per datasheet Figure 8-21. */
esp_err_t ads131m02_flush_fifo(ads131m02_t *dev);

/*
 * Read one conversion frame (NULL command).  On ESP_OK, *frame holds a
 * CRC-verified STATUS word + sign-extended CH0/CH1 samples.
 * ESP_ERR_INVALID_CRC on output-CRC mismatch (frame contents are for
 * diagnostics only in that case).
 */
esp_err_t ads131m02_read_frame(ads131m02_t *dev, ads131m02_frame_t *frame);

/* Single register read/write (verified write on _write_reg_verified). */
esp_err_t ads131m02_read_reg (ads131m02_t *dev, uint8_t addr, uint16_t *val);
esp_err_t ads131m02_write_reg(ads131m02_t *dev, uint8_t addr, uint16_t val);
esp_err_t ads131m02_write_reg_verified(ads131m02_t *dev, uint8_t addr,
                                       uint16_t val);

/* Re-verify the operating-point registers (periodic integrity check /
 * recovery diagnostics).  ESP_ERR_INVALID_RESPONSE on any mismatch. */
esp_err_t ads131m02_verify_config(ads131m02_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* ADS131M02_H */
