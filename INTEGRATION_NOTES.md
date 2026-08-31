# ADS131M02 + FIR integration notes

## What is integrated on this branch

This branch adds the ADS131M02 acquisition path and the validated 101-tap Q23 FIR/decimator to the real `PPG-PROJECT` repository.

The existing `state_machine.c` and `servo.c` were inspected before wiring the interfaces. That exposed one important correction to the isolated R2 package: the existing baseline servo is a **1200-SPS control loop**, so it must receive valid ADC samples before the 10x decimator. The 120-SPS FIR output is reserved for downstream physiological PPG, IMU/NLMS, and heart-rate processing.

```text
ADS131M02 CH0 valid conversion @ 1200 SPS
        |
        +--> raw_sample_cb --> servo_process_sample() @ 1200 SPS
        |
        +--> 101-tap Q23 FIR --> decimate by 10 --> 120 SPS
                                              |
                                              +--> downstream queue/callback
```

`fir.c` and `fir.h` are copied from the validated R2 package without changing their coefficients or arithmetic.

## Acquisition timing

DRDY defines the physical sample instant. The GPIO ISR only timestamps the edge with `esp_timer_get_time()` and posts an event. SPI, CRC/status validation, servo delivery, and FIR processing happen in task context.

Every accepted physical conversion is delivered exactly once to both the raw 1200-SPS callback and the FIR. The 120-SPS output retains the DRDY timestamp of the input sample that triggered that decimated output. The FIR group delay remains 50 input samples = 41.667 ms = 5 output samples and is compensated later at the fusion layer, not by altering acquisition timestamps.

## Missing-sample policy

The FIR may not silently bridge a missing ADC conversion because that would compress the time grid and shift decimation phase.

A failed SPI/CRC read may be retried only while the same ADC conversion is still recoverable within the bounded retry policy. If a conversion is genuinely lost, acquisition halts, the FIR is no longer touched, DRDY is disabled, and `error_cb` reports the condition to the existing supervisor. Recovery restarts acquisition through `ppg_acq_start()`, which resets the FIR, flushes/resynchronizes the ADC stream, discards settling conversions, and starts a new uniform sampling epoch.

The optional 120-SPS output queue is also cleared on stop/restart so pre-fault filtered outputs cannot leak into the new epoch.

## Existing supervisor ownership

No second recovery state machine is added. The existing supervisor remains authoritative.

Application wiring should use `supervisor_ppg_acquisition_active()` to decide when acquisition is active, call `ppg_acq_start()` when entering/re-entering the acquisition phase, call `ppg_acq_stop()` on recovery/fault entry, and map `ppg_acq` errors into the supervisor's existing recoverable/unrecoverable condition inputs.

The repository does not yet contain an ESP-IDF `main/` application or the final board GPIO map, so those concrete calls cannot be instantiated without guessing pins. This branch deliberately does not invent them.

## ADS131M02 operating point

The driver configures CH0 differential input, gain 1, 24-bit words, OSR 1024, HR mode, and no ADC DC-blocking filter. With CLKIN = 2.4576 MHz:

```text
fMOD  = 2.4576 MHz / 2 = 1.2288 MHz
fDATA = 1.2288 MHz / 1024 = 1200 SPS
```

A dedicated 2.4576-MHz oscillator remains the lowest-risk clock source. ESP32-C3 I2S MCLK can generate 2.4576 MHz exactly on average with a fractional divider, but its deterministic divider jitter should be bench-compared against a clean oscillator before using it as the production clock. LEDC does not hit the frequency exactly and is not used for CLKIN.

## Real-time callback contract

`raw_sample_cb` runs at 1200 SPS in the high-priority acquisition task and is intended for the existing bounded integer servo update. `output_cb` runs at 120 SPS and must also remain nonblocking. BLE, logging, NLMS-heavy work, and other blocking operations belong in consumer tasks. Prefer `output_queue_len > 0` for downstream DSP.

## Hardware validation still required

Host testing cannot validate DRDY jitter, real SPI signal integrity, same-conversion retry behavior, GPIO timing, the actual CLKIN waveform, ADC/TIA headroom, or end-to-end noise floor. Those remain bench tasks once the board and ESP-IDF application exist.
