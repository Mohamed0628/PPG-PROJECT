# Wearable PPG + IMU Heart-Rate Monitor

This is my senior-design wearable reflective PPG heart-rate monitor, built around an ESP32-C3, a custom optical/analog front end, synchronized inertial sensing, and BLE. The goal is not just to acquire a pulse waveform, but to build a signal chain whose rejection mechanisms correspond to the *physical source* of each major interference.

The current architecture is the result of a design review that replaced the earlier `TIA -> high-pass -> 50x gain -> low-pass -> ADC` chain. With a 24-bit differential delta-sigma ADC, the high-pass and gain stages were not only unnecessary, they created problems. The high-pass duplicated the baseline servo and would have broken its feedback by removing the DC quantity the servo needs to observe. The 50x stage added another op amp and its noise to the most sensitive part of the chain to solve a resolution problem the converter does not have.

## System Architecture

```text
Skin
  |
  v
Optical Capture Source (OCS)
  |
  v
TIA + analog baseline cancellation
  |
  v
ADS131M02 @ 1200 SPS
  |
  v
101-tap anti-alias FIR
  |
  v
Decimate by 10 -> 120 SPS
  |
  v
Ambient-reference subtraction
  |
  v
IMU-referenced NLMS motion cancellation
  |
  v
Beat detection / heart-rate estimation
  |
  v
BLE
```

## Why the Interference Rejection Is Split Up

A central design decision is to separate interference by **physical discriminant**, not by frequency alone. A single linear filter cannot solve disturbances that overlap the PPG signal in frequency.

| Interference | Rejection mechanism | Discriminant |
|---|---|---|
| Large DC photocurrent and slow drift | Analog baseline servo | Time scale |
| 120 Hz mains/lighting flicker and other out-of-band content | FIR before decimation | Frequency |
| Motion artifact | NLMS with IMU reference | Correlation with motion |
| In-band ambient-light variation | Second photodiode reference | Correlation with ambient light |

Motion is the clearest example. Walking cadence can be around 2 Hz while a brisk-walk heart rate can also be around 2 Hz. No ordinary frequency-selective filter can know which 2 Hz component is blood-volume pulsation and which is motion. The IMU supplies an independent observation of the interferer, which gives the adaptive filter information that is not present in the PPG waveform alone.

The same reasoning applies to slowly varying ambient light. The baseline servo must be slow enough not to erase the pulse, so there is an in-band region it cannot reject. A second photodiode provides the missing physical reference.

## Optical Capture Source

I call the LED + photodiode assembly the **Optical Capture Source (OCS)**.

The primary detector is an ams-OSRAM SFH 2430-Z with a 7.02 mm² active area, approximately 0.14 A/W responsivity at 525 nm, and approximately 950 pF junction capacitance at the 1.65 V reverse-bias operating point. It is a Vlambda-filtered detector with a 400-900 nm spectral range and peak sensitivity near 570 nm. The Vlambda filter reduces green responsivity relative to a broadband PIN photodiode, but it also rejects near-IR ambient energy that the old detector would have admitted. The device has about 0.1 nA typical dark current and a 5 nA maximum specified at 5 V reverse bias. Dark current is DC and is removed by the baseline servo, but worst-case part variation can consume meaningful cancellation range relative to an expected pulsatile current on the order of 10 nA.

The LED is green, 525-535 nm, with a narrow beam and a nominal 5 mm center-to-center spacing from the primary detector. The SFH 2430-Z package is 6.45 x 3.85 mm, substantially larger than the previous 3.2 x 2.0 mm detector, so the 5 mm spacing must be rechecked against the actual LED footprint and package orientation before layout is frozen; it is now mechanically tight rather than an assumed-safe spacing.

The ambient reference uses a second SFH 2430-Z with no intended LED optical path. The nominal 15 mm LED-to-reference-detector spacing remains optically reasonable, but the larger detector package and the OCS board outline must be checked to confirm that the reference detector, isolation structures, and skin-contact area still fit without crowding.

Optical crosstalk is treated as a PCB/package problem rather than only a spacing problem. The OCS uses a 1 mm routed slot through the board because FR-4 glass weave can pipe light between footprints, an opaque black epoxy dam standing roughly 1-2 mm above the board, and black soldermask.

The LED is driven by a MOSFET current sink from the battery rail. A green LED with roughly 3.1 V forward voltage leaves too little headroom for a simple series-resistor drive from a nominal 3.3 V rail.

## Analog Front End

The TIA is referenced to 1.65 V and follows

$$
V_{OUT} = V_{REF} - I_{ERR}R_f
$$

with

$$
I_{ERR} = I_{PD} - I_{CTRL}.
$$

The primary photodiode cathode is tied to 3.3 V and its anode is connected to the TIA summing node, giving approximately 1.65 V reverse bias around the nominal operating point.

The current TIA uses a 100 kΩ feedback resistor, a 10 nF C0G feedback capacitor, and an OPA2325-class amplifier. With approximately 960 pF total inverting-node capacitance and a 10 MHz amplifier gain-bandwidth product, the estimated minimum compensation capacitance is approximately 12.36 pF. The 10 nF feedback capacitor is therefore deliberately heavy compensation rather than a minimum-stability choice. It places the feedback pole around 159 Hz, about 20 times above the 8 Hz digital passband edge, while limiting the high-frequency noise gain to approximately 1.096.

The lower feedback resistance is required by the SFH 2430-Z's much larger ambient photocurrent. The detector produces about 6.3 µA at 1000 lx, so ordinary 300-500 lx indoor illumination can contribute roughly 2-3 µA before the LED is enabled. With about 1.6 V of usable output swing, 470 kΩ would clip near 3.4 µA, whereas 100 kΩ tolerates roughly 16 µA of residual current before reaching the same swing limit.

At an assumed 2 µA DC photocurrent, the current-noise estimate is still shot-noise dominated, but less strongly than before:

| Source | Input-referred current noise |
|---|---:|
| Photodiode shot noise | 0.80 pA/√Hz |
| 100 kΩ feedback-resistor Johnson noise | 0.41 pA/√Hz |
| Coarse cancellation-resistor Johnson noise | 0.33 pA/√Hz |
| Estimated total | 0.96 pA/√Hz |

The listed terms combine to about 0.96 pA/√Hz RMS spectral density. Over a 4.5 Hz bandwidth that is about 2.03 pA RMS, giving an estimated 73.8 dB SNR for a 10 nA pulsatile signal. Shot noise remains the largest single term and contributes about 70% of the listed noise power, so the front end is still reasonably described as shot-noise dominated, though the feedback-resistor contribution is no longer negligible. This is a calculated design estimate, not yet an on-skin measurement.

## Dual-Range Baseline Cancellation

The baseline actuator injects a controlled cancellation current into the TIA summing node:

$$
I_{CTRL} = \frac{V_{CTRL}-V_{SUM}}{R_{CTRL}}.
$$

The neutral point is therefore $V_{CTRL}=V_{SUM}$. This is important at startup: the cancellation PWM must initialize to its mid-scale/neutral condition **before the LED turns on**. Zero PWM duty is not neutral in this topology and can command a large cancellation current into the summing node.

The actuator has two ranges:

| Path | Resistance | Range | Step size | Purpose |
|---|---:|---:|---:|---|
| Coarse | 150 kΩ | ±11 µA | 1342 pA | Authority |
| Fine | 3.3 MΩ | ±500 nA | 61 pA | Resolution |

The commanded cancellation is clamped to ±11 µA. With the 100 kΩ TIA feedback resistor, the full coarse-path authority corresponds to about ±1.1 V at the TIA, which remains inside the amplifier's approximate ±1.65 V headroom. The cancellation hardware, rather than TIA swing, is now the binding limit.

The dual range remains justified. A 1342 pA coarse step produces about 134.2 µV at the TIA with 100 kΩ feedback, while a 10 nA pulsatile signal produces about 1.0 mV. The coarse step is therefore still about 13.4% of the expected PPG signal. The 61 pA fine step produces about 6.10 µV, or about 0.61% of that signal. Lowering $R_f$ scales the actuator-step voltage and PPG voltage together, so their ratio is essentially unchanged.

## Baseline Servo

The servo has separate fast-acquisition and slow-tracking modes. The proportional term is parameterized as

$$
K_p = \beta\left(\frac{V_{LSB}}{R_f}\right),
$$

while the integral gain is kept separately tunable.

| Mode | β | γ | Baseline-estimator corner | Simulated result |
|---|---:|---:|---:|---|
| Fast acquire | 0.01 | 3 s⁻¹ | 2 Hz | 0.97 s settling, critically damped, 99.7% PPG retained |
| Slow track | 0.01 | 0.1 s⁻¹ | 0.05 Hz | 100.0% PPG retained, no meaningful ringing |

A parameter sweep showed that settling time is controlled primarily by the integral term rather than by β. The proportional term's more important jobs are damping and limiting how much of the pulsatile signal is followed by the loop.

The servo runs from the valid **1200 SPS raw ADC stream**. It is intentionally upstream of the 10x digital decimator because its current estimator and convergence timing were designed for a 1200 Hz update rate.

## ADC and Acquisition

The converter is a TI ADS131M02: a two-channel, 24-bit delta-sigma ADC with simultaneous sampling. The primary channel is measured differentially with the TIA output on IN+ and $V_{REF}$ on IN-. This makes the reference cancel algebraically at the ADC input and allows reference disturbances to appear largely as common mode. Sampling the TIA and reference separately and subtracting them later in firmware would first digitize both noise contributions.

The ADC free-runs from its own clock and asserts DRDY for each conversion. DRDY defines the sample instant; the ESP32-C3 timestamps that edge and performs the SPI transfer in task context. This keeps BLE and other MCU scheduling activity from determining the sampling instant.

The acquisition rate is 1200 SPS. Simultaneous channel sampling is important for the ambient-reference path because it eliminates an otherwise unnecessary channel-to-channel timing error before subtraction.

## Anti-Alias FIR and Decimation

The digital anti-alias filter is implemented and host-validated. It is a 101-tap Parks-McClellan equiripple FIR with Q23 coefficients and a 64-bit accumulator:

```text
1200 SPS -> 101-tap FIR -> decimate by 10 -> 120 SPS
```

The filter runs **before** downsampling. This is essential because any content above the final 60 Hz Nyquist frequency can otherwise fold into the 0-60 Hz output band, including directly into the physiological PPG band.

| Metric | Result |
|---|---:|
| Passband | 0-8 Hz |
| Passband ripple | ±0.0057 dB |
| Gain at 1.2 Hz | -0.0046 dB |
| Worst alias into 0-8 Hz | -83.7 dB at 242.7 Hz |
| Attenuation at 120 Hz | -96.8 dB |
| Decimation | 10x |
| Output rate | 120 SPS |
| Group delay | 5 output samples |

The group delay is derived from the tap count and decimation factor rather than hard-coded because the IMU reference has to be delayed/aligned by the same amount before NLMS adaptation.

I chose 1200 -> 120 SPS rather than 1000 -> 100 SPS for an alias-placement reason. At 100 SPS output, interference near 100 Hz folds toward DC and can land directly inside the PPG band. At 120 SPS, 100 Hz folds to 20 Hz instead. The FIR still provides the actual rejection, but the rate choice makes the alias geometry less hostile to the physiological band.

## Motion Artifact Rejection

Motion is measured with an ICM-42670-P over I²C. Its internal filtering is configured before reducing the IMU rate, with a target low-pass region around 20-30 Hz. This is anti-alias protection for the **reference** signal: MEMS resonance, strap ringing, and impulsive events such as heel strike can contain broadband energy that would otherwise fold into the motion-reference band.

That matters because a corrupted NLMS reference does more than add noise. It can teach the adaptive filter to subtract components that were not actually caused by motion.

The motion stage uses NLMS with $\mu=0.05$ and bounded weights. Adaptation is held whenever the baseline servo is not in slow tracking so the adaptive filter does not learn through a changing analog operating point.

The ADC, IMU, and MCU have three independent clocks with no common steerable clock input. Instead of pretending they can remain phase-locked, firmware resamples the IMU onto the PPG timebase and then applies the required FIR group-delay alignment before adaptation.

## Ambient-Light Rejection

Servo simulation showed an important limitation: the slow baseline loop rejects very slow ambient drift below roughly 0.01 Hz, provides essentially no rejection above about 0.02 Hz, and can slightly amplify disturbances around 0.05 Hz.

That is not fixed by simply making the servo faster. A loop fast enough to track a 0.05 Hz disturbance begins moving toward the same band that contains the desired pulse. The approximately 0.02-5 Hz ambient region therefore does not have a clean frequency-domain solution.

The second photodiode supplies an independent ambient reference. After calibration of the coupling coefficient between detectors, the reference can be subtracted across the in-band region without requiring the heartbeat and ambient disturbance to occupy different frequencies. The achievable rejection will ultimately be limited by how well the two detectors observe the same ambient coupling path.

## Supervisor and Recovery

The firmware supervisor owns startup and recovery sequencing:

```text
POWER_OFF
    -> POWER_WAIT
    -> ANALOG_SETTLE
    -> SERVO_INIT
    -> LED_ENABLE
    -> FAST_ACQUIRE
    -> RUN

Additional states: SETTLING, RECOVERY, FAULT
```

The supervisor publishes system state; individual modules decide locally what that state means for them. State transitions occur when dependencies are satisfied rather than by treating startup as a list of arbitrary delays, and each transition has a timeout that can produce a fault.

The cancellation actuator is initialized neutral before LED enable. On recovery, acquisition and filtering are restarted as a new sampling epoch rather than allowing samples from before and after a discontinuity to be treated as one uniformly sampled FIR sequence.

## Power Architecture

```text
1S LiPo
  |
  v
Charger / protection
  |
  v
TPS63030 buck-boost (~3.5 V)
  |
  +-------------------------------> Digital domain
  |                                  ESP32-C3
  |                                  ICM-42670-P
  |
  +--> ferrite + capacitors
          |
          v
      3.3 V LDO
          |
          v
      Analog domain only
```

A ferrite bead is useful for high-frequency isolation but is not a low-frequency regulator. Its useful corner is around tens of kilohertz; pushing a passive ferrite network down toward the frequencies relevant to slow analog disturbances would require impractical series impedance or capacitance. The analog domain therefore uses active regulation after the ferrite network.

Hardware sequencing also reflects that dependency: buck-boost power-good gates the analog LDO enable so the analog rail cannot start before the upstream rail is established.

## Validation

The FIR has been exercised with an 11-test host suite, including passband preservation, 115 Hz and 119 Hz alias rejection, exact 120 Hz suppression, decimation count, group delay, DC gain, impulse behavior, and a full-scale ADC overflow stress case. All 11 tests pass.

The servo and supervisor are also tested with a runtime simulation that compiles the **real `servo.c` and `state_machine.c`** and closes a simulated physical plant around them. This has already caught bugs that would have been difficult to see by reading the control equations alone.

One failure was in convergence detection. The original convergence test observed the baseline estimate; in fast mode that estimator passes enough heartbeat energy that the convergence condition could never remain satisfied, causing `FAST_ACQUIRE` to time out into `FAULT` even though the plant had actually settled.

A separate disturbance stress test injects contact-pressure steps, drift ramps, ambient bursts, and a saturating event. It exposed another failure: the servo's saturation flag was never asserted because the coarse cancellation path had no range check. These tests are intentionally aimed at validating the firmware around realistic disturbances rather than only proving that individual equations work in isolation.

## What Has Not Been Measured Yet

**The analog component values are still provisional.**

The present TIA and cancellation design now accounts for the SFH 2430-Z's approximately 6.3 µA photocurrent at 1000 lx and its much larger junction capacitance, but the actual on-skin LED-generated DC photocurrent and pulsatile AC/DC ratio have still not been measured with this OCS.

That means the following values must be treated as design hypotheses until hardware data exists:

- 100 kΩ TIA feedback resistance
- 10 nF feedback capacitance
- coarse/fine cancellation authority
- ±11 µA cancellation clamp
- servo convergence threshold

The PCB therefore keeps $R_f$ and $C_f$ on reworkable footprints. The first optical/AFE measurements are intended to determine whether the assumed photocurrent and AC/DC ratio are correct before those values are frozen.

Calculated noise and SNR values in this README are likewise design estimates, not measured system performance.

## Current Design

| Parameter | Current value |
|---|---:|
| MCU | ESP32-C3 |
| ADC | TI ADS131M02, 24-bit delta-sigma |
| Raw PPG sample rate | 1200 SPS |
| Processed PPG rate | 120 SPS |
| ADC channels | 2, simultaneous |
| Primary photodiode | ams-OSRAM SFH 2430-Z |
| Ambient photodiode | ams-OSRAM SFH 2430-Z |
| LED wavelength | 525-535 nm |
| LED/primary-PD spacing | 5 mm center-to-center, pending mechanical recheck |
| TIA feedback resistor | 100 kΩ, provisional |
| TIA feedback capacitor | 10 nF C0G, provisional |
| TIA amplifier | OPA2325-class |
| Analog reference | 1.65 V |
| Coarse cancellation resistor | 150 kΩ |
| Fine cancellation resistor | 3.3 MΩ |
| Cancellation clamp | ±11 µA, provisional |
| FIR | 101-tap Parks-McClellan, Q23 |
| FIR passband | 0-8 Hz |
| FIR group delay | 5 samples at 120 SPS |
| IMU | ICM-42670-P |
| IMU LPF target | 20-30 Hz |
| NLMS step size | 0.05 |
| Buck-boost | TPS63030 at ~3.5 V |
| Analog rail | Ferrite + capacitors + 3.3 V LDO |
| Wireless | BLE |

## Repository Layout

```text
components/
  ads131m02/       ADS131M02 SPI, register and frame handling
  ppg_acq/         DRDY acquisition, timestamping, FIR and 120-SPS stream

host_tests/        Host-side FIR and ADC/stream validation
tools/             Analysis utilities, including sinc³ + FIR response checks

servo.c / servo.h              Fixed-point baseline servo
state_machine.c / .h           Startup, run, settling, recovery and fault supervisor
```

Board-specific ESP-IDF application wiring and final GPIO assignments are still to be added as the hardware definition is finalized.

## Build and Test

The host tests are designed to run independently of the target hardware. From the repository root:

```bash
cd host_tests
make
```

The FIR can also be compiled with host sanitizers during development so integer undefined behavior and memory errors are caught before target bring-up. The hardware-dependent ADS131M02/ESP-IDF path still requires target-side validation once the board GPIO map and application layer are in place.

The combined ADS131M02 sinc³ and FIR response can be inspected with the analysis utility in `tools/`.

## Next Hardware Milestone

The next major milestone is measurement: bring up the revised SFH 2430-Z OCS and analog front end, measure real ambient and LED-generated DC photocurrent plus the pulsatile AC/DC ratio on skin, verify TIA stability with the approximately 960 pF inverting-node capacitance, verify ADC/DRDY timing and SPI integrity, and then retune $R_f$, $C_f$, cancellation authority, and convergence thresholds from those measurements.

This is an **engineering prototype and is not intended for medical diagnosis or clinical use**.