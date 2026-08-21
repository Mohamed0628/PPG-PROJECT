# Wearable PPG + IMU Heart Rate Monitor

This project is a custom wearable PPG heart-rate monitor that I am building from the ground up. The goal is to measure a PPG waveform from the wrist, process it with a custom analog front end, sample it with a microcontroller, and use IMU data to help remove motion artifacts. The final device will be battery powered, built on a custom PCB, and capable of sending data over Bluetooth Low Energy.

## PPG Sensor

PPG works by shining light into the skin and measuring how much light returns to a photodiode. As blood volume changes with each heartbeat, the amount of reflected light also changes. The photodiode converts these changes into a small current.

The difficult part is that the heartbeat is only a small variation on top of a much larger DC photocurrent. The analog front end has to remove most of this baseline, amplify the useful pulse signal, and keep everything inside the voltage range of the ADC.

## Analog Front End

The analog front end is currently:

```text
Photodiode -> TIA -> High-Pass Filter -> Gain -> Low-Pass Filter -> ADC
```

The circuit runs from a 3.3 V supply, so I use a 1.65 V reference as the midpoint of the analog signal. This lets the PPG waveform move above and below 1.65 V while still staying between the 0 V and 3.3 V ADC rails.

The first stage is a transimpedance amplifier that converts photodiode current into voltage. The current design uses a 27 kΩ feedback resistor and a 4.7 pF feedback capacitor. The photodiode capacitance used during simulation is approximately 17.6 pF.

The basic TIA relationship is:

```math
V_{OUT} = V_{REF} - I_{PD}R_F
```

For example, a 1 µA peak-to-peak change in photodiode current produces approximately 27 mVpp with a 27 kΩ feedback resistor.

After the TIA, a high-pass filter removes most of the DC baseline. The current design uses 330 kΩ and 1 µF, giving a cutoff frequency of approximately 0.48 Hz.

The remaining PPG signal then goes through a non-inverting amplifier with approximately 50x gain. The current values are 490 kΩ and 10 kΩ:

```math
A_V = 1 + \frac{490k}{10k} = 50
```

Finally, the signal passes through an approximately 8 Hz low-pass filter before reaching the ADC. This removes higher-frequency content that is not useful for the PPG measurement.

## IMU and Motion Artifact Removal

One of the biggest problems with wrist PPG is motion. Moving the wrist changes the contact between the sensor and skin and can create signals that are much larger than the actual heartbeat.

The design includes a BNO085 IMU so that motion can be recorded at the same time as the PPG signal. The plan is to use the IMU measurements as a reference for an NLMS adaptive filter. The filter will try to estimate the part of the PPG signal caused by movement and subtract it from the measured waveform.

The PPG and IMU data therefore need to be synchronized accurately for this to work.

## Embedded System

The microcontroller will sample the PPG signal at approximately 100 samples per second while also collecting IMU measurements. Firmware will handle data acquisition, synchronization, filtering, heart-rate calculation, battery monitoring, and BLE communication.

During development, raw PPG and IMU data will also be sent over BLE so that the signals can be analyzed on a computer before all of the processing is moved onto the embedded system.

## Power and PCB

The final device will run from a rechargeable lithium battery with a regulated 3.3 V supply. The current battery-life target is approximately 8 hours of continuous operation.

Everything will eventually be integrated onto a custom PCB. The analog layout is especially important because the photodiode and TIA input are sensitive to noise and parasitic capacitance. The photodiode-to-TIA connection and feedback loop will be kept short, while digital switching circuitry will be kept away from the sensitive analog front end.

## LTspice Simulation

Before building the PCB, I am testing the analog front end in LTspice. The simulation includes the photodiode current and capacitance, TIA, high-pass filter, gain stage, low-pass filter, 1.65 V reference, and 3.3 V supply.

I am also testing the circuit using real recorded PPG waveforms instead of only using sine waves. The recorded signals are converted into LTspice PWL current files and used as the simulated photodiode current.

For the current tests, the recordings are normalized around a 25 µA photocurrent with approximately 1 µA peak-to-peak variation:

```text
24.5 µA -> 25.5 µA
```

This keeps the shape of the real recorded heartbeat while giving me a controlled photodiode current to use when testing the circuit. Multiple recordings are being tested so the AFE is not designed around a single PPG waveform.

## Current Design

| Parameter | Value |
|---|---:|
| Supply | 3.3 V |
| Analog reference | 1.65 V |
| PPG sample rate | 100 SPS |
| TIA feedback resistor | 27 kΩ |
| TIA feedback capacitor | 4.7 pF |
| High-pass cutoff | ~0.48 Hz |
| Amplifier gain | ~50x |
| Low-pass cutoff | ~8 Hz |
| IMU | BNO085 |
| Wireless | BLE |
| Battery-life target | ~8 hours |

The project is currently in the analog simulation and component-selection stage. The next major steps are finishing the analog front end, completing the PCB, bringing up the hardware, collecting real wrist PPG and IMU data, and testing the motion-artifact rejection algorithm.

This is an engineering prototype and is not intended for medical diagnosis or clinical use.
