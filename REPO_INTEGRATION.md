# PPG ADC/FIR repository integration

This branch adds the validated ADS131M02 acquisition and 1200 -> 120 SPS FIR pipeline without inventing a board pin map or ESP-IDF `main/` that does not yet exist in the repository.

## Rate split

```text
ADS131M02 valid CH0 sample @ 1200 SPS
        |
        +--> raw_sample_cb --> servo_process_sample() @ 1200 SPS
        |
        +--> 101-tap Q23 FIR --> decimate by 10 --> 120 SPS downstream queue/callback
```

The existing servo must stay on the 1200-SPS path. Its filter constants and convergence timing are explicitly derived for 1200 updates/s. The FIR is only the downstream anti-alias/decimation path.

## Supervisor ownership

The existing supervisor remains the only recovery state machine. Application wiring should start/stop acquisition according to `supervisor_ppg_acquisition_active()` and map `ppg_acq` errors onto the existing `recoverable_condition` / recovery path.

## What is intentionally still pending

The repository currently has no ESP-IDF `main/` application and no board GPIO assignments for ADS131M02 SPI/DRDY/SYNC. Those values must come from the PCB/board definition; they are not guessed in this branch.
