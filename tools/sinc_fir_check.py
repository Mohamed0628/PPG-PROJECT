#!/usr/bin/env python3
"""
H_total(f) = H_ADC(f) * H_FIR(f) validation for the PPG signal chain.

Operating point (see ads131m02.h and the integration report):
    CLKIN = 2.4576 MHz  ->  fMOD = CLKIN/2 = 1.2288 MHz  (datasheet 8.3.6)
    OSR   = 1024        ->  fDATA = fMOD/OSR = 1200 SPS  (Table 8-2)

ADC digital filter at OSR = 1024: pure sinc^3 path (no sinc1 stage —
the sinc1 averager only engages for OSR >= 2048, datasheet 8.3.7.1.2,
Figure 8-3, Table 8-3).  Magnitude response, datasheet Equation 7:

    |H(f)| = | sin(N*pi*f/fMOD) / (N * sin(pi*f/fMOD)) | ^ 3 ,  N = OSR

H_FIR(f) is computed from the ACTUAL Q23 coefficients parsed out of
fir.c (single source of truth), evaluated at fs = 1200 Hz:

    H_FIR(f) = | sum_k h[k] * exp(-j*2*pi*f*k/1200) | / 2^23

No FIR redesign, no inverse-sinc compensation is performed here; this
script only verifies that the combined response stays acceptable.
"""

import cmath
import math
import re
import sys
from pathlib import Path

FCLKIN = 2_457_600.0
FMOD = FCLKIN / 2.0            # 1.2288 MHz
OSR = 1024                     # sinc^3, no sinc1 stage at this setting
FS = FMOD / OSR                # 1200.0 SPS exactly
Q = 23

FIR_C = Path(__file__).resolve().parent.parent / \
    "components" / "ppg_acq" / "fir.c"


def load_coeffs(path):
    text = path.read_text()
    m = re.search(r"fir_coeffs\[FIR_NUM_TAPS\]\s*=\s*\{(.*?)\};",
                  text, re.S)
    if not m:
        sys.exit("could not locate fir_coeffs in fir.c")
    coeffs = [int(tok) for tok in re.findall(r"-?\d+", m.group(1))]
    if len(coeffs) != 101:
        sys.exit(f"expected 101 taps, found {len(coeffs)}")
    return coeffs


def h_sinc3(f):
    """ADC sinc^3 magnitude (datasheet Eq. 7); exact limit 1.0 at DC/notches."""
    if f == 0.0:
        return 1.0
    num = math.sin(OSR * math.pi * f / FMOD)
    den = OSR * math.sin(math.pi * f / FMOD)
    if den == 0.0:
        return 1.0
    return abs(num / den) ** 3


def h_fir(f, coeffs):
    acc = sum(c * cmath.exp(-2j * math.pi * f * k / FS)
              for k, c in enumerate(coeffs))
    return abs(acc) / (1 << Q)


def db(x):
    if x <= 0.0:
        return float("-inf")
    return 20.0 * math.log10(x)


def main():
    coeffs = load_coeffs(FIR_C)

    print("ADS131M02 sinc^3 (OSR=1024) x external FIR combined response")
    print(f"  fCLKIN = {FCLKIN/1e6:.4f} MHz, fMOD = {FMOD/1e6:.4f} MHz, "
          f"fDATA = {FS:.3f} SPS")
    print(f"  FIR: 101 taps, Q23, fs = 1200 Hz "
          f"(coefficients parsed from fir.c, sum = {sum(coeffs)})")
    print()
    hdr = (f"{'f (Hz)':>8} | {'ADC-only (dB)':>14} | "
           f"{'FIR-only (dB)':>14} | {'combined (dB)':>14}")
    print(hdr)
    print("-" * len(hdr))

    freqs = [0.0, 1.2, 3.0, 8.0, 60.0, 115.0, 119.0, 120.0]
    results = {}
    for f in freqs:
        ha = h_sinc3(f)
        hf = h_fir(f, coeffs)
        ht = ha * hf
        results[f] = (db(ha), db(hf), db(ht))
        print(f"{f:8.1f} | {db(ha):14.4f} | {db(hf):14.4f} | "
              f"{db(ht):14.4f}")

    print()
    ok = True

    # Passband droop check (DC-8 Hz).  The ADC contribution alone at
    # 8 Hz decides whether inverse-sinc compensation is warranted.
    adc_droop_8 = results[8.0][0]
    comb_var = max(abs(results[f][2] - results[0.0][2])
                   for f in (1.2, 3.0, 8.0))
    print(f"ADC-only droop at 8 Hz: {adc_droop_8:+.4f} dB")
    print(f"Combined passband deviation from DC (worst of 1.2/3/8 Hz): "
          f"{comb_var:.4f} dB")
    if abs(adc_droop_8) > 0.01 or comb_var > 0.03:
        print("  -> passband droop SIGNIFICANT: inverse-sinc "
              "compensation would need review")
        ok = False
    else:
        print("  -> negligible: NO inverse-sinc compensation required")

    # Alias-band check: combined attenuation must remain >= 80 dB at the
    # frequencies that fold into the PPG band after decimation to 120 SPS.
    for f in (115.0, 119.0, 120.0):
        if results[f][2] > -80.0:
            print(f"  -> combined attenuation at {f} Hz only "
                  f"{results[f][2]:.2f} dB (< 80 dB): FAIL")
            ok = False
    if ok:
        print("Combined alias-band attenuation >= 80 dB at "
              "115/119/120 Hz: PASS")
        # The sinc^3 only ever ADDS attenuation below fDATA/2 relative
        # to the FIR alone, so the validated FIR margins are preserved.

    print()
    print("SINC+FIR COMBINED CHECK " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
