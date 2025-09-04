#!/usr/bin/env python3
"""
Analyze alternating sine frequencies from DAC output capture.
Loads test_values2.csv and finds the two frequencies + switch times.
Decodes into bits (0/1) assuming each bit is 50 sine periods long.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ---------- Config ----------
CSV_FILE = "test_values2.csv"  # <-- your CSV file here
PERIODS_PER_BIT = 50  # must match STM32 firmware

EXPECTED_BITS = 48
text = "Hello"
bitstream_solution = "".join(f"{ord(c):08b}"[::-1] for c in text)  # LSB-first
bitstream_solution = (bitstream_solution + "0" * EXPECTED_BITS)[:EXPECTED_BITS]


def main():
    csv_path = Path(CSV_FILE)

    # ---------- Load CSV ----------
    try:
        df = pd.read_csv(csv_path)
        if df.shape[1] < 2:
            df = pd.read_csv(csv_path, header=None)
    except Exception:
        df = pd.read_csv(csv_path, header=None)

    time_col = df.columns[0]
    val_col = df.columns[1]

    t = df[time_col].astype(float).to_numpy()
    x = df[val_col].astype(float).to_numpy()

    # Clean
    good = np.isfinite(t) & np.isfinite(x)
    t, x = t[good], x[good]
    order = np.argsort(t)
    t, x = t[order], x[order]

    # ---------- Zero-crossings ----------
    x_centered = x - np.median(x)
    sign = np.signbit(x_centered)
    cross_idx = np.where((sign[:-1] == True) & (sign[1:] == False))[0]

    zc_times = []
    for i in cross_idx:
        x0, x1 = x_centered[i], x_centered[i + 1]
        t0, t1 = t[i], t[i + 1]
        if x1 != x0:
            frac = -x0 / (x1 - x0)
            if 0 <= frac <= 1:
                zc_times.append(t0 + frac * (t1 - t0))
        else:
            zc_times.append((t0 + t1) / 2.0)
    zc_times = np.asarray(zc_times)

    if len(zc_times) < 4:
        print("Not enough zero-crossings detected.")
        return

    # ---------- Frequency estimation ----------
    periods = np.diff(zc_times)
    freq_inst = 1.0 / periods
    t_freq = 0.5 * (zc_times[1:] + zc_times[:-1])

    q1, q3 = np.percentile(freq_inst, [25, 75])
    iqr = q3 - q1
    lo = q1 - 2.0 * iqr
    hi = q3 + 2.0 * iqr
    mask = (freq_inst > 0) & (freq_inst >= max(1e-6, lo)) & (freq_inst <= hi)
    freq_inst, t_freq = freq_inst[mask], t_freq[mask]

    # ---------- Simple 2-cluster k-means ----------
    c1, c2 = np.percentile(freq_inst, [25, 75])
    for _ in range(50):
        dist1, dist2 = abs(freq_inst - c1), abs(freq_inst - c2)
        lab = (dist2 < dist1).astype(int)
        if np.any(lab == 0):
            c1_new = freq_inst[lab == 0].mean()
        else:
            c1_new = c1
        if np.any(lab == 1):
            c2_new = freq_inst[lab == 1].mean()
        else:
            c2_new = c2
        if np.isclose(c1_new, c1) and np.isclose(c2_new, c2):
            break
        c1, c2 = c1_new, c2_new

    f1, f2 = sorted([c1, c2])  # lowest freq = f1, highest = f2
    centers = np.array([f1, f2])
    lab = np.argmin(np.abs(freq_inst[:, None] - centers[None, :]), axis=1)

    # ---------- Switch detection ----------
    switch_idxs = np.where(np.diff(lab) != 0)[0]
    switch_times = t_freq[switch_idxs + 1]

    # ---------- Segment export with bit decoding ----------
    segments = []
    bitstream = []
    start_idx = 0
    for si in switch_idxs:
        seg_start, seg_end = t_freq[start_idx], t_freq[si + 1]
        seg_label = lab[start_idx]
        freq_val = f1 if seg_label == 0 else f2
        bit_val = 0 if seg_label == 0 else 1

        # how many periods fit in this segment?
        n_periods = (seg_end - seg_start) * freq_val
        n_bits = int(round(n_periods / PERIODS_PER_BIT))

        segments.append(
            [
                seg_start,
                seg_end,
                seg_end - seg_start,
                freq_val,
                bit_val,
                n_periods,
                n_bits,
            ]
        )
        bitstream.extend([bit_val] * n_bits)
        start_idx = si + 1

    # handle last segment
    seg_label = lab[start_idx]
    freq_val = f1 if seg_label == 0 else f2
    bit_val = 0 if seg_label == 0 else 1
    seg_start, seg_end = t_freq[start_idx], t_freq[-1]
    n_periods = (seg_end - seg_start) * freq_val
    n_bits = int(round(n_periods / PERIODS_PER_BIT))

    segments.append(
        [seg_start, seg_end, seg_end - seg_start, freq_val, bit_val, n_periods, n_bits]
    )
    bitstream.extend([bit_val] * n_bits)

    seg_df = pd.DataFrame(
        segments,
        columns=[
            "start_time_s",
            "end_time_s",
            "duration_s",
            "freq_hz",
            "bit",
            "periods",
            "n_bits",
        ],
    )
    out_path = csv_path.with_name("frequency_segments.csv")
    seg_df.to_csv(out_path, index=False)

    # ---------- Print results ----------
    print(f"Estimated frequencies: {f1:.2f} Hz (bit 0), {f2:.2f} Hz (bit 1)")
    if len(switch_times):
        print("Switch times (s):")
        for st in switch_times:
            print(f"  {st:.6f}")
    else:
        print("No switches detected.")

    decoded_str = "".join(str(b) for b in bitstream)
    print("Decoded bitstream:")
    print(decoded_str)

    if bitstream_solution:
        print("Expected bitstream:")
        print(bitstream_solution)
        print("Match:", decoded_str.startswith(bitstream_solution))

    print(f"Segments saved to {out_path}")

    # ---------- Plot ----------
    plt.figure(figsize=(10, 4))
    plt.plot(t_freq, freq_inst, ".", markersize=2, label="inst. freq")
    plt.axhline(f1, linestyle="--", label=f"{f1:.1f} Hz (bit 0)")
    plt.axhline(f2, linestyle="--", label=f"{f2:.1f} Hz (bit 1)")
    plt.xlabel("Time [s]")
    plt.ylabel("Frequency [Hz]")
    plt.title(f"Instantaneous Frequency + Bits from {CSV_FILE}")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
