"""Plot autotest CSV results.

Usage:
    python plot_autotest.py                      # loads the most recent autotest_*.csv
    python plot_autotest.py path/to/results.csv  # loads a specific file
"""

import glob
import os
import sys
import csv
import collections

import matplotlib.pyplot as plt


def load_csv(path: str):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            snr = None if row["snr_db"] == "" else float(row["snr_db"])
            rows.append({
                "snr_db":            snr,
                "f0":                float(row["f0"]),
                "f1":                float(row["f1"]),
                "frequency_pair":    row["frequency_pair"],
                "messages_found":    int(row["messages_found"]),
                "messages_expected": int(row["messages_expected"]),
            })
    return rows


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        pattern = os.path.join(os.path.dirname(__file__), "autotest_*.csv")
        candidates = sorted(glob.glob(pattern))
        if not candidates:
            print("No autotest_*.csv files found.")
            sys.exit(1)
        csv_path = candidates[-1]  # most recent by name (timestamp-sorted)

    print(f"Loading {csv_path}")
    rows = load_csv(csv_path)

    # Group by (frequency_pair, snr_db), sum messages across subfolders
    totals: dict[tuple, list[int, int]] = collections.defaultdict(lambda: [0, 0])
    for r in rows:
        key = (r["frequency_pair"], r["snr_db"])
        totals[key][0] += r["messages_found"]
        totals[key][1] += r["messages_expected"]

    # Collect unique sorted SNR values and frequency pairs
    snr_values = sorted({k[1] for k in totals if k[1] is not None})
    freq_pairs = sorted({k[0] for k in totals})

    # Separate clean (None SNR) baseline if present
    has_clean = any(k[1] is None for k in totals)

    fig, ax = plt.subplots(figsize=(10, 5))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for i, pair in enumerate(freq_pairs):
        color = colors[i % len(colors)]
        xs, ys = [], []
        for snr in snr_values:
            key = (pair, snr)
            if key in totals:
                found, expected = totals[key]
                pct = 100.0 * found / expected if expected else 0.0
                xs.append(snr)
                ys.append(pct)
        ax.plot(xs, ys, marker="o", linestyle="-", color=color, label=f"{pair} Hz")

        if has_clean:
            key_clean = (pair, None)
            if key_clean in totals:
                found, expected = totals[key_clean]
                pct = 100.0 * found / expected if expected else 0.0
                ax.axhline(pct, color=color, linestyle=":", linewidth=0.8, alpha=0.6)

    ax.axhline(100, color="black", linestyle="--", linewidth=0.8, alpha=0.4, label="100 %")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("Messages decoded (%)")
    ax.set_title(f"Demodulator robustness — {os.path.basename(csv_path)}")
    ax.legend(title="Frequency pair", fontsize=9)
    ax.grid(True, alpha=0.4)
    ax.set_ylim(-5, 115)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
