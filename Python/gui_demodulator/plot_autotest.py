"""Plot autotest CSV results.

Usage:
    python plot_autotest.py                        # loads all autotest_*.csv files found
    python plot_autotest.py path/to/results.csv    # loads a specific file
    python plot_autotest.py file1.csv file2.csv    # loads multiple specific files
"""

import glob
import os
import re
import sys
import csv
import collections

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


def load_csv(path: str):
    # Infer test type from filename: autotest_CABLE_... → "Cable"
    basename = os.path.basename(path)
    m = re.match(r"autotest_([A-Za-z]+)_", basename)
    test_type = m.group(1).capitalize() if m else "Unknown"

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
                "test_type":         test_type,
            })
    return rows


def main():
    if len(sys.argv) > 1:
        groups = [sys.argv[1:]]
    else:
        base = os.path.dirname(__file__)
        groups = []
        for pattern in ("autotest_CABLE_*.csv", "autotest_SPEAKER_*.csv"):
            matches = sorted(glob.glob(os.path.join(base, pattern)))
            if matches:
                groups.append(matches)
        if not groups:
            print("No autotest_*.csv files found.")
            sys.exit(1)

    for csv_paths in groups:
        all_rows = []
        for path in csv_paths:
            print(f"Loading {path}")
            all_rows.extend(load_csv(path))
        plot(all_rows, csv_paths)


def plot(all_rows, csv_paths):

    plt.rcParams.update({
        "font.size":        10,
        "axes.labelsize":   10,
        "axes.titlesize":   11,
        "xtick.labelsize":   9,
        "ytick.labelsize":   9,
        "legend.fontsize":   9,
        "legend.title_fontsize": 9,
        "lines.linewidth":   1.5,
        "lines.markersize":  5,
    })

    # Group by (test_type, frequency_pair, snr_db), sum messages across subfolders
    totals: dict[tuple, list[int, int]] = collections.defaultdict(lambda: [0, 0])
    for r in all_rows:
        key = (r["test_type"], r["frequency_pair"], r["snr_db"])
        totals[key][0] += r["messages_found"]
        totals[key][1] += r["messages_expected"]

    # Collect unique sorted values
    test_types = sorted({k[0] for k in totals})
    snr_values = sorted({k[2] for k in totals if k[2] is not None})
    freq_pairs = sorted({k[1] for k in totals}, key=lambda p: int(re.split(r"[-/]", p)[0]))

    has_clean = any(k[2] is None for k in totals)

    # Line styles cycle per test type so cable vs speaker are visually distinct
    type_styles = {t: ls for t, ls in zip(test_types, ["-", "--", ":", "-."])}
    type_markers = {t: m for t, m in zip(test_types, ["o", "s", "^", "D"])}

    fig, ax = plt.subplots(figsize=(6.5, 4.0))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for ttype in test_types:
        ls = type_styles[ttype]
        marker = type_markers[ttype]
        for i, pair in enumerate(freq_pairs):
            color = colors[i % len(colors)]
            xs, ys = [], []
            for snr in snr_values:
                key = (ttype, pair, snr)
                if key in totals:
                    found, expected = totals[key]
                    pct = 100.0 * found / expected if expected else 0.0
                    xs.append(snr)
                    ys.append(pct)
            if not xs:
                continue

            label = f"{pair} Hz ({ttype})" if len(test_types) > 1 else f"{pair} Hz"
            ax.plot(xs, ys, marker=marker, linestyle=ls, color=color, alpha=0.35, label=label)

            if len(xs) >= 4:
                try:
                    def sigmoid(x, x0, k):
                        return 100.0 / (1.0 + np.exp(-k * (x - x0)))
                    popt, _ = curve_fit(sigmoid, xs, ys, p0=[np.median(xs), 0.5], maxfev=2000)
                    xs_smooth = np.linspace(min(xs), max(xs), 200)
                    ax.plot(xs_smooth, sigmoid(xs_smooth, *popt), linestyle=ls, linewidth=2.5, color=color, alpha=0.85)
                except RuntimeError:
                    pass

            if has_clean:
                key_clean = (ttype, pair, None)
                if key_clean in totals:
                    found, expected = totals[key_clean]
                    pct = 100.0 * found / expected if expected else 0.0
                    ax.axhline(pct, color=color, linestyle=":", linewidth=1.0, alpha=0.6)

    ax.axhline(100, color="black", linestyle="--", linewidth=1.0, alpha=0.4, label="100 %")
    ax.set_xlabel("SNR (dB)")
    ax.set_ylabel("Messages decoded (%)")
    ax.set_title(" + ".join(test_types))
    legend_title = "Freq pair (test)" if len(test_types) > 1 else "Frequency pair"
    ax.legend(title=legend_title)
    ax.grid(True, alpha=0.4)
    ax.set_ylim(-5, 115)
    plt.tight_layout()

    # Save PNG alongside the (first) CSV, or combined name if multiple
    if len(csv_paths) == 1:
        png_path = os.path.splitext(csv_paths[0])[0] + ".png"
    else:
        dir_ = os.path.dirname(csv_paths[0])
        png_path = os.path.join(dir_, "autotest_combined.png")

    fig.savefig(png_path, dpi=300)
    print(f"Saved {png_path}")
    plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
