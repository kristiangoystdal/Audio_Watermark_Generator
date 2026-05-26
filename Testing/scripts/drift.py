#!/usr/bin/env python3
"""
RTC Drift Analyzer - Calculates module clock drift from CSV files in the drift folder.

Directory structure expected:
  <drift_dir>/
    module_A.csv
    module_B.csv
    ...

Outputs:
  <results_dir>/
    module_A/  drift_report.txt
    module_B/  drift_report.txt
    ...
  <results_dir>/drift_results.csv   (all modules combined)
  <results_dir>/drift_plot.png
"""

import csv
from pathlib import Path
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import matplotlib.dates as mdates

DRIFT_DIR = Path("test_files/rtc_measurements/drift")
RESULTS_DIR = Path("results/drift")


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_csv_file(filepath):
    """
    Return list of (real_datetime, offset_seconds).
    Real time is local (UTC+2); offset = real_utc - module_time.
    """
    measurements = []
    with open(filepath, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            real_dt = datetime.strptime(row["real_datetime"], "%Y-%m-%d %H:%M:%S")
            module_dt = datetime.strptime(row["module_datetime"], "%Y-%m-%d %H:%M:%S")
            real_utc = real_dt - timedelta(hours=2)
            offset = (real_utc - module_dt).total_seconds()
            measurements.append((real_dt, offset))
    return measurements


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def linear_regression(x, y):
    n = len(x)
    if n < 2:
        return 0.0, 0.0
    x_mean = sum(x) / n
    y_mean = sum(y) / n
    num = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, y))
    den = sum((xi - x_mean) ** 2 for xi in x)
    slope = num / den if den != 0 else 0.0
    intercept = y_mean - slope * x_mean
    return slope, intercept


def filter_outliers(measurements):
    """
    Remove measurements whose offset is an outlier (e.g. Module=00:00:00 glitches).
    Uses median absolute deviation; falls back to a 30-second hard fence when MAD=0.
    """
    if len(measurements) < 2:
        return measurements
    offsets = [m[1] for m in measurements]
    sorted_off = sorted(offsets)
    n = len(sorted_off)
    median = sorted_off[n // 2]
    mad = sorted([abs(o - median) for o in offsets])[n // 2]
    threshold = max(3.0 * mad, 30.0)
    return [(dt, o) for dt, o in measurements if abs(o - median) <= threshold]


def compute_daily_averages(measurements):
    """
    Group measurements by calendar date and return one (mean_datetime, mean_offset)
    per day.  Averaging within a day removes session noise while keeping the full
    inter-day time span for regression leverage.
    """
    from collections import defaultdict
    groups = defaultdict(list)
    for dt, offset in measurements:
        groups[dt.date()].append((dt, offset))

    daily = []
    for day in sorted(groups):
        pts = groups[day]
        mean_ts = sum(dt.timestamp() for dt, _ in pts) / len(pts)
        mean_offset = sum(o for _, o in pts) / len(pts)
        mean_dt = datetime.fromtimestamp(mean_ts)
        daily.append((mean_dt, mean_offset))

    return daily


def analyze_module(csv_path):
    """Read one module CSV and return analysis dict."""
    measurements = parse_csv_file(csv_path)
    if not measurements:
        return None

    clean = filter_outliers(measurements)
    if not clean:
        return None

    clean.sort(key=lambda m: m[0])

    daily = compute_daily_averages(clean)
    if len(daily) < 2:
        return None

    t0 = daily[0][0]
    x_daily = [(d[0] - t0).total_seconds() for d in daily]
    y_daily = [d[1] for d in daily]
    slope, intercept = linear_regression(x_daily, y_daily)

    offsets = [m[1] for m in clean]
    time_span_h = (daily[-1][0] - t0).total_seconds() / 3600

    return {
        "module": csv_path.stem,
        "all_measurements": clean,
        "daily_averages": daily,
        "drift_ppm": slope * 1e6,
        "drift_s_per_day": slope * 86400,
        "initial_offset": daily[0][1],
        "final_offset": daily[-1][1],
        "avg_offset": sum(offsets) / len(offsets),
        "n_measurements": len(clean),
        "discarded": len(measurements) - len(clean),
        "time_span_hours": time_span_h,
        "regression_slope": slope,
        "regression_intercept": intercept,
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def format_offset(seconds):
    sign = "-" if seconds < 0 else "+"
    seconds = abs(float(seconds))
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{sign}{h:02d}:{m:02d}:{s:02d}"


def write_module_report(result, report_path):
    lines = []
    lines.append("=" * 70)
    lines.append(f"RTC DRIFT REPORT — {result['module'].upper()}")
    lines.append("=" * 70)
    lines.append(f"Total measurements : {result['n_measurements']}")
    if result["discarded"]:
        lines.append(f"Discarded (outliers): {result['discarded']}")
    lines.append(f"Days measured      : {len(result['daily_averages'])}")
    lines.append(f"Time span          : {result['time_span_hours']:.2f} hours")
    lines.append(f"Initial offset     : {format_offset(result['initial_offset'])}")
    lines.append(f"Final offset       : {format_offset(result['final_offset'])}")
    lines.append(f"Average offset     : {format_offset(result['avg_offset'])}")
    lines.append(f"Overall drift rate : {result['drift_ppm']:.3f} ppm")
    lines.append(f"Drift per day      : {result['drift_s_per_day']:.2f} s/day")
    lines.append("")
    lines.append("Daily averages (used for drift regression):")
    lines.append("-" * 70)
    for dt, offset in result["daily_averages"]:
        lines.append(f"  {dt.strftime('%Y-%m-%d %H:%M:%S')}   {offset:+.3f} s")
    lines.append("")
    lines.append("All measurements (real datetime, offset s):")
    lines.append("-" * 70)
    for dt, offset in result["all_measurements"]:
        lines.append(f"  {dt.strftime('%Y-%m-%d %H:%M:%S')}   {offset:+.1f} s")
    lines.append("=" * 70)

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines))


def write_shared_csv(results, csv_path):
    fieldnames = [
        "module",
        "n_measurements",
        "time_span_hours",
        "initial_offset_s",
        "final_offset_s",
        "avg_offset_s",
        "drift_ppm",
        "drift_s_per_day",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(
                {
                    "module": r["module"],
                    "n_measurements": r["n_measurements"],
                    "time_span_hours": f"{r['time_span_hours']:.4f}",
                    "initial_offset_s": f"{r['initial_offset']:.1f}",
                    "final_offset_s": f"{r['final_offset']:.1f}",
                    "avg_offset_s": f"{r['avg_offset']:.2f}",
                    "drift_ppm": f"{r['drift_ppm']:.3f}",
                    "drift_s_per_day": f"{r['drift_s_per_day']:.4f}",
                }
            )


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def _draw_offset_ax(ax, datetimes, offsets, daily_datetimes, daily_offsets,
                    slope, intercept, label, color):
    daily_baseline = daily_offsets[0]

    raw_rel = [o - daily_baseline for o in offsets]
    daily_rel = [o - daily_baseline for o in daily_offsets]

    t0 = daily_datetimes[0]
    span_s = (daily_datetimes[-1] - t0).total_seconds()
    reg_datetimes = [t0, t0 + timedelta(seconds=span_s)]
    reg_y = [
        slope * 0 + intercept - daily_baseline,
        slope * span_s + intercept - daily_baseline,
    ]

    ax.scatter(datetimes, raw_rel, color=color, alpha=0.3, s=20, zorder=2)
    ax.plot(daily_datetimes, daily_rel, "o-", color=color, label=label,
            linewidth=2, markersize=8, zorder=3)
    ax.plot(reg_datetimes, reg_y, "--", color=color, alpha=0.6, linewidth=1.2)


def _style_offset_ax(ax, title):
    ax.set_title(title)
    ax.set_xlabel("Datetime")
    ax.set_ylabel("Offset change (s)")
    ax.legend()
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
    ax.xaxis.set_major_locator(mdates.AutoDateLocator())
    ax.tick_params(axis="x", rotation=30)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":")


# ---------------------------------------------------------------------------
# Per-module plot
# ---------------------------------------------------------------------------

def plot_module(result, plot_path):
    fig, ax = plt.subplots(figsize=(10, 5))
    label = result["module"].replace("module_", "Module ").title()
    datetimes = [m[0] for m in result["all_measurements"]]
    offsets = [m[1] for m in result["all_measurements"]]
    daily_dts = [d[0] for d in result["daily_averages"]]
    daily_offs = [d[1] for d in result["daily_averages"]]

    _draw_offset_ax(ax, datetimes, offsets, daily_dts, daily_offs,
                    result["regression_slope"], result["regression_intercept"],
                    label, "steelblue")
    _style_offset_ax(ax, f"{label} — offset drift over time (UTC-corrected)")

    fig.tight_layout(pad=2.0)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"    Plot:   {plot_path}")


# ---------------------------------------------------------------------------
# Combined all-modules plot
# ---------------------------------------------------------------------------

def plot_all_modules(results, plot_path):
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    colors = plt.cm.tab10.colors

    for i, r in enumerate(results):
        color = colors[i % len(colors)]
        label = r["module"].replace("module_", "Module ").title()
        datetimes = [m[0] for m in r["all_measurements"]]
        offsets = [m[1] for m in r["all_measurements"]]
        daily_dts = [d[0] for d in r["daily_averages"]]
        daily_offs = [d[1] for d in r["daily_averages"]]
        _draw_offset_ax(axes[0], datetimes, offsets, daily_dts, daily_offs,
                        r["regression_slope"], r["regression_intercept"],
                        label, color)

    _style_offset_ax(axes[0], "All modules — offset drift over time (UTC-corrected)")

    module_names = [r["module"].replace("module_", "").title() for r in results]
    ppms = [r["drift_ppm"] for r in results]
    bar_colors = [colors[i % len(colors)] for i in range(len(results))]

    bars = axes[1].bar(module_names, ppms, color=bar_colors, edgecolor="black", linewidth=0.7)
    axes[1].set_title("Overall drift rate per module")
    axes[1].set_xlabel("Module")
    axes[1].set_ylabel("Drift rate (ppm)")
    axes[1].axhline(0, color="black", linewidth=0.8)
    axes[1].grid(True, axis="y", alpha=0.3)

    ppm_range = max(ppms) - min(ppms) if len(ppms) > 1 else abs(ppms[0]) + 1
    for bar, ppm in zip(bars, ppms):
        axes[1].text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + ppm_range * 0.02,
                     f"{ppm:.1f}", ha="center", va="bottom", fontsize=9)

    fig.tight_layout(pad=2.0)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Combined plot saved: {plot_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    csv_files = sorted(DRIFT_DIR.glob("module_*.csv"))

    if not csv_files:
        print(f"No module CSV files found in {DRIFT_DIR}")
        return

    print(f"Found {len(csv_files)} module CSV(s): {[f.name for f in csv_files]}")

    results = []
    for csv_path in csv_files:
        result = analyze_module(csv_path)
        if result is None:
            print(f"  {csv_path.name}: insufficient data, skipping")
            continue

        print(f"  {result['module']}: {result['n_measurements']} measurements, "
              f"{result['drift_ppm']:.2f} ppm, {result['drift_s_per_day']:.2f} s/day")

        report_path = RESULTS_DIR / result["module"] / "drift_report.txt"
        write_module_report(result, report_path)
        print(f"    Report: {report_path}")

        plot_module(result, RESULTS_DIR / result["module"] / "drift_plot.png")

        results.append(result)

    if not results:
        print("No valid results to write.")
        return

    write_shared_csv(results, RESULTS_DIR / "drift_results.csv")
    print(f"CSV saved: {RESULTS_DIR / 'drift_results.csv'}")

    plot_all_modules(results, RESULTS_DIR / "drift_plot.png")

    print("\nSummary:")
    print(f"  {'Module':<20} {'Drift (ppm)':>12} {'Drift (s/day)':>14}")
    print(f"  {'-'*20} {'-'*12} {'-'*14}")
    for r in results:
        print(f"  {r['module']:<20} {r['drift_ppm']:>12.3f} {r['drift_s_per_day']:>14.4f}")


if __name__ == "__main__":
    main()
