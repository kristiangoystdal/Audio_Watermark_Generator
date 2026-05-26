#!/usr/bin/env python3
"""
RTC Drift Analyzer - Calculates module clock drift from all txt files in drift subfolders.

Directory structure expected:
  <drift_dir>/
    module_A/  *.txt
    module_B/  *.txt
    ...

Outputs:
  <results_dir>/
    module_A/  drift_report.txt
    module_B/  drift_report.txt
    ...
  <drift_dir>/drift_results.csv   (all modules combined)
  <drift_dir>/drift_plot.png
"""

import re
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

def parse_time(time_str):
    h, m, s = map(int, time_str.split(":"))
    return h * 3600 + m * 60 + s


def parse_log_file(filepath):
    """
    Return (start_dt, measurements) where measurements is a list of
    (real_datetime, offset_seconds).  offset = real_seconds - module_seconds.
    """
    text = Path(filepath).read_text()

    date_match = re.search(r"Started: (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})", text)
    if not date_match:
        return None, []
    start_dt = datetime.strptime(date_match.group(1), "%Y-%m-%d %H:%M:%S")

    trigger_pattern = r"\[TRIGGER\] Real: (\d{2}:\d{2}:\d{2}), Module: (\d{2}:\d{2}:\d{2})"
    measurements = []
    prev_real_secs = None
    day_offset = 0

    for match in re.finditer(trigger_pattern, text):
        real_secs = parse_time(match.group(1))
        module_secs = parse_time(match.group(2))

        if prev_real_secs is not None and real_secs < prev_real_secs:
            day_offset += 86400

        abs_real_secs = real_secs + day_offset
        abs_module_secs = module_secs + day_offset
        # Real time is local (UTC+2); convert to UTC before computing offset
        abs_real_utc = abs_real_secs - 7200
        offset = abs_real_utc - abs_module_secs

        midnight = start_dt.replace(hour=0, minute=0, second=0, microsecond=0)
        real_dt = midnight + timedelta(seconds=abs_real_secs)

        measurements.append((real_dt, offset))
        prev_real_secs = real_secs

    return start_dt, measurements


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
    # 3×MAD threshold; when MAD≈0 (all offsets identical) fall back to 30 s
    threshold = max(3.0 * mad, 30.0)
    return [(dt, o) for dt, o in measurements if abs(o - median) <= threshold]


def analyze_module(module_dir):
    """Combine all txt files for one module and return analysis dict."""
    txt_files = sorted(module_dir.glob("*.txt"))

    all_measurements = []
    file_results = []

    for filepath in txt_files:
        start_dt, measurements = parse_log_file(filepath)
        if not measurements:
            continue

        clean = filter_outliers(measurements)
        discarded = len(measurements) - len(clean)
        all_measurements.extend(clean)

        if len(clean) >= 2:
            x = [(m[0] - clean[0][0]).total_seconds() for m in clean]
            y = [m[1] for m in clean]
            slope, _ = linear_regression(x, y)
            file_ppm = slope * 1e6
        else:
            file_ppm = None

        file_results.append(
            {
                "file": filepath.name,
                "start": start_dt,
                "n": len(clean),
                "discarded": discarded,
                "drift_ppm": file_ppm,
            }
        )

    if len(all_measurements) < 2:
        return None

    all_measurements.sort(key=lambda m: m[0])

    t0 = all_measurements[0][0]
    x_all = [(m[0] - t0).total_seconds() for m in all_measurements]
    y_all = [m[1] for m in all_measurements]
    slope, intercept = linear_regression(x_all, y_all)

    drift_ppm = slope * 1e6
    drift_s_per_day = slope * 86400
    offsets = [m[1] for m in all_measurements]
    time_span_h = (all_measurements[-1][0] - t0).total_seconds() / 3600

    return {
        "module": module_dir.name,
        "all_measurements": all_measurements,
        "file_results": file_results,
        "drift_ppm": drift_ppm,
        "drift_s_per_day": drift_s_per_day,
        "initial_offset": offsets[0],
        "final_offset": offsets[-1],
        "avg_offset": sum(offsets) / len(offsets),
        "n_measurements": len(all_measurements),
        "time_span_hours": time_span_h,
        "regression_slope": slope,
        "regression_intercept": intercept,
        "x_all": x_all,
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
    lines.append(f"Time span          : {result['time_span_hours']:.2f} hours")
    lines.append(f"Initial offset     : {format_offset(result['initial_offset'])}")
    lines.append(f"Final offset       : {format_offset(result['final_offset'])}")
    lines.append(f"Average offset     : {format_offset(result['avg_offset'])}")
    lines.append(f"Overall drift rate : {result['drift_ppm']:.3f} ppm")
    lines.append(f"Drift per day      : {result['drift_s_per_day']:.2f} s/day")
    lines.append("")
    lines.append("Per-file breakdown:")
    lines.append("-" * 70)
    for fr in result["file_results"]:
        ppm_str = f"{fr['drift_ppm']:.2f} ppm" if fr["drift_ppm"] is not None else "n/a"
        discard_str = f"  ({fr['discarded']} discarded)" if fr["discarded"] else ""
        lines.append(f"  {fr['file']}  [{fr['n']} measurements]{discard_str}  {ppm_str}")
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

def _draw_offset_ax(ax, datetimes, offsets, slope, intercept, label, color):
    """Plot offset-from-baseline measurements and regression line onto ax."""
    baseline = offsets[0]
    drift_from_baseline = [o - baseline for o in offsets]

    t0 = datetimes[0]
    span_s = (datetimes[-1] - t0).total_seconds()
    reg_datetimes = [t0, t0 + timedelta(seconds=span_s)]
    reg_y = [
        slope * 0 + intercept - baseline,
        slope * span_s + intercept - baseline,
    ]

    ax.plot(datetimes, drift_from_baseline, "o-", color=color, label=label,
            linewidth=1.5, markersize=5)
    ax.plot(reg_datetimes, reg_y, "--", color=color, alpha=0.5, linewidth=1)


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
    """Two-panel plot: offset over time + per-file drift bar chart."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    color = "steelblue"
    label = result["module"].replace("module_", "Module ").title()

    datetimes = [m[0] for m in result["all_measurements"]]
    offsets = [m[1] for m in result["all_measurements"]]

    _draw_offset_ax(axes[0], datetimes, offsets,
                    result["regression_slope"], result["regression_intercept"],
                    label, color)
    _style_offset_ax(axes[0], f"{label} — offset drift over time (UTC-corrected)")

    # Bar chart: per-file drift ppm
    files_with_ppm = [fr for fr in result["file_results"] if fr["drift_ppm"] is not None]
    if files_with_ppm:
        names = [fr["file"].replace("drift_log_", "").replace(".txt", "") for fr in files_with_ppm]
        ppms = [fr["drift_ppm"] for fr in files_with_ppm]
        bars = axes[1].bar(names, ppms, color=color, edgecolor="black", linewidth=0.7)
        axes[1].set_title("Per-session drift rate")
        axes[1].set_xlabel("Session")
        axes[1].set_ylabel("Drift rate (ppm)")
        axes[1].axhline(0, color="black", linewidth=0.8)
        axes[1].grid(True, axis="y", alpha=0.3)
        axes[1].tick_params(axis="x", rotation=20)
        ppm_range = max(ppms) - min(ppms) if len(ppms) > 1 else abs(ppms[0]) + 1
        for bar, ppm in zip(bars, ppms):
            axes[1].text(bar.get_x() + bar.get_width() / 2,
                         bar.get_height() + ppm_range * 0.02,
                         f"{ppm:.1f}", ha="center", va="bottom", fontsize=9)
    else:
        axes[1].set_visible(False)

    fig.tight_layout(pad=2.0)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"    Plot:   {plot_path}")


# ---------------------------------------------------------------------------
# Combined all-modules plot
# ---------------------------------------------------------------------------

def plot_all_modules(results, plot_path):
    """Two-panel plot: all modules' offset over time + drift ppm bar chart."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    colors = plt.cm.tab10.colors

    for i, r in enumerate(results):
        color = colors[i % len(colors)]
        label = r["module"].replace("module_", "Module ").title()
        datetimes = [m[0] for m in r["all_measurements"]]
        offsets = [m[1] for m in r["all_measurements"]]
        _draw_offset_ax(axes[0], datetimes, offsets,
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
    module_dirs = sorted(d for d in DRIFT_DIR.iterdir() if d.is_dir())

    if not module_dirs:
        print(f"No module subdirectories found in {DRIFT_DIR}")
        return

    print(f"Found {len(module_dirs)} module(s): {[d.name for d in module_dirs]}")

    results = []
    for module_dir in module_dirs:
        result = analyze_module(module_dir)
        if result is None:
            print(f"  {module_dir.name}: insufficient data, skipping")
            continue

        print(f"  {module_dir.name}: {result['n_measurements']} measurements, "
              f"{result['drift_ppm']:.2f} ppm, {result['drift_s_per_day']:.2f} s/day")

        report_path = RESULTS_DIR / module_dir.name / "drift_report.txt"
        write_module_report(result, report_path)
        print(f"    Report: {report_path}")

        plot_module(result, RESULTS_DIR / module_dir.name / "drift_plot.png")

        results.append(result)

    if not results:
        print("No valid results to write.")
        return

    csv_path = RESULTS_DIR / "drift_results.csv"
    write_shared_csv(results, csv_path)
    print(f"CSV saved: {csv_path}")

    plot_all_modules(results, RESULTS_DIR / "drift_plot.png")

    print("\nSummary:")
    print(f"  {'Module':<20} {'Drift (ppm)':>12} {'Drift (s/day)':>14}")
    print(f"  {'-'*20} {'-'*12} {'-'*14}")
    for r in results:
        print(f"  {r['module']:<20} {r['drift_ppm']:>12.3f} {r['drift_s_per_day']:>14.4f}")


if __name__ == "__main__":
    main()
