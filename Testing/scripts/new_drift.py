#!/usr/bin/env python3
"""
RTC Drift Analyzer v2 - Calculates module clock drift from TXT files.

Reads [TRIGGER] logs with format:
  [TRIGGER] Real: HH:MM:SS.mmm, Module: HH:MM:SS

Directory structure expected:
  test_files/rtc_measurements/drift/
    module_A/log1.txt
    module_A/log2.txt   (multiple files per module are combined)
    module_B/log1.txt
    module_C.txt        (root-level files also supported)
    ...

Outputs:
  results/drift/
    module_A/  drift_report.txt
    module_A/  drift_plot.png
    module_B/  drift_report.txt
    module_B/  drift_plot.png
    ...
    drift_results.csv   (all modules combined)
    drift_plot_all.png  (combined comparison)
"""

import re
from pathlib import Path
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import matplotlib.dates as mdates

DRIFT_DIR = Path("test_files/rtc_measurements/drift")
RESULTS_DIR = Path("results/drift")


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_txt_file(filepath):
    """
    Parse [TRIGGER] log file.
    Returns list of (real_datetime, module_datetime, offset_seconds).

    Format:
      [TRIGGER] Real: HH:MM:SS.mmm, Module: HH:MM:SS

    Offset = real_time - module_time (positive = module is slow/behind)

    Both times assumed on same day; uses 2026-01-01 as reference.
    Preserves millisecond precision from real time.
    """
    measurements = []
    pattern = (
        r"\[TRIGGER\]\s+Real:\s+(\d+):(\d+):([\d.]+),\s+Module:\s+(\d+):(\d+):(\d+)"
    )

    with open(filepath, "r") as f:
        for line_num, line in enumerate(f, 1):
            match = re.search(pattern, line)
            if not match:
                continue

            try:
                rh = int(match.group(1))
                rm = int(match.group(2))
                rs_float = float(match.group(3))

                mh = int(match.group(4))
                mm = int(match.group(5))
                ms_int = int(match.group(6))

                # Convert to seconds since midnight
                real_seconds = rh * 3600 + rm * 60 + rs_float
                module_seconds = mh * 3600 + mm * 60 + ms_int

                # Offset: positive = module behind (module is slow)
                offset = real_seconds - module_seconds

                # Create datetime with microsecond precision
                rs_int = int(rs_float)
                rs_micros = int((rs_float - rs_int) * 1e6)
                real_dt = datetime(2026, 1, 1, rh, rm, rs_int, microsecond=rs_micros)
                module_dt = datetime(2026, 1, 1, mh, mm, ms_int)

                measurements.append((real_dt, module_dt, offset))

            except (ValueError, IndexError) as e:
                print(f"    Warning: Could not parse line {line_num}: {line.strip()}")
                continue

    return measurements


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------


def linear_regression(x, y):
    """
    Compute slope and intercept of best-fit line using least squares.

    Args:
        x: list of x values (time in seconds)
        y: list of y values (offset in seconds)

    Returns:
        (slope, intercept) where y = slope*x + intercept
    """
    n = len(x)
    if n < 2:
        return 0.0, 0.0

    x_mean = sum(x) / n
    y_mean = sum(y) / n
    numerator = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, y))
    denominator = sum((xi - x_mean) ** 2 for xi in x)

    if denominator == 0:
        return 0.0, y_mean

    slope = numerator / denominator
    intercept = y_mean - slope * x_mean
    return slope, intercept


def filter_outliers(measurements):
    """
    Remove outlier measurements using median absolute deviation (MAD).
    Outliers are measurements whose offset deviates > 3*MAD from the median.
    Falls back to 30-second threshold when MAD=0 (no variation).

    Args:
        measurements: list of (real_dt, module_dt, offset) tuples

    Returns:
        Filtered list of measurements
    """
    if len(measurements) < 2:
        return measurements

    offsets = [m[2] for m in measurements]
    sorted_off = sorted(offsets)
    n = len(sorted_off)
    median = sorted_off[n // 2]

    # Median absolute deviation
    deviations = sorted([abs(o - median) for o in offsets])
    mad = deviations[n // 2]

    # Threshold: 3*MAD, with small fallback for zero-variance case
    threshold = max(3.0 * mad, 0.01)

    filtered = [
        (dt_real, dt_mod, o)
        for dt_real, dt_mod, o in measurements
        if abs(o - median) <= threshold
    ]

    return filtered


def compute_rolling_mean(measurements, window=None):
    """
    Smooth measurements with a centered rolling mean.
    Window size is adaptive: ~1/10 of measurement count, clamped to [3, 30].
    Returns list of (datetime, smoothed_offset) — one per measurement.
    """
    n = len(measurements)
    if n == 0:
        return []
    if n == 1:
        return [(measurements[0][0], measurements[0][2])]

    if window is None:
        window = max(3, min(n // 10, 30))

    half = window // 2
    result = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        pts = measurements[lo:hi]
        mean_offset = sum(m[2] for m in pts) / len(pts)
        result.append((measurements[i][0], mean_offset))

    return result


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def analyze_module_files(module_name, txt_paths):
    """
    Read and analyze one or more TXT files for a module.

    Returns dict with analysis results, or None if insufficient data.
    """
    all_measurements = []
    for p in txt_paths:
        all_measurements.extend(parse_txt_file(p))

    if not all_measurements:
        return None

    all_measurements.sort(key=lambda m: m[0])

    # Keep only the first measurement for each unique module second
    seen_mod_sec = set()
    filtered = []
    for m in all_measurements:
        key = (m[1].hour, m[1].minute, m[1].second)
        if key not in seen_mod_sec:
            seen_mod_sec.add(key)
            filtered.append(m)

    # Drop first and last, then filter outliers on the deduplicated data
    filtered = filtered[1:-1] if len(filtered) > 2 else filtered
    clean = filter_outliers(filtered)
    if not clean:
        return None

    windows = compute_rolling_mean(clean)
    if len(windows) < 2:
        return None

    # Regression on windowed data, excluding last window (may have fewer samples)
    reg_windows = windows[:-1] if len(windows) > 2 else windows
    t0 = windows[0][0]
    x_windows = [(d[0] - t0).total_seconds() for d in reg_windows]
    y_windows = [d[1] for d in reg_windows]
    slope, intercept = linear_regression(x_windows, y_windows)

    offsets = [m[2] for m in clean]
    time_span_s = (clean[-1][0] - clean[0][0]).total_seconds()
    time_span_h = time_span_s / 3600

    return {
        "module": module_name,
        "all_measurements": clean,  # (real_dt, module_dt, offset)
        "windows": windows,  # (mean_dt, mean_offset)
        "drift_ppm": slope * 1e6,
        "drift_s_per_day": slope * 86400,
        "drift_s_per_hour": slope * 3600,
        "drift_s_per_minute": slope * 60,
        "initial_offset": clean[0][2],
        "final_offset": clean[-1][2],
        "avg_offset": sum(offsets) / len(offsets),
        "n_measurements": len(clean),
        "n_windows": len(windows),
        "discarded": len(all_measurements) - len(clean),
        "time_span_seconds": time_span_s,
        "time_span_hours": time_span_h,
        "regression_slope": slope,
        "regression_intercept": intercept,
    }


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def format_offset(seconds):
    """Format offset as ±HH:MM:SS"""
    sign = "-" if seconds < 0 else "+"
    seconds = abs(float(seconds))
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    return f"{sign}{h:02d}:{m:02d}:{s:02d}"


def write_module_report(result, report_path):
    """Write detailed drift report for one module."""
    lines = []
    lines.append("=" * 75)
    lines.append(f"RTC DRIFT REPORT — {result['module'].upper()}")
    lines.append("=" * 75)
    lines.append("")

    # Data summary
    lines.append("DATA SUMMARY")
    lines.append("-" * 75)
    lines.append(f"  Total measurements  : {result['n_measurements']}")
    lines.append(f"  Discarded (outliers): {result['discarded']}")
    lines.append(f"  Windowed points     : {result['n_windows']}")
    lines.append(
        f"  Time span           : {result['time_span_hours']:.3f} hours ({result['time_span_seconds']:.0f} sec)"
    )
    lines.append("")

    # Offset summary
    lines.append("OFFSET SUMMARY")
    lines.append("-" * 75)
    lines.append(f"  Initial offset      : {format_offset(result['initial_offset'])}")
    lines.append(f"  Final offset        : {format_offset(result['final_offset'])}")
    lines.append(f"  Average offset      : {format_offset(result['avg_offset'])}")
    lines.append("")

    # Drift rates
    lines.append("DRIFT RATE")
    lines.append("-" * 75)
    lines.append(f"  Overall drift rate  : {result['drift_ppm']:+.4f} ppm")
    lines.append(f"  Drift per minute    : {result['drift_s_per_minute']:+.6f} seconds")
    lines.append(f"  Drift per hour      : {result['drift_s_per_hour']:+.4f} seconds")
    lines.append(f"  Drift per day       : {result['drift_s_per_day']:+.2f} seconds")
    lines.append(
        f"  Drift per month     : {result['drift_s_per_day'] * 30:+.1f} seconds ({result['drift_s_per_day'] * 30 / 60:+.2f} min)"
    )
    lines.append(
        f"  Drift per year      : {result['drift_s_per_day'] * 365:+.0f} seconds ({result['drift_s_per_day'] * 365 / 3600:+.2f} hours)"
    )
    lines.append("")

    # Direction
    if result["drift_ppm"] > 0.01:
        direction = "SLOW — module losing time"
    elif result["drift_ppm"] < -0.01:
        direction = "FAST — module gaining time"
    else:
        direction = "STABLE — negligible drift"
    lines.append(f"  Direction           : {direction}")
    lines.append("")

    # Sample measurements
    lines.append("=" * 75)
    lines.append("SAMPLE MEASUREMENTS (first 50)")
    lines.append("=" * 75)
    for i, (real_dt, module_dt, offset) in enumerate(result["all_measurements"][:50]):
        real_str = real_dt.strftime("%H:%M:%S.%f")[:-3]
        module_str = module_dt.strftime("%H:%M:%S")
        lines.append(f"  {real_str}  {module_str}  offset={offset:+.3f}s")

    if len(result["all_measurements"]) > 50:
        lines.append(
            f"  ... ({len(result['all_measurements']) - 50} more measurements)"
        )

    lines.append("=" * 75)

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines))


def write_summary_csv(results, csv_path):
    """Write summary CSV with all modules."""
    lines = [
        "module,n_measurements,n_windows,time_span_hours,"
        "initial_offset_s,final_offset_s,avg_offset_s,"
        "drift_ppm,drift_s_per_hour,drift_s_per_day"
    ]

    for r in results:
        line = (
            f"{r['module']},"
            f"{r['n_measurements']},"
            f"{r['n_windows']},"
            f"{r['time_span_hours']:.4f},"
            f"{r['initial_offset']:.3f},"
            f"{r['final_offset']:.3f},"
            f"{r['avg_offset']:.3f},"
            f"{r['drift_ppm']:.4f},"
            f"{r['drift_s_per_hour']:.6f},"
            f"{r['drift_s_per_day']:.4f}"
        )
        lines.append(line)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.write_text("\n".join(lines))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------


def plot_module(result, plot_path):
    """Create detailed plot for one module showing offset drift over time."""
    fig, ax = plt.subplots(figsize=(14, 6))

    # Extract data
    real_times = [m[0] for m in result["all_measurements"]]
    offsets = [m[2] for m in result["all_measurements"]]
    plot_windows = result["windows"][:-1] if len(result["windows"]) > 2 else result["windows"]
    window_times = [d[0] for d in plot_windows]
    window_offs = [d[1] for d in plot_windows]

    # Relative offset (baseline = first window)
    baseline = window_offs[0] if window_offs else 0
    offsets_rel = [o - baseline for o in offsets]
    window_offs_rel = [o - baseline for o in window_offs]

    # Regression line
    t0 = window_times[0]
    t_end = window_times[-1]
    span_s = (t_end - t0).total_seconds()
    reg_times = [t0, t_end]
    reg_y = [
        result["regression_slope"] * 0 + result["regression_intercept"] - baseline,
        result["regression_slope"] * span_s + result["regression_intercept"] - baseline,
    ]

    # Plot
    ax.scatter(
        real_times,
        offsets_rel,
        alpha=0.25,
        s=15,
        color="steelblue",
        zorder=2,
        label="Raw measurements",
    )
    ax.plot(
        window_times,
        window_offs_rel,
        "o-",
        color="darkblue",
        linewidth=2.5,
        markersize=6,
        label=f"Rolling mean ({result['n_windows']} points)",
        zorder=3,
    )
    ax.plot(
        reg_times,
        reg_y,
        "--",
        color="red",
        alpha=0.8,
        linewidth=2,
        label=f"Fit: {result['drift_ppm']:+.3f} ppm ({result['drift_s_per_day']:+.2f} s/day)",
        zorder=4,
    )

    ax.set_title(
        f"RTC Drift — {result['module'].upper()} | {result['time_span_hours']:.2f}h span",
        fontsize=14,
        fontweight="bold",
    )
    ax.set_xlabel("Local Time", fontsize=11)
    ax.set_ylabel("Offset change (seconds)", fontsize=11)
    ax.legend(loc="best", fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":")

    # Format x-axis as time
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax.xaxis.set_major_locator(mdates.AutoDateLocator())
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")

    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_ms_at_second_change(result, plot_path=None):
    """Plot the millisecond value of real time at each module second change."""
    # all_measurements already filtered to first-of-each-module-second in analyze_module_files
    measurements = result["all_measurements"]
    real_times = [m[0] for m in measurements]

    def sub_second_ms(real_dt, mod_dt):
        real_s = (
            real_dt.hour * 3600
            + real_dt.minute * 60
            + real_dt.second
            + real_dt.microsecond / 1e6
        )
        mod_s = mod_dt.hour * 3600 + mod_dt.minute * 60 + mod_dt.second
        return ((real_s - mod_s) % 1) * 1000

    ms_values = [sub_second_ms(m[0], m[1]) for m in measurements]

    fig, ax = plt.subplots(figsize=(14, 5))

    ax.scatter(real_times, ms_values, s=12, alpha=0.6, color="steelblue", zorder=2)
    ax.plot(
        real_times, ms_values, linewidth=0.8, alpha=0.4, color="steelblue", zorder=1
    )

    ax.set_title(
        f"Real clock ms when module second ticks — {result['module'].upper()}",
        fontsize=13,
        fontweight="bold",
    )
    ax.set_xlabel("Real timestamp", fontsize=11)
    ax.set_ylabel("ms into real second (0–999)", fontsize=11)
    ax.set_ylim(-10, 1010)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":")
    ax.axhline(500, color="gray", linewidth=0.6, linestyle="--", alpha=0.5)
    ax.grid(True, alpha=0.3)

    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax.xaxis.set_major_locator(mdates.AutoDateLocator())
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")

    fig.tight_layout()

    if plot_path:
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")

    # plt.show()
    plt.close(fig)


def plot_ms_all_modules(results, plot_path=None):
    """Combined ms-at-second-change plot for all modules."""
    if not results:
        return

    fig, ax = plt.subplots(figsize=(14, 6))
    colors = plt.cm.tab10.colors

    def sub_second_ms(real_dt, mod_dt):
        real_s = (
            real_dt.hour * 3600
            + real_dt.minute * 60
            + real_dt.second
            + real_dt.microsecond / 1e6
        )
        mod_s = mod_dt.hour * 3600 + mod_dt.minute * 60 + mod_dt.second
        return ((real_s - mod_s) % 1) * 1000

    for i, result in enumerate(results):
        color = colors[i % len(colors)]
        measurements = result["all_measurements"]
        real_times = [m[0] for m in measurements]
        ms_values = [sub_second_ms(m[0], m[1]) for m in measurements]

        ax.scatter(real_times, ms_values, s=10, alpha=0.5, color=color, zorder=2)
        ax.plot(
            real_times,
            ms_values,
            linewidth=0.8,
            alpha=0.4,
            color=color,
            zorder=1,
            label=result["module"],
        )

    ax.set_title(
        "Real clock ms when module second ticks — all modules",
        fontsize=13,
        fontweight="bold",
    )
    ax.set_xlabel("Real timestamp", fontsize=11)
    ax.set_ylabel("ms into real second (0–999)", fontsize=11)
    ax.set_ylim(-10, 1010)
    ax.axhline(0, color="black", linewidth=0.8, linestyle=":")
    ax.axhline(500, color="gray", linewidth=0.6, linestyle="--", alpha=0.5)
    ax.legend(loc="best", fontsize=10)
    ax.grid(True, alpha=0.3)

    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax.xaxis.set_major_locator(mdates.AutoDateLocator())
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")

    fig.tight_layout()

    if plot_path:
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")

    # plt.show()
    plt.close(fig)


def plot_all_modules(results, plot_path):
    """Create combined comparison plot for all modules."""
    if not results:
        return

    fig, axes = plt.subplots(2, 1, figsize=(14, 11))
    colors = plt.cm.tab10.colors

    # Determine time unit based on the longest module span
    max_span_s = max(r["time_span_seconds"] for r in results)
    if max_span_s >= 7200:
        time_divisor, time_unit = 3600, "hours"
    elif max_span_s >= 120:
        time_divisor, time_unit = 60, "minutes"
    else:
        time_divisor, time_unit = 1, "seconds"

    # Top: all offsets on one graph
    for i, r in enumerate(results):
        color = colors[i % len(colors)]
        label = r["module"].replace("module_", "Module ").title()

        # Exclude last (potentially incomplete) window
        plot_windows = r["windows"][:-1] if len(r["windows"]) > 2 else r["windows"]
        if not plot_windows:
            continue

        t0 = r["all_measurements"][0][0]
        baseline = plot_windows[0][1]

        win_elapsed = [(d[0] - t0).total_seconds() / time_divisor for d in plot_windows]
        window_offs_rel = [d[1] - baseline for d in plot_windows]

        axes[0].plot(
            win_elapsed,
            window_offs_rel,
            "o-",
            color=color,
            linewidth=2,
            markersize=4,
            label=label,
            zorder=3,
        )

    axes[0].set_title(
        "All Modules — Offset Drift Over Time", fontsize=13, fontweight="bold"
    )
    axes[0].set_xlabel(f"Elapsed time ({time_unit})", fontsize=11)
    axes[0].set_ylabel("Offset change (seconds)", fontsize=11)
    axes[0].legend(ncol=min(3, len(results)), fontsize=10, loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0, color="black", linewidth=0.8, linestyle=":")
    axes[0].set_xlim(left=0)

    # Bottom: drift rates as bar chart
    module_names = [r["module"].replace("module_", "").title() for r in results]
    ppms = [r["drift_ppm"] for r in results]
    bar_colors = [colors[i % len(colors)] for i in range(len(results))]

    bars = axes[1].bar(
        module_names,
        ppms,
        color=bar_colors,
        edgecolor="black",
        linewidth=0.8,
        alpha=0.8,
    )
    axes[1].set_title("Drift Rate Comparison", fontsize=13, fontweight="bold")
    axes[1].set_xlabel("Module", fontsize=11)
    axes[1].set_ylabel("Drift rate (ppm)", fontsize=11)
    axes[1].axhline(0, color="black", linewidth=0.8)
    axes[1].grid(True, axis="y", alpha=0.3)

    # Add value labels on bars
    y_vals = ppms
    ppm_range = max(y_vals) - min(y_vals) if len(y_vals) > 1 else abs(y_vals[0]) + 1
    ppm_range = max(ppm_range, 1e-6)  # Avoid division by zero

    for bar, ppm in zip(bars, ppms):
        y_pos = (
            bar.get_height() + ppm_range * 0.05
            if ppm >= 0
            else bar.get_height() - ppm_range * 0.08
        )
        axes[1].text(
            bar.get_x() + bar.get_width() / 2,
            y_pos,
            f"{ppm:.3f}",
            ha="center",
            va="bottom" if ppm >= 0 else "top",
            fontsize=10,
            fontweight="bold",
        )

    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Combined plot: {plot_path}")


# ---------------------------------------------------------------------------
# Module discovery
# ---------------------------------------------------------------------------


def collect_module_files(drift_dir):
    """
    Map each TXT file to its own module entry.

    Returns dict: {module_name: [Path]} — one entry per file.
    Module name is "{folder}/{stem}" for files in subdirectories,
    or just "{stem}" for files in the root of drift_dir.
    """
    if not drift_dir.exists():
        print(f"Error: Directory not found: {drift_dir}")
        return {}

    groups = {}
    for txt_path in sorted(drift_dir.rglob("*.txt")):
        rel = txt_path.relative_to(drift_dir)

        if rel.parent == Path("."):
            module_name = txt_path.stem
        else:
            module_name = str(rel.parent / txt_path.stem)

        groups[module_name] = [txt_path]

    return dict(sorted(groups.items()))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    """Main entry point."""
    print()
    print("=" * 75)
    print("RTC DRIFT ANALYZER v2")
    print("=" * 75)
    print()

    module_files = collect_module_files(DRIFT_DIR)

    if not module_files:
        print(f"No TXT files found in {DRIFT_DIR}")
        return

    print(
        f"Found {sum(len(v) for v in module_files.values())} TXT file(s) across {len(module_files)} module(s):"
    )
    for mod, paths in module_files.items():
        print(f"\n  [{mod}]")
        for p in paths:
            print(f"    {p.relative_to(DRIFT_DIR)}")
    print()

    results = []
    for module_name, txt_paths in module_files.items():
        print(f"\n{'='*75}")
        print(f"Analyzing: {module_name}")
        print(f"{'='*75}")

        result = analyze_module_files(module_name, txt_paths)

        if result is None:
            print("  ⚠ SKIPPED — insufficient data (< 2 windows)")
            continue

        # Print summary
        print(
            f"  ✓ {result['n_measurements']} measurements (discarded: {result['discarded']})"
        )
        print(
            f"  ✓ {result['n_windows']} windows, {result['time_span_hours']:.3f} hours span"
        )
        print(f"  ✓ Drift: {result['drift_ppm']:+.4f} ppm")
        print(
            f"    └─ {result['drift_s_per_day']:+.2f} s/day | {result['drift_s_per_hour']:+.4f} s/hour"
        )
        print(
            f"    └─ Initial: {format_offset(result['initial_offset'])}, "
            f"Final: {format_offset(result['final_offset'])}"
        )

        # Write report
        report_path = RESULTS_DIR / result["module"] / "drift_report.txt"
        write_module_report(result, report_path)
        print(f"  → Report: {report_path}")

        # Plot drift over time
        plot_path = RESULTS_DIR / result["module"] / "drift_plot.png"
        plot_module(result, plot_path)
        print(f"  → Drift plot: {plot_path}")

        # Plot ms values at each module second change
        ms_plot_path = RESULTS_DIR / result["module"] / "ms_changes_plot.png"
        plot_ms_at_second_change(result, ms_plot_path)
        print(f"  → MS plot: {ms_plot_path}")

        results.append(result)

    if not results:
        print("\n⚠ No valid results to write.")
        return

    print(f"\n{'='*75}")
    print("Summary")
    print(f"{'='*75}\n")

    # Write summary CSV
    csv_path = RESULTS_DIR / "drift_results.csv"
    write_summary_csv(results, csv_path)
    print(f"Summary CSV: {csv_path}\n")

    # Plot all modules combined
    plot_all_path = RESULTS_DIR / "drift_plot_all.png"
    plot_all_modules(results, plot_all_path)

    # Combined ms-at-second-change plot for all modules
    ms_all_path = RESULTS_DIR / "ms_changes_plot_all.png"
    plot_ms_all_modules(results, ms_all_path)
    print(f"Combined MS plot: {ms_all_path}")

    # Print summary table — compute adaptive column widths from actual data
    directions = []
    for r in results:
        if r["drift_ppm"] > 0.01:
            directions.append("SLOW (losing)")
        elif r["drift_ppm"] < -0.01:
            directions.append("FAST (gaining)")
        else:
            directions.append("STABLE")

    col_module = max(len("Module"), max(len(r["module"]) for r in results))
    col_ppm = max(len("PPM"), max(len(f"{r['drift_ppm']:.4f}") for r in results))
    col_sday = max(len("S/Day"), max(len(f"{r['drift_s_per_day']:.4f}") for r in results))
    col_dir = max(len("Direction"), max(len(d) for d in directions))

    sep =f"+-{'-'*col_module}-+-{'-'*col_ppm}-+-{'-'*col_sday}-+-{'-'*col_dir}-+"
    print(f"\n{sep}")
    print(f"| {'Module':<{col_module}} | {'PPM':>{col_ppm}} | {'S/Day':>{col_sday}} | {'Direction':>{col_dir}} |")
    print(sep)
    for r, direction in zip(results, directions):
        print(
            f"| {r['module']:<{col_module}} | {r['drift_ppm']:>{col_ppm}.4f} | {r['drift_s_per_day']:>{col_sday}.4f} | {direction:>{col_dir}} |"
        )
    print(f"{sep}\n")


if __name__ == "__main__":
    main()
