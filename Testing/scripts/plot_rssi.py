import csv
import os
import re
from datetime import datetime, timedelta
from helper import *
import numpy as np
import matplotlib.pyplot as plt


def parse_timecode(tim):
    """Parse HHMMSSCCDDMMYYYY into a datetime (centiseconds ignored)."""
    return datetime.strptime(tim[:6] + tim[8:], "%H%M%S%d%m%Y")


def extract_rssi_data(log_file):
    """Parse RSSI, TIM, and MID values from log file."""
    rssi_data = []
    with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    rssi_pattern = r"RSSI:\s*(-?\d+)\s*dBm"
    tim_pattern = r"TIM(\d{16})"
    mid_pattern = r"MID([0-9A-Fa-f]+)"

    lines = content.split("Entering STOP mode...")
    for block in lines:
        rssi_match = re.search(rssi_pattern, block)
        tim_match = re.search(tim_pattern, block)
        if rssi_match and tim_match:
            rssi = int(rssi_match.group(1))
            tim = tim_match.group(1)
            mid_match = re.search(mid_pattern, block)
            mid = mid_match.group(1) if mid_match else None
            rssi_data.append((tim, rssi, mid))

    return rssi_data


def _has_duplicate_mids(rssi_data):
    """True if any MID appears more than once, or if no MIDs are present."""
    mids = [mid for _, _, mid in rssi_data if mid is not None]
    return not mids or len(mids) != len(set(mids))


def parse_range_params(filename):
    """Extract start distance and step from filename like '1_3_module_2.txt'."""
    match = re.match(r"^(\d+)_(\d+)", os.path.basename(filename))
    if match:
        return int(match.group(1)), int(match.group(2))
    return None, None


def parse_trailing_number(filename):
    """Extract trailing group number from filename stem like 'foo bar 3.txt' -> 3."""
    stem = os.path.splitext(os.path.basename(filename))[0]
    match = re.search(r" (\d+)$", stem)
    return int(match.group(1)) if match else None


def process_group(
    log_files, start, step, subfolder_name, result_subfolder, trailing_num=None
):
    """Plot and save one group of files sharing the same start and step values."""
    module_series = {}

    if start is not None and step != 0:
        # Collect all data first so we can align across files
        all_rssi_data = {}
        for log_file in sorted(log_files):
            module_name = os.path.basename(log_file).replace(".txt", "")
            rssi_data = extract_rssi_data(log_file)
            all_rssi_data[module_name] = rssi_data
            print(
                f"    {module_name}: start={start} m, step={step} m, {len(rssi_data)} packets"
            )

        use_time = any(_has_duplicate_mids(data) for data in all_rssi_data.values())

        if not use_time:
            print("    Alignment: MID-based")
            all_mids = sorted(
                set(
                    mid
                    for data in all_rssi_data.values()
                    for _, _, mid in data
                    if mid is not None
                )
            )
            mid_to_rank = {mid: i for i, mid in enumerate(all_mids)}
            for module_name, rssi_data in all_rssi_data.items():
                pairs = sorted(
                    (start + mid_to_rank[mid] * step, rssi)
                    for _, rssi, mid in rssi_data
                    if mid is not None and mid in mid_to_rank
                )
                module_series[module_name] = pairs
        else:
            print("    Alignment: time-based (duplicate or missing MIDs detected)")
            all_samples = []
            for module_name, rssi_data in all_rssi_data.items():
                for tim, rssi, _mid in rssi_data:
                    all_samples.append((parse_timecode(tim), module_name, rssi))
            all_samples.sort(key=lambda x: x[0])

            clusters = []
            cluster_buf = []
            for dt, module, rssi in all_samples:
                if not cluster_buf or (dt - cluster_buf[-1][0]) > timedelta(seconds=2):
                    if cluster_buf:
                        clusters.append(cluster_buf)
                    cluster_buf = [(dt, module, rssi)]
                else:
                    cluster_buf.append((dt, module, rssi))
            if cluster_buf:
                clusters.append(cluster_buf)

            for cluster_idx, cluster in enumerate(clusters):
                x = start + cluster_idx * step
                for _, module, rssi in cluster:
                    module_series.setdefault(module, []).append((x, rssi))
    else:
        for log_file in sorted(log_files):
            module_name = os.path.basename(log_file).replace(".txt", "")
            rssi_data = extract_rssi_data(log_file)
            print(
                f"    {module_name}: start={start} m, step={step} m, {len(rssi_data)} packets"
            )

            if start is None:
                pairs = [(idx, rssi) for idx, (_, rssi, _mid) in enumerate(rssi_data)]
            else:  # step == 0
                pairs = sorted(
                    [(parse_timecode(tim), rssi) for tim, rssi, _mid in rssi_data],
                    key=lambda x: x[0],
                )
            module_series[module_name] = pairs

    all_modules = sorted(module_series.keys())

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.tab10(range(len(all_modules)))

    line_styles = ["-", "--", ":", "-."]
    markers = ["o", "s", "^", "D", "v", "P", "*", "X"]

    for i, (module, color) in enumerate(zip(all_modules, colors)):
        pairs = module_series[module]
        ax.plot(
            [p[0] for p in pairs],
            [p[1] for p in pairs],
            marker=markers[i % len(markers)],
            linestyle=line_styles[i % len(line_styles)],
            label=module,
            color=color,
            linewidth=2,
            markersize=6,
        )

    if start is None:
        x_label = "Measurement #"
        title_suffix = "sequential"
    elif step == 0:
        x_label = "Time"
        title_suffix = f"fixed at {start} m"
        ax.xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter("%H:%M:%S"))
        plt.xticks(rotation=45, ha="right")
    else:
        x_label = "Distance (m)"
        title_suffix = f"from {start} m, {step} m step"

    ax.set_xlabel(x_label, fontsize=12)
    ax.set_ylabel("RSSI (dBm)", fontsize=12)
    group_tag = f" [group {trailing_num}]" if trailing_num is not None else ""
    ax.set_title(
        f"RSSI vs Distance — {subfolder_name}{group_tag} ({title_suffix})",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    suffix = "sequential" if start is None else f"from{start}_step{step}"
    if trailing_num is not None:
        suffix += f"_group{trailing_num}"
    save_plot_to_results_folder(fig, result_subfolder, f"rssi_vs_distance_{suffix}.png")
    plt.close(fig)

    headers = ["Module", x_label, "RSSI (dBm)"]
    rows = [
        {"Module": module, x_label: x, "RSSI (dBm)": rssi}
        for module in all_modules
        for x, rssi in module_series[module]
    ]
    save_results_to_csv(headers, rows, result_subfolder, f"rssi_results_{suffix}.csv")


def process_subfolder(subfolder_path, subfolder_name, result_subfolder):
    """Process files directly in this folder, then recurse into subdirectories."""
    entries = sorted(os.scandir(subfolder_path), key=lambda e: e.name)
    txt_files = [
        e.path for e in entries if e.is_file() and e.name.lower().endswith(".txt")
    ]
    subdirs = [e for e in entries if e.is_dir()]

    if txt_files:
        groups = {}
        for log_file in txt_files:
            start, step = parse_range_params(log_file)
            trailing_num = parse_trailing_number(log_file)
            groups.setdefault((start, step, trailing_num), []).append(log_file)

        for (start, step, trailing_num), files in sorted(
            groups.items(),
            key=lambda x: (x[0][0] is None, x[0][0], x[0][1], x[0][2] is None, x[0][2]),
        ):
            group_tag = f" group={trailing_num}" if trailing_num is not None else ""
            if start is None:
                print(f"  Group sequential{group_tag} ({len(files)} file(s)):")
            else:
                print(
                    f"  Group start={start} m, step={step} m{group_tag} ({len(files)} file(s)):"
                )
            process_group(
                files, start, step, subfolder_name, result_subfolder, trailing_num
            )

        plot_step0_averages(txt_files, result_subfolder, subfolder_name)

    for entry in subdirs:
        sub_name = entry.name
        print(f"\n--- {subfolder_name.upper()}/{sub_name.upper()} ---")
        process_subfolder(
            entry.path,
            f"{subfolder_name}/{sub_name}",
            os.path.join(result_subfolder, sub_name),
        )


def extract_module_id(filename):
    """Strip the leading 'start_step_' prefix from a filename stem to get the module id."""
    stem = os.path.splitext(os.path.basename(filename))[0]
    match = re.match(r"^\d+_\d+_(.*)", stem)
    return match.group(1) if match else stem


def _fit_best_regression(xs, ys):
    """Try linear and logarithmic regression; return (name, r2, predict_fn, label) for the better fit."""
    ss_tot = np.sum((ys - np.mean(ys)) ** 2)

    def r2(y_pred):
        ss_res = np.sum((ys - y_pred) ** 2)
        return 1 - ss_res / ss_tot if ss_tot > 0 else 0

    candidates = []

    # Linear
    c1 = np.polyfit(xs, ys, 1)
    candidates.append(
        (
            "linear",
            r2(np.polyval(c1, xs)),
            lambda x, c=c1: np.polyval(c, x),
            f"y = {c1[0]:.3f}x + {c1[1]:.3f}",
        )
    )

    # Logarithmic (requires x > 0)
    if np.all(xs > 0):
        c_log = np.polyfit(np.log10(xs), ys, 1)
        candidates.append(
            (
                "logarithmic",
                r2(np.polyval(c_log, np.log10(xs))),
                lambda x, c=c_log: np.polyval(c, np.log10(x)),
                f"y = {c_log[0]:.3f}·log₁₀(x) + {c_log[1]:.3f}",
            )
        )

    return max(candidates, key=lambda t: t[1])


def plot_step0_averages(txt_files, result_folder, subfolder_name):
    """From a flat list of files, compute average RSSI per module per distance for step=0 files and plot."""
    # module_id -> {distance -> [rssi values]}
    data = {}

    for fpath in txt_files:
        start, step = parse_range_params(fpath)
        if start is None or step != 0:
            continue

        module_id = extract_module_id(fpath)
        rssi_data = extract_rssi_data(fpath)
        if not rssi_data:
            continue

        avg = sum(r for _, r, _ in rssi_data) / len(rssi_data)
        data.setdefault(module_id, {}).setdefault(start, []).append(avg)

    if not data:
        print("No step=0 files found for average plot.")
        return

    # Collapse multiple measurements at the same distance by averaging
    module_series = {}
    for module_id, dist_map in data.items():
        pairs = sorted((dist, sum(vals) / len(vals)) for dist, vals in dist_map.items())
        module_series[module_id] = pairs

    all_modules = sorted(module_series.keys())

    THRESHOLD = -100
    print(
        f"  Linear regression — estimated distance where RSSI crosses {THRESHOLD} dBm:"
    )
    for module in all_modules:
        pairs = module_series[module]
        if len(pairs) < 2:
            print(f"    {module}: not enough data points")
            continue
        xs = np.array([p[0] for p in pairs], dtype=float)
        ys = np.array([p[1] for p in pairs], dtype=float)
        slope, intercept = np.polyfit(xs, ys, 1)
        if slope >= 0:
            print(
                f"    {module}: RSSI not decreasing with distance (slope={slope:.3f}), no crossing predicted"
            )
        else:
            crossing = (THRESHOLD - intercept) / slope
            print(
                f"    {module}: ~{crossing:.1f} m  (slope={slope:.3f} dBm/m, intercept={intercept:.1f} dBm)"
            )

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.tab10(range(len(all_modules)))
    line_styles = ["-", "--", ":", "-."]
    markers = ["o", "s", "^", "D", "v", "P", "*", "X"]

    for i, (module, color) in enumerate(zip(all_modules, colors)):
        pairs = module_series[module]
        ax.plot(
            [p[0] for p in pairs],
            [p[1] for p in pairs],
            marker=markers[i % len(markers)],
            linestyle=line_styles[i % len(line_styles)],
            label=module,
            color=color,
            linewidth=2,
            markersize=6,
        )

    ax.set_xlabel("Distance (m)", fontsize=12)
    ax.set_ylabel("Average RSSI (dBm)", fontsize=12)
    ax.set_title(
        f"Average RSSI vs Distance — {subfolder_name} (step=0 files)",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    save_plot_to_results_folder(fig, result_folder, "rssi_averages_step0.png")
    plt.close(fig)

    headers = ["Module", "Distance (m)", "Average RSSI (dBm)"]
    rows = [
        {"Module": module, "Distance (m)": dist, "Average RSSI (dBm)": round(avg, 2)}
        for module in all_modules
        for dist, avg in module_series[module]
    ]
    save_results_to_csv(headers, rows, result_folder, "rssi_averages_step0.csv")

    _plot_combined_average_with_regression(module_series, result_folder, subfolder_name)


def read_csv_data(csv_path):
    """Read range-test CSV and return list of (distance_m, module, rssi)."""
    rows = []
    with open(csv_path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                (int(row["Distance (m)"]), row["Module"], int(row["RSSI (dBm)"]))
            )
    return rows


def process_csv_files(csv_files, folder_name, result_folder):
    """Plot average RSSI vs distance (m) for a collection of range-test CSV files."""
    data = {}
    for fpath in csv_files:
        for dist, module, rssi in read_csv_data(fpath):
            data.setdefault(module, {}).setdefault(dist, []).append(rssi)

    if not data:
        return

    module_series = {
        module: sorted((dist, sum(vals) / len(vals)) for dist, vals in dist_map.items())
        for module, dist_map in data.items()
    }

    all_modules = sorted(module_series.keys())

    all_distances = sorted({d for dist_map in data.values() for d in dist_map})
    expected_per_dist = {
        d: max(len(data[m].get(d, [])) for m in data) for d in all_distances
    }
    total_expected = sum(expected_per_dist.values())
    print(f"\n  Packet statistics (expected: {total_expected} packets per module):")
    for module in all_modules:
        received = sum(len(data[module].get(d, [])) for d in all_distances)
        lost = total_expected - received
        rate = received / total_expected * 100 if total_expected else 0
        print(
            f"    {module}: {received} received, {lost} lost, success rate: {rate:.1f}%"
        )
        for d in all_distances:
            exp = expected_per_dist[d]
            got = len(data[module].get(d, []))
            d_lost = exp - got
            d_rate = got / exp * 100 if exp else 0
            print(f"      {d:>5} cm: {got}/{exp} packets, {d_lost} lost ({d_rate:.0f}%)")

    colors = plt.cm.tab10(range(len(all_modules)))
    line_styles = ["-", "--", ":", "-."]
    markers = ["o", "s", "^", "D", "v", "P", "*", "X"]

    fig_pdr, ax_pdr = plt.subplots(figsize=(10, 6))
    for i, (module, color) in enumerate(zip(all_modules, colors)):
        rates = [
            (d, len(data[module].get(d, [])) / expected_per_dist[d] * 100)
            for d in all_distances
        ]
        ax_pdr.plot(
            [r[0] for r in rates],
            [r[1] for r in rates],
            marker=markers[i % len(markers)],
            linestyle=line_styles[i % len(line_styles)],
            label=module,
            color=color,
            linewidth=2,
            markersize=6,
        )
    ax_pdr.set_xlabel("Distance (cm)", fontsize=12)
    ax_pdr.set_ylabel("Packet Reception Rate (%)", fontsize=12)
    ax_pdr.set_title(
        f"Packet Reception Rate vs Distance — {folder_name}",
        fontsize=14,
        fontweight="bold",
    )
    ax_pdr.set_ylim(0, 105)
    ax_pdr.grid(True, alpha=0.3)
    ax_pdr.legend(fontsize=10)
    save_plot_to_results_folder(fig_pdr, result_folder, "packet_reception_rate.png")
    plt.close(fig_pdr)

    fig, ax = plt.subplots(figsize=(10, 6))
    for i, (module, color) in enumerate(zip(all_modules, colors)):
        pairs = module_series[module]
        ax.plot(
            [p[0] for p in pairs],
            [p[1] for p in pairs],
            marker=markers[i % len(markers)],
            linestyle=line_styles[i % len(line_styles)],
            label=module,
            color=color,
            linewidth=2,
            markersize=6,
        )

    ax.set_xlabel("Distance (m)", fontsize=12)
    ax.set_ylabel("Average RSSI (dBm)", fontsize=12)
    ax.set_title(
        f"Average RSSI vs Distance — {folder_name}",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    save_plot_to_results_folder(fig, result_folder, "rssi_vs_distance.png")
    plt.close(fig)

    headers = ["Module", "Distance (cm)", "Average RSSI (dBm)", "Packet Reception Rate (%)"]
    rows_out = [
        {
            "Module": module,
            "Distance (cm)": dist,
            "Average RSSI (dBm)": round(avg, 2),
            "Packet Reception Rate (%)": round(
                len(data[module].get(dist, [])) / expected_per_dist[dist] * 100, 1
            ),
        }
        for module in all_modules
        for dist, avg in module_series[module]
    ]
    save_results_to_csv(headers, rows_out, result_folder, "rssi_vs_distance.csv")

    _plot_combined_average_with_regression(
        module_series, result_folder, folder_name, x_label="Distance (m)"
    )


def _plot_combined_average_with_regression(
    module_series, result_folder, subfolder_name, x_label="Distance (m)"
):
    """Average all modules at each distance, fit best regression, and save the plot."""
    # Collect all per-distance averages across modules
    dist_to_values = {}
    for pairs in module_series.values():
        for dist, avg in pairs:
            dist_to_values.setdefault(dist, []).append(avg)

    if len(dist_to_values) < 2:
        print("  Not enough distances for combined average regression plot.")
        return

    combined = sorted(
        (dist, sum(vals) / len(vals)) for dist, vals in dist_to_values.items()
    )
    xs = np.array([p[0] for p in combined], dtype=float)
    ys = np.array([p[1] for p in combined], dtype=float)

    name, r2, predict_fn, eq_label = _fit_best_regression(xs, ys)
    print(
        f"  Combined average regression ({subfolder_name}): {name}  {eq_label}  R²={r2:.4f}"
    )

    xs_smooth = np.linspace(xs.min(), xs.max(), 300)
    ys_smooth = predict_fn(xs_smooth)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(
        xs, ys, color="steelblue", s=60, zorder=3, label="Average RSSI (all modules)"
    )
    ax.plot(
        xs_smooth,
        ys_smooth,
        color="tomato",
        linewidth=2,
        label=f"{name} fit: {eq_label}\n$R^2 = {r2:.4f}$",
    )

    ax.set_xlabel(x_label, fontsize=12)
    ax.set_ylabel("Average RSSI (dBm)", fontsize=12)
    ax.set_title(
        f"Combined Average RSSI vs Distance — {subfolder_name}\n(best-fit regression)",
        fontsize=14,
        fontweight="bold",
    )
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)

    save_plot_to_results_folder(
        fig, result_folder, "rssi_combined_average_regression.png"
    )
    plt.close(fig)

    headers = ["Distance (m)", "Combined Average RSSI (dBm)", "Regression fit"]
    rows = [
        {
            "Distance (m)": dist,
            "Combined Average RSSI (dBm)": round(avg, 2),
            "Regression fit": round(float(predict_fn(dist)), 2),
        }
        for dist, avg in combined
    ]
    save_results_to_csv(
        headers, rows, result_folder, "rssi_combined_average_regression.csv"
    )


def main():
    base_folder = os.path.join("test_files", "range_tests")

    if not os.path.isdir(base_folder):
        print(f"Folder not found: {base_folder}")
        return

    entries = sorted(os.scandir(base_folder), key=lambda e: e.name)
    csv_files = [
        e.path for e in entries if e.is_file() and e.name.lower().endswith(".csv")
    ]
    subfolders = [e for e in entries if e.is_dir()]

    if csv_files:
        print(f"\n--- RANGE TESTS (CSV) ---")
        process_csv_files(csv_files, "range_tests", "range_test")

    for entry in subfolders:
        result_subfolder = os.path.join("range_test", entry.name)
        print(f"\n--- {entry.name.upper()} ---")
        process_subfolder(entry.path, entry.name, result_subfolder)

    if not csv_files and not subfolders:
        print(f"No files or subfolders found in {base_folder}")
        return

    print("\nDone!")


if __name__ == "__main__":
    main()
