from helper.helper import *

import csv
from datetime import datetime, timedelta
from pathlib import Path

OFFSET_DIR = Path("test_files/rtc_measurements/offset")

csv_files = sorted(OFFSET_DIR.glob("module_*.csv"))
print(f"Found {len(csv_files)} files:")
for f in csv_files:
    print(f"  {f}")

all_offsets = {}

for csv_path in csv_files:
    print(f"\nProcessing file: {csv_path}")

    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("  No data found in file.")
        continue

    offsets = []
    for row in rows:
        fmt = "%Y-%m-%d %H:%M:%S.%f" if "." in row["real_datetime"] else "%Y-%m-%d %H:%M:%S"
        real_dt = datetime.strptime(row["real_datetime"], fmt)
        module_dt = datetime.strptime(row["module_datetime"], "%Y-%m-%d %H:%M:%S")
        real_utc = real_dt - timedelta(hours=2)
        offsets.append((module_dt - real_utc).total_seconds())

    label = csv_path.stem
    all_offsets[label] = offsets

    print(
        f"  {len(offsets)} measurements, mean offset: {sum(offsets)/len(offsets):.2f}s"
    )

# --- Summary plot ---
all_offsets_flat = [o for offsets in all_offsets.values() for o in offsets]

mean_offset = np.mean(all_offsets_flat)
std_offset = np.std(all_offsets_flat)

int_vals = sorted(set(int(round(o)) for o in all_offsets_flat))
bin_edges = [v - 0.5 for v in int_vals] + [int_vals[-1] + 0.5]

fig, ax = plt.subplots(figsize=(5, 3))
ax.hist(
    all_offsets_flat,
    bins=bin_edges,
    edgecolor="black",
    alpha=0.7,
    orientation="horizontal",
    rwidth=0.5,
)
# ax.set_title("DS3231 RTC Offset Distribution")
ax.set_xlabel("Count")
ax.set_ylabel("Offset (s)")
ax.set_yticks(int_vals)
ax.set_ylim(int_vals[0] - 0.5, int_vals[-1] + 0.5)
ax.grid(True, axis="x")

print(
    f"\nOverall stats: mean={mean_offset:.2f}s, std={std_offset:.2f}s, min={min(all_offsets_flat):.2f}s, max={max(all_offsets_flat):.2f}s"
)

plt.tight_layout()
save_plot_to_results_folder(fig, "offset", "offset_summary.png", dpi=300)

# --- CSV report ---
report_rows = [
    {
        "module": label,
        "n_measurements": len(offsets),
        "mean_offset_s": f"{sum(offsets)/len(offsets):.3f}",
        "std_offset_s": f"{np.std(offsets):.3f}",
        "min_offset_s": f"{min(offsets):.1f}",
        "max_offset_s": f"{max(offsets):.1f}",
    }
    for label, offsets in all_offsets.items()
]
report_rows.append(
    {
        "module": "total",
        "n_measurements": len(all_offsets_flat),
        "mean_offset_s": f"{mean_offset:.3f}",
        "std_offset_s": f"{std_offset:.3f}",
        "min_offset_s": f"{min(all_offsets_flat):.1f}",
        "max_offset_s": f"{max(all_offsets_flat):.1f}",
    }
)
save_results_to_csv(
    list(report_rows[0].keys()),
    report_rows,
    "offset",
    "offset_results.csv",
)
