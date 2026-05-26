from helper import *

from datetime import datetime, timedelta

files = find_txt_files("rtc_measurements/offset")
print(f"Found {len(files)} files:")
for file in files:
    print(f"  {file}")

all_offsets = {}

for file in files:
    print(f"\nProcessing file: {file}")

    with open(file, "r") as f:
        lines = f.readlines()

    real_times = []
    module_times = []

    if len(lines) < 2:
        print("  Not enough data in file.")
        continue

    for line in lines:
        if "[TRIGGER]" in line:
            times_str = line.split("[TRIGGER]")[1].strip()
            real_time_str, module_time_str = times_str.split(",")

            real_time_split = real_time_str.split(":")
            module_time_split = module_time_str.split(":")

            real_time = f"{real_time_split[1].strip()}:{real_time_split[2].strip()}:{real_time_split[3].strip()}"
            module_time = f"{module_time_split[1].strip()}:{module_time_split[2].strip()}:{module_time_split[3].strip()}"

            real_times.append(real_time)
            module_times.append(module_time)

    if not real_times or not module_times:
        print("  No valid time data found in file.")
        continue

    adjusted_module_times = []
    for module_time in module_times:
        module_time_obj = datetime.strptime(module_time, "%H:%M:%S")
        adjusted_module_time_obj = module_time_obj + timedelta(hours=2)
        adjusted_module_times.append(adjusted_module_time_obj.strftime("%H:%M:%S"))

    offsets = []
    for real_time, module_time in zip(real_times, adjusted_module_times):
        real_time_obj = datetime.strptime(real_time, "%H:%M:%S")
        module_time_obj = datetime.strptime(module_time, "%H:%M:%S")
        offsets.append((module_time_obj - real_time_obj).total_seconds())

    label = extract_subfolder_name(file) or file.split("/")[-1].replace(".txt", "")
    label = file.split("/")[-1].replace(".txt", "")
    all_offsets[label] = offsets

    print(f"  {len(offsets)} measurements, mean offset: {sum(offsets)/len(offsets):.2f}s")

# --- Summary plot ---
all_offsets_flat = [o for offsets in all_offsets.values() for o in offsets]

mean_offset = np.mean(all_offsets_flat)
std_offset = np.std(all_offsets_flat)

int_vals = sorted(set(int(round(o)) for o in all_offsets_flat))
bin_edges = [v - 0.5 for v in int_vals] + [int_vals[-1] + 0.5]

fig, ax = plt.subplots(figsize=(5, 3))
ax.hist(all_offsets_flat, bins=bin_edges, edgecolor="black", alpha=0.7, orientation="horizontal", rwidth=0.5)
ax.set_title("RTC Offset Distribution (All Modules)")
ax.set_xlabel("Count")
ax.set_ylabel("Offset (s)")
ax.set_yticks(int_vals)
ax.set_ylim(int_vals[0] - 0.5, int_vals[-1] + 0.5)
ax.grid(True, axis="x")

print(f"\nOverall stats: mean={mean_offset:.2f}s, std={std_offset:.2f}s, min={min(all_offsets_flat):.2f}s, max={max(all_offsets_flat):.2f}s")

plt.tight_layout()
save_plot_to_results_folder(fig, "offset", "offset_summary.png")
plt.show()
