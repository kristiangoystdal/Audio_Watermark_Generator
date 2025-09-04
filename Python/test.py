import csv
import os
import matplotlib.pyplot as plt

# Example DAC values (scaled version for reference)
sine_vals = [2047, 3250, 3994, 3994, 3250, 2047, 844, 100, 100, 844, 2047]
real_vals = [
    ((x - min(sine_vals)) / (max(sine_vals) - min(sine_vals))) * 3.3 for x in sine_vals
]

print("Reference DAC burst values (scaled to 3.3V):", real_vals)

# --- SETTINGS ---
filename = "test_values.csv"
output_folder = "filtered_segments"
os.makedirs(output_folder, exist_ok=True)

# --- READ FILE ---
rows = []
with open(filename, newline="", encoding="utf-8") as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        rows.append(row)

# --- FIND BURST REGIONS BASED ON BURST PULSE WIDTH (column[2]) ---
new_rows = []
temp_segment = []
for row in rows:
    if float(row[2]) > 1:  # inside burst gate
        temp_segment.append(row)
    else:
        if temp_segment:
            new_rows.append(temp_segment)
            temp_segment = []

# --- SEGMENT BURSTS BY START/END MARKERS ---
segmented = []
for segment in new_rows:
    start_idx = 0
    i = 0
    while i < len(segment) - 1:
        # Find "end-of-sequence marker" (two consecutive values near 0)
        if float(segment[i][1]) < 0.05 and float(segment[i + 1][1]) < 0.05:
            if i + 1 > start_idx:
                subseg = segment[start_idx : i + 1]
                # Keep only if it has both start and end identifiers
                if len(subseg) > len(
                    sine_vals
                ):  # crude filter: must contain full burst
                    segmented.append(subseg)
            start_idx = i + 1
            i += 1
        i += 1
    if start_idx < len(segment):
        subseg = segment[start_idx:]
        if len(subseg) > len(sine_vals):
            segmented.append(subseg)

# --- PLOT RESULTS ---
for idx, segment in enumerate(segmented):
    y_vals = [float(row[1]) for row in segment]
    plt.plot(y_vals, label=f"Segment {idx+1}")
    print(f"Segment {idx+1}: {len(y_vals)} samples")

plt.xlabel("Sample index")
plt.ylabel("Voltage (V)")
plt.title("Detected Bursts with Start+End Markers")
plt.grid()
plt.legend()
plt.show()

# --- SAVE CSV SEGMENTS ---
for i, segment in enumerate(segmented):
    output_path = os.path.join(output_folder, f"burst_with_markers_{i+1}.csv")
    with open(output_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(segment)
