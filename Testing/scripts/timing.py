import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from helper.helper import *

interval_min = 1

filenames_to_test = find_txt_files("timing")


def calculate_and_plot_time_differences(filename):
    """Parse data file and analyze time differences."""
    try:
        with open(filename, "r") as f:
            data = f.read()
    except FileNotFoundError:
        print(f"Warning: {filename} not found, skipping.")
        return None

    lines = data.strip().split("\n")

    start_times = []
    end_times = []
    messages = []
    timestamps = []
    valid_indices = []  # Track which rows have valid timestamps

    for line in lines:
        if not line.strip():
            continue

        parts = line.split("\t")
        if len(parts) < 3:
            continue

        try:
            start_time = float(parts[0])
            end_time = float(parts[1])
            message_part = "\t".join(parts[2:])  # Handle tabs in message
        except ValueError:
            continue

        start_times.append(start_time)
        end_times.append(end_time)
        messages.append(message_part)

        # Extract timestamp if present
        if "Time:" in message_part:
            try:
                time_part = message_part.split("|")[0].strip()
                timestamp = time_part.split("Time:")[1].strip()
                timestamp_time = timestamp.split(" on ")[0].strip()
                timestamps.append(timestamp_time)
                valid_indices.append(len(start_times) - 1)
            except (IndexError, ValueError):
                pass

    if not start_times:
        print(f"No valid data in {filename}")
        return None

    # Calculate time differences between consecutive start times
    time_diffs = np.diff(start_times)

    # Calculate time differences between consecutive timestamps
    timestamp_diffs = []
    if len(timestamps) > 1:
        for i in range(1, len(timestamps)):
            try:
                t1 = datetime.strptime(timestamps[i - 1], "%H:%M:%S")
                t2 = datetime.strptime(timestamps[i], "%H:%M:%S")
                diff = (t2 - t1).total_seconds()
                timestamp_diffs.append(diff)
            except ValueError:
                pass

    # Plot
    plt.figure(figsize=(10, 5))
    plt.plot(time_diffs, label="Start Time Differences", marker="o")
    # if timestamp_diffs:
    # plt.plot(timestamp_diffs, label="Timestamp Differences", marker="x")
    plt.title(f"Time Differences for {filename}")
    plt.xlabel("Index")
    plt.ylabel("Time Difference (seconds)")
    plt.legend()
    plt.grid()

    extracted_subfolder = extract_subfolder_name(filename)
    save_path = f"{filename.split('/')[-1].split('.')[0]}_time_differences.png"
    print(f"Saving plot to: {save_path}")

    save_plot_to_results_folder(
        plt,
        "timing/" + extracted_subfolder,
        save_path,
    )
    plt.close()

    # Save results
    base_name = filename.split("/")[-2].split(".")[0]
    with open(base_name + "_time_differences.txt", "w") as f:
        f.write("Difference between start times:\n")
        f.write("\n".join([str(diff) for diff in time_diffs]))
        if timestamp_diffs:
            f.write("\n\nDifference between timestamps:\n")
            f.write("\n".join([str(diff) for diff in timestamp_diffs]))

    return {
        "filename": filename,
        "time_diffs": time_diffs,
        "timestamp_diffs": timestamp_diffs,
    }


# Process all files
results = []
for filename in filenames_to_test:
    result = calculate_and_plot_time_differences(filename)
    if result:
        results.append(result)

# Print averages
print("\n--- Average Time Differences ---")
for result in results:
    filename = result["filename"]
    avg_time = np.mean(result["time_diffs"]) if len(result["time_diffs"]) > 0 else 0
    avg_timestamp = (
        np.mean(result["timestamp_diffs"]) if len(result["timestamp_diffs"]) > 0 else 0
    )
    print(
        f"{filename}: Start times avg = {avg_time:.2f}s, Timestamps avg = {avg_timestamp:.2f}s"
    )

# Plot all files together
if results:
    plt.figure(figsize=(12, 6))
    min_length = min([len(r["time_diffs"]) for r in results]) if results else 0

    for result in results:
        filename = result["filename"]
        time_diffs = result["time_diffs"][:min_length]  # Truncate to common length
        plt.plot(time_diffs, label=filename.split(".")[0], marker="o")

    plt.title("Time Differences Between Start Times (All Files)")
    plt.xlabel("Index")
    plt.ylabel("Time Difference (seconds)")
    plt.legend()
    plt.grid()
    extracted_subfolder = extract_subfolder_name(results[0]["filename"])
    save_plot_to_results_folder(
        plt,
        "timing/" + extracted_subfolder,
        "all_time_differences.png",
    )
    plt.close()
    print("\nPlots saved: individual files and all_time_differences.png")
