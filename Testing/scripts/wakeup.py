import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
from helper import *

file_names = find_txt_files("rtc_measurements")

file_lines = []
for file_name in file_names:
    with open(file_name, "r") as f:
        data = f.read()
        lines = data.strip().split("\n")
    file_lines.append(lines)


def calculate_stddev(times):
    """Calculate the standard deviation of intervals between wakeup times."""

    # Convert time strings to seconds since midnight
    time_seconds = []
    for t in times:
        dt = datetime.strptime(t, "%H:%M:%S")
        seconds = dt.hour * 3600 + dt.minute * 60 + dt.second
        time_seconds.append(seconds)

    # Calculate intervals
    intervals = np.diff(time_seconds)

    # Remove outliers (intervals not in the range of 50 to 70 seconds)
    intervals = intervals[(intervals >= 50) & (intervals <= 70)]

    # Calculate standard deviation
    stddev = np.std(intervals)
    return stddev


def plot_wakeup_times(times):
    """Plot interval between wakeup times to visualize intervals."""

    # Convert time strings to seconds since midnight
    time_seconds = []
    for t in times:
        dt = datetime.strptime(t, "%H:%M:%S")
        seconds = dt.hour * 3600 + dt.minute * 60 + dt.second
        time_seconds.append(seconds)
    # Calculate intervals
    intervals = np.diff(time_seconds)

    # Remove outliers (intervals not in the range of 50 to 70 seconds)
    filtered_intervals = intervals[(intervals >= 50) & (intervals <= 70)]

    # Plot intervals
    plt.figure(figsize=(10, 5))
    plt.plot(filtered_intervals, marker="o")
    plt.title("Intervals Between Wakeup Times")
    plt.xlabel("Interval Index")
    plt.ylabel("Interval (seconds)")
    plt.grid()
    save_plot_to_results_folder(plt, "wakeup", "wakeup_intervals.png")


if __name__ == "__main__":
    wakeup_times_files = []
    
    for lines in file_lines:
        file_times = []
        for line in lines:
            if "Module:" in line:
                wakeup_time = line.split("Module:")[1].strip()
                file_times.append(wakeup_time)
        wakeup_times_files.append(file_times)

    for i, file_times in enumerate(wakeup_times_files):
        stddev = calculate_stddev(file_times)
        print(
            f"Standard Deviation of Wakeup Intervals (File {i+1}): {stddev:.2f} seconds"
        )
        # plot_wakeup_times(file_times)

    # print(f"Standard Deviation of Wakeup Intervals (Group 1): {stddev_1:.2f} seconds")
    # print(f"Standard Deviation of Wakeup Intervals (Group 2): {stddev_2:.2f} seconds")
    # plot_wakeup_times(wakeup_times_1)
    # plot_wakeup_times(wakeup_times_2)
    # plot_wakeup_times(wakeup_times_3)
