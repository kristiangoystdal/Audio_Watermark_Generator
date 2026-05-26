import numpy as np
import matplotlib.pyplot as plt

drift_rates = [0.01, 0.59, 0.88, 0.90, 0.93, 2.01]  # extra seconds per minute

baseline = list(range(1, 61))  # ideal 1 to 60 minutes
drifts = {drift_rate: [] for drift_rate in drift_rates}

for minute in range(1, 61):
    for drift_rate in drift_rates:
        # RTC interval in minutes, but real time adds drift_rate seconds per minute
        actual_seconds = minute * 60 + (minute * drift_rate)
        actual_minutes = actual_seconds / 60
        drifts[drift_rate].append(actual_minutes)

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(
    range(1, 61),
    baseline,
    label="Baseline (ideal)",
    linewidth=2,
    color="black",
    linestyle="--",
)

colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
for drift_rate, color in zip(drift_rates, colors):
    plt.plot(
        range(1, 61),
        drifts[drift_rate],
        label=f"Drift: +{drift_rate} sec/min",
        linewidth=2,
        color=color,
    )

plt.title("Clock Drift Over Time", fontsize=14, fontweight="bold")
plt.xlabel("RTC Interval Setting (minutes)", fontsize=12)
plt.ylabel("Real Time Elapsed (minutes)", fontsize=12)
plt.legend(loc="upper left", fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("clock_drift.png", dpi=300, bbox_inches="tight")
print("✓ Saved clock_drift.png")

# Print final drift in seconds after 60 minutes
print("\n=== Final Drift After 60 Minutes ===")
for drift_rate in drift_rates:
    final_drift = (drifts[drift_rate][-1] - baseline[-1]) * 60
    print(f"+{drift_rate} sec/min → {final_drift:.1f} seconds")
