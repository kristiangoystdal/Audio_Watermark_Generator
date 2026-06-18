import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from helper.helper import *

MEASURES_DIR = Path(__file__).parent.parent / "test_files" / "current_measures"
WINDOW = 100

arrow_files = sorted(MEASURES_DIR.glob("*.arrow"))
if not arrow_files:
    print(f"No .arrow files found in {MEASURES_DIR}")
    exit(1)

for filepath in arrow_files:
    df = pd.read_feather(filepath)

    df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    current_raw = df["Channel A Current"] * 1000
    first_active_idx = (current_raw > 0).idxmax()
    df = df.iloc[first_active_idx:].copy()
    df["timestamp"] = df["timestamp"] - df["timestamp"].iloc[0]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    fig.suptitle(filepath.name)

    voltage = df["Channel A Voltage"]
    axes[0].plot(df["timestamp"], voltage, alpha=0.4, label="raw")
    axes[0].plot(df["timestamp"], voltage.rolling(WINDOW, center=True).mean(), label=f"rolling mean ({WINDOW})")
    axes[0].set_ylabel("Voltage (V)")
    axes[0].legend()

    current = df["Channel A Current"] * 1000
    axes[1].plot(df["timestamp"], current, alpha=0.4, label="raw")
    axes[1].plot(df["timestamp"], current.rolling(WINDOW, center=True).mean(), label=f"rolling mean ({WINDOW})")
    axes[1].set_ylabel("Current (mA)")
    axes[1].legend()

    power = df["Channel A Power"] * 1000
    axes[2].plot(df["timestamp"], power, alpha=0.4, label="raw")
    axes[2].plot(df["timestamp"], power.rolling(WINDOW, center=True).mean(), label=f"rolling mean ({WINDOW})")
    axes[2].set_ylabel("Power (mW)")
    axes[2].legend()

    axes[2].set_xlabel("Time (s)")
    plt.tight_layout()
    # plt.show()
    save_plot_to_results_folder(fig, "current_measures", filepath.stem + ".png", dpi=300)
