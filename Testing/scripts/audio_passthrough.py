import os
import time
import numpy as np
import soundfile as sf
import pandas as pd
import matplotlib.pyplot as plt
from helper.helper import *

file_paths = find_wav_files("audio_passthrough")
print(f"Found {len(file_paths)} .wav files for testing.")

file_name_list = []
dominant_freq_list = []
amplitude_db_list = []
amplitude_list = []
samplerate_list = []
duration_list = []

# Perform FFT on each file and collect results
fft_data = []
for file_path in file_paths:
    data, samplerate = sf.read(file_path)
    # If stereo, take only one channel
    if len(data.shape) > 1:
        data = data[:, 0]
    # Crop to 10 seconds
    data = data[: samplerate * 10]
    # Perform FFT
    fft_result = np.fft.fft(data)
    freqs = np.fft.fftfreq(len(data), 1 / samplerate)
    # Get dominant frequency (positive frequencies only)
    half = len(fft_result) // 2
    idx = np.argmax(np.abs(fft_result[:half]))
    freq = freqs[idx]
    amp = np.abs(fft_result[idx])
    amp_db = 20 * np.log10(amp) if amp > 0 else -np.inf
    sr = samplerate
    print(
        f"File: {os.path.basename(file_path)}, Dominant Frequency: {freq:.2f} Hz, Amplitude: {amp:.2f} ({amp_db:.2f} dB)"
    )
    print(f"Samplerate: {sr} Hz, Duration: {len(data) / sr:.2f} seconds")

    file_name_list.append(os.path.basename(file_path))
    dominant_freq_list.append(f"{freq:.2f}")
    amplitude_db_list.append(f"{amp_db:.2f}")
    amplitude_list.append(f"{amp:.2f}")
    samplerate_list.append(f"{sr}")
    duration_list.append(f"{len(data) / sr:.2f}")

    fft_data.append((file_path, fft_result, freqs, sr))

# Compute global y-axis limits across all plots
all_db = np.concatenate([
    20 * np.log10(np.abs(fft_result[: len(fft_result) // 2]) + 1e-12)
    for _, fft_result, _, _ in fft_data
])
y_min, y_max = all_db.min(), all_db.max()
y_margin = (y_max - y_min) * 0.05

# Save FFT plots with a consistent scale
for file_path, fft_result, freqs, sr in fft_data:
    plt.figure(figsize=(10, 6))
    plt.plot(
        freqs[: len(freqs) // 2],
        20 * np.log10(np.abs(fft_result[: len(fft_result) // 2]) + 1e-12),
    )
    plt.title(f"FFT of {os.path.basename(file_path)}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude (dB)")
    plt.xlim(0, sr / 2)
    plt.ylim(y_min - y_margin, y_max + y_margin)
    plt.grid()

    save_plot_to_results_folder(
        plt, "audio_passthrough", f"{os.path.basename(file_path)}_fft.png"
    )

# Save results to CSV with headers
results_df = pd.DataFrame(
    {
        "File Name": file_name_list,
        "Dominant Frequency (Hz)": dominant_freq_list,
        "Amplitude (dB)": amplitude_db_list,
        "Amplitude": amplitude_list,
        "Samplerate (Hz)": samplerate_list,
        "Duration (s)": duration_list,
    }
)

# Sort the data based on the frequency in the filename (if it contains a frequency) or by the dominant frequency
results_df["Dominant Frequency (Hz)"] = results_df["Dominant Frequency (Hz)"].astype(
    float
)
results_df = results_df.sort_values(by="Dominant Frequency (Hz)").reset_index(drop=True)

save_results_to_csv(
    [
        "File Name",
        "Dominant Frequency (Hz)",
        "Amplitude (dB)",
        "Amplitude",
        "Samplerate (Hz)",
        "Duration (s)",
    ],
    results_df.to_dict(orient="records"),
    "audio_passthrough",
    "audio_passthrough_results.csv",
)
