import os
import numpy as np
import soundfile as sf
import pandas as pd
import matplotlib.pyplot as plt
from helper.helper import *

file_paths = find_wav_files("audio_passthrough")
print(f"Found {len(file_paths)} .wav files for testing.")

# --- Per-file FFT analysis ---
fft_data = {}  # key: basename -> (fft_result, freqs, sr, amp_db)

for file_path in file_paths:
    data, samplerate = sf.read(file_path)
    if len(data.shape) > 1:
        data = data[:, 0]
    data = data[: samplerate * 10]

    fft_result = np.fft.fft(data)
    freqs = np.fft.fftfreq(len(data), 1 / samplerate)

    half = len(fft_result) // 2
    freq_resolution = freqs[1] - freqs[0]

    # Dominant frequency via argmax (diagnostic only)
    peak_idx = np.argmax(np.abs(fft_result[:half]))
    dominant_freq = freqs[peak_idx]

    # Amplitude pinned to the known test frequency from the filename
    basename = os.path.basename(file_path)
    test_freq_str = basename.split("_")[0]
    try:
        target_freq = int(test_freq_str)
        target_idx = int(round(target_freq / freq_resolution))
        target_idx = min(target_idx, half - 1)
    except ValueError:
        target_freq = dominant_freq
        target_idx = peak_idx

    amp = np.abs(fft_result[target_idx])
    amp_db = 20 * np.log10(amp) if amp > 0 else -np.inf

    fft_data[basename] = {
        "file_path": file_path,
        "fft_result": fft_result,
        "freqs": freqs,
        "sr": samplerate,
        "target_freq": target_freq,
        "dominant_freq": dominant_freq,
        "amplitude": amp,
        "amplitude_db": amp_db,
        "duration": len(data) / samplerate,
    }
    print(f"{basename}: target={target_freq} Hz, {amp_db:.2f} dB, peak={dominant_freq:.2f} Hz, sr={samplerate} Hz")

# --- Compute global y-axis limits ---
all_db = np.concatenate(
    [
        20 * np.log10(np.abs(v["fft_result"][: len(v["fft_result"]) // 2]) + 1e-12)
        for v in fft_data.values()
    ]
)
y_min, y_max = all_db.min(), all_db.max()
y_margin = (y_max - y_min) * 0.05

# --- Save FFT plots ---
for basename, d in fft_data.items():
    fft_result, freqs, sr = d["fft_result"], d["freqs"], d["sr"]
    half = len(fft_result) // 2
    plt.figure(figsize=(10, 6))
    plt.plot(freqs[:half], 20 * np.log10(np.abs(fft_result[:half]) + 1e-12))
    plt.title(f"FFT of {basename}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude (dB)")
    plt.xlim(0, sr / 2)
    plt.ylim(y_min - y_margin, y_max + y_margin)
    plt.grid()
    save_plot_to_results_folder(plt, "audio_passthrough", f"{basename}_fft.png")

# --- Attenuation comparison ---
test_freqs = ["2500", "7500", "15000"]
attenuation_rows = []

for f in test_freqs:
    direct_key = f"{f}_direct.wav"
    pass_key = f"{f}_pass.wav"

    if direct_key not in fft_data or pass_key not in fft_data:
        print(f"WARNING: Missing files for {f} Hz pair, skipping.")
        continue

    direct = fft_data[direct_key]
    bypassed = fft_data[pass_key]

    attenuation = direct["amplitude_db"] - bypassed["amplitude_db"]

    print(f"\n--- {f} Hz ---")
    print(
        f"  Direct:   {direct['amplitude_db']:.2f} dB (peak at: {direct['dominant_freq']:.2f} Hz)"
    )
    print(
        f"  Bypassed: {bypassed['amplitude_db']:.2f} dB (peak at: {bypassed['dominant_freq']:.2f} Hz)"
    )
    print(f"  Attenuation: {attenuation:.2f} dB")

    attenuation_rows.append(
        {
            "Test Frequency (Hz)": int(f),
            "Direct Amplitude (dB)": round(direct["amplitude_db"], 2),
            "Bypassed Amplitude (dB)": round(bypassed["amplitude_db"], 2),
            "Attenuation (dB)": round(attenuation, 2),
            "Direct Peak Freq (Hz)": round(direct["dominant_freq"], 2),
            "Bypassed Peak Freq (Hz)": round(bypassed["dominant_freq"], 2),
        }
    )

attenuation_df = (
    pd.DataFrame(attenuation_rows)
    .sort_values("Test Frequency (Hz)")
    .reset_index(drop=True)
)
print("\nAttenuation Results:")
print(attenuation_df.to_string(index=False))

save_results_to_csv(
    list(attenuation_df.columns),
    attenuation_df.to_dict(orient="records"),
    "audio_passthrough",
    "audio_passthrough_results.csv",
)
