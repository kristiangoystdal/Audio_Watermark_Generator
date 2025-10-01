import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from scipy.signal import spectrogram

wav_path = "signal_out_min_1.wav"

# Load audio
data, sample_rate = sf.read(wav_path)
if data.ndim > 1:
    data = data[:, 0]

# Spectrogram
nperseg = 1024
noverlap = nperseg // 2
freqs, times, Sxx = spectrogram(
    data,
    fs=sample_rate,
    nperseg=nperseg,
    noverlap=noverlap,
    scaling="spectrum",
    mode="magnitude",
)

# Convert to dB safely
Sxx_db = 20 * np.log10(Sxx + 1e-12)

# 1) Pool energy over frequencies >= 20 kHz (rows -> mask, then average across rows)
band_mask = freqs >= 20000
if not np.any(band_mask):
    print(
        "No spectrogram rows at or above 20 kHz (try increasing sample rate / nperseg)."
    )
else:
    band_trace = Sxx_db[band_mask, :].mean(axis=0)  # one value per time column

    # Optional smoothing to reduce flicker (moving average over ~3 frames)
    if band_trace.size >= 3:
        kernel = np.ones(3) / 3
        band_trace = np.convolve(band_trace, kernel, mode="same")

    # 2) Choose a threshold to mark 'above 20 kHz present'
    #    You can tweak either of these strategies:
    # A) Adaptive: median + 6 dB
    thr = np.median(band_trace) + 6.0
    # B) Or percentile-based (e.g., 75th): thr = np.percentile(band_trace, 75)

    active = band_trace > thr

    # 3) Find contiguous active regions = pulses
    #    Edges are where active changes True<->False
    edges = np.diff(active.astype(int))
    starts = np.where(edges == 1)[0] + 1
    ends = np.where(edges == -1)[0] + 1

    # Handle if activity starts at t=0 or ends at final sample
    if active[0]:
        starts = np.r_[0, starts]
    if active[-1]:
        ends = np.r_[ends, active.size]

    if len(starts) == 0:
        print("No pulses detected above threshold in the >=20 kHz band.")
    else:
        # Pulse start/end times
        pulse_starts = times[starts]
        pulse_ends = times[np.clip(ends - 1, 0, len(times) - 1)]
        pulse_durs = pulse_ends - pulse_starts

        # Interval between pulses (start of previous to start of next)
        if len(pulse_starts) >= 2:
            gaps = pulse_starts[1:] - pulse_starts[:-1]
        else:
            gaps = np.array([])

        # Report
        print(f"Detected {len(pulse_starts)} pulse(s) in the >=20 kHz band.")
        for i, (ts, te, dur) in enumerate(zip(pulse_starts, pulse_ends, pulse_durs), 1):
            print(f"  Pulse {i}: {ts:.6f}s → {te:.6f}s  (duration {dur*1000:.2f} ms)")
        if gaps.size:
            print("Interval between starts of pulses:")
            for i, g in enumerate(gaps, 1):
                print(f"  Interval {i}: {g:.2f} s")
        else:
            print("No inter-pulse intervals (only one pulse or pulses merged).")

        # Optional debug plots
        # plt.figure(figsize=(10, 4))
        # plt.plot(times, band_trace, label="Band power (>=20 kHz)")
        # plt.axhline(thr, linestyle="--", label="Threshold")
        # for ts, te in zip(pulse_starts, pulse_ends):
        #     plt.axvspan(ts, te, alpha=0.2)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Level [dB]")
        # plt.title(">=20 kHz band activity over time")
        # plt.legend()
        # plt.tight_layout()
        # plt.show()

        # Spectrogram plot
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.pcolormesh(times, freqs / 1000, Sxx_db, shading="gouraud", cmap="viridis")
        plt.colorbar(label="Magnitude [dB]")
        plt.ylim(0, sample_rate / 2000)  # Show up to
        plt.ylabel("Frequency [kHz]")
        plt.title("Spectrogram")
        for ts, te in zip(pulse_starts, pulse_ends):
            plt.axvspan(ts, te, color="red", alpha=0.3)
        plt.subplot(2, 1, 2)
        plt.plot(times, band_trace, label="Band power (>=20 kHz)")
        plt.axhline(thr, linestyle="--", label="Threshold")
        for ts, te in zip(pulse_starts, pulse_ends):
            plt.axvspan(ts, te, color="red", alpha=0.3)
        plt.xlabel("Time [s]")
        plt.ylabel("Level [dB]")
        plt.title(">=20 kHz band activity over time")
        plt.legend()
        plt.tight_layout()
        plt.show()
