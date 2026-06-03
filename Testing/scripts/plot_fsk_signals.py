#!/usr/bin/env python3
"""Plot waveform + spectrogram of the first FSK signal in each demodulator_autotest file.

CABLE files are combined into one figure (4 rows × 2 cols: waveform | spectrogram).
SPEAKER files are each saved as individual figures.

Reads only the first SCAN_SECONDS of each file to avoid loading multi-hour recordings.
SILENCE_PAD seconds of audio is kept on both sides of the detected signal.
"""

import os
import re
import sys

import matplotlib.pyplot as plt
plt.rcParams.update({"font.size": 16})
import numpy as np
import soundfile as sf

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "Software"))
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "..", "Software", "gui_demodulator"),
)
import gui_demodulator.demodulator as _demodulator

_demodulator.DEBUG_PRINTS = False
from gui_demodulator.demodulator import find_all_fsk_regions

AUTOTEST_DIR = os.path.join(
    os.path.dirname(__file__), "..", "test_files", "demodulator_autotest"
)
OUTPUT_DIR = os.path.join(
    os.path.dirname(__file__), "..", "results", "fsk_signal_plots"
)

SCAN_SECONDS = 120      # seconds to read from the start of each file when scanning
SILENCE_PAD  = 0.25      # seconds of audio to keep before / after the detected region
FOLDER_RE    = re.compile(r"^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)")


def _find_wav_files(base_dir: str) -> list[tuple[str, float, float]]:
    """Return (wav_path, f0, f1) sorted by group then frequency."""
    results = []
    for root, dirs, files in os.walk(base_dir):
        dirs.sort()
        folder = os.path.basename(root)
        m = FOLDER_RE.match(folder)
        if m is None:
            continue
        f0, f1 = float(m.group(1)), float(m.group(2))
        for fname in sorted(files):
            if fname.lower().endswith(".wav"):
                results.append((os.path.join(root, fname), f0, f1))
    return results


def _read_chunk(path: str, seconds: float) -> tuple[np.ndarray, int]:
    """Read up to `seconds` of audio from the start of `path`."""
    with sf.SoundFile(path) as f:
        fs = f.samplerate
        n_frames = min(f.frames, int(seconds * fs))
        audio = f.read(n_frames, dtype="float32", always_2d=False)
    if audio.ndim > 1:
        audio = audio[:, 0]
    return audio, fs


def _extract_segment(wav_path: str, f0: float, f1: float):
    """Load a chunk, detect the first FSK region, return (segment, t_axis, t_start, t_end, region)."""
    audio, fs = _read_chunk(wav_path, SCAN_SECONDS)
    regions   = find_all_fsk_regions(audio, fs, f0, f1)
    region    = min(regions, key=lambda r: r["start_time"])

    t_start = max(0.0, region["start_time"] - SILENCE_PAD)
    t_end   = min(len(audio) / fs, region["end_time"] + SILENCE_PAD)
    i_start = int(t_start * fs)
    i_end   = int(t_end   * fs)
    segment = audio[i_start:i_end]
    t_axis  = np.arange(len(segment)) / fs + t_start
    return segment, t_axis, t_start, t_end, fs, region


def _draw_waveform(ax, segment, t_axis):
    ax.plot(t_axis, segment, color="steelblue", linewidth=0.4, rasterized=True)
    ax.set_xlim(t_axis[0], t_axis[-1])
    ax.set_ylim(-1, 1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude")


def _compute_spectrogram(segment, fs, t_start):
    """Return (Pxx_db, freqs, bins_abs) without drawing."""
    nfft     = 128
    noverlap = nfft * 3 // 4
    fig_tmp, ax_tmp = plt.subplots()
    Pxx, freqs, bins, _ = ax_tmp.specgram(
        segment.astype(float), NFFT=nfft, Fs=fs,
        noverlap=noverlap, cmap="inferno", scale="dB", vmin=-80,
    )
    plt.close(fig_tmp)
    return 10 * np.log10(Pxx + 1e-30), freqs, bins + t_start


def _draw_spectrogram(ax, Pxx_db, freqs, bins_abs, t_start, t_end, vmin, vmax):
    ax.pcolormesh(bins_abs, freqs, Pxx_db, cmap="inferno", vmin=vmin, vmax=vmax,
                  shading="auto", rasterized=True)
    ax.set_ylim(0, 24000)
    ax.set_xlim(t_start, t_end)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")


def _plot_group(files: list[tuple[str, float, float]], out_path: str) -> None:
    """Create one figure with 2 columns (waveform | spectrogram) per file row."""
    # First pass: extract all segments and compute spectrograms
    data = []
    for wav_path, f0, f1 in files:
        rel = os.path.relpath(wav_path, AUTOTEST_DIR)
        print(f"  {rel} (f0={f0:.0f} Hz, f1={f1:.0f} Hz)", end="", flush=True)
        segment, t_axis, t_start, t_end, fs, region = _extract_segment(wav_path, f0, f1)
        print(f"  →  signal {region['start_time']:.2f}s – {region['end_time']:.2f}s")
        Pxx_db, freqs, bins_abs = _compute_spectrogram(segment, fs, t_start)
        data.append((f0, f1, segment, t_axis, t_start, t_end, Pxx_db, freqs, bins_abs))

    vmin   = -80
    vmax   = -30

    n = len(files)
    fig, axes = plt.subplots(
        n, 2, figsize=(16, 4 * n),
        gridspec_kw={"hspace": 0.55, "wspace": 0.35},
    )
    if n == 1:
        axes = [axes]

    for row, (f0, f1, segment, t_axis, t_start, t_end, Pxx_db, freqs, bins_abs) in enumerate(data):
        label = f"f0 = {f0/1000:.0f} kHz, f1 = {f1/1000:.0f} kHz"
        _draw_waveform(axes[row][0], segment, t_axis)
        axes[row][0].set_title(f"Waveform  {label}")

        _draw_spectrogram(axes[row][1], Pxx_db, freqs, bins_abs, t_start, t_end, vmin, vmax)
        axes[row][1].set_title(f"Spectrogram  {label}")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved → {os.path.relpath(out_path, os.path.dirname(__file__))}")



def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    all_files = _find_wav_files(AUTOTEST_DIR)
    if not all_files:
        print(f"No WAV files found under {AUTOTEST_DIR}")
        return

    cable_files   = sorted(
        [(p, f0, f1) for p, f0, f1 in all_files if "CABLE"   in p], key=lambda x: x[1], reverse=True
    )
    speaker_files = sorted(
        [(p, f0, f1) for p, f0, f1 in all_files if "SPEAKER" in p], key=lambda x: x[1], reverse=True
    )

    # Remove old plots
    for fname in os.listdir(OUTPUT_DIR):
        if fname.lower().endswith(".png"):
            os.remove(os.path.join(OUTPUT_DIR, fname))

    print(f"Found {len(all_files)} file(s).\n")

    if cable_files:
        print("=== CABLE (combined figure) ===")
        _plot_group(cable_files, os.path.join(OUTPUT_DIR, "cable_all.png"))

    if speaker_files:
        print("\n=== SPEAKER (combined figure) ===")
        _plot_group(speaker_files, os.path.join(OUTPUT_DIR, "speaker_all.png"))

    print(f"\nDone. Figures saved to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
