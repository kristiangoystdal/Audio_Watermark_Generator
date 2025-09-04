#!/usr/bin/env python3
"""
Play a captured signal (time,value CSV) as audio, and save a WAV.

- Expects CSV with two columns: time_seconds, value
- Infers original sample times from the first column
- Rescales signal to [-1, 1]
- Resamples to AUDIO_FS (default 44100 Hz)
- Plays via sounddevice and saves "signal_out.wav"

Install deps (once):
    pip install numpy pandas sounddevice scipy

Run:
    python play_signal.py
"""

import numpy as np
import pandas as pd
import sounddevice as sd
from scipy.io.wavfile import write as wavwrite
from pathlib import Path

# ----------- Config -----------
CSV_FILE = "test_values2.csv"  # your input capture
AUDIO_FS = 44100  # output audio sample rate
WAV_OUT = "signal_out2.wav"  # saved WAV file name
# -------------------------------


def load_time_series(csv_path: Path):
    """Load time/value columns from CSV (header or no header)."""
    try:
        df = pd.read_csv(csv_path)
        if df.shape[1] < 2:
            df = pd.read_csv(csv_path, header=None)
    except Exception:
        df = pd.read_csv(csv_path, header=None)

    if df.shape[1] < 2:
        raise ValueError("CSV must have at least 2 columns: time[s], value")

    t = df.iloc[:, 0].astype(float).to_numpy()
    x = df.iloc[:, 1].astype(float).to_numpy()

    # Clean and sort by time
    good = np.isfinite(t) & np.isfinite(x)
    t, x = t[good], x[good]
    order = np.argsort(t)
    t, x = t[order], x[order]

    # Remove exact duplicates in time (if any)
    uniq = np.diff(t, prepend=t[0] - 1) > 0
    t, x = t[uniq], x[uniq]

    return t, x


def normalize_to_audio(x):
    """Map arbitrary numeric range to [-1, 1] safely."""
    # If looks like 12-bit DAC (0..4095), map to [-1, 1] around midscale
    xmin, xmax = np.min(x), np.max(x)
    rng = xmax - xmin

    if rng == 0:
        return np.zeros_like(x, dtype=np.float32)

    # Heuristic: if typical DAC-like range, center at mid and scale to peak
    if 3500 <= xmax <= 5000 and -500 <= xmin <= 500:
        mid = (xmax + xmin) / 2.0
        y = (x - mid) / (rng / 2.0)
    else:
        # General min-max to [-1, 1]
        y = 2.0 * (x - xmin) / rng - 1.0

    # Clip any overshoot
    y = np.clip(y, -1.0, 1.0)
    return y.astype(np.float32)


def resample_to_audio(t, y, fs_out):
    """
    Resample an irregular/regular time series (t in seconds, y in [-1,1])
    to a uniform grid at fs_out using linear interpolation.
    """
    if len(t) < 2:
        return np.zeros(1, dtype=np.float32)

    t0, t1 = float(t[0]), float(t[-1])
    if t1 <= t0:
        return np.zeros(1, dtype=np.float32)

    dur = t1 - t0
    n_out = max(1, int(np.round(dur * fs_out)))
    t_uniform = np.linspace(t0, t1, n_out, endpoint=False)
    y_uniform = np.interp(t_uniform, t, y).astype(np.float32)
    return y_uniform


def main():
    csv_path = Path(CSV_FILE)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    # 1) Load time/value
    t, x = load_time_series(csv_path)

    # 2) Normalize to audio range [-1, 1]
    y = normalize_to_audio(x)

    # 3) Resample to AUDIO_FS for playback
    audio = resample_to_audio(t, y, AUDIO_FS)

    # 4) Play
    print(
        f"Playing {CSV_FILE}: {len(audio)} samples at {AUDIO_FS} Hz "
        f"(duration ~ {len(audio)/AUDIO_FS:.2f} s)"
    )
    sd.play(audio, samplerate=AUDIO_FS, blocking=True)

    # 5) Save WAV (16-bit PCM)
    wav = np.int16(np.clip(audio, -1, 1) * 32767)
    wavwrite(WAV_OUT, AUDIO_FS, wav)
    print(f"WAV saved to {WAV_OUT}")


if __name__ == "__main__":
    main()
