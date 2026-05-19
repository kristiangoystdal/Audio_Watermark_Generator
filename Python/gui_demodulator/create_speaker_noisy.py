"""Generate SPEAKER_NOISY from SPEAKER by adding white noise at -5 dB SNR.

For each WAV file in every SPEAKER subfolder the FSK signal power is
estimated by bandpass-filtering the active region around f0 and f1.
White noise is then added at -5 dB SNR (noise RMS = FSK RMS / 10^(5/20))
and the result is written to the mirrored path under SPEAKER_NOISY.

Usage:
    python create_speaker_noisy.py
"""

import os
import re
import sys

import numpy as np
import soundfile as sf
from scipy.signal import butter, sosfilt

from demodulator import find_likely_fsk_region

SPEAKER_DIR = os.path.join(os.path.dirname(__file__), "SPEAKER")
NOISY_DIR   = os.path.join(os.path.dirname(__file__), "SPEAKER_NOISY")
FOLDER_RE   = re.compile(r"^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)")
BANDPASS_HZ = 150   # half-bandwidth around each tone for RMS estimation


def bandpassed_rms(signal: np.ndarray, fs: float, fc: float) -> float:
    nyq = fs / 2.0
    low  = max(0.001, (fc - BANDPASS_HZ) / nyq)
    high = min(0.999, (fc + BANDPASS_HZ) / nyq)
    if low >= high:
        return 0.0
    sos = butter(6, [low, high], btype="band", output="sos")
    return float(np.sqrt(np.mean(sosfilt(sos, signal) ** 2)))


def estimate_fsk_rms(audio: np.ndarray, fs: float, f0: float, f1: float) -> float:
    """Estimate RMS of the FSK tones using the detected active region."""
    try:
        region = find_likely_fsk_region(audio, fs, f0, f1)
        start  = max(0, int(round(region["start_time"] * fs)))
        end    = min(len(audio), int(round(region["end_time"] * fs)))
        active = audio[start:end]
    except Exception:
        active = audio

    if len(active) < 256:
        active = audio

    rms0 = bandpassed_rms(active, fs, f0)
    rms1 = bandpassed_rms(active, fs, f1)
    return max(rms0, rms1, 1e-10)


def process_file(src: str, dst: str, f0: float, f1: float) -> None:
    audio, fs = sf.read(src)
    if audio.ndim > 1:
        audio = audio[:, 0]
    audio = audio.astype(float)

    SNR_DB    = -5.0
    fsk_rms   = estimate_fsk_rms(audio, fs, f0, f1)
    noise_rms = fsk_rms / (10 ** (SNR_DB / 20.0))
    noise     = np.random.default_rng().normal(0.0, noise_rms, len(audio))
    noisy     = np.clip(audio + noise, -1.0, 1.0)

    os.makedirs(os.path.dirname(dst), exist_ok=True)
    sf.write(dst, noisy, fs)
    print(f"  FSK RMS={fsk_rms:.6f}  noise RMS={noise_rms:.6f} ({SNR_DB:+.0f} dB)  →  {os.path.basename(dst)}")


def main() -> None:
    subfolders = sorted(
        d for d in os.listdir(SPEAKER_DIR)
        if os.path.isdir(os.path.join(SPEAKER_DIR, d)) and FOLDER_RE.match(d)
    )

    if not subfolders:
        print(f"No matching subfolders found in {SPEAKER_DIR}")
        sys.exit(1)

    for folder in subfolders:
        m  = FOLDER_RE.match(folder)
        f0 = float(m.group(1))
        f1 = float(m.group(2))

        src_dir = os.path.join(SPEAKER_DIR, folder)
        dst_dir = os.path.join(NOISY_DIR,   folder)
        os.makedirs(dst_dir, exist_ok=True)

        wav_files = sorted(f for f in os.listdir(src_dir) if f.lower().endswith(".wav"))
        print(f"\n{folder}  (f0={f0:.0f} Hz, f1={f1:.0f} Hz, {len(wav_files)} file(s))")

        for filename in wav_files:
            process_file(
                src=os.path.join(src_dir, filename),
                dst=os.path.join(dst_dir, filename),
                f0=f0,
                f1=f1,
            )

    print(f"\nDone. Noisy files written to {NOISY_DIR}\n")


if __name__ == "__main__":
    main()
