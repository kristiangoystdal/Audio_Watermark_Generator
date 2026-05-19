"""Run decode_fsk on every WAV file in INPUT_FOLDER at multiple SNR levels.

Subfolder names follow the pattern <f0>-<f1>[_anything].
Frequencies are read from the subfolder name.
White noise is added in-memory at each SNR level; no files are written to disk.
SNR is calibrated against the FSK signal power estimated via bandpass filtering.
"""

import csv
import datetime
import io
import os
import re
import sys
import time
import tempfile

import numpy as np
import soundfile as sf
from scipy.signal import butter, sosfilt

import demodulator as _demodulator
_demodulator.DEBUG_PRINTS = False  # silence demodulator output during automated testing
from demodulator import decode_fsk, find_likely_fsk_region

# ══════════════════════════════════════════════════════════════
#  Configuration — edit these to change the test
# ══════════════════════════════════════════════════════════════
INPUT_FOLDER      = "SPEAKER"   # folder containing frequency subfolders
EXPECTED_MESSAGES = 16        # expected decoded messages per subfolder
SNR_LEVELS_DB     = [2, 1, 0, -1, -2, -3, -4, -5, -6, -8]
# None → clean (no noise), numbers → SNR in dB relative to FSK signal power
# ══════════════════════════════════════════════════════════════

FOLDER_RE   = re.compile(r"^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)")
BASE_DIR    = os.path.dirname(__file__)
BANDPASS_HZ = 150  # half-bandwidth (Hz) used to estimate FSK signal RMS


# ── Noise helpers ──────────────────────────────────────────────────────────

def _bandpassed_rms(signal: np.ndarray, fs: float, fc: float) -> float:
    nyq  = fs / 2.0
    low  = max(0.001, (fc - BANDPASS_HZ) / nyq)
    high = min(0.999, (fc + BANDPASS_HZ) / nyq)
    if low >= high:
        return 0.0
    sos = butter(6, [low, high], btype="band", output="sos")
    return float(np.sqrt(np.mean(sosfilt(sos, signal) ** 2)))


def _estimate_fsk_rms(audio: np.ndarray, fs: float, f0: float, f1: float) -> float:
    """Estimate RMS of the FSK tones by bandpass-filtering the active region."""
    try:
        # Suppress progress prints from find_likely_fsk_region
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            region = find_likely_fsk_region(audio, fs, f0, f1)
        finally:
            sys.stdout = old_stdout
        start  = max(0, int(round(region["start_time"] * fs)))
        end    = min(len(audio), int(round(region["end_time"] * fs)))
        active = audio[start:end]
    except Exception:
        active = audio
    if len(active) < 256:
        active = audio
    return max(_bandpassed_rms(active, fs, f0), _bandpassed_rms(active, fs, f1), 1e-10)


def _add_noise(audio: np.ndarray, fs: float, f0: float, f1: float, snr_db: float) -> np.ndarray:
    fsk_rms   = _estimate_fsk_rms(audio, fs, f0, f1)
    noise_rms = fsk_rms / (10 ** (snr_db / 20.0))
    noise     = np.random.default_rng().normal(0.0, noise_rms, len(audio))
    return np.clip(audio + noise, -1.0, 1.0)


# ── Per-subfolder runner ───────────────────────────────────────────────────

def _snr_label(snr_db) -> str:
    return "Clean" if snr_db is None else f"{snr_db:+.0f} dB"


def run_subfolder(folder_path: str, f0: float, f1: float, snr_db) -> tuple[int, float, str]:
    """Decode all WAVs in folder_path at the given SNR. Returns (total, elapsed, error_note)."""
    wav_files  = sorted(f for f in os.listdir(folder_path) if f.lower().endswith(".wav"))
    total      = 0
    error_note = ""
    t0         = time.monotonic()

    for filename in wav_files:
        src = os.path.join(folder_path, filename)
        print(f"\n  File: {filename}")
        try:
            audio, fs = sf.read(src)
            if audio.ndim > 1:
                audio = audio[:, 0]
            audio = audio.astype(float)

            if snr_db is not None:
                audio = _add_noise(audio, fs, f0, f1, snr_db)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp_path = tmp.name
            try:
                sf.write(tmp_path, audio, fs)
                count = decode_fsk(
                    tmp_path,
                    f0=f0, f1=f1,
                    generate_debug=False,
                    segmentation=True,
                    use_ecc=True,
                    ecc_nsym=20,
                ) or 0
            finally:
                os.unlink(tmp_path)

            print(f"  → {count} message(s) decoded")
            total += count
        except Exception as exc:
            print(f"  ERROR: {exc}")
            if not error_note:
                error_note = str(exc)

    return total, time.monotonic() - t0, error_note


# ── Main ───────────────────────────────────────────────────────────────────

def main() -> None:
    suite_dir  = os.path.join(BASE_DIR, INPUT_FOLDER)
    subfolders = sorted(
        d for d in os.listdir(suite_dir)
        if os.path.isdir(os.path.join(suite_dir, d)) and FOLDER_RE.match(d)
    )
    if not subfolders:
        print(f"No matching subfolders found in {suite_dir}")
        sys.exit(1)

    # results[snr_label][folder] = (total, status, note, elapsed)
    all_results: dict[str, dict] = {}
    t_total = time.monotonic()

    for snr_db in SNR_LEVELS_DB:
        label = _snr_label(snr_db)
        all_results[label] = {}

        print(f"\n\n{'#'*60}")
        print(f"#  {INPUT_FOLDER}  —  SNR: {label}")
        print(f"{'#'*60}")

        for folder in subfolders:
            m  = FOLDER_RE.match(folder)
            f0 = float(m.group(1))
            f1 = float(m.group(2))
            fp = os.path.join(suite_dir, folder)

            print(f"\n{'='*60}")
            print(f"Folder : {folder}")
            print(f"f0={f0:.0f} Hz  f1={f1:.0f} Hz  |  SNR: {label}")
            print(f"{'='*60}")

            total, elapsed, err = run_subfolder(fp, f0, f1, snr_db)
            status = "PASS" if total == EXPECTED_MESSAGES else "FAIL"
            note   = err or ("" if status == "PASS" else f"got {total}, expected {EXPECTED_MESSAGES}")
            print(f"\n  Total: {total}/{EXPECTED_MESSAGES} messages → {status}  ({elapsed:.1f}s)")
            all_results[label][folder] = (total, status, note, elapsed)

    # ── Summary ────────────────────────────────────────────────────────────
    elapsed_total = time.monotonic() - t_total

    count_cell = f"{EXPECTED_MESSAGES}/{EXPECTED_MESSAGES}"   # widest possible cell
    col_snr    = max(len("SNR Level"), max(len(_snr_label(s)) for s in SNR_LEVELS_DB))
    col_cell   = max(len(f) for f in subfolders)             # folder name drives column width
    col_cell   = max(col_cell, len(count_cell))
    col_score  = max(len("Score"), len(f"{len(subfolders)}/{len(subfolders)}"))

    header = f"  {'SNR Level':<{col_snr}}  |"
    for folder in subfolders:
        header += f"  {folder:^{col_cell}}  |"
    header += f"  {'Score':^{col_score}}  |"
    sep = "-" * len(header)

    print(f"\n\n{'='*60}")
    print(f"  SUMMARY  —  {INPUT_FOLDER}  (target: {EXPECTED_MESSAGES} msgs/subfolder)")
    print(f"{'='*60}")
    print(sep)
    print(header)
    print(sep)

    overall_pass = True
    for snr_db in SNR_LEVELS_DB:
        label  = _snr_label(snr_db)
        row    = f"  {label:<{col_snr}}  |"
        passed = 0
        for folder in subfolders:
            total, status, _, _ = all_results[label][folder]
            cell = f"{total}/{EXPECTED_MESSAGES}"
            row += f"  {cell:^{col_cell}}  |"
            if status == "PASS":
                passed += 1
            else:
                overall_pass = False
        score_str = f"{passed}/{len(subfolders)}"
        row += f"  {score_str:^{col_score}}  |"
        print(row)

    print(sep)
    print(f"\n  Total time: {elapsed_total:.1f}s")
    print(f"  {'ALL TESTS PASSED' if overall_pass else 'SOME TESTS FAILED'}\n")

    # ── CSV export ─────────────────────────────────────────────────────────
    timestamp  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path   = os.path.join(BASE_DIR, f"autotest_{INPUT_FOLDER}_{timestamp}.csv")
    with open(csv_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["snr_db", "f0", "f1", "frequency_pair", "folder",
                         "messages_found", "messages_expected", "passed"])
        for snr_db in SNR_LEVELS_DB:
            label = _snr_label(snr_db)
            snr_value = "" if snr_db is None else snr_db
            for folder in subfolders:
                m     = FOLDER_RE.match(folder)
                f0    = float(m.group(1))
                f1    = float(m.group(2))
                total, status, _, _ = all_results[label][folder]
                writer.writerow([snr_value, f0, f1, f"{f0:.0f}-{f1:.0f}",
                                 folder, total, EXPECTED_MESSAGES,
                                 status == "PASS"])
    print(f"  Results saved to {csv_path}\n")

    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
