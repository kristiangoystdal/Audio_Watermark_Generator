"""Run decode_fsk on every WAV file in INPUT_FOLDER at multiple SNR levels.

Subfolder names follow the pattern <f0>-<f1>[_anything].
Frequencies are read from the subfolder name.
Pink noise is added in-memory at each SNR level; no files are written to disk.
SNR is calibrated against the FSK signal power estimated via bandpass filtering.
"""

import csv
import datetime
import io
import multiprocessing as mp
import os
import re
import sys
import time
import tempfile

import numpy as np
import soundfile as sf
from scipy.signal import lfilter

import demodulator as _demodulator
_demodulator.DEBUG_PRINTS = False  # silence demodulator output during automated testing
from demodulator import decode_fsk, find_likely_fsk_region

# ══════════════════════════════════════════════════════════════
#  Configuration — edit these to change the test
# ══════════════════════════════════════════════════════════════
INPUT_FOLDER      = "CABLE"   # folder containing frequency subfolders
EXPECTED_MESSAGES = 125        # expected decoded messages per subfolder
SNR_LEVELS_DB     =  list(np.arange(10, -25, -0.3))  # 5 dB down to -25 dB in 0.5 dB steps
# None → clean (no noise), numbers → SNR in dB relative to FSK signal power
# ══════════════════════════════════════════════════════════════

FOLDER_RE   = re.compile(r"^(\d+(?:\.\d+)?)-(\d+(?:\.\d+)?)")
BASE_DIR    = os.path.dirname(__file__)
# ── Noise helpers ──────────────────────────────────────────────────────────

def _estimate_fsk_rms(audio: np.ndarray, fs: float, f0: float, f1: float) -> float:
    """Estimate RMS of the FSK signal from the active region."""
    try:
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
    return max(float(np.sqrt(np.mean(active ** 2))), 1e-10)


def _pink_noise(rng: np.random.Generator, n: int) -> np.ndarray:
    """Generate unit-variance pink (1/f) noise via IIR filtering of white noise.

    Uses Paul Kellet's IIR approximation — O(n) memory vs O(n·16) for FFT approach.
    """
    # Generate float32 directly to halve the white-noise allocation (~700 MB vs ~1.4 GB).
    white = rng.standard_normal(n, dtype=np.float32)
    b = np.array([0.049922035, -0.095993537, 0.050612699, -0.004408786], dtype=np.float64)
    a = np.array([1.0, -2.494956002, 2.017265875, -0.522189400], dtype=np.float64)
    pink = lfilter(b, a, white)
    del white  # free early — lfilter is done with it
    std = float(pink.std()) or 1.0
    pink /= std
    return pink.astype(np.float32)


def _add_noise(audio: np.ndarray, fs: float, f0: float, f1: float, snr_db: float) -> np.ndarray:
    fsk_rms   = _estimate_fsk_rms(audio, fs, f0, f1)
    noise_rms = fsk_rms / (10 ** (snr_db / 20.0))
    noise     = _pink_noise(np.random.default_rng(), len(audio))
    noise    *= noise_rms        # in-place scale — avoids a full-size temporary
    audio    += noise            # in-place add  — avoids another full-size temporary
    del noise
    np.clip(audio, -1.0, 1.0, out=audio)  # in-place clip
    return audio


# ── Per-subfolder runner ───────────────────────────────────────────────────

# All five of these markers must appear in a label-track line for the message
# to be considered valid (time, message text, device ID, location, temperature).
_REQUIRED_FIELD_MARKERS = ("Time: ", "Message: ", "Device ID: ", "Location: ", "Temperature: ")


def _count_valid_messages(label_path: str) -> int:
    """Count label-track lines that contain all required field markers."""
    if not os.path.exists(label_path):
        return 0
    count = 0
    with open(label_path) as f:
        for line in f:
            if all(marker in line for marker in _REQUIRED_FIELD_MARKERS):
                count += 1
    return count


def _snr_label(snr_db) -> str:
    return "Clean" if snr_db is None else f"{snr_db:+.1f} dB"


def _decode_file_worker(src: str, f0: float, f1: float, snr_db, result_queue: mp.Queue) -> None:
    """Decode one WAV file and put (count, error_str) on result_queue.

    Runs in a dedicated subprocess so all memory — including the demodulator's
    internal state — is freed when this process exits.
    """
    try:
        audio, fs = sf.read(src)
        if audio.ndim > 1:
            audio = audio[:, 0]
        audio = audio.astype(np.float32)

        if snr_db is not None:
            audio = _add_noise(audio, fs, f0, f1, snr_db)

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name
        label_path = os.path.splitext(tmp_path)[0] + ".txt"
        try:
            sf.write(tmp_path, audio, fs)
            del audio
            decode_fsk(
                tmp_path,
                f0=f0, f1=f1,
                generate_debug=False,
                segmentation=True,
                use_ecc=True,
                ecc_nsym=20,
            )
            count = _count_valid_messages(label_path)
        finally:
            os.unlink(tmp_path)
            if os.path.exists(label_path):
                os.unlink(label_path)

        result_queue.put((count, ""))
    except Exception as exc:
        result_queue.put((0, str(exc)))


def run_subfolder(folder_path: str, f0: float, f1: float, snr_db) -> tuple[int, float, str]:
    """Decode all WAVs in folder_path at the given SNR. Returns (total, elapsed, error_note).

    Each file is decoded in a fresh subprocess so accumulated demodulator memory
    cannot carry over between files or SNR levels.
    """
    wav_files  = sorted(f for f in os.listdir(folder_path) if f.lower().endswith(".wav"))
    total      = 0
    error_note = ""
    t0         = time.monotonic()
    ctx        = mp.get_context("spawn")

    for filename in wav_files:
        src = os.path.join(folder_path, filename)
        print(f"\n  File: {filename}", flush=True)

        q = ctx.Queue()
        p = ctx.Process(target=_decode_file_worker, args=(src, f0, f1, snr_db, q))
        p.start()
        p.join()

        if p.exitcode != 0 or q.empty():
            msg = f"subprocess exited with code {p.exitcode}"
            print(f"  ERROR: {msg}")
            if not error_note:
                error_note = msg
            continue

        count, err = q.get_nowait()
        print(f"  → {count} message(s) decoded")
        total += count
        if err and not error_note:
            error_note = err

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
