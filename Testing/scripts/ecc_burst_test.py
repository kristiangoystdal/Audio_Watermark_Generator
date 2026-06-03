"""T8 - ECC Burst Error Correction test.

Tests Reed-Solomon ECC robustness by injecting a single burst error of
increasing duration into the active FSK region of pre-recorded WAV files,
each encoded with a different number of ECC parity bytes (0, 10, 20, ..., 100).

Approach
--------
One centred burst of silence (zeroed samples) is applied to each file at
durations from 0 ms up to MAX_BURST_MS in BURST_STEP_MS increments.

Burst errors interact with the demodulator as follows:
  - Silenced f1-bits (bit=1) are misread as 0 → byte errors.
  - Silenced f0-bits (bit=0) produce silence=0 and are decoded correctly.
  - Only ~50 % of burst-covered bits become errors, so in practice the ECC
    system tolerates roughly twice the burst duration predicted by raw RS theory.

Theoretical RS correction limit (lower bound on observed tolerance):
    burst_limit_ms = floor(ecc_nsym / 2) * 8 * SYMBOL_MS

The test records whether decode_fsk succeeds for every (ecc_bytes, burst_ms)
combination, then plots the result as a heatmap with the theoretical boundary
overlaid.

Configuration
-------------
Edit the constants below the imports to change the test scope.
"""

import csv
import datetime
import io
import os
import sys
import tempfile
import time

import matplotlib.pyplot as plt
import numpy as np
import soundfile as sf

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "Software"))
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "..", "Software", "gui_demodulator"),
)
import gui_demodulator.demodulator as _demodulator

_demodulator.DEBUG_PRINTS = False
from gui_demodulator.demodulator import decode_fsk, find_likely_fsk_region

# ══════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════
ECC_DIR = os.path.join(os.path.dirname(__file__), "..", "test_files", "ECC")

# (ecc_bytes, filename, use_ecc, ecc_nsym_for_decode)
# For 0 ECC bytes: RS is disabled; ecc_nsym=20 is passed only to control the
# gap-bridge width in find_message_ranges, not to attempt RS correction.
ECC_FILES = [
    (0, "0ECC.WAV", False, 20),
    (10, "10ECC.WAV", True, 10),
    (20, "20ECC.WAV", True, 20),
    (30, "30ECC.WAV", True, 30),
    (40, "40ECC.WAV", True, 40),
    (50, "50ECC.WAV", True, 50),
    (60, "60ECC.WAV", True, 60),
    (70, "70ECC.WAV", True, 70),
    (80, "80ECC.WAV", True, 80),
    (90, "90ECC.WAV", True, 90),
    (100, "100ECC.WAV", True, 100),
]

F0 = 22000  # FSK tone for bit 0 [Hz]
F1 = 23000  # FSK tone for bit 1 [Hz]

# Approximate symbol period (s): p0 = 66 periods of f0 ≈ 66/22176 ≈ 2.98 ms
SYMBOL_MS = 66 / 22176.29 * 1000

MAX_BURST_MS = 2000  # longest burst to test [ms]
BURST_STEP_MS = 25  # increment between burst durations [ms]
# ══════════════════════════════════════════════════════════════

BASE_DIR = os.path.dirname(__file__)
RESULTS_DIR = os.path.normpath(
    os.path.join(BASE_DIR, "..", "results", "ecc_burst_test")
)


def _find_active_region(audio: np.ndarray, fs: int) -> dict:
    """Return the FSK active region dict without printing progress."""
    old = sys.stdout
    sys.stdout = io.StringIO()
    try:
        region = find_likely_fsk_region(audio, fs, F0, F1)
    finally:
        sys.stdout = old
    return region


def _inject_burst(
    audio: np.ndarray, fs: int, region: dict, burst_ms: float
) -> np.ndarray:
    """Return a copy of *audio* with a centred burst of silence injected.

    The burst is centred at the midpoint of the active FSK region.  Its
    length is ``burst_ms`` milliseconds.  Samples are clamped to the audio
    boundaries so the burst never spills outside the recording.
    """
    corrupted = audio.copy()
    if burst_ms <= 0:
        return corrupted
    burst_samples = int(round(burst_ms / 1000.0 * fs))
    centre = int(round(region["center_time"] * fs))
    start = max(0, centre - burst_samples // 2)
    end = min(len(audio), start + burst_samples)
    corrupted[start:end] = 0.0
    return corrupted


_REQUIRED_FIELD_MARKERS = (
    "Time: ",
    "Message: ",
    "Device ID: ",
    "Location: ",
    "Temperature: ",
)


def _get_message_payloads(label_path: str) -> list[str]:
    """Return the payload strings of lines that contain all required field markers.

    Each label line is tab-delimited: start_time, end_time, payload.
    Only the payload (3rd column) is returned.
    """
    if not os.path.exists(label_path):
        return []
    payloads = []
    with open(label_path) as f:
        for line in f:
            if all(marker in line for marker in _REQUIRED_FIELD_MARKERS):
                parts = line.strip().split("\t", 2)
                payloads.append(parts[2] if len(parts) >= 3 else line.strip())
    return payloads


def _decode_to_payloads(
    audio: np.ndarray, fs: int, use_ecc: bool, ecc_nsym: int
) -> list[str]:
    """Write *audio* to a temp WAV, run decode_fsk, return valid message payloads."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        tmp_path = tmp.name
    label_path = os.path.splitext(tmp_path)[0] + ".txt"
    try:
        sf.write(tmp_path, audio, fs)
        decode_fsk(
            tmp_path,
            f0=F0,
            f1=F1,
            generate_debug=False,
            segmentation=True,
            use_ecc=use_ecc,
            ecc_nsym=ecc_nsym,
        )
        return _get_message_payloads(label_path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        if os.path.exists(label_path):
            os.unlink(label_path)


def _decode_audio(
    audio: np.ndarray,
    fs: int,
    use_ecc: bool,
    ecc_nsym: int,
    reference: str | None = None,
) -> bool:
    """Decode *audio* and return True if ≥1 message passes validation.

    When *reference* is supplied the decoded payload must match it exactly,
    catching cases where corrupted data still produces structurally valid
    field markers (e.g. 0-ECC files with small burst errors).
    """
    payloads = _decode_to_payloads(audio, fs, use_ecc, ecc_nsym)
    if not payloads:
        return False
    if reference is None:
        return True
    return any(payload == reference for payload in payloads)


def run_test() -> tuple[list[int], list[float], np.ndarray]:
    """Run the full ECC burst test matrix.

    Returns
    -------
    ecc_levels : list[int]
        ECC parity byte counts (rows).
    burst_durations : list[float]
        Burst durations in ms (columns).
    results : np.ndarray, shape (len(ecc_levels), len(burst_durations))
        1 = decoded successfully, 0 = failed.
    """
    burst_durations = [
        float(x) for x in np.arange(0, MAX_BURST_MS + BURST_STEP_MS, BURST_STEP_MS)
    ]
    ecc_levels = [row[0] for row in ECC_FILES]
    results = np.zeros((len(ECC_FILES), len(burst_durations)), dtype=int)

    t_start = time.monotonic()
    total_runs = len(ECC_FILES) * len(burst_durations)
    run_idx = 0

    for row_idx, (ecc_bytes, fname, use_ecc, ecc_nsym) in enumerate(ECC_FILES):
        fpath = os.path.normpath(os.path.join(BASE_DIR, ECC_DIR, fname))
        audio, fs = sf.read(fpath)
        if audio.ndim > 1:
            audio = audio[:, 0]
        audio = audio.astype(np.float32)

        region = _find_active_region(audio, fs)
        active_dur_ms = (region["end_time"] - region["start_time"]) * 1000

        # Decode the clean audio once to obtain the reference payload.
        ref_payloads = _decode_to_payloads(audio, fs, use_ecc, ecc_nsym)
        reference: str | None = ref_payloads[0] if ref_payloads else None
        ref_status = "ok" if reference else "WARNING: no reference decode"

        print(
            f"\n[{ecc_bytes:3d} ECC bytes]  {fname}  "
            f"active={active_dur_ms:.0f} ms  "
            f"centre={region['center_time']:.2f} s  "
            f"ref={ref_status}"
        )

        for col_idx, burst_ms in enumerate(burst_durations):
            run_idx += 1
            corrupted = _inject_burst(audio, fs, region, burst_ms)
            success = _decode_audio(corrupted, fs, use_ecc, ecc_nsym, reference)
            results[row_idx, col_idx] = int(success)
            symbol = "+" if success else "."
            print(f"  {burst_ms:5.0f} ms → {symbol}", end="", flush=True)

        elapsed = time.monotonic() - t_start
        done_frac = run_idx / total_runs
        eta = (elapsed / done_frac) * (1 - done_frac) if done_frac > 0 else 0
        print(f"\n  Row done  ({elapsed:.0f}s elapsed, ETA {eta:.0f}s)")

    print(f"\nTotal time: {time.monotonic() - t_start:.1f}s")
    return ecc_levels, burst_durations, results


def save_csv(ecc_levels, burst_durations, results, timestamp: str) -> str:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    csv_path = os.path.join(RESULTS_DIR, f"ecc_burst_{timestamp}.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ecc_bytes", "burst_ms", "success"])
        for row_idx, ecc in enumerate(ecc_levels):
            for col_idx, burst in enumerate(burst_durations):
                writer.writerow([ecc, burst, int(results[row_idx, col_idx])])
    print(f"CSV saved to {csv_path}")
    return csv_path


def plot_results(ecc_levels, burst_durations, results, csv_path: str) -> None:
    """Plot the result matrix as a heatmap with the theoretical correction boundary."""
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
        }
    )

    fig, ax = plt.subplots(figsize=(10, 5))

    # Heatmap: green = decoded, red = failed
    cmap = plt.colormaps["RdYlGn"]
    ax.imshow(
        results,
        aspect="auto",
        cmap=cmap,
        vmin=0,
        vmax=1,
        origin="lower",
        extent=(
            burst_durations[0] - BURST_STEP_MS / 2,
            burst_durations[-1] + BURST_STEP_MS / 2,
            ecc_levels[0] - 5,
            ecc_levels[-1] + 5,
        ),
        interpolation="nearest",
    )

    # Theoretical correction boundary:
    #   correctable byte errors = floor(ecc_nsym / 2)
    #   burst covers ≈ burst_ms / SYMBOL_MS bits = burst_ms / (SYMBOL_MS * 8) bytes
    #   boundary: burst_limit_ms = floor(ecc_nsym / 2) * 8 * SYMBOL_MS
    theory_ecc = np.array(ecc_levels, dtype=float)
    theory_burst = np.floor(theory_ecc / 2) * 8 * SYMBOL_MS
    ax.plot(
        theory_burst,
        theory_ecc,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label=f"RS limit: ⌊ECC/2⌋ × 8 × {SYMBOL_MS:.1f} ms",
        zorder=5,
    )
    ax.scatter(theory_burst, theory_ecc, color="black", s=18, zorder=6)

    from matplotlib.patches import Patch

    ax.set_xlabel("Burst error duration (ms)")
    ax.set_ylabel("ECC parity bytes")
    # ax.set_title("ECC Burst Error Correction — single centred burst, zeroed samples")
    ax.set_yticks(ecc_levels)
    ax.set_xlim(burst_durations[0] - BURST_STEP_MS, burst_durations[-1] + BURST_STEP_MS)
    ax.grid(False)

    cmap = plt.colormaps["RdYlGn"]
    legend_entries = [
        Patch(facecolor=cmap(1.0), label="Pass"),
        Patch(facecolor=cmap(0.0), label="Fail"),
        ax.get_lines()[0],  # RS limit line
    ]
    legend = ax.legend(handles=legend_entries, loc="upper right")
    legend.get_frame().set_alpha(0.7)
    legend.get_frame().set_linewidth(0.5)

    plt.tight_layout()

    png_path = os.path.splitext(csv_path)[0] + ".png"
    fig.savefig(png_path, dpi=150)
    print(f"Plot saved to {png_path}")
    plt.show()
    plt.close(fig)


def load_csv(csv_path: str) -> tuple[list[int], list[float], np.ndarray]:
    """Load a previously saved CSV and reconstruct the result matrix."""
    ecc_set: dict[int, dict[float, int]] = {}
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ecc = int(row["ecc_bytes"])
            burst = float(row["burst_ms"])
            success = int(row["success"])
            ecc_set.setdefault(ecc, {})[burst] = success

    ecc_levels = sorted(ecc_set.keys())
    burst_durations = sorted(next(iter(ecc_set.values())).keys())
    results = np.zeros((len(ecc_levels), len(burst_durations)), dtype=int)
    for r, ecc in enumerate(ecc_levels):
        for c, burst in enumerate(burst_durations):
            results[r, c] = ecc_set[ecc][burst]
    return ecc_levels, burst_durations, results


def main() -> None:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    print("=" * 60)
    print("  T8 — ECC Burst Error Correction Test")
    print(f"  f0={F0} Hz  f1={F1} Hz")
    print(f"  Burst range: 0 – {MAX_BURST_MS} ms  step: {BURST_STEP_MS} ms")
    n_steps = int(MAX_BURST_MS / BURST_STEP_MS) + 1
    est_min = len(ECC_FILES) * n_steps * 0.7 / 60
    print(
        f"  Files: {len(ECC_FILES)}  ×  Burst steps: {n_steps}  (est. {est_min:.0f} min)"
    )
    print("=" * 60)

    ecc_levels, burst_durations, results = run_test()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = save_csv(ecc_levels, burst_durations, results, timestamp)
    plot_results(ecc_levels, burst_durations, results, csv_path)


if __name__ == "__main__":
    # Set to a CSV path to plot without rerunning the test, e.g.:
    # PLOT_CSV = "results/ecc_burst_test/ecc_burst_20240101_120000.csv"
    PLOT_CSV = "results/ecc_burst_test/ecc_burst_20260603_115059.csv"

    if PLOT_CSV:
        if not os.path.exists(PLOT_CSV):
            print(f"Error: file not found: {PLOT_CSV}")
            sys.exit(1)
        ecc_levels, burst_durations, results = load_csv(PLOT_CSV)
        plot_results(ecc_levels, burst_durations, results, PLOT_CSV)
    else:
        confirm = input("This will run ~500 decode attempts. Proceed? (y/n): ")
        if confirm.strip().lower() != "y":
            print("Cancelled.")
            sys.exit(0)
        main()
