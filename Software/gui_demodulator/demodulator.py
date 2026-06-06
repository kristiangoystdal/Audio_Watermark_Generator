"""
FSK demodulator utilities for extracting bitstreams and messages from audio files.

Example command-line usage:
python demodulator.py -i test.wav --f0 21333 --f1 22325 --use-ecc --ecc-parity-bytes 20 --generate-debug --segmentation
"""

import math
import os
import sys
import numpy as np
import soundfile as sf
from scipy.signal import windows
from reed_solomon import NSYM as DEFAULT_ECC_NSYM

MIN_MESSAGE_BITS = 16
MIN_REGION_INTERVAL = 55.0  # Minimum expected interval between FSK transmissions (seconds)
DAC_FS = 1920000

DEBUG_PRINTS = True
DEBUG_PLOTS = False
DEBUG_PLOTS_ALL_COMBINATIONS = False  # Plot every N/offset candidate in select_best_symbol_timing_and_offset

if DEBUG_PLOTS:
    import matplotlib.pyplot as plt
    #make PLOTS folder and clear existing contents for debug plot output
    import shutil
    if os.path.exists("PLOTS"):
        shutil.rmtree("PLOTS")
    os.makedirs("PLOTS", exist_ok=True)


days_of_week = [
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday",
]

def _dprint(*args, **kwargs):
    if DEBUG_PRINTS:
        print(*args, **kwargs)  # noqa: T201

def _dwrite(text: str):
    if DEBUG_PRINTS:
        sys.stdout.write(text)
        sys.stdout.flush()

def _compute_fsk_window_scores(
    x, fs, f0, f1,
    window_duration=0.5, hop_duration=0.05, threshold_sigma=3.0,
    progress_callback=None,
):
    """Score overlapping windows by FSK tone power. Returns (window_times, window_scores, active, threshold)."""
    if window_duration <= 0 or hop_duration <= 0:
        raise ValueError("window_duration and hop_duration must be > 0")
    x = np.asarray(x, dtype=float)
    if x.ndim != 1:
        raise ValueError("x must be a 1D audio signal")
    window_size = max(1024, int(round(window_duration * fs)))
    hop_size = max(1, int(round(hop_duration * fs)))
    if len(x) < window_size:
        raise ValueError("Audio is shorter than one analysis window")

    window = windows.hann(window_size, sym=False)
    starts = np.arange(0, len(x) - window_size + 1, hop_size, dtype=int)
    frames = np.lib.stride_tricks.sliding_window_view(x, window_size)[::hop_size]
    n = np.arange(window_size, dtype=float)
    window_energy = float(np.sum(window ** 2))
    osc_f0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    osc_f1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)

    total_windows = len(starts)
    batch_size = max(1, min(2048, total_windows))
    window_scores = np.empty(total_windows, dtype=float)
    progress_width = 24

    def print_progress(done_windows: int):
        fraction = done_windows / total_windows if total_windows else 1.0
        filled = int(round(progress_width * fraction))
        bar = "#" * filled + "-" * (progress_width - filled)
        _dwrite(f"\rFinding active FSK region [{bar}] {fraction * 100:5.1f}%")
        if progress_callback is not None:
            progress_callback(fraction, "Finding active FSK region")

    print_progress(0)
    for batch_start in range(0, total_windows, batch_size):
        batch_end = min(batch_start + batch_size, total_windows)
        batch_frames = frames[batch_start:batch_end] * window
        tone0_power = np.abs(batch_frames @ np.conjugate(osc_f0)) ** 2 / window_energy
        tone1_power = np.abs(batch_frames @ np.conjugate(osc_f1)) ** 2 / window_energy
        window_scores[batch_start:batch_end] = np.maximum(tone0_power, tone1_power)
        print_progress(batch_end)
    _dwrite("\n")

    window_times = (starts + 0.5 * window_size) / fs
    baseline = float(np.median(window_scores))
    mad = float(np.median(np.abs(window_scores - baseline)))
    robust_sigma = 1.4826 * mad
    threshold = baseline if robust_sigma == 0.0 else baseline + threshold_sigma * robust_sigma
    active = window_scores >= threshold
    return window_times, window_scores, active, threshold


def _groups_to_regions(groups, window_scores, window_times, window_duration, duration):
    """Convert index groups from flatnonzero into region dicts."""
    regions = []
    for group in groups:
        regions.append({
            "start_time": float(max(0.0, window_times[group[0]] - 0.5 * window_duration)),
            "end_time": float(min(duration, window_times[group[-1]] + 0.5 * window_duration)),
            "center_time": float(np.mean(window_times[group])),
            "score": float(np.mean(window_scores[group])),
        })
    return regions


def find_likely_fsk_region(
    x,
    fs,
    f0,
    f1,
    window_duration=0.5,
    hop_duration=0.05,
    threshold_sigma=3.0,
    progress_callback=None,
):
    """
    Find the time region most likely to contain the FSK signal.

    The signal is scanned in overlapping windows. Each window is scored by the
    matched tone power at `f0` and `f1`. Contiguous windows with
    unusually high tone power are merged into candidate regions, and the region
    with the highest average score is returned.
    """
    x = np.asarray(x, dtype=float)
    duration = len(x) / fs
    window_times, window_scores, active, threshold = _compute_fsk_window_scores(
        x, fs, f0, f1, window_duration, hop_duration, threshold_sigma, progress_callback
    )

    if not np.any(active):
        _dprint("Warning: No active FSK windows found above threshold; falling back to peak window")
        best_idx = int(np.argmax(window_scores))
        result = {
            "start_time": float(max(0.0, window_times[best_idx] - 0.5 * window_duration)),
            "end_time": float(min(duration, window_times[best_idx] + 0.5 * window_duration)),
            "center_time": float(window_times[best_idx]),
            "score": float(window_scores[best_idx]),
            "threshold": float(threshold),
        }
    else:
        if np.all(active):
            _dprint("Warning: All windows are above threshold; FSK region spans the entire file")
        active_idx = np.flatnonzero(active)
        split_points = np.where(np.diff(active_idx) > 1)[0] + 1
        groups = np.split(active_idx, split_points)
        regions = _groups_to_regions(groups, window_scores, window_times, window_duration, duration)
        result = max(regions, key=lambda r: r["score"])
        result["threshold"] = float(threshold)

    _dprint(
        "Active FSK region found: "
        f"{result['start_time']:.3f}s to {result['end_time']:.3f}s "
        f"(center {result['center_time']:.3f}s, score {result['score']:.6f})"
    )
    return result


def find_all_fsk_regions(
    x,
    fs,
    f0,
    f1,
    window_duration=0.5,
    hop_duration=0.05,
    threshold_sigma=3.0,
    progress_callback=None,
):
    """
    Find all FSK-active regions in the audio signal.

    Returns a list of region dicts sorted by start_time, each with keys:
    start_time, end_time, center_time, score, threshold.
    Falls back to a single region around the peak window if no active windows
    are found above the threshold.
    """
    x = np.asarray(x, dtype=float)
    duration = len(x) / fs
    window_times, window_scores, active, threshold = _compute_fsk_window_scores(
        x, fs, f0, f1, window_duration, hop_duration, threshold_sigma, progress_callback
    )

    if not np.any(active):
        _dprint("Warning: No active FSK windows found above threshold; treating full file as one region")
        best_idx = int(np.argmax(window_scores))
        return [{
            "start_time": float(max(0.0, window_times[best_idx] - 0.5 * window_duration)),
            "end_time": float(min(duration, window_times[best_idx] + 0.5 * window_duration)),
            "center_time": float(window_times[best_idx]),
            "score": float(window_scores[best_idx]),
            "threshold": float(threshold),
        }]

    active_idx = np.flatnonzero(active)
    split_points = np.where(np.diff(active_idx) > 1)[0] + 1
    groups = np.split(active_idx, split_points)
    regions = _groups_to_regions(groups, window_scores, window_times, window_duration, duration)

    # Discard regions that are much weaker than the strongest one.
    # Genuine FSK regions should be in the same ballpark; noise hits that
    # just barely crossed the window threshold are orders of magnitude weaker.
    # The sigma-based filter fails here because extreme outliers inflate the MAD,
    # so we use a ratio against the best score instead.
    _dprint(f"Window threshold: {threshold:.6f} (baseline={float(np.median(window_scores)):.6f}, "
          f"{np.sum(active)}/{len(window_scores)} windows active)")

    # Cap regions using the known minimum transmission interval, enforcing
    # a minimum spacing between selected regions (non-maximum suppression).
    # Picking top-N by score alone can select nearby false-positive pairs,
    # placing a split point inside a real transmission.
    if MIN_REGION_INTERVAL > 0 and len(regions) > 1:
        max_regions = max(1, math.floor(duration / MIN_REGION_INTERVAL))
        min_spacing = MIN_REGION_INTERVAL
        _dprint(f"Interval filter: max {max_regions} region(s) allowed "
              f"(≤1 per {MIN_REGION_INTERVAL:.0f}s, min spacing {min_spacing:.0f}s, "
              f"{duration:.0f}s file), {len(regions)} before filter")
        if len(regions) > max_regions:
            # Greedy NMS: pick strongest region, then next strongest that is
            # at least min_spacing away from every already-selected region.
            candidates = sorted(regions, key=lambda r: r["score"], reverse=True)
            selected = []
            for r in candidates:
                if all(abs(r["center_time"] - s["center_time"]) >= min_spacing
                       for s in selected):
                    selected.append(r)
                if len(selected) >= max_regions:
                    break
            regions = selected
        _dprint(f"{len(regions)} region(s) after interval filter")

    for r in regions:
        r["threshold"] = float(threshold)
    regions.sort(key=lambda r: r["start_time"])
    _dprint(f"Found {len(regions)} FSK-active region(s).")
    return regions

def fsk_symbol_metrics(
    x,
    fs,
    f0,
    f1,
    N_samples,
    N_err,
    start,
    initial_acc_err=0.0,
):
    """
    Compute per-symbol energy at f0 and f1 using a sliding Hann window.

    Supports a non-integer symbol length (N_err) by gradually adjusting the
    segment start index to track the fractional sample drift.
    """
    W = windows.hann(N_samples, sym=False)
    n = np.arange(N_samples)
    e_0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    e_1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)
    E0, E1, S = [], [], []
    acc_err = float(initial_acc_err)
    for seg_start in range(start, len(x) - N_samples + 1, N_samples):
        acc_err += N_err
        seg_start -= int(math.floor(acc_err))
        if seg_start < 0 or seg_start + N_samples > len(x):
            continue
        seg = x[seg_start : seg_start + N_samples] * W
        X0 = np.vdot(e_0, seg)
        X1 = np.vdot(e_1, seg)
        denom = np.sum(W**2)
        e0v = (np.abs(X0) ** 2) / denom
        e1v = (np.abs(X1) ** 2) / denom
        E0.append(e0v)
        E1.append(e1v)
        S.append(e1v - e0v)
    return np.array(E0), np.array(E1), np.array(S)


def compute_symbol_start_samples(num_symbols, N_samples, N_err, start=0):
    """Return symbol start samples using the same accumulated drift correction as demodulation."""
    starts = np.empty(num_symbols, dtype=int)
    acc_err = 0.0
    seg_start = start
    for idx in range(num_symbols):
        acc_err += N_err
        corrected_start = seg_start - int(math.floor(acc_err))
        starts[idx] = corrected_start
        seg_start += N_samples
    return starts

def define_thresholds(scores, region_mask=None):
    """Derive thresholds from symbol score distribution to mask noise.

    Thresholds are computed from scores within the active region only
    (region_mask). Signal strength is estimated from the 75th percentile of
    each tail (robust against noise contamination of the low tail), and the
    threshold is placed at a fraction of that estimate. This avoids the failure
    mode where a low percentile falls into near-zero noise values when the
    region contains a mix of signal and silence symbols.
    """
    SIGNAL_PERCENTILE = 75   # use upper portion of each tail to estimate signal level
    THRESHOLD_FRACTION = 0.25  # threshold as fraction of that signal-level estimate
    scores = np.asarray(scores, dtype=float)
    ref = scores[region_mask] if region_mask is not None else scores
    pos_vals = ref[ref > 0]
    neg_vals = ref[ref < 0]
    th1 = float(np.percentile(pos_vals, SIGNAL_PERCENTILE)) * THRESHOLD_FRACTION if len(pos_vals) >= 5 else 0.0
    th0 = float(np.percentile(neg_vals, 100 - SIGNAL_PERCENTILE)) * THRESHOLD_FRACTION if len(neg_vals) >= 5 else 0.0
    return th0, th1

def generate_mask(th0, th1, scores):
    """Return boolean mask of scores considered reliable for decoding."""
    scores = np.asarray(scores, dtype=float)
    return (scores > th1) | (scores < th0)

def print_message(message, start_time, end_time, debug, label_track):
    """Write decoded message info to debug and label_track streams.

    Returns True if the message was valid and written to label_track.
    """
    debug.write("Decoded Message: \"" + message + "\"\n")
    debug.write(f"{len(message)} characters\n")
    if len(message) < 5:
        debug.write("Warning: Message too short\n")
        return False
    elif message[0] != "/":
        if len(message) > 6 and message.count("/") >= 2:
            message = message[message.index("/"):]
            debug.write("Warning: Message truncated to first '/' due to missing preamble\n")
        else:
            debug.write("Warning: Message might be corrupted (missing '/' preamble)\n")
            return False
    elif message[-1] != "/":
        if len(message) > 6 and message.count("/") >= 2:
            message = message[:message.rfind("/") + 1]
            debug.write("Warning: Message truncated to last '/' due to missing termination\n")
        else:
            debug.write("Warning: Message might be corrupted (missing '/' termination)\n")
            return False
    fields = []
    for line in message.split("/"):
        if line[0:3] == "STR":
            fields.append("Message: " + line[3:])
        elif line[0:3] == "LOC":
            fields.append("Location: " + line[3:])
        elif line[0:3] == "MID":
            fields.append("Message ID: " + line[3:])
        elif line[0:3] == "DID":
            fields.append("Device ID: " + line[3:])
        elif line[0:3] == "TMP":
            fields.append("Temperature: " + line[3:] + "°C")
        elif line[0:3] == "TIM":
            try:
                day_idx = int(line[9:11]) - 1
                day_str = days_of_week[day_idx] if 0 <= day_idx < 7 else "?"
                fields.append("Time: " + f"{line[3:5]}:{line[5:7]}:{line[7:9]} on {day_str} {line[11:13]}/{line[13:15]}/{line[15:19]}")
            except (ValueError, IndexError):
                fields.append("Time: [corrupt]")
    debug.write("\n")
    if not fields:
        debug.write("Warning: Message contains no recognised identifiers, skipping label track\n")
        return False
    label_track.write(f"{start_time:.6f}\t{end_time:.6f}\t" + " | ".join(fields) + "\n")
    return True

def seconds_to_hms(total_seconds):
    """Convert seconds to a human-readable hours/minutes/seconds string."""
    hours = int(total_seconds // 3600)
    minutes = int((total_seconds % 3600) // 60)
    seconds = int(total_seconds % 60)
    milliseconds = int((total_seconds % 1) * 1000)

    if hours > 0:
        return f"{hours}h {minutes}m {seconds}s {milliseconds}ms"
    elif minutes > 0:
        return f"{minutes}m {seconds}s {milliseconds}ms"
    else:
        return f"{seconds}s {milliseconds}ms"
    
def refine_fsk_tones_fft(
    x,
    fs: float,
    f0: float,
    f1: float,
    search_width_hz: float = 200.0,
    avg_bin_width: int = 20,
):
    """
    FFT the full signal and find the most prominent peak within:
      [f0 - search_width_hz, f0 + search_width_hz]
      [f1 - search_width_hz, f1 + search_width_hz]

    Returns
    -------
    f0_new, f1_new : float
        Peak frequencies (Hz) inside each band.
    """
    x = np.asarray(x, dtype=float)
    if len(x) / fs > 20.0:
        return f0, f1
    nyq = fs / 2.0
    if search_width_hz <= 0:
        raise ValueError("search_width_hz must be > 0")

    # Window to reduce spectral leakage
    w = np.hanning(len(x))
    xw = x * w

    # FFT (one-sided)
    X = np.fft.rfft(xw)
    power = X.real * X.real + X.imag * X.imag
    df = fs / len(xw)

    # Smooth magnitudes by averaging neighboring bins only in the search bands.
    if avg_bin_width < 1:
        raise ValueError("avg_bin_width must be >= 1")
    if avg_bin_width % 2 == 0:
        avg_bin_width += 1
    kernel = None
    half_width = avg_bin_width // 2
    if avg_bin_width > 1:
        kernel = np.ones(avg_bin_width, dtype=float) / avg_bin_width

    def hz_to_bin(freq_hz: float) -> int:
        return int(np.clip(np.round(freq_hz / df), 0, len(power) - 1))

    def smooth_band_slice(k_lo: int, k_hi: int):
        if kernel is None:
            return power[k_lo : k_hi + 1]

        ext_lo = max(0, k_lo - half_width)
        ext_hi = min(len(power) - 1, k_hi + half_width)
        smoothed_ext = np.convolve(power[ext_lo : ext_hi + 1], kernel, mode="same")
        rel_lo = k_lo - ext_lo
        rel_hi = rel_lo + (k_hi - k_lo + 1)
        return smoothed_ext[rel_lo:rel_hi]

    def peak_in_band(f_center: float) -> float:
        lo = max(0.0, f_center - search_width_hz)
        hi = min(nyq, f_center + search_width_hz)
        k_lo = hz_to_bin(lo)
        k_hi = hz_to_bin(hi)
        if k_hi < k_lo:
            raise ValueError(f"No FFT bins in band [{lo}, {hi}] Hz")
        band_smooth = smooth_band_slice(k_lo, k_hi)
        k = k_lo + int(np.argmax(band_smooth))

        # Quadratic (parabolic) interpolation around k for sub-bin peak estimate
        # Use log-magnitude to behave better for sharp peaks.
        if 1 <= k < len(power) - 1:
            y0 = np.log(power[k - 1] + 1e-30)
            y1 = np.log(power[k] + 1e-30)
            y2 = np.log(power[k + 1] + 1e-30)
            denom = (y0 - 2.0 * y1 + y2)
            if denom != 0.0:
                delta = 0.5 * (y0 - y2) / denom  # in bins, typically [-0.5, 0.5]
                f_est = k * df + delta * df
                # Clamp back into the band, just in case
                return float(min(max(f_est, lo), hi))
        return float(k * df)

    f0_new = peak_in_band(f0)
    f1_new = peak_in_band(f1)

    if DEBUG_PLOTS:
        freqs = np.fft.rfftfreq(len(xw), d=1.0 / fs)
        magnitude_db = 10 * np.log10(power + 1e-30)
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(freqs, magnitude_db, color="steelblue", linewidth=0.7, label="FFT magnitude")
        # Shade search bands
        for f_in, f_out, color in [(f0, f0_new, "tab:orange"), (f1, f1_new, "tab:green")]:
            band_lo = max(0.0, f_in - search_width_hz)
            band_hi = min(nyq, f_in + search_width_hz)
            ax.axvspan(band_lo, band_hi, alpha=0.15, color=color)
            ax.axvline(f_in, color=color, linestyle="--", linewidth=1.2, label=f"in {f_in:.1f} Hz")
            ax.axvline(f_out, color=color, linestyle="-", linewidth=1.5, label=f"out {f_out:.1f} Hz")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Power (dB)")
        ax.set_title("refine_fsk_tones_fft — FFT with input/output tone markers")
        ax.legend(fontsize=8)
        ax.set_xlim(
            max(0, min(f0, f1) - search_width_hz * 3),
            min(nyq, max(f0, f1) + search_width_hz * 3),
        )
        plt.tight_layout()
        plt.savefig(f"PLOTS/refine_fsk_fft_f0_{f0:.0f}_f1_{f1:.0f}.png", dpi=150)
        plt.close(fig)

    return f0_new, f1_new

def compute_p0(f0):
    m = np.round(DAC_FS * (3000 / 1_000_000))
    n0 = np.floor(DAC_FS / f0)
    return int(np.round(m / n0))

def _trim_sparse_flanks(group, min_isolation_gap=20, min_flank=None):
    """
    Trim sparse noise clusters from the leading and trailing edges of a group.

    If the cluster of reliable symbols on one side of a gap >= min_isolation_gap
    has fewer than min_flank symbols, that cluster is stripped from the group.
    Internal gaps with enough mass on both sides are left untouched so a real
    message with an occasional weak stretch is not split.
    """
    if min_flank is None:
        min_flank = MIN_MESSAGE_BITS * 2
    if len(group) < 2:
        return group

    diffs = np.diff(group)
    big_gaps = np.where(diffs >= min_isolation_gap)[0]
    if len(big_gaps) == 0:
        return group

    start = 0
    end = len(group)

    # Trim left: if the cluster before the first big gap is too small
    first_big = big_gaps[0]
    if first_big + 1 < min_flank:
        start = first_big + 1

    # Trim right: if the cluster after the last big gap is too small
    valid_gaps = big_gaps[big_gaps >= start]
    if len(valid_gaps) > 0:
        last_big = valid_gaps[-1]
        if end - (last_big + 1) < min_flank:
            end = last_big + 1

    return group[start:end]


def find_message_ranges(mask, ecc_nsym):
    """
    Find contiguous message ranges in the original bitstream using reliable symbols.

    Returns a list of (start_idx, end_idx) on the unmasked bit array.
    """
    reliable_idx = np.flatnonzero(mask)
    if len(reliable_idx) == 0:
        return []

    max_gap = ecc_nsym * 8
    min_reliable = MIN_MESSAGE_BITS * 2  # reject ranges backed by only a handful of noise hits
    max_bits = 256 * 8  # 256 bytes — no valid codeword can be longer than this
    split_at = np.where(np.diff(reliable_idx) > max_gap)[0] + 1
    groups = np.split(reliable_idx, split_at)
    trimmed = [_trim_sparse_flanks(g) for g in groups]
    return [
        (int(g[0]), int(g[-1]))
        for g in trimmed
        if len(g) >= min_reliable and (g[-1] - g[0] + 1) <= max_bits
    ]

def bits_to_bytes(bits, bit_offset=0):
    """Pack a bit array into bytes, optionally skipping an initial bit offset."""
    if bit_offset < 0 or bit_offset >= 8:
        raise ValueError("bit_offset must be in range [0, 7]")

    usable_bits = len(bits) - bit_offset
    if usable_bits <= 0:
        return b""

    n_full_bytes = usable_bits // 8
    trailing_bits = usable_bits % 8
    n_bytes = n_full_bytes + (1 if trailing_bits > 4 else 0)
    if n_bytes <= 0:
        return b""

    out = bytearray()
    idx = bit_offset
    for byte_idx in range(n_bytes):
        value = 0
        bits_in_byte = 8
        if byte_idx == n_full_bytes and trailing_bits > 4:
            bits_in_byte = trailing_bits
        for _ in range(bits_in_byte):
            value = (value << 1) | int(bits[idx])
            idx += 1
        value <<= 8 - bits_in_byte
        out.append(value)
    return bytes(out)

def decode_message_with_rs(msg_bits, rsc, nsym, decode_codeword_fn, rs_error_type):
    """
    Attempt RS decode from a bit segment.

    Tries all bit alignments [0..7] and, for each, trims leading and trailing
    bytes up to _MAX_EDGE_TRIM bytes each. Leading-byte trimming handles ranges
    extended from the front by noise hits near the message start (which shift
    the codeword start beyond the 0-7 bit-offset search). Trailing-byte
    trimming handles ranges extended from the back by trailing silence or noise.

    Head trim is capped at a small constant (_MAX_HEAD_TRIM) because leading-
    edge misalignment is at most a few bytes regardless of nsym.  Tail trim
    must still search up to nsym because msg_bits is intentionally extended
    by ecc_nsym*8 bits in the caller; that extension has to be peeled back to
    find the exact codeword boundary.  Total RS-decode calls are therefore
    O(_MAX_HEAD_TRIM × nsym) rather than O(nsym²).
    """
    _MAX_HEAD_TRIM = 8  # bytes — realistic leading-edge timing misalignment
    max_len_bytes = len(msg_bits) // 8
    max_trim = min(_MAX_HEAD_TRIM, max(0, max_len_bytes - nsym - 1))
    for head_trim in range(max_trim + 1):
        for bit_offset in range(8):
            codeword = bits_to_bytes(
                msg_bits[head_trim * 8:], bit_offset=bit_offset
            )
            tail_max = min(nsym, max(0, len(codeword) - nsym - 1))
            for tail_trim in range(tail_max + 1):
                trimmed = codeword if tail_trim == 0 else codeword[: len(codeword) - tail_trim]
                if len(trimmed) <= nsym:
                    break
                try:
                    payload = decode_codeword_fn(trimmed, codec=rsc)
                    if payload and payload[:1] == b"/" and payload[-1:] == b"/":
                        return payload, bit_offset
                except rs_error_type:
                    pass
    return None

def decode_message_without_ecc(
    msg_bits=None,
    payload_bytes=None,
    strip_trailing_bytes=0,
    trim_to_trailing_slash=False,
):
    """Decode bytes directly from a bit segment or byte payload, trying byte alignments."""
    def normalize_payload(payload: bytes) -> bytes:
        if strip_trailing_bytes > 0:
            payload = payload[:-strip_trailing_bytes]
        if trim_to_trailing_slash:
            slash_idx = payload.rfind(b"/")
            payload = payload[: slash_idx + 1] if slash_idx >= 0 else b""
        return payload

    if payload_bytes is not None:
        payload = normalize_payload(payload_bytes)
        if not payload:
            return b""
        text = payload.decode("ascii", errors="replace")
        if text.startswith("/") and text.endswith("/"):
            return payload
        return payload

    if msg_bits is None:
        return b""

    for bit_offset in range(8):
        payload = normalize_payload(bits_to_bytes(msg_bits, bit_offset=bit_offset))
        if not payload:
            continue
        text = payload.decode("ascii", errors="replace")
        if text.startswith("/") and text.endswith("/"):
            return payload
    return normalize_payload(bits_to_bytes(msg_bits, bit_offset=0))

def select_best_symbol_timing_and_offset(
    x,
    fs,
    refined_f0,
    f1,
    p0,
    region_info,
    n_true_search_radius=1,
    n_true_step=0.1,
    stepsize=25,
    progress_callback=None,
):
    """Jointly search N and symbol offset while keeping refined_f0 fixed."""
    if n_true_search_radius < 0:
        raise ValueError("n_true_search_radius must be >= 0")
    if n_true_step <= 0:
        raise ValueError("n_true_step must be > 0")
    if stepsize <= 0:
        raise ValueError("stepsize must be > 0")

    region_start = max(0, int(round(region_info["start_time"] * fs)))
    region_end = min(len(x), int(round(region_info["end_time"] * fs)))
    region_x = x[region_start:region_end]
    if len(region_x) == 0:
        region_x = x
        region_start = 0
        region_end = len(x)

    symbol_time = p0 / refined_f0
    nominal_N_true = fs * symbol_time
    candidate_N_trues = np.arange(
        max(n_true_step, nominal_N_true - n_true_search_radius),
        nominal_N_true + n_true_search_radius + 0.5 * n_true_step,
        n_true_step,
        dtype=float,
    )

    _dprint(
        f"Testing {len(candidate_N_trues)} N_true candidates around "
        f"N_true={nominal_N_true:.4f} samples using refined f0={refined_f0:.2f} Hz..."
    )

    total_candidates = sum(
        max(1, len(range(0, max(1, int(math.ceil(candidate_N_true))), stepsize)))
        for candidate_N_true in candidate_N_trues
    )
    progress_width = 24

    def print_progress(done_candidates: int):
        fraction = done_candidates / total_candidates if total_candidates else 1.0
        filled = int(round(progress_width * fraction))
        bar = "#" * filled + "-" * (progress_width - filled)
        _dwrite(f"\rFinding best timing/offset [{bar}] {fraction * 100:5.1f}%")
        if progress_callback is not None:
            progress_callback(fraction, "Finding best timing/offset")

    def score_offset(N: int, N_err: float, offset: int):
        if len(region_x) < N:
            return float("-inf"), np.array([], dtype=float)

        def corrected_symbol_start(symbol_idx: int) -> int:
            nominal_start = offset + symbol_idx * N
            return nominal_start - int(math.floor((symbol_idx + 1) * N_err))

        effective_symbol_step = N - N_err
        if effective_symbol_step <= 0:
            return float("-inf"), np.array([], dtype=float)
        symbol_idx = max(0, int(math.floor((region_start - offset) / effective_symbol_step)))
        while symbol_idx > 0 and corrected_symbol_start(symbol_idx - 1) >= region_start:
            symbol_idx -= 1
        while corrected_symbol_start(symbol_idx) < region_start:
            symbol_idx += 1

        # Align the local start to the drift-corrected symbol boundary that is
        # actually evaluated by fsk_symbol_metrics for this symbol index.
        local_offset = corrected_symbol_start(symbol_idx) - region_start
        E0, E1, scores = fsk_symbol_metrics(
            region_x,
            fs,
            refined_f0,
            f1,
            N,
            N_err,
            start=local_offset,
        )
        scores = E1 - E0
        if len(scores) == 0:
            return float("-inf"), scores
        return float(np.mean(np.abs(scores))), scores

    best_setup = None
    best_scores_for_plot = None
    done_candidates = 0
    print_progress(0)
    for candidate_N_true in candidate_N_trues:
        N = max(1, int(math.ceil(candidate_N_true)))
        N_err = N - candidate_N_true
        offsets = list(range(0, N, stepsize)) or [0]
        best_offset = 0
        best_score = float("-inf")
        best_scores_this_N = None

        for offset in offsets:
            score, scores = score_offset(N, N_err, offset)
            done_candidates += 1
            if DEBUG_PLOTS and DEBUG_PLOTS_ALL_COMBINATIONS and len(scores) > 0:
                fig, ax = plt.subplots(figsize=(10, 4))
                ax.scatter(range(len(scores)), scores, s=10)
                ax.set_title(
                    f"N_true={candidate_N_true:.4f}, N={N}, offset={offset} — "
                    f"score={score if np.isfinite(score) else 0:.6f}"
                )
                ax.set_xlabel("Symbol Index")
                ax.set_ylabel("Score (E1 - E0)")
                ax.grid()
                plt.tight_layout()
                plt.savefig(f"PLOTS/timing_Ntrue_{candidate_N_true:.4f}_offset_{offset}.png", dpi=100)
                plt.close(fig)
            if score > best_score:
                best_score = score
                best_offset = offset
                best_scores_this_N = scores
            print_progress(done_candidates)

        if best_setup is None or best_score > best_setup["avg_abs_score"]:
            best_scores_for_plot = (candidate_N_true, N, best_offset, best_score, best_scores_this_N)
            best_setup = {
                "f0": float(refined_f0),
                "p0": int(p0),
                "symbol_time": float(candidate_N_true / fs),
                "N": int(N),
                "N_true": float(candidate_N_true),
                "N_err": float(N_err),
                "offset": int(best_offset),
                "avg_abs_score": float(best_score if np.isfinite(best_score) else 0.0),
                "region_start_time": float(region_info["start_time"]),
                "region_end_time": float(region_info["end_time"]),
            }

    _dwrite("\n")
    if best_setup is None:
        raise ValueError("No valid N/offset candidate produced demodulation scores")

    if DEBUG_PLOTS and best_scores_for_plot is not None:
        candidate_N_true, N, offset, score, scores = best_scores_for_plot
        plt.scatter(range(len(scores)), scores, s=10)
        plt.title(
            f"N_true={candidate_N_true:.4f}, N={N}, offset={offset} Symbol Scores. "
            f"Score: {score if np.isfinite(score) else float(0):.6f}"
        )
        plt.xlabel("Symbol Index")
        plt.ylabel("Score (E1 - E0)")
        plt.grid()
        plt.savefig(f"PLOTS/Ntrue_{candidate_N_true:.4f}_N_{N}_offset_{offset}_scores.png")
        plt.close()

    _dprint(
        "Best timing/offset found: "
        f"N={best_setup['N']}, "
        f"N_true={best_setup['N_true']:.4f}, "
        f"offset={best_setup['offset']}"
    )

    return best_setup

def decode_fsk(input_filename: str,
               f0: float = 20833.33,
               f1: float = 22222.22,
               generate_debug: bool = False,
               segmentation: bool = False,
               use_ecc: bool = True,
               ecc_nsym: int = DEFAULT_ECC_NSYM,
               progress_callback=None):
    """
    Demodulate an FSK-modulated WAV file and create message labels/debug text files.

    Parameters
    ----------
    input_filename : str
        Path to the input WAV file.
    f0, f1 : float
        Carrier frequencies representing bits 0 and 1.
    generate_debug : bool
        Whether to write debug metrics to a companion text file.
    segmentation : bool
        When True, all FSK-active regions are detected automatically and the
        audio is split at the midpoints between them.  Each resulting segment
        is processed independently.  When False the whole file is one segment.
    use_ecc : bool
        Enable Reed-Solomon error correction on each decoded message segment.
    ecc_nsym : int
        Number of RS parity bytes to use when ECC is enabled.
    progress_callback : callable, optional
        Called with (fraction: float, message: str) to report progress.
        fraction is in [0, 1].

    Returns
    -------
    int
        Number of successfully decoded messages.
    """

    if use_ecc and (ecc_nsym <= 0 or ecc_nsym >= 255):
        raise ValueError("ecc_nsym must be in range [1, 254] when ECC is enabled")

    def report(fraction, message=""):
        if progress_callback is not None:
            progress_callback(fraction, message)

    message_count = 0
    p0 = compute_p0(f0)
    base, _ = os.path.splitext(input_filename)
    if generate_debug:
        debug_filename = base + "_debug.txt"
    else:
        debug_filename = os.devnull
    labels_filename = base + ".txt"
    with open(labels_filename, "w") as label_track, open(debug_filename, "w") as debug:
        rsc = None
        decode_codeword_fn = None
        rs_error_type = Exception
        if use_ecc:
            try:
                from reedsolo import ReedSolomonError
                from reed_solomon import build_codec_with_nsym, decode_codeword
            except ModuleNotFoundError as exc:
                raise ModuleNotFoundError(
                    "ECC is enabled but 'reedsolo' is not installed. "
                    "Install reedsolo or disable ECC."
                ) from exc
            rsc = build_codec_with_nsym(ecc_nsym)
            decode_codeword_fn = decode_codeword
            rs_error_type = ReedSolomonError
        report(0.0, "Reading audio file")
        _dprint(f"\nReading audio from {input_filename}...")
        audio, fs = sf.read(input_filename)
        if audio.ndim > 1:
            audio = audio[:, 0] # take right channel if stereo
        report(0.05, "Finding active FSK region(s)")
        _dprint("Locating likely FSK-active region(s)...")

        def _region_progress(frac, msg):
            report(0.05 + frac * 0.20, msg)

        if segmentation:
            all_regions = find_all_fsk_regions(audio, fs, f0, f1, progress_callback=_region_progress)
            tone_ref_region = max(all_regions, key=lambda r: r["score"])
        else:
            all_regions = None
            tone_ref_region = find_likely_fsk_region(audio, fs, f0, f1, progress_callback=_region_progress)

        if DEBUG_PLOTS:
            times = np.arange(len(audio)) / fs
            plt.plot(times, audio, label="Audio Signal")
            if all_regions:
                for r in all_regions:
                    plt.axvspan(r["start_time"], r["end_time"], color="orange", alpha=0.3)
                plt.axvspan(0, 0, color="orange", alpha=0.3, label="Detected FSK Regions")
            else:
                plt.axvspan(tone_ref_region["start_time"], tone_ref_region["end_time"], color="orange", alpha=0.3, label="Detected FSK Region")
            plt.title("Audio Signal with Detected FSK Region(s)")
            plt.xlabel("Time (s)")
            plt.ylabel("Amplitude")
            plt.grid()
            plt.savefig("PLOTS/detected_fsk_region.png")
            plt.close()

        active_region_start_sample = max(0, int(round(tone_ref_region["start_time"] * fs)))
        active_region_end_sample = min(len(audio), int(round(tone_ref_region["end_time"] * fs)))
        active_audio = audio[active_region_start_sample:active_region_end_sample]
        if len(active_audio) == 0:
            active_audio = audio
        report(0.25, "Refining FSK tones")
        _dprint("Refining FSK tones from detected active region...")
        f0, f1 = refine_fsk_tones_fft(active_audio, fs, f0, f1)
        _dprint(
            f"Demodulation parameters: f0={f0:.2f} Hz, f1={f1:.2f} Hz, "
            f"p0={p0} ({p0} period(s) per symbol, symbol_time={p0/f0*1000:.3f} ms), "
            f"tone-ref-region={tone_ref_region['start_time']:.3f}s-{tone_ref_region['end_time']:.3f}s"
        )
        audio_len = audio.shape[0]
        if segmentation and all_regions:
            split_times = [
                (all_regions[i]["end_time"] + all_regions[i + 1]["start_time"]) / 2
                for i in range(len(all_regions) - 1)
            ]
            split_samples = [0] + [int(round(t * fs)) for t in split_times] + [audio_len]
            segments = [
                (split_samples[i], audio[split_samples[i] : split_samples[i + 1]])
                for i in range(len(split_samples) - 1)
            ]
            _dprint(f"Auto-segmentation: {len(all_regions)} FSK region(s) → {len(segments)} segment(s).")
        else:
            segments = [(0, audio)]
        report(0.30, "Processing audio segments")
        _dprint(f"Processing {len(segments)} audio segment(s) for demodulation...")
        num_segments = len(segments)
        segmentindex = 0
        _all_scores_t: list = []
        _all_scores_v: list = []
        _segment_thresholds: list = []  # (t_start, t_end, th0, th1)
        _segment_split_times: list = split_times if (segmentation and all_regions and len(all_regions) > 1) else []
        for segment_start_sample, audio in segments:
            seg_base = 0.30 + (segmentindex / num_segments) * 0.70
            seg_span = 0.70 / num_segments
            _dprint(f"\nProcessing segment {segmentindex+1}...")

            def _seg_region_progress(frac, msg, _base=seg_base, _span=seg_span):
                report(_base + frac * _span * 0.30, msg)

            min_window_samples = max(1024, int(round(0.25 * fs)))
            if len(audio) < min_window_samples:
                _dprint(f"Segment {segmentindex+1} too short ({len(audio)} samples), skipping.")
                segmentindex += 1
                continue

            if segmentation and all_regions and segmentindex < len(all_regions):
                r = all_regions[segmentindex]
                seg_offset_s = segment_start_sample / fs
                seg_duration_s = len(audio) / fs
                seg_region_info = {
                    "start_time": max(0.0, r["start_time"] - seg_offset_s),
                    "end_time": min(seg_duration_s, r["end_time"] - seg_offset_s),
                    "center_time": r["center_time"] - seg_offset_s,
                    "score": r["score"],
                    "threshold": r.get("threshold", 0.0),
                }
                _dprint(f"Using pre-computed region for segment {segmentindex+1}: "
                        f"{seg_region_info['start_time']:.3f}s-{seg_region_info['end_time']:.3f}s")
                report(seg_base + seg_span * 0.30, "Using pre-computed FSK region")
            else:
                seg_region_info = find_likely_fsk_region(
                    audio, fs, f0, f1, progress_callback=_seg_region_progress
                )

            _dprint(f"Finding the best timing/offset for segment {segmentindex+1}...")

            def _timing_progress(frac, msg, _base=seg_base, _span=seg_span):
                report(_base + _span * 0.30 + frac * _span * 0.55, msg)

            best_setup = select_best_symbol_timing_and_offset(
                audio, fs, f0, f1, p0,
                region_info=seg_region_info,
                progress_callback=_timing_progress,
            )
            N = best_setup["N"]
            N_err = best_setup["N_err"]
            best_offset = best_setup["offset"]
            best_offset_score = best_setup["avg_abs_score"]
            _dprint(
                f"Decoding bits for segment {segmentindex+1} with N={N}, offset {best_offset} "
                f"(avg separation {best_offset_score:.6f})..."
            )
            report(seg_base + seg_span * 0.85, "Decoding bits")
            E0, E1, scores = fsk_symbol_metrics(audio, fs, f0, f1, N, N_err, start=best_offset)
            bits = (E1 > E0).astype(int)
            report(seg_base + seg_span * 0.87, "Decoding messages")

            if len(bits) == 0:
                segmentindex += 1
                continue
            start_samples = compute_symbol_start_samples(len(bits), N, N_err, start=best_offset)
            if DEBUG_PLOTS:
                _all_scores_t.extend((start_samples + segment_start_sample) / fs)
                _all_scores_v.extend(scores)
            if len(start_samples) > 1:
                next_start_samples = np.empty_like(start_samples)
                next_start_samples[:-1] = start_samples[1:]
                next_start_samples[-1] = (
                    start_samples[-1]
                    + N
                    - int(math.floor((len(bits) + 1) * N_err))
                    + int(math.floor(len(bits) * N_err))
                )
            else:
                next_start_samples = np.array(
                    [
                        start_samples[0]
                        + N
                        - int(math.floor((len(bits) + 1) * N_err))
                        + int(math.floor(len(bits) * N_err))
                    ],
                    dtype=int,
                )
            region_start_sample = max(0, int(round(seg_region_info["start_time"] * fs)))
            region_end_sample = min(len(audio), int(round(seg_region_info["end_time"] * fs)))
            region_mask = (start_samples >= region_start_sample) & (start_samples < region_end_sample)
            th0, th1 = define_thresholds(scores, region_mask=region_mask)
            _dprint(f"Segment {segmentindex+1} thresholds: th0={th0:.6f} (f0), th1={th1:.6f} (f1)")
            if DEBUG_PLOTS:
                seg_t = (start_samples + segment_start_sample) / fs
                _segment_thresholds.append((float(seg_t[0]), float(seg_t[-1]), th0, th1))
            if DEBUG_PLOTS:
                plt.scatter(range(len(scores)), scores, s=10)
                plt.axhline(th0, color="red", linestyle="--", label=f"th0={th0:.4f}")
                plt.axhline(th1, color="green", linestyle="--", label=f"th1={th1:.4f}")
                plt.title(f"Segment {segmentindex+1} Symbol Scores")
                plt.xlabel("Symbol Index")
                plt.ylabel("Score (E1 - E0)")
                plt.legend()
                plt.grid()
                plt.savefig(f"PLOTS/segment_{segmentindex+1}_scores.png")
                plt.close()
            mask = generate_mask(th0, th1, scores)
            msg_ranges = find_message_ranges(mask, ecc_nsym)
            num_msg_ranges = max(len(msg_ranges), 1)
            for range_idx, (start_idx, end_idx) in enumerate(msg_ranges):
                report(
                    seg_base + seg_span * (0.87 + (range_idx / num_msg_ranges) * 0.12),
                    f"Decoding message {range_idx + 1}/{len(msg_ranges)}",
                )
                msg_bits = bits[start_idx : min(len(bits), end_idx + 1 + ecc_nsym * 8)]
                if len(msg_bits) == 0:
                    continue
                message_start_sample = segment_start_sample + start_samples[start_idx]
                message_end_sample = segment_start_sample + next_start_samples[end_idx]
                time_in_recording = (
                    message_start_sample / fs
                )
                if len(msg_bits) < MIN_MESSAGE_BITS:
                    continue
                else:
                    debug.write("\nTime in recording: " + seconds_to_hms(time_in_recording) + "\n")
                    _dprint(f"Decoded message bits: {len(msg_bits)}\n")
                start_time = time_in_recording
                end_time = (
                    message_end_sample / fs
                )

                mid_bits = np.array([b for byte in b"/MID" for b in (int(byte) >> i & 1 for i in range(7, -1, -1))], dtype=int)
                mid_offset = None
                for mid_i in range(len(msg_bits) - len(mid_bits) + 1):
                    if np.array_equal(msg_bits[mid_i:mid_i + len(mid_bits)], mid_bits):
                        mid_offset = mid_i
                        msg_bits = msg_bits[mid_i:]
                        break

                if mid_offset is not None:
                    mid_bit_idx = start_idx + mid_offset
                    if mid_bit_idx < len(start_samples):
                        start_time = (segment_start_sample + start_samples[mid_bit_idx]) / fs

                if use_ecc:
                    rs_result = decode_message_with_rs(
                        msg_bits,
                        rsc,
                        ecc_nsym,
                        decode_codeword_fn,
                        rs_error_type,
                    )
                    if rs_result is not None:
                        decoded_payload, ecc_bit_offset = rs_result
                        if mid_offset is None:
                            mid_bit_idx = start_idx + ecc_bit_offset
                            if mid_bit_idx < len(start_samples):
                                start_time = (segment_start_sample + start_samples[mid_bit_idx]) / fs
                        _dprint("ECC decode successful")
                    else:
                        _dprint("ECC decode failed")
                        debug.write("Warning: RS decode failed for this message segment\n")
                        decoded_payload = decode_message_without_ecc(
                            msg_bits=msg_bits,
                            trim_to_trailing_slash=True,
                        )
                        if not decoded_payload:
                            debug.write("Warning: Could not decode bytes for this message segment\n")
                            continue
                else:
                    decoded_payload = decode_message_without_ecc(msg_bits)
                    if not decoded_payload:
                        debug.write("Warning: Could not decode bytes for this message segment\n")
                        continue

                message = decoded_payload.decode("ascii", errors="replace")
                if print_message(message, start_time, end_time, debug, label_track):
                    message_count += 1

            segmentindex += 1

        if DEBUG_PLOTS and _all_scores_t:
            _, ax = plt.subplots(figsize=(14, 4))
            ax.scatter(_all_scores_t, _all_scores_v, s=5)
            for t in _segment_split_times:
                ax.axvline(t, color="red", linewidth=0.8, linestyle="--")
            th0_labeled = th1_labeled = False
            for t_start, t_end, th0, th1 in _segment_thresholds:
                ax.hlines(th1, t_start, t_end, colors="tab:green", linewidths=1.5,
                          label="th1 (f1)" if not th1_labeled else "_nolegend_")
                ax.hlines(th0, t_start, t_end, colors="tab:orange", linewidths=1.5,
                          label="th0 (f0)" if not th0_labeled else "_nolegend_")
                th0_labeled = th1_labeled = True
            if _segment_thresholds:
                ax.legend(fontsize=8)
            ax.set_title("Symbol Scores — Full Audio File")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Score (E1 - E0)")
            ax.grid()
            plt.tight_layout()
            plt.show()

        report(1.0, "Done")
        return message_count



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="FSK demodulator")
    parser.add_argument("-i", "--input", required=True, help="Input WAV file")
    parser.add_argument("--f0", type=float, default=20833.33, help="Frequency for bit 0 [Hz]")
    parser.add_argument("--f1", type=float, default=22222.22, help="Frequency for bit 1 [Hz]")
    parser.add_argument("--generate-debug", action="store_true",
                        help="Generate debug file (.txt)")
    parser.add_argument(
        "--segmentation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Auto-detect all FSK regions and split audio at midpoints between them",
    )
    parser.add_argument(
        "--use-ecc",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable/disable Reed-Solomon error correction (default: enabled)",
    )
    parser.add_argument(
        "--ecc-parity-bytes",
        type=int,
        default=DEFAULT_ECC_NSYM,
        help=f"Reed-Solomon parity bytes (nsym) when ECC is enabled (default: {DEFAULT_ECC_NSYM})",
    )

    args = parser.parse_args()

    decode_fsk(
        input_filename=args.input,
        f0=args.f0,
        f1=args.f1,
        generate_debug=args.generate_debug,
        segmentation=args.segmentation,
        use_ecc=args.use_ecc,
        ecc_nsym=args.ecc_parity_bytes,
    )
