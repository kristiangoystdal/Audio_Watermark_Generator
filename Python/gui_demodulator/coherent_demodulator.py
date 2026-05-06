"""
Coherent FSK demodulator — drop-in replacement for demodulator.py.

Core algorithm (per symbol window):
  1. lo_f0[n] = exp(-j·2π·f0·n/fs)   ← local oscillator at f0
     lo_f1[n] = exp(-j·2π·f1·n/fs)   ← local oscillator at f1
  2. mixed_f0 = signal[n] · lo_f0[n]  ← mix (multiply)
     mixed_f1 = signal[n] · lo_f1[n]
  3. corr_f0  = Σ mixed_f0            ← integrate (matched-filter output)
     corr_f1  = Σ mixed_f1
  4. bit = 1  if |corr_f1|² > |corr_f0|²

Drop-in API (identical to demodulator.py):
  decode_fsk(input_filename, f0, f1, generate_debug, minutes_per_segment,
             use_ecc, ecc_nsym, progress_callback) → int

CLI:
  python coherent_demodulation.py -i FILE --f0 8347 --f1 9056
"""

import argparse
import math
import os
import sys

import numpy as np
import soundfile as sf
from scipy.signal import windows

# Re-use output-formatting and ECC utilities from demodulator.py so the
# written file formats are byte-for-byte identical.
from demodulator import (
    MIN_MESSAGE_BITS,
    compute_p0,
    compute_symbol_start_samples,
    decode_message_with_rs,
    decode_message_without_ecc,
    define_thresholds,
    find_message_ranges,
    generate_mask,
    print_message,
    seconds_to_hms,
)
from reed_solomon import NSYM as DEFAULT_ECC_NSYM


# ---------------------------------------------------------------------------
# FSK region detection — coherent LO-power scan
# ---------------------------------------------------------------------------

def _find_fsk_region(
    audio: np.ndarray,
    fs: float,
    f0: float,
    f1: float,
    window_sec: float = 0.25,
    hop_sec: float = 0.10,
    sigma: float = 3.0,
    progress_callback=None,
) -> dict:
    """
    Locate the time region most likely containing the FSK signal.

    Each analysis window is scored by mixing with local oscillators at f0
    and f1 and taking the maximum power.  The highest-scoring contiguous
    group of windows above the noise threshold is returned.

    Returns a dict with the same keys as demodulator.find_likely_fsk_region.
    """
    win_size = max(1024, int(round(window_sec * fs)))
    hop = max(1, int(round(hop_sec * fs)))

    n = np.arange(win_size, dtype=float)
    lo_f0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    lo_f1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)
    hann = windows.hann(win_size, sym=False)
    hann_energy = float(np.sum(hann ** 2))

    starts = np.arange(0, len(audio) - win_size + 1, hop)
    total = len(starts)
    scores = np.empty(total, dtype=float)
    progress_width = 24

    def _print_progress(done: int):
        frac = done / total if total else 1.0
        filled = int(round(progress_width * frac))
        bar = "#" * filled + "-" * (progress_width - filled)
        sys.stdout.write(f"\rFinding active FSK region [{bar}] {frac * 100:5.1f}%")
        sys.stdout.flush()
        if progress_callback is not None:
            progress_callback(frac, "Finding active FSK region")

    _print_progress(0)
    for i, s in enumerate(starts):
        seg = audio[s : s + win_size] * hann
        p0 = (np.abs(np.dot(seg, np.conj(lo_f0))) ** 2) / hann_energy
        p1 = (np.abs(np.dot(seg, np.conj(lo_f1))) ** 2) / hann_energy
        scores[i] = max(p0, p1)
        if (i + 1) % max(1, total // 20) == 0:
            _print_progress(i + 1)
    _print_progress(total)
    sys.stdout.write("\n")

    center_times = (starts + win_size // 2) / fs

    median = np.median(scores)
    mad = np.median(np.abs(scores - median))
    threshold = median + sigma * 1.4826 * mad

    active = scores >= threshold
    if not np.any(active):
        best = int(np.argmax(scores))
        t_c = center_times[best]
        t_start = max(0.0, t_c - 0.5 * window_sec)
        t_end = min(len(audio) / fs, t_c + 0.5 * window_sec)
    else:
        active_idx = np.flatnonzero(active)
        splits = np.where(np.diff(active_idx) > 1)[0] + 1
        groups = np.split(active_idx, splits)
        best_group = max(groups, key=lambda g: float(np.mean(scores[g])))
        t_start = max(0.0, center_times[best_group[0]] - 0.5 * window_sec)
        t_end = min(len(audio) / fs, center_times[best_group[-1]] + 0.5 * window_sec)

    result = {
        "start_time": float(t_start),
        "end_time": float(t_end),
        "center_time": float((t_start + t_end) / 2),
        "score": float(np.max(scores)),
        "threshold": float(threshold),
        "window_times": center_times,
        "window_scores": scores,
    }
    print(
        f"Active FSK region found: {t_start:.3f}s to {t_end:.3f}s "
        f"(center {result['center_time']:.3f}s, score {result['score']:.6f})"
    )
    return result


# ---------------------------------------------------------------------------
# Carrier frequency refinement — FFT peak search on active region
# ---------------------------------------------------------------------------

def _refine_carrier_frequencies(
    audio: np.ndarray,
    fs: float,
    f0: float,
    f1: float,
    search_hz: float = 200.0,
) -> tuple[float, float]:
    """
    Refine f0 and f1 by finding FFT power peaks within ±search_hz of each
    nominal frequency.  Only runs for clips ≤ 20 s (same guard as
    demodulator.refine_fsk_tones_fft).
    """
    if len(audio) / fs > 20.0:
        return f0, f1

    w = np.hanning(len(audio))
    X = np.fft.rfft(audio * w)
    power = X.real ** 2 + X.imag ** 2
    df = fs / len(audio)
    nyq = fs / 2.0

    def _peak(f_center: float) -> float:
        lo = max(0.0, f_center - search_hz)
        hi = min(nyq, f_center + search_hz)
        k_lo = int(np.clip(np.round(lo / df), 0, len(power) - 1))
        k_hi = int(np.clip(np.round(hi / df), 0, len(power) - 1))
        k = k_lo + int(np.argmax(power[k_lo : k_hi + 1]))
        if 1 <= k < len(power) - 1:
            y0 = np.log(power[k - 1] + 1e-30)
            y1 = np.log(power[k] + 1e-30)
            y2 = np.log(power[k + 1] + 1e-30)
            denom = y0 - 2 * y1 + y2
            if denom != 0.0:
                return float(np.clip(k * df + 0.5 * (y0 - y2) / denom * df, lo, hi))
        return float(k * df)

    return _peak(f0), _peak(f1)


# ---------------------------------------------------------------------------
# Core coherent mixing helpers
# ---------------------------------------------------------------------------

def _make_local_oscillators(
    N: int, fs: float, f0: float, f1: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return complex LO vectors for one symbol window of N samples."""
    n = np.arange(N, dtype=float)
    lo_f0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    lo_f1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)
    return lo_f0, lo_f1


def _mix_and_integrate(
    segment: np.ndarray,
    lo_f0: np.ndarray,
    lo_f1: np.ndarray,
    hann: np.ndarray,
) -> tuple[complex, complex]:
    """
    Coherent demodulation for one symbol window.

    Applies Hann window → mix with each LO → integrate (inner product).
    Returns (corr_f0, corr_f1) complex matched-filter outputs.
    """
    seg_w = segment * hann
    corr_f0 = np.dot(seg_w, np.conj(lo_f0))   # mix f0 + integrate
    corr_f1 = np.dot(seg_w, np.conj(lo_f1))   # mix f1 + integrate
    return corr_f0, corr_f1


# ---------------------------------------------------------------------------
# Joint timing search — N_true × offset grid
# ---------------------------------------------------------------------------

def _find_best_timing(
    audio: np.ndarray,
    fs: float,
    f0: float,
    f1: float,
    nominal_N_true: float,
    region_info: dict,
    n_true_radius: float = 1.0,
    n_true_step: float = 0.1,
    offset_step: int = 25,
    progress_callback=None,
) -> dict:
    """
    Joint 2-D grid search over symbol-length (N_true) and start offset.

    For each (N_true, offset) pair the mean |E1 - E0| across all symbols
    in the active region is computed via coherent LO mixing.  The pair
    that maximises this score is returned.

    Return dict matches the shape of demodulator.select_best_symbol_timing_and_offset.
    """
    region_start = max(0, int(round(region_info["start_time"] * fs)))
    region_end = min(len(audio), int(round(region_info["end_time"] * fs)))
    region = audio[region_start:region_end]
    if len(region) == 0:
        region = audio
        region_start = 0

    n_true_vals = np.arange(
        max(n_true_step, nominal_N_true - n_true_radius),
        nominal_N_true + n_true_radius + 0.5 * n_true_step,
        n_true_step,
    )

    total_candidates = sum(
        len(range(0, max(1, int(math.ceil(nv))), max(1, offset_step)))
        for nv in n_true_vals
    )
    progress_width = 24
    done = [0]

    def _print_progress():
        frac = done[0] / total_candidates if total_candidates else 1.0
        filled = int(round(progress_width * frac))
        bar = "#" * filled + "-" * (progress_width - filled)
        sys.stdout.write(f"\rFinding best timing/offset [{bar}] {frac * 100:5.1f}%")
        sys.stdout.flush()
        if progress_callback is not None:
            progress_callback(frac, "Finding best timing/offset")

    _print_progress()

    best_setup = None

    for N_true_cand in n_true_vals:
        N = max(1, int(math.ceil(N_true_cand)))
        N_err = N - N_true_cand
        lo_f0, lo_f1 = _make_local_oscillators(N, fs, f0, f1)
        hann = windows.hann(N, sym=False)

        best_offset_this_N = 0
        best_score_this_N = float("-inf")

        for offset in range(0, N, max(1, offset_step)):
            sep_vals = []
            acc_err = 0.0
            s = offset
            while s + N <= len(region):
                acc_err += N_err
                cs = s - int(math.floor(acc_err))
                if 0 <= cs and cs + N <= len(region):
                    seg = region[cs : cs + N]
                    cf0, cf1 = _mix_and_integrate(seg, lo_f0, lo_f1, hann)
                    sep_vals.append(abs(abs(cf1) ** 2 - abs(cf0) ** 2))
                s += N
            done[0] += 1
            _print_progress()

            if sep_vals:
                avg = float(np.mean(sep_vals))
                if avg > best_score_this_N:
                    best_score_this_N = avg
                    best_offset_this_N = offset

        if best_setup is None or best_score_this_N > best_setup["avg_abs_score"]:
            best_setup = {
                "f0": float(f0),
                "symbol_time": float(N_true_cand / fs),
                "N": int(N),
                "N_true": float(N_true_cand),
                "N_err": float(N_err),
                "offset": int(best_offset_this_N),
                "avg_abs_score": float(best_score_this_N if np.isfinite(best_score_this_N) else 0.0),
                "region_start_time": float(region_info["start_time"]),
                "region_end_time": float(region_info["end_time"]),
            }

    sys.stdout.write("\n")
    if best_setup is None:
        raise ValueError("No valid N/offset candidate produced demodulation scores")

    print(
        f"Best timing/offset found: N={best_setup['N']}, "
        f"N_true={best_setup['N_true']:.4f}, offset={best_setup['offset']}"
    )
    return best_setup


# ---------------------------------------------------------------------------
# Coherent bit decoding
# ---------------------------------------------------------------------------

def _decode_bits_coherent(
    audio: np.ndarray,
    fs: float,
    f0: float,
    f1: float,
    N: int,
    N_err: float,
    start: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Decode FSK bits via coherent LO mixing with fractional drift correction.

    Mirrors the interface of demodulator.fsk_symbol_metrics but uses
    explicit LO multiplication instead of Goertzel/DFT correlation.

    Returns (E0, E1, scores) where scores = E1 - E0, matching the shape
    expected by demodulator.define_thresholds and generate_mask.
    """
    lo_f0, lo_f1 = _make_local_oscillators(N, fs, f0, f1)
    hann = windows.hann(N, sym=False)

    E0_list, E1_list = [], []
    acc_err = 0.0

    for s in range(start, len(audio) - N + 1, N):
        acc_err += N_err
        cs = s - int(math.floor(acc_err))
        if cs < 0 or cs + N > len(audio):
            continue
        seg = audio[cs : cs + N]
        cf0, cf1 = _mix_and_integrate(seg, lo_f0, lo_f1, hann)
        e0 = abs(cf0) ** 2
        e1 = abs(cf1) ** 2
        E0_list.append(e0)
        E1_list.append(e1)

    E0 = np.array(E0_list, dtype=float)
    E1 = np.array(E1_list, dtype=float)
    return E0, E1, E1 - E0


# ---------------------------------------------------------------------------
# Main entry point — identical signature to demodulator.decode_fsk
# ---------------------------------------------------------------------------

def decode_fsk(
    input_filename: str,
    f0: float = 20833.33,
    f1: float = 22222.22,
    generate_debug: bool = False,
    minutes_per_segment: int = -1,
    use_ecc: bool = True,
    ecc_nsym: int = DEFAULT_ECC_NSYM,
    progress_callback=None,
) -> int:
    """
    Demodulate an FSK-modulated WAV file using coherent LO mixing and write
    Audacity-compatible label and optional debug files.

    Parameters and return value are identical to demodulator.decode_fsk so
    this module can be used as a drop-in replacement.

    Parameters
    ----------
    input_filename : str
        Path to the input WAV file.
    f0, f1 : float
        Carrier frequencies for bits 0 and 1.
    generate_debug : bool
        Write a companion *_debug.txt file.
    minutes_per_segment : int
        Segment length in minutes (-1 = whole file).
    use_ecc : bool
        Enable Reed-Solomon error correction.
    ecc_nsym : int
        RS parity bytes (1–254).
    progress_callback : callable, optional
        Called as progress_callback(fraction: float, message: str).

    Returns
    -------
    int
        Number of successfully decoded messages.
    """
    if use_ecc and not (1 <= ecc_nsym <= 254):
        raise ValueError("ecc_nsym must be in range [1, 254] when ECC is enabled")

    def report(fraction, message=""):
        if progress_callback is not None:
            progress_callback(fraction, message)

    message_count = 0
    p0 = compute_p0(f0)
    base, _ = os.path.splitext(input_filename)
    debug_filename = base + "_debug.txt" if generate_debug else os.devnull
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
        print(f"Reading audio from {input_filename}...")
        audio, fs = sf.read(input_filename)
        if audio.ndim > 1:
            audio = audio[:, 0]

        report(0.05, "Finding active FSK region")
        print("Locating likely FSK-active region...")

        def _region_progress(frac, msg):
            report(0.05 + frac * 0.20, msg)

        region_info = _find_fsk_region(audio, fs, f0, f1, progress_callback=_region_progress)

        active_start = max(0, int(round(region_info["start_time"] * fs)))
        active_end = min(len(audio), int(round(region_info["end_time"] * fs)))
        active_audio = audio[active_start:active_end] if active_end > active_start else audio

        report(0.25, "Refining FSK tones")
        print("Refining FSK tones from detected active region...")
        f0, f1 = _refine_carrier_frequencies(active_audio, fs, f0, f1)
        print(
            f"Demodulation parameters: f0={f0:.2f} Hz, f1={f1:.2f} Hz, p0={p0}, "
            f"search-region={region_info['start_time']:.3f}s-{region_info['end_time']:.3f}s"
        )

        if minutes_per_segment <= 0:
            minutes_per_segment = len(audio) / fs / 60
        seg_len = int(round(minutes_per_segment * 60 * fs))
        audio_len = audio.shape[0]
        segments = [(i, audio[i : i + seg_len]) for i in range(0, audio_len, seg_len)]

        report(0.30, "Processing audio segments")
        print(f"Processing {len(segments)} audio segment(s) for demodulation...")
        num_segments = len(segments)

        for seg_idx, (segment_start_sample, seg_audio) in enumerate(segments):
            seg_base = 0.30 + (seg_idx / num_segments) * 0.70
            seg_span = 0.70 / num_segments
            print(f"\nProcessing segment {seg_idx + 1}...")

            def _seg_region_progress(frac, msg, _b=seg_base, _s=seg_span):
                report(_b + frac * _s * 0.30, msg)

            seg_region_info = _find_fsk_region(
                seg_audio, fs, f0, f1, progress_callback=_seg_region_progress
            )

            nominal_N_true = (p0 / f0) * fs
            print(f"Finding the best timing/offset for segment {seg_idx}...")

            def _timing_progress(frac, msg, _b=seg_base, _s=seg_span):
                report(_b + _s * 0.30 + frac * _s * 0.55, msg)

            best_setup = _find_best_timing(
                seg_audio,
                fs,
                f0,
                f1,
                nominal_N_true,
                region_info=seg_region_info,
                progress_callback=_timing_progress,
            )

            N = best_setup["N"]
            N_err = best_setup["N_err"]
            best_offset = best_setup["offset"]
            best_offset_score = best_setup["avg_abs_score"]
            print(
                f"Decoding bits for segment {seg_idx} with N={N}, offset {best_offset} "
                f"(avg separation {best_offset_score:.6f})..."
            )

            report(seg_base + seg_span * 0.85, "Decoding bits")
            E0, E1, scores = _decode_bits_coherent(
                seg_audio, fs, f0, f1, N, N_err, start=best_offset
            )
            bits = (E1 > E0).astype(int)

            report(seg_base + seg_span * 0.87, "Decoding messages")

            if len(bits) == 0:
                continue

            start_samples = compute_symbol_start_samples(
                len(bits), N, N_err, start=best_offset
            )
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
            region_end_sample = min(len(seg_audio), int(round(seg_region_info["end_time"] * fs)))
            region_mask = (start_samples >= region_start_sample) & (
                start_samples < region_end_sample
            )

            th0, th1 = define_thresholds(scores, region_mask=region_mask)
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
                time_in_recording = message_start_sample / fs

                if len(msg_bits) < MIN_MESSAGE_BITS:
                    continue

                debug.write("Time in recording: " + seconds_to_hms(time_in_recording) + "\n")
                print(f"Decoded message bits: {len(msg_bits)}\n")
                start_time = time_in_recording
                end_time = message_end_sample / fs

                # Align to /MID preamble if present
                mid_bits = np.array(
                    [b for byte in b"/MID" for b in (int(byte) >> i & 1 for i in range(7, -1, -1))],
                    dtype=int,
                )
                mid_offset = None
                for mid_i in range(len(msg_bits) - len(mid_bits) + 1):
                    if np.array_equal(msg_bits[mid_i : mid_i + len(mid_bits)], mid_bits):
                        mid_offset = mid_i
                        msg_bits = msg_bits[mid_i:]
                        break

                if mid_offset is not None:
                    mid_bit_idx = start_idx + mid_offset
                    if mid_bit_idx < len(start_samples):
                        start_time = (segment_start_sample + start_samples[mid_bit_idx]) / fs

                if use_ecc:
                    rs_result = decode_message_with_rs(
                        msg_bits, rsc, ecc_nsym, decode_codeword_fn, rs_error_type
                    )
                    if rs_result is not None:
                        decoded_payload, ecc_bit_offset = rs_result
                        if mid_offset is None:
                            mid_bit_idx = start_idx + ecc_bit_offset
                            if mid_bit_idx < len(start_samples):
                                start_time = (
                                    segment_start_sample + start_samples[mid_bit_idx]
                                ) / fs
                        print("ECC decode successful:" + decoded_payload.decode("ascii", errors="replace"))
                    else:
                        print("ECC decode failed")
                        debug.write("Warning: RS decode failed for this message segment\n")
                        decoded_payload = decode_message_without_ecc(
                            msg_bits=msg_bits, trim_to_trailing_slash=True
                        )
                        if not decoded_payload:
                            debug.write("Warning: Could not decode bytes for this message segment\n")
                            continue
                        print("Attempting to decode without ECC...")
                        print("message: " + decoded_payload.decode("ascii", errors="replace") + "\n")
                else:
                    decoded_payload = decode_message_without_ecc(msg_bits)
                    if not decoded_payload:
                        debug.write("Warning: Could not decode bytes for this message segment\n")
                        continue

                message = decoded_payload.decode("ascii", errors="replace")
                if print_message(message, start_time, end_time, debug, label_track):
                    message_count += 1

        report(1.0, "Done")
        return message_count


# ---------------------------------------------------------------------------
# CLI — identical flags to demodulator.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Coherent FSK demodulator (drop-in replacement for demodulator.py)"
    )
    parser.add_argument("-i", "--input", required=True, help="Input WAV file")
    parser.add_argument("--f0", type=float, default=20833.33, help="Frequency for bit 0 [Hz]")
    parser.add_argument("--f1", type=float, default=22222.22, help="Frequency for bit 1 [Hz]")
    parser.add_argument("--generate-debug", action="store_true", help="Generate debug file (.txt)")
    parser.add_argument(
        "--minutes-per-segment",
        type=int,
        default=-1,
        help="Length of each segment in minutes (-1 = process full file)",
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
        help=f"Reed-Solomon parity bytes when ECC is enabled (default: {DEFAULT_ECC_NSYM})",
    )
    args = parser.parse_args()

    decode_fsk(
        input_filename=args.input,
        f0=args.f0,
        f1=args.f1,
        generate_debug=args.generate_debug,
        minutes_per_segment=args.minutes_per_segment,
        use_ecc=args.use_ecc,
        ecc_nsym=args.ecc_parity_bytes,
    )
