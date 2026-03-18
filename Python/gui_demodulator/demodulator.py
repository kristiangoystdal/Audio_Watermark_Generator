"""
FSK demodulator utilities for extracting bitstreams and messages from audio files.

Example command-line usage:
python demodulator.py -i test.wav --f0 21333 --f1 22325 --use-ecc --ecc-parity-bytes 20 --generate-debug --minutes-per-segment -1
"""

import math
import os
import sys
import numpy as np
import soundfile as sf
from scipy.signal import firwin, lfilter, windows
from reed_solomon import NSYM as DEFAULT_ECC_NSYM

MIN_MESSAGE_BITS = 16

def find_likely_fsk_region(
    x,
    fs,
    f0,
    f1=None,
    window_duration=0.25,
    hop_duration=0.1,
    band_width_hz=150.0,
    threshold_sigma=3.0,
):
    """
    Find the time region most likely to contain the FSK signal.

    The signal is scanned in overlapping windows. Each window is scored by the
    matched tone power at `f0` and optionally `f1`. Contiguous windows with
    unusually high tone power are merged into candidate regions, and the region
    with the highest average score is returned.

    `band_width_hz` is kept for call-site compatibility but is not used by the
    fast path, which evaluates only the exact tone frequencies.
    """
    if window_duration <= 0 or hop_duration <= 0:
        raise ValueError("window_duration and hop_duration must be > 0")
    x = np.asarray(x, dtype=float)
    if x.ndim != 1:
        raise ValueError("x must be a 1D audio signal")
    _ = band_width_hz

    window_size = max(8, int(round(window_duration * fs)))
    hop_size = max(1, int(round(hop_duration * fs)))
    if len(x) < window_size:
        raise ValueError("Audio is shorter than one analysis window")

    window = windows.hann(window_size, sym=False)
    starts = np.arange(0, len(x) - window_size + 1, hop_size, dtype=int)
    frames = np.lib.stride_tricks.sliding_window_view(x, window_size)[::hop_size]
    n = np.arange(window_size, dtype=float)
    window_energy = float(np.sum(window ** 2))

    osc_f0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    osc_f1 = None
    if f1 is not None:
        osc_f1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)

    total_windows = len(starts)
    batch_size = max(1, min(2048, total_windows))
    window_scores = np.empty(total_windows, dtype=float)
    progress_width = 24

    def print_progress(done_windows: int):
        fraction = done_windows / total_windows if total_windows else 1.0
        filled = int(round(progress_width * fraction))
        bar = "#" * filled + "-" * (progress_width - filled)
        sys.stdout.write(
            f"\rFinding active FSK region [{bar}] {fraction * 100:5.1f}%"
        )
        sys.stdout.flush()

    print_progress(0)
    for batch_start in range(0, total_windows, batch_size):
        batch_end = min(batch_start + batch_size, total_windows)
        batch_frames = frames[batch_start:batch_end] * window
        tone0_power = np.abs(batch_frames @ np.conjugate(osc_f0)) ** 2 / window_energy
        if osc_f1 is not None:
            tone1_power = np.abs(batch_frames @ np.conjugate(osc_f1)) ** 2 / window_energy
            window_scores[batch_start:batch_end] = np.maximum(tone0_power, tone1_power)
        else:
            window_scores[batch_start:batch_end] = tone0_power
        print_progress(batch_end)
    sys.stdout.write("\n")
    window_times = (starts + 0.5 * window_size) / fs

    baseline = float(np.median(window_scores))
    mad = float(np.median(np.abs(window_scores - baseline)))
    robust_sigma = 1.4826 * mad
    if robust_sigma == 0.0:
        threshold = baseline
    else:
        threshold = baseline + threshold_sigma * robust_sigma

    active = window_scores >= threshold
    if not np.any(active):
        best_idx = int(np.argmax(window_scores))
        start_time = max(0.0, window_times[best_idx] - 0.5 * window_duration)
        end_time = min(len(x) / fs, window_times[best_idx] + 0.5 * window_duration)
        result = {
            "start_time": float(start_time),
            "end_time": float(end_time),
            "center_time": float(window_times[best_idx]),
            "score": float(window_scores[best_idx]),
            "threshold": float(threshold),
            "window_times": window_times,
            "window_scores": window_scores,
        }
    else:
        active_idx = np.flatnonzero(active)
        split_points = np.where(np.diff(active_idx) > 1)[0] + 1
        groups = np.split(active_idx, split_points)

        best_region = None
        for group in groups:
            region_score = float(np.mean(window_scores[group]))
            region_start = max(0.0, window_times[group[0]] - 0.5 * window_duration)
            region_end = min(len(x) / fs, window_times[group[-1]] + 0.5 * window_duration)
            if best_region is None or region_score > best_region["score"]:
                best_region = {
                    "start_time": float(region_start),
                    "end_time": float(region_end),
                    "center_time": float(np.mean(window_times[group])),
                    "score": region_score,
                    "threshold": float(threshold),
                    "window_times": window_times,
                    "window_scores": window_scores,
                }
        result = best_region

    print(
        "Active FSK region found: "
        f"{result['start_time']:.3f}s to {result['end_time']:.3f}s "
        f"(center {result['center_time']:.3f}s, score {result['score']:.6f})"
    )

    return result

def fsk_symbol_metrics(x, fs, f0, f1, N_samples, N_err, start):
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
    acc_err = 0.0
    for seg_start in range(start, len(x) - N_samples+ 1, N_samples):
        acc_err += N_err
        seg_start -= int(math.floor(acc_err))
        seg = x[seg_start : seg_start + N_samples] * W
        X0 = np.vdot(e_0, seg)
        X1 = np.vdot(e_1, seg)
        denom = np.sum(W**2)
        e0v = (np.abs(X0) ** 2) / denom
        e1v = (np.abs(X1) ** 2) / denom
        E0.append(e0v)
        E1.append(e1v)
        S.append(abs(e1v - e0v))
    return np.array(E0), np.array(E1), np.array(S)

def fsk_decode(x, fs, f0, f1, N, N_err):
    """Decode bits for a signal chunk using zero symbol offset."""
    E0, E1, _ = fsk_symbol_metrics(
        x, fs, f0, f1, N, N_err, start=0)
    bits = (E1 > E0).astype(int)
    scores = E1 - E0
    return bits, scores

def compute_symbol_start_samples(num_symbols, N_samples, N_err):
    """Return symbol start samples using the same accumulated drift correction as demodulation."""
    starts = np.empty(num_symbols, dtype=int)
    acc_err = 0.0
    seg_start = 0
    for idx in range(num_symbols):
        acc_err += N_err
        corrected_start = seg_start - int(math.floor(acc_err))
        starts[idx] = corrected_start
        seg_start += N_samples
    return starts

def define_thresholds(scores):
    """Derive loose thresholds from symbol score distribution to mask noise."""
    pos_vals = [s for s in scores if s > 0.0003]
    neg_vals = [s for s in scores if s < -0.0003]
    
    th1 = 0.2 * (sum(pos_vals) / len(pos_vals)) if pos_vals else 0.0
    th0 = 0.2 * (sum(neg_vals) / len(neg_vals)) if neg_vals else 0.0
    
    return th0, th1

def generate_mask(th0, th1, scores):
    """Return boolean mask of scores considered reliable for decoding."""
    scores = np.asarray(scores, dtype=float)
    return (scores > th1) | (scores < th0)

days_of_week = [
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday",
]

def print_message(message, start_time, end_time, debug, labels):
    """Write decoded message info to debug and labels streams."""
    debug.write("Decoded Message: \"" + message + "\"\n")
    if len(message) == 0:
        debug.write("Warning: 0 length message\n")
        return
    elif message[0] != "/":
        debug.write("Warning: Message might be corrupted (missing '/' preamble)\n")
        return
    elif message[-1] != "/":
        debug.write("Warning: Message might be corrupted (missing '/' termination)\n")
        return
    else:
        labels.write(f"{start_time:.6f}\t{end_time:.6f}\t")
        debug.write(f"{len(message)} characters\n")

        messages_lines = message.split("/")
        for line in messages_lines:
            if line[0:3] == "STR":
                labels.write("Message: " + line[3:] + " | ")
            elif line[0:3] == "LOC":
                labels.write("Location: " + line[3:] + " | ")
            elif line[0:3] == "DID":
                labels.write("Device ID: " + line[3:] + " | ")
            elif line[0:3] == "TMP":
                labels.write("Temperature: " + line[3:] + "°C | ")
            elif line[0:3] == "TIM":
                labels.write("Time: " + f"{line[3:5]}:{line[5:7]}:{line[7:9]} on {days_of_week[int(line[9:11]) - 1]} {line[11:13]}/{line[13:15]}/{line[15:19]}" + "\n")
        debug.write("\n")

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
    
def split_messages_by_gap(bits, times, symbol_time):
    """Split bitstream into messages based on time gaps larger than 3 symbols."""
    if len(bits) == 0:
        return []

    gaps = np.diff(times, prepend=times[0])
    split_idx = np.where(gaps > 3 * symbol_time)[0]  # start of a new message
    bit_segments = np.split(bits, split_idx)
    time_segments = np.split(times, split_idx)
    return [(b, t) for b, t in zip(bit_segments, time_segments) if len(b) > 0]

import numpy as np

def refine_fsk_tones_fft(
    x,
    fs: float,
    f0: float,
    f1: float,
    search_width_percent: int = 2,
    avg_bin_width: int = 200,
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
    nyq = fs / 2.0

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
        lo = max(0.0, f_center - search_width_percent / 100.0 * f_center)
        hi = min(nyq, f_center + search_width_percent / 100.0 * f_center)
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

    return f0_new, f1_new

import numpy as np

def compute_p0(f0, DAC_fs=960000):
    m = DAC_fs * (3000 / 1_000_000)
    n0 = np.floor(DAC_fs / f0)
    return int(np.round(m / n0))


def find_message_ranges(mask, max_gap_symbols=3):
    """
    Find contiguous message ranges in the original bitstream using reliable symbols.

    Returns a list of (start_idx, end_idx) on the unmasked bit array.
    """
    reliable_idx = np.flatnonzero(mask)
    if len(reliable_idx) == 0:
        return []

    split_at = np.where(np.diff(reliable_idx) > max_gap_symbols)[0] + 1
    groups = np.split(reliable_idx, split_at)
    return [(int(group[0]), int(group[-1])) for group in groups if len(group) > 0]


def bits_to_bytes(bits, bit_offset=0):
    """Pack a bit array into bytes, optionally skipping an initial bit offset."""
    if bit_offset < 0 or bit_offset >= 8:
        raise ValueError("bit_offset must be in range [0, 7]")

    usable_bits = len(bits) - bit_offset
    n_full_bytes = usable_bits // 8
    if n_full_bytes <= 0:
        return b""

    out = bytearray()
    idx = bit_offset
    for _ in range(n_full_bytes):
        value = 0
        for _ in range(8):
            value = (value << 1) | int(bits[idx])
            idx += 1
        out.append(value)
    return bytes(out)


def decode_message_with_rs(msg_bits, rsc, nsym, decode_codeword_fn, rs_error_type):
    """
    Attempt RS decode from a bit segment.

    Tries all bit alignments [0..7] and returns the first successful payload.
    """
    for bit_offset in range(8):
        codeword = bits_to_bytes(msg_bits, bit_offset=bit_offset)
        if len(codeword) <= nsym:
            continue
        try:
            payload = decode_codeword_fn(codeword, codec=rsc)
            return payload
        except rs_error_type:
            continue
    return None


def decode_message_without_ecc(msg_bits):
    """Decode bytes directly from bit segment, trying byte alignments."""
    for bit_offset in range(8):
        payload = bits_to_bytes(msg_bits, bit_offset=bit_offset)
        if not payload:
            continue
        text = payload.decode("ascii", errors="replace")
        if text.startswith("/") and text.endswith("/"):
            return payload
    return bits_to_bytes(msg_bits, bit_offset=0)

def select_best_f0_setup(x, fs, refined_f0, f1, p0, search_span_hz=100.0, step_hz=1.0, region_info=None):
    """Sweep f0 around the refined tone using a fixed p0 and absolute Hz step spacing."""
    if step_hz <= 0:
        raise ValueError("step_hz must be > 0")

    candidate_f0s = np.arange(
        refined_f0 - search_span_hz,
        refined_f0 + search_span_hz + 0.5 * step_hz,
        step_hz,
        dtype=float,
    )
    candidate_f0s = candidate_f0s[candidate_f0s > 0]
    if region_info is None:
        print("Locating likely FSK-active region for f0 sweep...")
        region_info = find_likely_fsk_region(x, fs, refined_f0, f1=f1)
    region_start = max(0, int(round(region_info["start_time"] * fs)))
    region_end = min(len(x), int(round(region_info["end_time"] * fs)))
    region_x = x[region_start:region_end]
    if len(region_x) == 0:
        region_x = x

    print(
        f"Testing {len(candidate_f0s)} f0 candidates over "
        f"{search_span_hz:.1f} Hz around refined f0={refined_f0:.2f} Hz..."
    )

    best_setup = None
    for candidate_f0 in candidate_f0s:
        symbol_time = p0 / candidate_f0
        N = int(round(fs * symbol_time))
        N_true = fs * symbol_time
        if N < N_true:
            N += 1
        N_err = N - N_true
        if N <= 0:
            continue

        _, scores = fsk_decode(region_x, fs, candidate_f0, f1, N, N_err)
        avg_abs_score = float(np.mean(np.abs(scores))) if len(scores) else float("-inf")

        if best_setup is None or avg_abs_score > best_setup["avg_abs_score"]:
            best_setup = {
                "f0": float(candidate_f0),
                "p0": int(p0),
                "symbol_time": float(symbol_time),
                "N": int(N),
                "N_err": float(N_err),
                "avg_abs_score": float(avg_abs_score),
                "region_start_time": float(region_info["start_time"]),
                "region_end_time": float(region_info["end_time"]),
            }

    if best_setup is None:
        raise ValueError("No valid f0 candidate produced demodulation scores")

    return best_setup

def decode_fsk(input_filename: str, 
               f0: float = 20833.33, 
               f1: float = 22222.22,
               generate_debug: bool = False,
               minutes_per_segment: int = -1,
               use_ecc: bool = True,
               ecc_nsym: int = DEFAULT_ECC_NSYM):
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
    minutes_per_segment : int
        Length in minutes of each processed segment (-1 processes the whole file).
    use_ecc : bool
        Enable Reed-Solomon error correction on each decoded message segment.
    ecc_nsym : int
        Number of RS parity bytes to use when ECC is enabled.
    """

    if use_ecc and (ecc_nsym <= 0 or ecc_nsym >= 255):
        raise ValueError("ecc_nsym must be in range [1, 254] when ECC is enabled")
    user_f0 = f0
    p0 = compute_p0(user_f0)

    base, _ = os.path.splitext(input_filename)
    if generate_debug:
        debug_filename = base + "_debug.txt"
    else:
        debug_filename = os.devnull
    labels_filename = base + ".txt"
    with open(labels_filename, "w") as labels, open(debug_filename, "w") as debug:
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
        print(f"Reading audio from {input_filename}...")
        audio, fs = sf.read(input_filename)
        if audio.ndim > 1:
            audio = audio.mean(axis=1)
        audio = (audio - np.mean(audio))
        print("Locating likely FSK-active region...")
        region_info = find_likely_fsk_region(audio, fs, f0, f1=f1)
        region_start = max(0, int(round(region_info["start_time"] * fs)))
        region_end = min(len(audio), int(round(region_info["end_time"] * fs)))
        active_audio = audio[region_start:region_end]
        if len(active_audio) == 0:
            active_audio = audio
        print("Refining FSK tones from detected active region...")
        f0, f1 = refine_fsk_tones_fft(active_audio, fs, f0, f1)
        print("Selecting best f0 for demodulation...")
        best_f0_setup = select_best_f0_setup(audio, fs, f0, f1, p0, region_info=region_info)
        f0 = best_f0_setup["f0"]
        p0 = best_f0_setup["p0"]
        Ts = 1 / fs
        symbol_time = best_f0_setup["symbol_time"]
        N = best_f0_setup["N"]
        N_err = best_f0_setup["N_err"]
        print(
            f"Demodulation parameters: f0={f0:.2f} Hz, f1={f1:.2f} Hz, p0={p0}, "
            f"search-region={best_f0_setup['region_start_time']:.3f}s-"
            f"{best_f0_setup['region_end_time']:.3f}s"
        )
        if minutes_per_segment <= 0:
            minutes_per_segment = len(audio) / fs / 60          
        seg_len = int(round(minutes_per_segment * 60 * fs))
        n = audio.shape[0]
        segments = [(i, audio[i : i + seg_len]) for i in range(0, n, seg_len)]
        print(f"Processing {len(segments)} audio segment(s) for demodulation...")
        segmentindex = 0
        for segment_start_sample, audio in segments:
            print(f"decoding bits for segment {segmentindex}...")
            bits, scores = fsk_decode(audio, fs, f0, f1, N, N_err)
            if len(bits) == 0:
                segmentindex += 1
                continue
            start_samples = compute_symbol_start_samples(len(bits), N, N_err)
            if len(start_samples) > 1:
                next_start_samples = np.empty_like(start_samples)
                next_start_samples[:-1] = start_samples[1:]
                next_start_samples[-1] = start_samples[-1] + N - int(math.floor((len(bits) + 1) * N_err)) + int(math.floor(len(bits) * N_err))
            else:
                next_start_samples = np.array([start_samples[0] + N], dtype=int)
            th0, th1 = define_thresholds(scores)
            mask = generate_mask(th0, th1, scores)
            DMA_reset_delay = 25  # samples
            msg_ranges = find_message_ranges(mask)
            for start_idx, end_idx in msg_ranges:
                msg_bits = bits[start_idx : end_idx + 1]
                if len(msg_bits) == 0:
                    continue
                message_start_sample = segment_start_sample + start_samples[start_idx]
                message_end_sample = segment_start_sample + next_start_samples[end_idx]
                time_in_recording = (
                    message_start_sample / fs - DMA_reset_delay * Ts
                )
                if len(msg_bits) < MIN_MESSAGE_BITS:
                    continue
                else:
                    debug.write("Time in recording: " + seconds_to_hms(time_in_recording) + "\n")
                    print(f"Decoded message bits: {len(msg_bits)}\n")
                start_time = time_in_recording
                end_time = (
                    message_end_sample / fs - DMA_reset_delay * Ts
                )
                if use_ecc:
                    decoded_payload = decode_message_with_rs(
                        msg_bits,
                        rsc,
                        ecc_nsym,
                        decode_codeword_fn,
                        rs_error_type,
                    )
                    if decoded_payload is None:
                        debug.write("Warning: RS decode failed for this message segment\n")

                        message = decode_message_without_ecc(msg_bits)
                        print("Attempting to decode without ECC...")
                        print("message: " + message.decode("ascii", errors="replace") + "\n")
                        continue
                else:
                    decoded_payload = decode_message_without_ecc(msg_bits)
                    if not decoded_payload:
                        debug.write("Warning: Could not decode bytes for this message segment\n")
                        continue

                message = decoded_payload.decode("ascii", errors="replace")
                print_message(message, start_time, end_time, debug, labels)

            segmentindex += 1



if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="FSK demodulator")
    parser.add_argument("-i", "--input", required=True, help="Input WAV file")
    parser.add_argument("--f0", type=float, default=20833.33, help="Frequency for bit 0 [Hz]")
    parser.add_argument("--f1", type=float, default=22222.22, help="Frequency for bit 1 [Hz]")
    parser.add_argument("--generate-debug", action="store_true",
                        help="Generate debug file (.txt)")
    parser.add_argument("--minutes-per-segment", type=int, default=-1,
                        help="Length of each segment in minutes (-1 = process full file)")
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
        minutes_per_segment=args.minutes_per_segment,
        use_ecc=args.use_ecc,
        ecc_nsym=args.ecc_parity_bytes,
    )
