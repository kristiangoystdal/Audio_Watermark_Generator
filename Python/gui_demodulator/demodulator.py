"""
FSK demodulator utilities for extracting bitstreams and messages from audio files.

Example command-line usage:
python demodulator.py -i /path/to/recording.wav --f0 20833.33 --f1 22222.22 --p0 60 --use-ecc --ecc-parity-bytes 8 --generate-debug --minutes-per-segment -1
"""

import math
import os
import numpy as np
import soundfile as sf
from scipy.signal import firwin, lfilter, windows
from reed_solomon import NSYM as DEFAULT_ECC_NSYM

def bandlimit(signal, f_lower, f_upper, fs, numtaps=257):
    """Apply an FIR bandpass filter and return filtered signal and group delay."""
    bp = firwin(numtaps, [f_lower, f_upper], pass_zero=False, fs=fs)
    group_delay = (numtaps - 1) // 2
    return lfilter(bp, [1.0], signal), group_delay

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

def find_best_offset(x, fs, f0, f1, N, N_err, stepsize=25):
    """Search for the symbol alignment offset that maximizes average separation."""
    best_offset, best_val = 0, -np.inf
    for offset in range(0, N, stepsize):
        _, _, S = fsk_symbol_metrics(
            x, fs, f0, f1, N, N_err, start=offset
        )
        if len(S) == 0:
            continue
        val = np.mean(S)
        if val > best_val:
            best_val, best_offset = val, offset
    return best_offset, best_val

def fsk_decode_aligned(x, fs, f0, f1, N, N_err, offset):
    """Decode bits for a signal chunk given a known alignment offset."""
    E0, E1, _ = fsk_symbol_metrics(
        x, fs, f0, f1, N, N_err, start=offset)
    bits = (E1 > E0).astype(int)
    scores = E1 - E0
    return bits, scores

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

def decode_fsk(input_filename: str, 
               f0: float = 20833.33, 
               f1: float = 22222.22, 
               p0: int = 60,
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
    p0 : int
        Number of cycles for a '0' bit (defines symbol length via p0/f0).
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
        audio, fs = sf.read(input_filename)
        if audio.ndim > 1:
            audio = audio.mean(axis=1)
        Ts = 1 / fs
        symbol_time = p0 / f0
        N = int(round(fs * symbol_time))
        N_true = fs * symbol_time
        if N < N_true:
            N += 1 
        N_err = N - N_true
        audio = (audio - np.mean(audio))
        audio, filter_delay = bandlimit(audio, f0 - 1000, f1 + 1000, fs)
        if minutes_per_segment <= 0:
            minutes_per_segment = len(audio) / fs / 60          
        seg_len = int(round(minutes_per_segment * 60 * fs))
        n = audio.shape[0]
        segments = [audio[i : i + seg_len] for i in range(0, n, seg_len)]
        segmentindex = 0
        for audio in segments:
            print(f"\nSegment {segmentindex}: ")
            print("Finding the best offset for segment...")
            best_offset, _ = find_best_offset(audio, fs, f0, f1, N, N_err)
            bits, scores = fsk_decode_aligned(audio, fs, f0, f1, N, N_err, best_offset)
            if len(bits) == 0:
                print("No bits found in this segment")
                segmentindex += 1
                continue
            idx = np.arange(len(bits))
            start_samples = best_offset + idx * N
            t = start_samples / fs
            th0, th1 = define_thresholds(scores)
            print(f"Thresholds: f0: {th0:.4f} f1: {th1:.4f}")
            mask = generate_mask(th0, th1, scores)
            DMA_reset_delay = 25  # samples
            msg_ranges = find_message_ranges(mask)
            for start_idx, end_idx in msg_ranges:
                msg_bits = bits[start_idx : end_idx + 1]
                msg_times = t[start_idx : end_idx + 1]
                if len(msg_times) == 0:
                    continue
                first_bit_time = msg_times[0]
                time_in_recording = (
                    segmentindex * 60 * minutes_per_segment
                    + first_bit_time - filter_delay * Ts - DMA_reset_delay * Ts
                )
                debug.write("Time in recording: " + seconds_to_hms(time_in_recording) + "\n")
                start_time = time_in_recording
                end_time = time_in_recording + symbol_time * len(msg_bits)
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
    parser.add_argument("--p0", type=int, default=60, help="Cycles per '0' bit (defines symbol length via p0/f0)")
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
        p0=args.p0,
        generate_debug=args.generate_debug,
        minutes_per_segment=args.minutes_per_segment,
        use_ecc=args.use_ecc,
        ecc_nsym=args.ecc_parity_bytes,
    )
