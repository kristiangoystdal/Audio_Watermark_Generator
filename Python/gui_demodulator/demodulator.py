import numpy as np
from scipy.signal import firwin, lfilter, windows
import soundfile as sf
import os
import datetime
import matplotlib.pyplot as plt


def bandlimit(signal, f_lower, f_upper, fs, numtaps=257):
    """Band-pass filter the signal between f_lower and f_upper."""
    bp = firwin(numtaps, [f_lower, f_upper], pass_zero=False, fs=fs)
    group_delay = (numtaps - 1) // 2
    return lfilter(bp, [1.0], signal), group_delay


def fsk_symbol_metrics(x, fs, f0, f1, N, start, stepN):
    """Compute E0, E1, score per symbol for windows starting at 'start', stepping by N."""
    W = windows.hann(N, sym=False)
    n = np.arange(N)
    e0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    e1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)
    E0, E1, S = [], [], []
    for s in range(start, len(x) - N + 1, stepN):
        seg = x[s : s + N] * W
        c0 = np.vdot(e0, seg)
        c1 = np.vdot(e1, seg)
        denom = np.sum(W**2)
        e0v = (np.abs(c0) ** 2) / denom
        e1v = (np.abs(c1) ** 2) / denom
        E0.append(e0v)
        E1.append(e1v)
        S.append(abs(e1v - e0v))
    return np.array(E0), np.array(E1), np.array(S)


def find_best_offset(x, fs, f0, f1, N, stepsize=10):
    """Brute-force offset search: pick r maximizing mean |E1-E0|."""
    best_r, best_val = 0, -np.inf
    for r in range(0, N, stepsize):
        _, _, S = fsk_symbol_metrics(
            x, fs, f0, f1, N, start=r, stepN=N
        )
        if len(S) == 0:
            continue
        val = np.mean(S)
        if val > best_val:
            best_val, best_r = val, r
    return best_r, best_val


def fsk_decode_aligned(x, fs, f0, f1, N, offset):
    E0, E1, _ = fsk_symbol_metrics(
        x, fs, f0, f1, N, start=offset, stepN=N
    )
    bits = (E1 > E0).astype(int)
    scores = E1 - E0
    return bits, scores, E0, E1


def define_threshold(scores):
    if len(scores) == 0:
        return 0.0
    else:
        return np.max(np.abs(scores)) * 0.2


days_of_week = [
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday",
]


def print_message(message, start_time, end_time, output, labels):
    labels.write(f"{start_time:.6f}\t{end_time:.6f}\t")

    if len(message) == 0:
        print("No message decoded")
        output.write("No message decoded\n")
        labels.write("Error: 0 length message\n")
        return
    elif message[0] != "/":
        print("Error: Message might be corrupted (missing '/' preamble)")
        output.write("Error: Message might be corrupted (missing '/' preamble)\n")
        labels.write("Error: missing '/' preamble\n")
        return
    elif message[-1] != "/":
        print("Error: Message might be corrupted (missing '/' termination)")
        output.write("Error: Message might be corrupted (missing '/' termination)\n")
        labels.write("Error: missing '/' termination\n")
        return

    print("Decoded Message: ", message)
    output.write("Decoded Message: " + message + "\n")
    print(len(message), " characters")
    output.write(f"{len(message)} characters\n")
    messages_lines = message.split("/")
    for line in messages_lines:
        if line[0:3] == "STR":
            print("Message: ", line[3:])
            output.write("Message: " + line[3:] + "\n")
            labels.write("Message: " + line[3:] + " | ")
        elif line[0:3] == "LOC":
            print("Location: ", line[3:])
            output.write("Location: " + line[3:] + "\n")
            labels.write("Location: " + line[3:] + " | ")
        elif line[0:3] == "DID":
            print("Device ID: ", line[3:])
            output.write("Device ID: " + line[3:] + "\n")
            labels.write("Device ID: " + line[3:] + " | ")
        elif line[0:3] == "TMP":
            print("Temperature: ", line[3:])
            output.write("Temperature: " + line[3:] + "°C\n")
            labels.write("Temperature: " + line[3:] + "°C | ")
        elif line[0:3] == "TIM":
            print(f"Timestamp: {line[3:5]}:{line[5:7]}:{line[7:9]} on {days_of_week[int(line[9:11]) - 1]} {line[11:13]}/{line[13:15]}/{line[15:19]}")
            output.write(f"Timestamp: {line[3:5]}:{line[5:7]}:{line[7:9]} on {days_of_week[int(line[9:11]) - 1]} {line[11:13]}/{line[13:15]}/{line[15:19]}\n")
            labels.write("Time: " + f"{line[3:5]}:{line[5:7]}:{line[7:9]} on {days_of_week[int(line[9:11]) - 1]} {line[11:13]}/{line[13:15]}/{line[15:19]}" + "\n")

    output.write("\n")

def seconds_to_hms(total_seconds):
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
    
def split_messages_by_gap(bits, times, sym_time):
    """
    Split a bit stream into messages wherever the time gap between consecutive
    symbols exceeds 3 × sym_time.

    Returns: list of (bits_segment, times_segment)
    """
    if len(bits) == 0:
        return []

    gaps = np.diff(times, prepend=times[0])
    split_idx = np.where(gaps > 3 * sym_time)[0]  # start of a new message
    bit_segments = np.split(bits, split_idx)
    time_segments = np.split(times, split_idx)
    return [(b, t) for b, t in zip(bit_segments, time_segments) if len(b) > 0]



def decode_fsk(input_filename: str, 
               f0: float = 20833.33, 
               f1: float = 22222.22, 
               p0: int = 60,
               generate_readable: bool = False,
               minutes_per_segment: int = -1):

    
    base, _ = os.path.splitext(input_filename)

    # Only create a readable file if asked for it
    if generate_readable:
        output_filename = base + "_readable.txt"
    else:
        output_filename = os.devnull  # discard writes, no file created
    
    labels_filename = base + ".txt"

    with open(labels_filename, "w") as labels, open(output_filename, "w") as output:

        audio, fs = sf.read(input_filename)

        if audio.ndim > 1:
            audio = audio.mean(axis=1)

        Ts = 1 / fs

        sym_time = p0 / f0

        N = int(round(fs * sym_time))

        audio = (audio - np.mean(audio)) * 25  # Scale and center

        audio, filter_delay = bandlimit(audio, f0 - 1000, f1 + 1000, fs)

        if minutes_per_segment <= 0:
            minutes_per_segment = len(audio) / fs / 60  # full length in minutes
            
        seg_len = int(round(minutes_per_segment * 60 * fs))  # samples per segment
        n = audio.shape[0]
        segments = [audio[i : i + seg_len] for i in range(0, n, seg_len)]

        segmentindex = 0

        for audio in segments:
            print(f"\nSegment {segmentindex}: ")
            print("Finding the best offset for segment...")
            best_offset, _ = find_best_offset(audio, fs, f0, f1, N)
            bits, scores, E0, E1 = fsk_decode_aligned(audio, fs, f0, f1, N, best_offset)

            if len(bits) == 0:
                print("No bits found in this segment")
                segmentindex += 1
                continue

            idx = np.arange(len(bits))
            start_samples = best_offset + idx * N
            t = start_samples / fs

            threshold = max(0.5, define_threshold(scores))
            print("Threshold: ", threshold)
            mask = np.abs(scores) > threshold

            masked_bits = bits[mask]

            masked_time = t[mask]

            DMA_reset_delay = 25  # samples

            # Split into per-message bitstreams in advance
            msg_segments = split_messages_by_gap(masked_bits, masked_time, sym_time)

            message = ""
            for msg_bits, msg_times in msg_segments:
                # Compute start time for this message
                first_bit_time = msg_times[0]
                time_in_recording = (
                    segmentindex * 60 * minutes_per_segment
                    + first_bit_time - filter_delay * Ts - DMA_reset_delay * Ts
                )

                print("Time in recording:", seconds_to_hms(time_in_recording))
                output.write("Time in recording: " + seconds_to_hms(time_in_recording) + "\n")

                start_time = time_in_recording
                end_time = time_in_recording + sym_time * len(msg_bits)

                # Convert bits to ASCII text
                bitstring = ""
                for i, bit in enumerate(msg_bits, start=1):
                    bitstring += str(bit)
                    if (i % 8) == 0:
                        message += chr(int(bitstring, 2))
                        bitstring = ""
                print_message(message, start_time, end_time, output, labels)

                message = ""  # reset before the next message
            segmentindex += 1




if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="FSK demodulator")
    parser.add_argument("-i", "--input", required=True, help="Input WAV file")
    parser.add_argument("--f0", type=float, default=20833.33, help="Frequency for bit 0 [Hz]")
    parser.add_argument("--f1", type=float, default=22222.22, help="Frequency for bit 1 [Hz]")
    parser.add_argument("--p0", type=int, default=60, help="Cycles per '0' bit (defines symbol length via p0/f0)")
    parser.add_argument("--generate-readable", action="store_true",
                        help="Generate human-readable output file (.txt)")
    parser.add_argument("--minutes-per-segment", type=int, default=-1,
                        help="Length of each segment in minutes (-1 = process full file)")

    args = parser.parse_args()

    decode_fsk(
        input_filename=args.input,
        f0=args.f0,
        f1=args.f1,
        p0=args.p0,
        generate_readable=args.generate_readable,
        minutes_per_segment=args.minutes_per_segment,
    )
