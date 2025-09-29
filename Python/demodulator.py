import numpy as np
from scipy.signal import firwin, lfilter, windows
import soundfile as sf
import matplotlib.pyplot as plt


def bandlimit(signal, f_lower, f_upper, fs, numtaps=257):
    """Band-pass filter the signal between f_lower and f_upper."""
    bp = firwin(numtaps, [f_lower, f_upper], pass_zero=False, fs=fs)
    group_delay = (numtaps - 1) // 2
    return lfilter(bp, [1.0], signal), group_delay


def fsk_symbol_metrics(x, fs, f0, f1, N, start, stepN, apply_window=True):
    """Compute E0, E1, score per symbol for windows starting at 'start', stepping by N."""
    W = windows.hann(N, sym=False) if apply_window else 1.0
    n = np.arange(N)
    e0 = np.exp(-1j * 2 * np.pi * f0 * n / fs)
    e1 = np.exp(-1j * 2 * np.pi * f1 * n / fs)
    E0, E1, S = [], [], []
    for s in range(start, len(x) - N + 1, stepN):
        seg = x[s : s + N] * W
        c0 = np.vdot(e0, seg)
        c1 = np.vdot(e1, seg)
        denom = np.sum(W**2) if apply_window else N
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
            x, fs, f0, f1, N, start=r, stepN=N, apply_window=True
        )
        if len(S) == 0:
            continue
        val = np.mean(S)
        if val > best_val:
            best_val, best_r = val, r
    return best_r, best_val


def fsk_decode_aligned(x, fs, f0, f1, N, offset):
    E0, E1, _ = fsk_symbol_metrics(
        x, fs, f0, f1, N, start=offset, stepN=N, apply_window=True
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


def print_message(message):
    print("Decoded Message: ")
    print(len(message), " characters")
    messages_lines = message.split("/")
    for line in messages_lines:
        if line[0:3] == "STR":
            print("Message: ", line[3:])
        elif line[0:3] == "LOC":
            print("Location: ", line[3:])
        elif line[0:3] == "DID":
            print("Device ID: ", line[3:])
        elif line[0:3] == "TMP":
            print("Temperature: ", line[3:])
        elif line[0:3] == "TIM":
            print(
                f"Timestamp: {line[3:5]}:{line[5:7]}:{line[7:9]} on {days_of_week[int(line[9:11]) - 1]} {line[11:13]}/{line[13:15]}/{line[15:19]}"
            )


if __name__ == "__main__":

    f0, f1 = 20833.33, 22222.22  # Hz
    p0, p1 = 60, 64  # number of cycles per symbol
    filename = "signal_out_min_1_test.wav"  # Input WAV file

    audio, fs = sf.read(filename)
    Ts = 1 / fs

    sym_time = p0 / f0

    N = int(round(fs * sym_time))

    audio, fs = sf.read(filename)

    audio = (audio - np.mean(audio)) * 25  # Scale and center

    audio, filter_delay = bandlimit(audio, f0 - 1000, f1 + 1000, fs)

    mins_per_segment = 1  # int(input("Time between messages [minutes]: "))

    seg_len = int(round(mins_per_segment * 60 * fs))  # samples per segment
    n = audio.shape[0]
    segments = [audio[i : i + seg_len] for i in range(0, n, seg_len)]

    segmentindex = 0

    for audio in segments:
        print()
        print(f"Segment {segmentindex}: ")
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

        threshold = define_threshold(scores)
        print("Threshold: ", threshold)
        mask = np.abs(scores) > threshold

        masked_bits = bits[mask]

        masked_time = t[mask]

        bitstring = ""
        time_index = 0
        index = 0
        message = ""
        prev_bit_time = -1

        for bit in masked_bits:
            if (masked_time[index] - prev_bit_time) > 3 * sym_time:
                if len(message) > 0:
                    message += "\n"
                bitstring = ""
                index = 0
                message += f"Time in recording: {segmentindex*60*mins_per_segment + masked_time[time_index] + filter_delay*Ts:.3f}s\nMessage: "
            bitstring += str(bit)
            # print(str(bit),end="")
            if ((index + 1) % 8) == 0:
                message += chr(int(bitstring, 2))
                if len(message) == 3:
                    if message != "STR":
                        print(
                            "Warning: Message might be corrupted (missing 'STR' preamble)"
                        )
                bitstring = ""
            prev_bit_time = masked_time[index]
            index += 1
            time_index += 1

        print_message(message)
        segmentindex += 1
