import numpy as np
from scipy.signal import firwin, lfilter, windows
import soundfile as sf
import matplotlib.pyplot as plt

print()

def bandlimit_20_24k(x, fs, numtaps=257):
    # Linear-phase FIR band-pass around your tones
    bp = firwin(numtaps, [19000, 23800], pass_zero=False, fs=fs)
    return lfilter(bp, [1.0], x)

def fsk_symbol_metrics(x, fs, f0, f1, N, start, stepN, apply_window=True):
    """Compute E0, E1, score per symbol for windows starting at 'start', stepping by N."""
    W = windows.hann(N, sym=False) if apply_window else 1.0
    n = np.arange(N)
    e0 = np.exp(-1j * 2*np.pi * f0 * n / fs)
    e1 = np.exp(-1j * 2*np.pi * f1 * n / fs)
    E0, E1, S = [], [], []
    for s in range(start, len(x) - N + 1, stepN):
        seg = x[s:s+N] * W
        c0 = np.vdot(e0, seg)
        c1 = np.vdot(e1, seg)
        denom = np.sum(W**2) if apply_window else N
        e0v = (np.abs(c0)**2) / denom
        e1v = (np.abs(c1)**2) / denom
        E0.append(e0v); E1.append(e1v); S.append(abs(e1v - e0v))
    return np.array(E0), np.array(E1), np.array(S)

def find_best_offset(x, fs, f0, f1, N):
    """Brute-force modulo-N offset search: pick r maximizing mean |E1-E0|."""
    best_r, best_val = 0, -np.inf
    # To keep it fast on long files, subsample offsets (optional):
    # offsets = range(0, N, 1)
    for r in range(N):
        _, _, S = fsk_symbol_metrics(x, fs, f0, f1, N, start=r, stepN=N, apply_window=True)
        if len(S) == 0: continue
        val = np.mean(S)
        if val > best_val:
            best_val, best_r = val, r
    return best_r, best_val

def fsk_decode_aligned(x, fs, f0, f1, N, offset):
    E0, E1, _ = fsk_symbol_metrics(x, fs, f0, f1, N, start=offset, stepN=N, apply_window=True)
    bits = (E1 > E0).astype(int)
    scores = E1 - E0
    return bits, scores, E0, E1


if __name__ == "__main__":

    f0, f1 = 20833.33, 22222.22
    filename = "signal_out_min_1_long.wav"

    audio, fs = sf.read(filename)

    sym_time = 60.0 / f0         
    
    N = int(round(fs * sym_time))

    audio, fs = sf.read(filename)

    audio = (audio - np.mean(audio))*25 # Scale and center

    audio = bandlimit_20_24k(audio[500:], fs)

    seg_mins = int(input("Time between messages [minutes]: "))

    seg_len = int(round(seg_mins*60 * fs))   # samples per segment
    n = audio.shape[0]
    segments = [audio[i:i+seg_len] for i in range(0, n, seg_len)]

    segmentindex = 0

    for audio in segments:
        print()
        print(f"Segment {segmentindex}: ")

        print("Finding the best offset for segment...")
        best_offset, _ = find_best_offset(audio[:(48000*60)], fs, f0, f1, N)

        print("Decoding...")
        bits, scores, E0, E1 = fsk_decode_aligned(audio, fs, f0, f1, N, best_offset)

        idx = np.arange(len(bits))
        start_samples = best_offset + idx * N
        center_samples = start_samples + N // 2
        t = center_samples / fs

        threshold = 0.1
        mask = np.abs(scores) > threshold

        time = t[mask]

        bitstring = ""
        time_index = 0
        index = 0
        message = ""
        prev_bit_time = 0

        for bit in bits[mask]:
            if((time[index] - prev_bit_time) > 0.01):
                bitstring = ""
                index = 0
                message += f"Time in recording: {segmentindex*60*seg_mins + time[time_index]:.2f} Message: "
            bitstring += str(bit)
            #print(str(bit),end="")
            if ((index+1) % 8) == 0:
                message += chr(int(bitstring,2))
                bitstring = ""
            prev_bit_time = time[index]
            index += 1
            time_index += 1

        print(message)
        segmentindex += 1

        plt.stem(time, bits[mask])
