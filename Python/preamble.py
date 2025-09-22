import numpy as np
import matplotlib.pyplot as plt
from scipy.io.wavfile import write

fs = 48000
f0, f1 = 20833.0, 22222.0   # Hz
p0, p1 = 60, 64             # periods per bit
bitseq = "0010110111010100"

def gen_bit_until_pos_zero(f, periods, fs):
    """
    Generate a sine that contains at least `periods` full periods of frequency f,
    then continues until the first positive-going zero crossing, and returns:
      - samples (1D float array)
      - the index where the positive-going zero crossing occurs (last sample included is just BEFORE the crossing)
    The boundary will be exactly between the last sample of this array and the next sample.
    """
    # Target phase advance
    target_phase = 2 * np.pi * periods
    # Per-sample phase increment
    dphi = 2 * np.pi * f / fs

    # Grow samples until we reach target phase, then continue until the first positive-going zero crossing
    phases = []
    phi = 0.0
    xs = []

    # Generate at least the target number of periods (phase >= target_phase)
    while phi < target_phase:
        xs.append(np.sin(phi))
        phi += dphi

    # Now continue sample by sample until the first positive-going zero crossing
    # We need a sign change from negative to positive.
    # Ensure we have at least one previous sample to check sign.
    prev = xs[-1]
    while True:
        xs.append(np.sin(phi))
        cur = xs[-1]
        if prev <= 0.0 and cur > 0.0:
            # The positive zero crossing is between the last two samples (prev -> cur).
            # We stop BEFORE the crossing, so this segment ends at index len(xs)-2.
            # Trim off the last sample (which is already positive), so boundary lies between samples.
            xs.pop()  # remove 'cur'; boundary is now right after the last element
            break
        prev = cur
        phi += dphi

    return np.array(xs, dtype=np.float64)

# Build the full waveform
segments = []
for b in bitseq:
    if b == '0':
        seg = gen_bit_until_pos_zero(f0, p0, fs)
    else:
        seg = gen_bit_until_pos_zero(f1, p1, fs)
    segments.append(seg)

signal = np.concatenate(segments)

# Save to WAV (16-bit PCM)
write("fsk_sequence_zero_cross.wav", fs, (signal * 32767).astype(np.int16))
print(f"Total samples: {len(signal)}")
