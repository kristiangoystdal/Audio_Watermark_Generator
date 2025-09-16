import numpy as np
import matplotlib.pyplot as plt

def make_preamble(fs, f0, f1):
    periods1 = 64
    N1 = int(round(periods1 * fs / f1))
    t1 = np.arange(N1) / fs
    signal1 = np.sin(2 * np.pi * f1 * t1)
    f0 = 20833
    periods2 = 60
    N2 = int(round(periods2 * fs / f0))
    t2 = np.arange(N2) / fs
    signal2 = np.sin(2 * np.pi * f0 * t2)
    pattern = np.concatenate([signal1, signal2])
    return np.tile(pattern, 8)

preamble = make_preamble(48000, 20833, 22222)

# Plot a portion to inspect
plt.plot(preamble)  # Plot just one pattern for clarity
plt.xlabel("Sample")
plt.ylabel("Amplitude")
plt.title("Pattern: 64 periods of 22 222 Hz + 60 periods of 20 833 Hz (repeated 8x)")
plt.show()
