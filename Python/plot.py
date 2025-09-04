import numpy as np
import matplotlib.pyplot as plt
import math

# Constants
Fs = 1_000_000  # sample rate (1 MHz)
n21 = 48  # samples for ~21 kHz
n22 = 45  # samples for ~22 kHz
n25 = 40  # samples for ~25 kHz


# Find LCM of n21 and n22
def lcm(a, b):
    return abs(a * b) // math.gcd(a, b)


num_samples = lcm(n21, n22)

# Generate 21 kHz sine wave
sine_21k = [
    (4095.0 / 2.0) * (1.0 + np.sin(2.0 * np.pi * i / n21)) for i in range(num_samples)
]
sine_21k = np.array(sine_21k)

# Generate 22 kHz sine wave
sine_22k = [
    (4095.0 / 2.0) * (1.0 + np.sin(2.0 * np.pi * i / n22)) for i in range(num_samples)
]
sine_22k = np.array(sine_22k)

# Generate 25 kHz sine wave
sine_25k = [
    (4095.0 / 2.0) * (1.0 + np.sin(2.0 * np.pi * i / n25)) for i in range(num_samples)
]
sine_25k = np.array(sine_25k)

# Time axis (in microseconds)
t = np.arange(num_samples) / Fs * 1e6

# Plot both waves in the same figure
plt.figure(figsize=(12, 4))
plt.plot(t, sine_21k, label="21 kHz", marker="o", markersize=3)
plt.plot(t, sine_22k, label="22 kHz", marker="x", markersize=3)
plt.plot(t, sine_25k, label="25 kHz", marker="s", markersize=3)
plt.title("21kHz and 22kHz Sine Waves (sampled at 1 MHz, phase matched)")
plt.xlabel("Time (µs)")
plt.ylabel("DAC Value (0–4095)")
plt.legend(loc="upper right")
plt.grid(True)
plt.show()


# Calculate and print the actual frequencies of the two sine waves
freq_21k = Fs / n21
freq_22k = Fs / n22
freq_25k = Fs / n25
print(f"Actual frequency for n21 ({n21} samples): {freq_21k/1000:.2f} kHz")
print(f"Actual frequency for n22 ({n22} samples): {freq_22k/1000:.2f} kHz")
print(f"Actual frequency for n25 ({n25} samples): {freq_25k/1000:.2f} kHz")
