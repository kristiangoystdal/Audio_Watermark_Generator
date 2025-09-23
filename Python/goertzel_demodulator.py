import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from scipy.signal import correlate, butter, filtfilt, firwin, lfilter, windows

def goertzel(samples, sample_rate, target_frequency):
    """
    Implements the Goertzel algorithm to detect a specific frequency.

    Args:
        samples (list or np.array): The input signal samples.
        sample_rate (int): The sampling rate of the input signal.
        target_frequency (float): The frequency to detect.

    Returns:
        float: The magnitude squared of the target frequency component.
    """
    N = len(samples)
    k = int(0.5 + (N * target_frequency / sample_rate))  # Closest integer DFT bin
    
    # Handle edge cases where k is out of bounds for the target frequency
    if k < 0 or k >= N:
        return 0.0

    w = (2.0 * np.pi * k) / N
    cosine = np.cos(w)
    sine = np.sin(w)
    coeff = 2.0 * cosine

    q1 = 0.0
    q2 = 0.0

    for sample in samples:
        q0 = sample + coeff * q1 - q2
        q2 = q1
        q1 = q0

    # Calculate the magnitude squared
    magnitude_squared = q1**2 + q2**2 - q1 * q2 * coeff
    return magnitude_squared

f_0 = 20833
f_1 = 22222


filename = "signal_out_9.wav"

data, sampling_rate = sf.read(filename)

seconds_per_bit = 0.00288003
samples_per_bit = round(sampling_rate*seconds_per_bit)

def bandlimit_20_24k(x, fs, numtaps=257):
    # Linear-phase FIR band-pass around your tones
    bp = firwin(numtaps, [19000, 24000], pass_zero=False, fs=fs)
    return lfilter(bp, [1.0], x)





# Pre-filter-Parameters
fs = sampling_rate        # Sampling rate (Hz)
cutoff = 15000    # High-pass cutoff (Hz)
order = 6         # Filter order (steepness)

b, a = butter(order, cutoff / (fs/2), btype='high', analog=False)

if np.issubdtype(data.dtype, np.integer):
    data = data / np.iinfo(data.dtype).max
data = data.astype(np.float64)

data_centered = (data - np.mean(data))*20

data_filtered = filtfilt(b, a, data_centered)

plt.plot(data_filtered)
plt.show()

def goertzel_mag2(x, fs, f0):
    N = len(x)
    k = int(round(f0 * N / fs))          # nearest DFT bin
    w = 2*np.pi*k/N
    coeff = 2*np.cos(w)
    s0 = s1 = s2 = 0.0
    for n in range(N):
        s0 = x[n] + coeff*s1 - s2
        s2, s1 = s1, s0
    # standard Goertzel tail
    real = s1 - s2*np.cos(w)
    imag = s2*np.sin(w)
    return real*real + imag*imag 

goertzel_signal = goertzel_mag2(data_filtered,fs, f_0)

plt.plot(goertzel_signal)
plt.show()

message = ""

for byte_index in range(15):
    bitstring=""
    for bit_index in range(8):
        start_sample = (byte_index*8 + bit_index)*samples_per_bit
        end_sample = start_sample+samples_per_bit
        g_0 = goertzel(data_centered[start_sample:end_sample],sampling_rate,f_0)
        g_1 = goertzel(data_centered[start_sample:end_sample],sampling_rate,f_1)
        bitstring += "0" if g_0 > g_1 else "1"
    print(bitstring)
    message += chr(int(bitstring,2))

print()
print(message)
print()