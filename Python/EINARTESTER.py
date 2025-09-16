import numpy as np
import wave
import matplotlib.pyplot as plt
from scipy.io import wavfile
import soundfile as sf

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

filename = "signal_out_6.wav"
data, sampling_rate = sf.read(filename)

seconds_per_bit = 0.00288003
samples_per_bit = round(sampling_rate*seconds_per_bit)

if np.issubdtype(data.dtype, np.integer):
    data = data / np.iinfo(data.dtype).max
data = data.astype(np.float64)

data_cropped = data[11:20000]*20
data_centered = data_cropped - np.mean(data_cropped)

message = ""

for byte_index in range(15):
    bitstring=""
    for bit_index in range(8):
        start_sample = (byte_index*8 + bit_index)*samples_per_bit
        end_sample = start_sample+samples_per_bit
        g_0 = goertzel(data_centered[start_sample:end_sample],sampling_rate,f_0)
        g_1 = goertzel(data_centered[start_sample:end_sample],sampling_rate,f_1)
        bitstring += "0" if g_0 > g_1 else "1"
    message += chr(int(bitstring,2))

print()
print(message)
print()