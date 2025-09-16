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

filename = "signal_out_6.wav"

data, sampling_rate = sf.read(filename)



if np.issubdtype(data.dtype, np.integer):
    data = data / np.iinfo(data.dtype).max
data = data.astype(np.float64)

data_cropped = data[11:25957]*20

data_centered = data_cropped - np.mean(data_cropped)

for symbol_index in range(24):
    start_sample = symbol_index*140
    end_sample = (symbol_index+1)*140-1


    #print(f"Symbol {symbol_index} going from sample {start_sample} to {end_sample}")


    mag_0 = goertzel(data_centered[start_sample:end_sample],sampling_rate,20833)
    mag_1 = goertzel(data_centered[start_sample:end_sample],sampling_rate,22222)
    if(mag_0 > mag_1):
        print(0, end="")
    else:
        print(1, end="")
    


freq = []
g = []

for tf in range(20000,25000,1):
    mag_sq = goertzel(data_centered,sampling_rate,tf)
    freq.append(tf)
    g.append(mag_sq)

