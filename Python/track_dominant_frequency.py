import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt

audio_file_path = "sweep_test.wav"
frame_ms = 40
hop_ms = 10
minimum_frequency_hz = 20
maximum_frequency_hz = 20000

sample_rate, data = wavfile.read(audio_file_path)

if np.issubdtype(data.dtype, np.integer):
    data = data / np.iinfo(data.dtype).max

frame_length = int(round(frame_ms* 0.001 * sample_rate))
hop_length = int(round(hop_ms * 0.001 * sample_rate))

hanning_window = np.hanning(frame_length)

n_fft = 1 << (frame_length - 1).bit_length()

frequencies = np.fft.rfftfreq(n_fft, d=1.0/sample_rate)

dominant_frequencies = []
times = []

for start in range(0, len(data)-frame_length + 1, hop_length):
    frame = data[start:start+frame_length] * hanning_window
    fft = np.fft.rfft(frame, n=n_fft)
    magnitude = np.abs(fft)

    dominant_frequency_index = np.argmax(magnitude)
    dominant_frequency 