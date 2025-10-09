import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from scipy.signal import spectrogram

wav_path = "signal_out_min_1.wav"

# Load audio
data, sample_rate = sf.read(wav_path)
if data.ndim > 1:
    data = data[:, 0]
    
average_amplitude = np.mean(np.abs(data))
print(f"Average amplitude: {average_amplitude:.4f}")
    
# Plot waveform
plt.figure(figsize=(12, 4))
plt.plot(np.arange(len(data)) / sample_rate, data)
plt.title("Waveform")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.grid()
plt.tight_layout()
plt.show()