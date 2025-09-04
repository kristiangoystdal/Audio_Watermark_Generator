import numpy as np
import wave
import struct

# Parameters
fs = 1_000_000        # Sample rate (1 MHz)
f0 = 20000            # Frequency for bit 0
f1 = 21000            # Frequency for bit 1
bit_duration = 0.001  # Bit length in seconds (1 ms per bit = 1000 bps)
amplitude = 16000     # Amplitude (16-bit audio max ~32767)

def string_to_bits(s):
    return ''.join(format(ord(c), '08b') for c in s)

def generate_fsk(bits, f0, f1, fs, bit_duration, amplitude):
    samples_per_bit = int(fs * bit_duration)
    signal = []

    t = np.arange(samples_per_bit) / fs
    for bit in bits:
        f = f1 if bit == '1' else f0
        wave = amplitude * np.sin(2 * np.pi * f * t)
        signal.extend(wave)
    return np.array(signal, dtype=np.int16)

def save_wav(filename, signal, fs):
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)  # Mono
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(fs)
        wf.writeframes(signal.tobytes())

# Example usage
message = "HELLO"
bits = string_to_bits(message)
print("Message:", message)
print("Bits:", bits)

fsk_signal = generate_fsk(bits, f0, f1, fs, bit_duration, amplitude)
save_wav("fsk_hello.wav", fsk_signal, fs)

print("FSK signal saved to fsk_hello.wav")


import matplotlib.pyplot as plt

# Plot a portion of the FSK signal as a sine wave
plt.figure(figsize=(12, 4))
time_axis = np.arange(len(fsk_signal)) / fs
plt.plot(time_axis, fsk_signal)
plt.title("FSK Signal (Sine Wave Representation)")
plt.xlabel("Time (seconds)")
plt.ylabel("Amplitude")
plt.xlim(0, bit_duration * len(bits))  # Show only the duration of the message
plt.tight_layout()
plt.show()