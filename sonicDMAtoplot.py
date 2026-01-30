import numpy as np
import matplotlib.pyplot as plt

FS = 48000  # sample rate

x = np.loadtxt("C:/Users/am200/Downloads/sonicDMAreadings.txt", dtype=np.int32)
s24 = (x >> 4) & 0xFFFFFF # extract audio bits
s24 = s24.astype(np.int32) # sign-extend 24-bit
s24[s24 & 0x800000 != 0] -= 1 << 24
y = s24 / float(1 << 23) # normalize to [-1, 1]
# Frequency domain math
NFFT = 4095
window = np.hanning(NFFT)
yf = y[:NFFT] * window
Y = np.fft.rfft(yf)
freq = np.fft.rfftfreq(NFFT, d=1/FS)
magnitude = 20 * np.log10(np.abs(Y) + 1e-12)
# Plotting
fig, axs = plt.subplots(2, 1, figsize=(12, 7))
axs[0].plot(y[:2000]) # Time domain
axs[0].set_title("Audio samples")
axs[0].set_xlabel("Sample index")
axs[0].set_ylabel("Amplitude")
axs[0].grid(True)
axs[1].plot(freq, magnitude) # Frequency domain
axs[1].set_title("FFT of captured audio")
axs[1].set_xlabel("Frequency (Hz)")
axs[1].set_ylabel("Magnitude (dB)")
axs[1].set_xlim(0, 8000)
axs[1].grid(True)
plt.tight_layout()
plt.show()
