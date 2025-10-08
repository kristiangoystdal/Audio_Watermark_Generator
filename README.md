# 🎧 Audio Watermark Generator

**Real-time embedded audio watermark generator and decoder built for STM32 microcontrollers.**  
This project implements a complete audio watermarking system capable of generating, embedding, and decoding digital identifiers in audio signals — designed for resource-constrained embedded environments.

---

## 🚀 Overview

The system runs on an **STM32G4 series** microcontroller and transmits watermarks as **ultrasonic FSK signals** (typically around 21–22 kHz).  
It includes:

- A configurable **watermark bitstream generator**
- **Real-time audio modulation** using FSK (frequency-shift keying)
- **Timer-driven DAC output** with circular DMA buffering
- Integration with an external **DS3231 RTC** for timestamping
- Metadata embedding: `/STR…/DID…/LOC…/TMP…/TIM…/` format
- A **Python decoder and analysis suite** for validation and visualization

---

## 🧠 Features

- ⚙️ **Configurable parameters** via user header file (`user_config.h`)
- 🕒 **RTC integration** (DS3231) with battery-backed clock
- 🧩 **Preamble, silence, and polarity** control via macros
- 🎚 **Fixed-point DSP arithmetic** for efficient modulation
- 🧮 **FSK frequency pair definitions** stored in `frequency_pairs.h`
- 🔄 **DMA-driven continuous audio output** (no blocking delays)
- 🧰 **Python tools** for decoding, spectrogram visualization, and testing

---

## 🗂 Project Structure

```
Audio_Watermark_Generator/
├── STM32/                  # Embedded source (C, CubeMX, HAL)
│   ├── Core/
│   ├── Drivers/
│   ├── CMakeLists.txt
│   └── STM32G431XX_FLASH.ld
├── Python/                 # Decoder, analysis, and utilities
│   ├── demodulator.py
│   ├── decoder.py
│   └── utils/
├── .github/workflows/      # CI build workflows
└── README.md
```

---

## 🧱 Build Instructions

### 🔹 Requirements
- **STM32CubeIDE** or **CubeMX + CMake + Ninja**
- **GNU Arm Embedded Toolchain** (13.3.1+st.9 or newer)
- **OpenOCD / ST-Link V3** for flashing
- macOS, Linux, or Windows host

### 🔹 Build and Flash
```bash
cd STM32
cube-cmake -B build/Debug -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug
cube-cmake --build build/Debug
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build/Debug/STM_TESTING.elf verify reset exit"
```

---

## 🧩 Python Decoder (Optional)

A companion decoder written in Python can analyze captured watermarked audio files.

### Run the decoder:
```bash
cd Python
python3 decoder.py input.wav
```

This script demodulates and extracts the embedded message from the audio signal.

---

## 🧪 Current Status

This repository is under **active development**.  
A macOS `.app` build (`STM32Tool.app`) is available as a **beta release** for easier build and flash automation.  
👉 [Download Beta Release](../../releases/latest)

---

## 👥 Contributors

- **Kristian Gøystdal** – Developer  
- **Einar Bergslid** – Contributor  

---

## 📝 License

This project is released under the **MIT License**.  
You’re free to use, modify, and distribute it for educational and research purposes.

---

## 📡 Tags

`stm32` · `audio-watermarking` · `embedded-systems` · `signal-processing` · `dsp` · `c` · `c++` · `real-time`
