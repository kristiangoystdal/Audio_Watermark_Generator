# 🎧 Audio Watermark Generator

**Real-time embedded audio watermark generator for STM32 microcontrollers, with supporting configuration and decoding tools.**

This project implements a complete audio watermarking system capable of generating and embedding digital identifiers into audio signals, designed specifically for resource-constrained embedded environments.

Supporting tools are also provided to:

- configure the system with user-defined variables (e.g., location, device id)
- define transmission intervals
- decode and extract the watermark from recorded audio files.

---

## 🚀 Overview

The system runs on an **STM32G4 series** microcontroller and transmits watermarks as **FSK signals** (with multiple options for frequency band).

It includes:

- A configurable **watermark bitstream generator**
- **Real-time audio modulation** using FSK (frequency-shift keying)
- **Timer-driven DAC output** with circular DMA buffering
- Integration with an external **DS3231 RTC** for timestamping
- Metadata embedding: `/STR…/DID…/LOC…/TMP…/TIM…/` format
- A **Python decoder and analysis suite** for validation and visualization in Audacity

---

## 🧠 Features

- ⚙️ **Configurable parameters** via the `user_config.h` header file
- 🕒 **RTC integration** using the DS3231 battery-backed real-time clock
- 🧩 **Preamble, silence, and polarity** control through preprocessor macros
- 🎚 **Fixed-point DSP arithmetic** for fast and efficient signal generation
- 🧮 **Frequency pair definitions** stored in `frequency_pairs.h`
- 🔄 **Non-blocking audio streaming** using DMA-driven circular buffers
- 🧰 **Python tools** for system configuration and watermark decoding

---

## 🗂 Project Structure

```
Audio_Watermark_Generator/
├── STM32/                      # Embedded firmware source for STM32 (CubeMX, HAL, DSP)
│   ├── Core/                   # Application logic (main.c, etc.)
│   ├── Drivers/                # HAL and low-level peripheral drivers
│   ├── CMakeFiles/             # Auto-generated CMake build files
│   ├── cmake/                  # CMake config and toolchain files
│   ├── .mxproject              # STM32CubeMX project definition
│   ├── .project                # Eclipse/IDE project file
│   ├── .vscode/                # VSCode project settings
│   ├── .settings/              # Additional IDE configurations
│   ├── .clangd                 # Clangd config for auto-completion and linting
│   ├── .gitignore              # Git exclusions
│   └── CMakeLists.txt          # Root build file for STM32 firmware
│
├── Python/                     # Supporting tools: GUI + decoding utilities
│   ├── gui/                    # GUI for configuration and toolchain setup
│   │   ├── scripts/            # Helper scripts for the gui
│   │   │   ├── build.py
│   │   │   ├── paths.py
│   │   │   └── user_config.py
│   │   ├── gui.py              # Configurator GUI application
│   │   ├── STM32Tool.spec      # PyInstaller spec for configurator GUI
│   │   └── icon.icns           # App icon for macOS bundling
│   │
│   ├── gui_demodulator/        # GUI decoder for analyzing recorded watermarked audio
│   │   ├── demodulator.py      # Core demodulation logic
│   │   ├── demodulator_gui.py  # GUI frontend for audio decoding
│   │   ├── Demodulator.spec    # PyInstaller spec for decoder GUI
│   │   └── icon.icns           # App icon for macOS bundling
│
├── .gitignore                  # Git exclusions (repo root)
└── README.md                   # Project overview and usage instructions

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
# Navigate to STM32 firmware directory
cd STM32

# Generate build system with CMake and Ninja
cube-cmake -B build/Debug -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Compile the firmware
cube-cmake --build build/Debug

# Flash binary to STM32 via ST-Link
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/Debug/STM_TESTING.elf verify reset exit"
```

---

## 🧰 Python Tools (GUI & CLI)

This repo includes two optional Python GUI applications:


### 🔧 1. STM32 Configurator GUI

Configure the watermark content, frequency settings, and export headers for STM32 firmware.

```bash
cd Python/gui
python3 gui.py
```


### 🔍 2. Watermark Decoder GUI

Analyze recorded `.wav` files and extract watermark messages using FSK demodulation.

```bash
cd Python/gui_demodulator
python3 demodulator_gui.py
```


### ⚙️ Dependencies

Install required Python packages:

```bash
pip install numpy scipy matplotlib soundfile
```

> For GUI apps, `tkinter` is also required (usually bundled with Python).


---

## 🧪 Current Status

## 🧪 Current Status

This repository is under **active development**.

Beta `.app` builds are available for macOS:

- 🛠 **Audio Watermark Flash Tool.app** – GUI for configuring watermark and timing parameters, and flashing the STM32 microcontroller
- 🔍 **FSK Audio Demodulator.app** – GUI for decoding and extracting watermarks from recorded audio files

👉 [Download Latest Beta Release](../../releases/latest)

---

## 👥 Contributors

- **Kristian Gøystdal** – Developer
- **Einar Bergslid** – Developer

---

## 📝 License

This project is released under the **MIT License**.  
You’re free to use, modify, and distribute it for educational and research purposes.

---

## 📡 Tags

`stm32` · `audio-watermarking` · `embedded-systems` · `signal-processing` · `dsp` · `c` · `c++` · `real-time`
