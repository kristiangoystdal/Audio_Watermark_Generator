# Audio Watermark Generator

Real-time embedded audio watermark generator for STM32 microcontrollers, with supporting hardware design, configuration tools, and decoding software.

This project implements a complete audio watermarking system that embeds digital identifiers into audio signals in real-time, designed for resource-constrained embedded environments. Supporting tools allow you to configure the device, define transmission intervals, and decode watermarks from recorded audio.

---

## Overview

The system runs on an **STM32G431** microcontroller and transmits watermarks as **FSK signals** across a configurable frequency band. It can output via a speaker or a **CC1101 RF transceiver**, and includes battery-backed timestamping via a **DS3231 RTC**.

The watermark payload follows a structured metadata format: `/STR…/DID…/LOC…/TMP…/TIM…/`

---

## Repository Structure

```
Audio_Watermark_Generator/
├── Firmware/               # STM32G4 embedded firmware (CubeMX + CMake + HAL)
│   ├── Core/               # Application logic, drivers, and headers
│   ├── Drivers/            # HAL and CMSIS drivers
│   ├── USB_Device/         # USB CDC interface
│   ├── cmake/              # Toolchain and build config
│   └── CMakeLists.txt
│
├── Hardware/               # KiCad PCB design files
│   ├── *.kicad_sch         # Schematic sheets (MCU, RTC, mixer, speaker, transceiver, battery)
│   ├── *.kicad_pcb         # PCB layout
│   ├── Gerber/             # Fabrication outputs
│   └── JCLPCB/             # BOM and CPL for assembly
│
├── Software/               # Python GUI tools
│   ├── gui/                # STM32 configurator and flash tool
│   └── gui_demodulator/    # FSK watermark decoder and visualizer
│
├── Testing/                # Automated test scripts and results
│   ├── scripts/            # Test scripts (drift, range, localization, timing, etc.)
│   ├── test_files/         # Input audio and measurement files
│   └── results/            # Test output plots and reports
│
└── README.md
```

---

## Firmware Features

- **FSK watermark generation** with configurable frequency pairs (`frequency_config.h`)
- **Fixed-point DSP** for efficient real-time signal synthesis
- **DMA-driven circular DAC output** for non-blocking audio streaming
- **Reed-Solomon FEC** for error-resilient watermark encoding
- **DS3231 RTC** integration for battery-backed timestamping
- **CC1101 RF transceiver** support for wireless transmission
- **USB CDC** interface for serial communication with the host
- **Wakeup / sleep** power management with configurable intervals
- **LED feedback** and structured error codes
- Configurable parameters via `user_config.h`

---

## Hardware

The custom PCB is designed in **KiCad** and includes:

- STM32G431 microcontroller
- DS3231 RTC with battery backup
- CC1101 RF transceiver
- Audio mixer and speaker driver
- USB connector
- Power management (battery input)

Gerber files and JLCPCB assembly files (BOM + CPL) are included in `Hardware/Gerber/` and `Hardware/JCLPCB/`.

---

## Software Tools

### Configurator GUI

Configure watermark content (device ID, location, timing intervals), generate `user_config.h`, and flash the STM32 firmware — all from a single GUI.

```bash
cd Software/gui
python3 gui.py
```

### Watermark Decoder GUI

Load a recorded `.wav` file and extract the embedded watermark using FSK demodulation and Reed-Solomon decoding, with visualization of the demodulated spectrum.

```bash
cd Software/gui_demodulator
python3 demodulator_gui.py
```

### Dependencies

```bash
pip install numpy scipy matplotlib soundfile
```

> `tkinter` is required for the GUIs and is typically bundled with Python.

---

## Build & Flash (Firmware)

### Requirements

- STM32CubeIDE or **CubeMX + CMake + Ninja**
- GNU Arm Embedded Toolchain (13.3.1 or newer)
- OpenOCD / ST-Link V3

### Build

```bash
cd Firmware

# Configure
cmake -B build/Debug -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Compile
cmake --build build/Debug
```

### Flash

```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/Debug/Audio_Watermark_Generator.elf verify reset exit"
```

---

## Testing

The `Testing/` directory contains scripts for characterizing system performance:

| Script | Description |
|---|---|
| `autotest.py` | Automated demodulator correctness tests |
| `drift.py` | Measures frequency drift over time |
| `offset_calculation.py` | Computes timing offset between devices |
| `localize.py` | Acoustic localization from multi-receiver recordings |
| `audio_passthrough.py` | Validates audio signal chain |
| `spec_leak.py` | Spectral purity / leakage analysis |
| `timing.py` | Transmission timing validation |

```bash
cd Testing
python3 run_all_tests.py
```

---

## Current Status

This repository is under **active development**.

Beta `.app` builds are available for macOS:

- **Audio Watermark Flash Tool** — configure watermark parameters and flash the STM32
- **FSK Audio Demodulator** — decode and extract watermarks from recorded audio

[Download the latest beta release](../../releases/latest)

---

## Contributors

- **Kristian Gøystdal** — Developer
- **Einar Bergslid** — Developer

---

## License

Released under the **MIT License**. Free to use, modify, and distribute for educational and research purposes.

---

`stm32` · `audio-watermarking` · `embedded-systems` · `fsk` · `dsp` · `reed-solomon` · `kicad` · `python`
