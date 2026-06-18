# Acoustic Metadata Multiplexer

A low-cost, low-power system for embedding time-synchronized FSK metadata signals into audio recordings made by Autonomous Recording Units (ARUs). Designed for bioacoustic field deployments where cross-unit time synchronization and metadata recovery from audio alone are required.

The system encodes a structured metadata payload as a BFSK signal mixed into the audio recorded by any ARU model — either acoustically via a speaker, or electrically via a 3.5 mm cabled pass-through. A desktop demodulator extracts the metadata from recorded `.wav` files post-deployment.

---

## How It Works

The system operates in a **star topology**: one unit acts as a base station and the others are receivers, one per ARU.

1. **Base station** wakes on a fixed schedule, generates a common payload (message ID, timestamp, user string), and broadcasts it over RF at 433 MHz using the CC1101 transceiver.
2. **Receiver units** listen on the CC1101 in continuous RX mode. On receiving a packet, each receiver generates a BFSK-modulated audio signal encoding the shared payload combined with its own local data (device ID, GPS coordinates, live temperature from DS3231).
3. The FSK signal is mixed into the audio captured by the ARU — either by playing it through a small speaker near the ARU microphone, or by mixing it into a microphone signal over a cabled 3.5 mm connection.
4. Since all receivers embed at the same time (triggered by the same RF packet), the embedded signal appears at the same absolute moment across all ARU recordings — enabling post-hoc time alignment via the message ID.
5. A recorded `.wav` containing the embedded signal can be decoded using the desktop demodulator to recover both the metadata and the synchronization reference point.

**Standalone mode** skips RF communication entirely: the unit generates and embeds its own payload on a fixed schedule, using only locally configured and measured data.

The metadata payload format is:

```
/MID<id>/TIM<timestamp>/STR<string>/DID<device_id>/LOC<lat,lon>/TMP<temperature>/
```

Each field is optional and individually togglable via `user_config.h` or the GUI. `MID` (message ID) is the synchronization anchor — recordings are aligned by matching identical message IDs across units.

---

## Repository Structure

```
Audio_Watermark_Generator/
├── Firmware/                   # STM32G431 embedded firmware
│   ├── Core/
│   │   ├── Inc/                # Headers (drivers, config, DSP)
│   │   └── Src/                # Source (main loop, peripherals, FEC)
│   ├── Drivers/                # HAL, CMSIS, and DSP libraries
│   ├── Middlewares/            # USB device library
│   ├── USB_Device/             # USB CDC interface
│   ├── cmake/                  # CMake toolchain config
│   ├── CMakeLists.txt
│   └── CMakePresets.json
│
├── Hardware/                   # KiCad PCB design
│   ├── *.kicad_sch             # Schematics (MCU, RTC, mixer, speaker, transceiver, battery)
│   ├── *.kicad_pcb             # PCB layout
│   ├── Components/             # Custom footprints, symbols, and 3D models
│   ├── Gerber/                 # Fabrication files
│   └── JCLPCB/                 # BOM and CPL for JLCPCB assembly
│
├── Software/                   # Desktop tools (Python)
│   ├── gui/                    # Configurator and flash tool
│   │   ├── gui.py              # Main GUI application
│   │   ├── build_app.py        # PyInstaller build script
│   │   ├── scripts/            # Config generation, build automation, paths
│   │   └── tools/              # Bundled toolchain (ARM GCC, CMake, Ninja, dfu-util)
│   │
│   └── gui_demodulator/        # Metadata decoder
│       ├── demodulator_gui.py  # Decoder GUI
│       ├── demodulator.py      # FSK demodulation engine
│       └── reed_solomon.py     # Reed-Solomon decoder
│
├── Testing/                    # Test scripts and data
│   ├── scripts/                # Individual test scripts
│   │   └── helper/             # Shared utilities
│   ├── test_files/             # Input audio and measurement data
│   ├── results/                # Output plots and reports
│   └── run_all_tests.py        # Run all tests
│
└── README.md
```

---

## Getting Started

### Prerequisites

| Component       | Requirement                                  |
| --------------- | -------------------------------------------- |
| Microcontroller | STM32G431-based board (custom PCB or Nucleo) |
| Toolchain       | GNU Arm Embedded Toolchain 13.2+             |
| Build system    | CMake + Ninja                                |
| Programmer      | ST-Link V3 or DFU over USB                   |
| Python          | 3.8+ with `tkinter`                          |

### Install Python Dependencies

```bash
pip install numpy scipy matplotlib soundfile
```

> `tkinter` is included with most Python installations. On Linux, you may need `sudo apt install python3-tk`.

---

## Firmware

### Key Configuration

All metadata and system parameters are defined in [`Firmware/Core/Inc/user_config.h`](Firmware/Core/Inc/user_config.h):

| Parameter                                          | Default             | Description                                                           |
| -------------------------------------------------- | ------------------- | --------------------------------------------------------------------- |
| `USER_STRING`                                      | `"Hello World"`     | User-defined string payload                                           |
| `DEVICE_ID`                                        | `42`                | Numeric identifier for this unit                                      |
| `LOCATION`                                         | `"63.4190,10.4015"` | GPS coordinates of this ARU                                           |
| `TEMPERATURE`                                      | `20`                | Static temperature fallback (overridden by DS3231 reading at runtime) |
| `INCLUDE_USER_STRING` / `INCLUDE_DEVICE_ID` / etc. | `true`              | Toggle individual payload fields on/off                               |
| `FSK_LOWER_FREQUENCY`                              | `10000` Hz          | FSK f0 tone (auto-aligned to DAC sample rate)                         |
| `FSK_HIGHER_FREQUENCY`                             | `12000` Hz          | FSK f1 tone (auto-aligned to DAC sample rate)                         |
| `SIGNAL_ATTENUATION`                               | `100`               | Output level (0 = silence, 100 = full)                                |
| `USE_REED_SOLOMON_ERROR_CORRECTION`                | `true`              | Enable/disable RS error correction                                    |
| `RS_ERROR_CORRECTION_SYMBOLS`                      | `20`                | Number of RS parity symbols                                           |
| `OPERATION_MODE`                                   | `0`                 | 0 = Receiver, 1 = Base Station, 2 = Standalone                        |
| `STARTING_HOUR` / `END_HOUR`                       | `0` / `23`          | Daily schedule window (24h format)                                    |
| `RUN_MINUTES` / `SLEEP_MINUTES`                    | `3` / `0`           | Duty cycle within the schedule window                                 |
| `USE_CABLE_TRANSMISSION`                           | `false`             | Cable (3.5 mm) output instead of speaker                              |

You can edit this file manually or use the GUI configurator (see below).

### Operation Modes

**Receiver (`OPERATION_MODE=0`)**
Listens on the CC1101 in continuous RX mode. On receiving a packet, the unit generates a metadata string combining shared fields from the RF packet (message ID, timestamp, user string) with its own local fields (device ID, location, temperature), then outputs the BFSK signal. One receiver unit is deployed at each ARU.

**Base Station (`OPERATION_MODE=1`)**
Wakes on a fixed RTC-based schedule, generates a common payload (message ID, RTC timestamp, user string), and transmits it via the CC1101 RF transceiver. One unit per deployment must be configured as the base station.

**Standalone (`OPERATION_MODE=2`)**
No RF communication. The unit wakes on a fixed RTC-based schedule, generates a payload from locally configured and measured values, and outputs the BFSK signal. Used when cross-unit synchronization is not needed.

### Features

- **BFSK metadata embedding** — DMA-driven circular DAC output with pre-computed sine LUTs; frequencies are auto-aligned to integer samples-per-period for zero spectral leakage
- **Reed-Solomon FEC** — GF(2^8) encoder with configurable parity symbols for burst-error-resilient decoding
- **CC1101 sub-GHz RF** — bit-banged SPI driver for 433 MHz base station to receiver synchronization; receiver uses continuous RX mode
- **DS3231 RTC** — I2C temperature-compensated clock with battery backup; software-controlled power gating to save current between reads
- **Scheduled operation** — configurable active hours (`STARTING_HOUR`/`END_HOUR`) and run/sleep duty cycle aligned to AudioMoth duty-cycle timing; RTC wakeup timer for Base Station/Standalone, GPIO interrupt (GDO0) for Receiver (mode 0)
- **Audio output paths** — speaker (GPIO-switched amplifier) or cable (relay-switched gain-neutral mixing mode with op-amp signal chain)
- **Battery monitoring** — ADC reading through a resistor divider; halts with error code if voltage drops below 3.0 V
- **USB CDC** — serial interface for host communication and first-boot RTC time sync
- **Flash-persistent flags** — records whether time sync has been performed (survives power cycles)
- **LED status codes** — 4-bit binary blink patterns encoding error/status codes, repeated 3 times
- **STOP1 low-power mode** — full peripheral suspend between transmissions; clock and I2C re-initialized on wake

### Build

```bash
cd Firmware
cmake -B build/Debug -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

### Flash

**Via ST-Link:**

```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
  -c "program build/Debug/Audio_Watermark_Generator.elf verify reset exit"
```

**Via DFU (USB):**
The GUI flash tool handles DFU flashing automatically. Alternatively, use `dfu-util` directly.

---

## Software Tools

### Configurator & Flash Tool

A desktop GUI for configuring metadata parameters and flashing the firmware to the STM32.

```bash
cd Software/gui
python3 gui.py
```

**What it does:**

- Configure metadata fields per operation mode (base station, receiver, standalone)
- Set FSK frequencies, attenuation, and Reed-Solomon error correction
- Set operation schedule (active hours, run/sleep intervals)
- Generate `user_config.h`, build the firmware, and flash over DFU

The tool bundles its own ARM toolchain, CMake, Ninja, and dfu-util in `Software/gui/tools/`, so no manual toolchain setup is needed when using the GUI.

### Metadata Decoder

A desktop GUI for extracting embedded metadata from recorded audio files.

```bash
cd Software/gui_demodulator
python3 demodulator_gui.py
```

**What it does:**

- Load a `.wav` recording containing an embedded BFSK signal
- Perform FSK demodulation and Reed-Solomon decoding
- Display the decoded metadata payload
- Visualize the demodulated spectrum and signal

### Pre-built macOS Apps

Beta `.app` builds are available for macOS:

- **Audio Watermark Flash Tool** — configure and flash
- **FSK Audio Demodulator** — decode metadata from recordings

[Download the latest release](../../releases/latest)

---

## Testing

The `Testing/` directory contains scripts for characterizing and validating system performance.

### Test Scripts

| Script                      | Description                                          |
| --------------------------- | ---------------------------------------------------- |
| `autotest.py`               | Automated demodulator correctness testing            |
| `drift.py`                  | Frequency drift measurement over time                |
| `ecc_burst_test.py`         | Reed-Solomon burst error correction validation       |
| `offset_calculation.py`     | Timing offset between devices                        |
| `localize.py`               | Acoustic localization from multi-receiver recordings |
| `localize_tdoa.py`          | Time-difference-of-arrival (TDOA) localization       |
| `audio_passthrough.py`      | Audio signal chain validation (FFT analysis)         |
| `spec_leak.py`              | Spectral purity and leakage analysis                 |
| `timing.py`                 | Transmission timing validation                       |
| `wakeup.py`                 | Sleep/wakeup mechanism testing                       |
| `plot_fsk_signals.py`       | FSK signal visualization                             |
| `plot_bird_spectrograms.py` | Spectrogram analysis of bird audio                   |
| `plot_current.py`           | Current consumption analysis                         |
| `plot_rssi.py`              | RF signal strength plotting                          |
| `plot_temperature.py`       | Temperature variation analysis                       |

### Test Data

Test input data is organized in `Testing/test_files/`:

- `demodulator_autotest/` — Pre-recorded samples for demodulator validation
- `acoustic_localization/` — Multi-receiver recordings for TDOA testing
- `rtc_measurements/` — RTC drift data
- `range_tests/` — Distance-dependent RF signal quality
- `current_measures/` — Power consumption logs
- `temperature_range/` — Temperature sweep recordings

### Run All Tests

```bash
cd Testing
python3 run_all_tests.py
```

---

## Hardware

The custom 4-layer PCB is designed in KiCad and includes:

- **STM32G431** microcontroller
- **DS3231** RTC with battery backup
- **CC1101** RF transceiver (433 MHz, sub-GHz)
- Audio mixer with gain-neutral pass-through and speaker driver
- USB connector
- Battery power management

Design files are in `Hardware/`:

- Schematics: individual `.kicad_sch` files for each subsystem
- PCB layout: `Audio_watermark_generator.kicad_pcb`
- 3D model: `Audio_watermark_generator.step`
- Fabrication: `Gerber/` directory with Gerber + drill files
- Assembly: `JCLPCB/` with BOM and CPL for JLCPCB ordering
- Custom components: `Components/` with footprints, symbols, and 3D models

---

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes
4. Push and open a pull request

---

## Contributors

- **Kristian Gøystdal** — Developer
- **Einar Bergslid** — Developer

---

## License

Released under the **MIT License**. Free to use, modify, and distribute.

---

`stm32` · `bioacoustics` · `autonomous-recording-unit` · `time-synchronization` · `fsk` · `metadata-embedding` · `reed-solomon` · `kicad` · `python`
