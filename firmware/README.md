# WT VCO firmware

WT VCO runs on two microcontrollers:

| Folder | MCU | Role |
|---|---|---|
| [`wtvco/`](wtvco/) | RP2350 (Waveshare RP2350-Zero) | Wavetable oscillator and audio output (I2S → PCM5102A). Reads V/OCT through an ADS1115 and its own ADC, reads knobs and CVs from the PIC, and runs a USB serial console for calibration. |
| [`pic_wtvco/`](pic_wtvco/) | PIC16F18426 | Works as an external ADC. It samples the 4 knobs and 4 CV inputs and sends one 18-byte frame every millisecond to the RP2350 over UART (1 Mbit/s, TX only). |

The RP2350 runs the audio on core0 and the inputs and console on core1.
See the schematic in [`../hardware/`](../hardware/) for the wiring.

## RP2350 (`wtvco/`)

### Requirements

- [Arduino CLI](https://arduino.github.io/arduino-cli/) or Arduino IDE 2
- The [arduino-pico](https://github.com/earlephilhower/arduino-pico) core **6.1.0** (`rp2040:rp2040`).
  Board manager URL: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
- Board: **Waveshare RP2350 Zero** (`rp2040:rp2040:waveshare_rp2350_zero`)
- No extra libraries.

### Build and flash

```sh
arduino-cli core install rp2040:rp2040@6.1.0
arduino-cli compile --fqbn rp2040:rp2040:waveshare_rp2350_zero firmware/wtvco --output-dir build/wtvco
```

- **First flash:** hold BOOT while you plug in USB. A `RP2350` drive appears; copy `build/wtvco/wtvco.ino.uf2` to it.
- **Later updates:** `arduino-cli upload -p <COM port> --fqbn rp2040:rp2040:waveshare_rp2350_zero firmware/wtvco`

### Host-side unit tests

Needs `g++` (C++17) on the PATH:

```sh
sh firmware/wtvco/tests/run_tests.sh
```

### Generated tables

`wavetable.h` and `exp2_lut.h` are generated files. They are checked in, so you do not need Python to build.
To change the waveforms, edit `tools/wt_frames.py` and run the generator (Python 3 + NumPy):

```sh
python firmware/wtvco/tools/gen_wavetable.py   # writes wavetable.h (+ a preview SVG)
python firmware/wtvco/tools/gen_exp2_lut.py    # writes exp2_lut.h
```

`wt_frames.py` can morph between harmonic spectra (default: sine → triangle → saw → square).
It can also read a Serum-style mono WAV that holds the frames as consecutive single cycles (`SOURCE = "wav"`).

## PIC (`pic_wtvco/`)

### Requirements

- MPLAB XC8 **v3.10**
- PIC16F1xxxx_DFP 1.31.465 (installed with MPLAB X v6.35)
- A programmer. MPLAB Snap works but cannot power the target, so it programs in low-voltage mode. Keep the `LVP = ON` configuration bit as it is.

### Build

```sh
sh firmware/pic_wtvco/build.sh        # -> build/pic_wtvco.hex
sh firmware/pic_wtvco/build.sh test   # link-test build: sends fixed patterns instead of ADC values
```

The script uses the default Windows install paths. To use other paths, set `XC8` (the `xc8-cc` binary) and `DFP` (the DFP folder).

### Programming

The board has **no ICSP header**. The PIC sits in a DIP socket, so program it off the board, for example on a breadboard with external 5 V:

| Snap pin | PIC16F18426 pin |
|---|---|
| 1 MCLR | 4 |
| 2 VTG | 1 (VDD) |
| 3 GND | 14 (VSS) |
| 4 PGD | 13 |
| 5 PGC | 12 |

With MPLAB IPE from the command line:

```sh
ipecmd -TPSNAP -P16F18426 -M -Fbuild/pic_wtvco.hex -Y -OL
```

Check the orientation of pin 1 before you insert the PIC into the socket.

## Serial console and calibration

Open the RP2350's USB serial port (any baud rate; 115200 is fine) and type `help`.

| Command | Purpose |
|---|---|
| `show`, `adc`, `ads`, `fus`, `pic`, `cal` | Status: shared variables and load, coarse ADC, ADS1115, V/OCT fusion, PIC link, relative calibration |
| `calmode on` / `off` | Enter or leave calibration mode (needed for the commands below) |
| `czero` | With **all CV inputs unplugged**: measure the CV zero points |
| `vcal <mV>` | With V/OCT held at a voltage you measured with a multimeter: record one point |
| `vcal fit` | Fit the ADS1115 gain and offset over the recorded points |
| `calsave` | Save everything to flash. Audio is muted briefly. |
| `calshow` / `caldefault` | Show the values, or go back to the theoretical values (not saved) |
| `tune` | Try other time constants and dead bands (RAM only) |

Typical first calibration:

1. `calmode on`
2. Unplug every CV input, then run `czero`.
3. Hold V/OCT at about 1 V, measure it, and run `vcal <mV>`. Repeat at about 4 V. The two points must be at least 1 V apart.
4. `vcal fit`, then `calsave`, then `calmode off`.

The relative calibration between the RP2350 ADC and the ADS1115 learns itself while you play. `calsave` stores it too.

Test overrides such as `coarse`, `voct` or `ksrc stub` disconnect the knobs. Send `default` to give control back to the knobs.

## Build options

`wtvco/config.h` holds the compile-time constants. `CFG_DIAG` (default 1) adds the diagnostic console commands. They do nothing unless you type them.
