# WT VCO

A wavetable VCO module for [AE Modular](https://www.tangiblewaves.com/) (5 V, 0–5 V CV).

- Wavetable oscillator on an RP2350 (Waveshare RP2350-Zero) with a PCM5102A DAC. The output is 1–4 V, centred on 2.5 V.
- V/OCT is read by two converters, an ADS1115 and the RP2350's own ADC, and the readings are fused into one value. The ADS1115 gives the accuracy and the RP2350 ADC gives the speed.
- A PIC16F18426 reads the knobs and the WT / FM CV inputs. Each CV has a unipolar and a bipolar input.
- Knobs: FREQ COARSE, FREQ FINE, WT, FM DEPTH.
- Calibration runs from a USB serial console and is stored in flash.

## Repository layout

| Folder | Contents |
|---|---|
| [`hardware/`](hardware/) | Schematic (PDF, SVG), BOM (CSV) and the EasyEDA Pro project (`.epro`) |
| [`firmware/`](firmware/) | RP2350 firmware (Arduino) and PIC firmware (XC8). See [`firmware/README.md`](firmware/README.md) for build, flashing and calibration. |
| [`spec/`](spec/) | Technical specification: design, design rationale and verification results. [CV and knob inputs](spec/wtvco_input_spec.md) and [audio processing](spec/wtvco_audio_process.md) (Japanese versions: `*.ja.md`) |

## License

| Folder | License |
|---|---|
| `hardware/` | [CERN-OHL-P v2](hardware/LICENSE) |
| `firmware/` | [MIT](firmware/LICENSE) |
| `spec/` | [CC BY 4.0](spec/LICENSE) |

Copyright (c) 2026 Shota Azuma.

You may use, modify, build and sell this design, including commercially, under these licenses.
Please keep the copyright notice and credit the original.

### Brand name: SHOT MODULES

WT VCO is designed and sold by **SHOT MODULES**.

These licenses cover the design files only. **They grant no right to use the name "SHOT MODULES" or its logo.**
Do not sell or promote products made from this design under that name or logo, or in a way that suggests SHOT MODULES made or endorses them.
You are free to use the name "WT VCO".
