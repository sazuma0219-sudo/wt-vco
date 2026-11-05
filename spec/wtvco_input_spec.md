# WT VCO technical specification: CV and knob inputs

English | [日本語](wtvco_input_spec.ja.md)

This document describes the design of the **CV and knob input section** of WT VCO, a wavetable VCO module for AE Modular (RP2350-Zero, **1 channel**), the reasons behind it, and the verification results.

- **V/OCT** is handled by the RP2350 alone. It is read by two converters, the ADS1115 and the RP2350's internal ADC, and the two readings are fused into one value (sensor fusion). The PIC has nothing to do with V/OCT.
- **WT CV, FM CV (each with a unipolar and a bipolar input) and the knobs** are read by the ADC of a PIC16F18426 and sent to the RP2350 over UART. The PIC works as an "external ADC that only sends raw values".

Scope: the input processing of the RP2350 firmware (core1) and the PIC firmware.
Sound generation (core0) is described in `wtvco_audio_process.md`. The audio side reads the shared variables defined here (chapter 10).

The schematic and BOM are in `../hardware/`, the firmware is in `../firmware/`. This document shows circuit excerpts where they help explain the design.

Datasheets referenced: ADS1115 = TI SBAS444B, PCM5102A = TI SLAS859C, PIC16F18426 = Microchip DS40001985C.

---

## 1. Overview

### 1.1 Inputs

| Input | Kind | Speed | Accuracy | Read by |
|---|---|---|---|---|
| V/OCT1 | CV | Fast | High | ADS1115 + RP2350 ADC (coarse) |
| WT1 CV UNI (unipolar) | CV | Medium | Low | PIC ADC → UART |
| WT1 CV BI (bipolar) | CV | Medium | Low | PIC ADC → UART |
| FREQ1 COARSE | Knob | Slow | High, **stability first** | PIC ADC → UART (ratiometric) |
| FREQ1 FINE | Knob | Slow | High | PIC ADC → UART (ratiometric) |
| WT1 KNOB | Knob | Slow | Medium | PIC ADC → UART (ratiometric) |
| FM1 CV UNI (unipolar) | CV | Medium (slow modulation only) | Low | PIC ADC → UART |
| FM1 CV BI (bipolar) | CV | Medium (slow modulation only) | Low | PIC ADC → UART |
| FM1 DEPTH | Knob | Slow | Low | PIC ADC → UART (ratiometric) |

### 1.2 Output (for reference; handled by the audio side)

| Output | Description |
|---|---|
| OUT1 | Oscillator output. The PCM5102A's OUTL (±1.5 V around GND) is shifted by the output buffer to **1–4 V (centred on 2.5 V)** (`wtvco_audio_process.md` §1.3) |

### 1.3 How the pitch is set (for reference; computed by the audio side)

The frequency is

```
f = f_C0 × 2^((coarse_cent + fine_cent + voct_cent + fm_cent) / 1200)
```

| Term | Range | Set by |
|---|---|---|
| `f_C0` | Constant | Frequency of C0 |
| `coarse_cent` | 0–7200 cent | FREQ COARSE. Moves continuously over 72 semitones, C0–C6 |
| `fine_cent` | −100 to +100 cent | FREQ FINE. Corrects the pitch set roughly with COARSE (0 at the centre) |
| `voct_cent` | 0–6000 cent | V/OCT input. 0–5 V (clamped to 0–6000. When unplugged it does not read 0 but about 0.2–0.3 V higher; chapter 6) |
| `fm_cent` | −6000 to +18000 cent | FM (exponential FM = pitch modulation). FM CV (−2.5 to +7.5 V) × FM DEPTH (0–2400 cent/V). 0 when unplugged (2.5, 8.6) |

- Note names use scientific pitch notation with A4 = 440 Hz (C0 ≈ 16.35 Hz, C3 ≈ 130.81 Hz, C6 ≈ 1046.5 Hz).
- COARSE spans 7200 cent with the PIC's 12-bit ADC, so **one ADC LSB ≈ 1.76 cent**. A 1-LSB wobble is heard directly as a pitch wobble,
  so COARSE is processed with stability as the top priority (7.2, 8.3). The averaged value is sent with 14 bits (7.3), so it reaches the RP2350 in 0.44-cent steps.
- Without FM the highest pitch, with COARSE, FINE and V/OCT all at maximum, is 13300 cent = C11 + 100 cent (about 35.5 kHz).
  With FM the possible range widens to −6100 to +31300 cent, but the audio side clamps the pitch to C0 – just below C11.
  The handling of the upper and lower ends (clamping, anti-aliasing) belongs to the audio side.

### 1.4 The V/OCT problem

- On AE Modular, V/OCT is 0–5 V with an output impedance of about 1 kΩ. At 1 V/oct, 0–5 V = 0–6000 cent.
- **Accuracy requirement**: in steady state, 1 LSB ≤ 3 cent (= 2.5 mV).
- **Speed requirement**: following a note-on within ≤ 500 µs (including the audio output).

The ADS1115 meets the accuracy requirement (in the ±6.144 V range 1 LSB = 187.5 µV ≈ 0.22 cent; input impedance
10 MΩ common-mode and 22 MΩ differential (datasheet Table 2 etc.), so it can be connected directly without a buffer or divider).
But at most 860 SPS = nominally 1.163 ms per conversion (1.057–1.292 ms with the ±10% spread of the internal oscillator), it does not meet the speed requirement.

The RP2350's internal ADC (12 bit, 500 kSPS, ENOB about 9.2) meets the speed requirement but not the accuracy requirement.

### 1.5 The V/OCT solution

Read both at the same time, and use **the ADS1115 in steady state and the internal (coarse) ADC while the input is changing**.

- For about 1.1–2.7 ms after a note-on (5.5) the accuracy is coarse (an error of a few cent to a dozen or so cent). This is accepted.
- An error of the order of 1200 cent (the "squelch" heard when the previous note's value lingers) is **not** accepted. Preventing it is the reason this mechanism exists.

### 1.6 Terms

| Term | Meaning |
|---|---|
| Node | The point after the 1 kΩ right behind the V/OCT jack, where the ADS1115 input, the buffer input and the pull-down meet |
| Coarse ADC / coarse value | V/OCT read by the RP2350's internal ADC (through the buffer and divider) |
| Raw sample | One conversion of the coarse ADC. **Every 2 µs** (500 kSPS, 3.1) |
| Coarse sample `p` | The average of each block of 16 raw samples (**every 32 µs**, 3.2). Used for window statistics |
| Moving average `s` | The moving average of the last 16 raw samples (updated **every raw sample = every 2 µs**, 3.2). Used for jump detection and as the FAST-mode output |
| Clean conversion | An ADS1115 conversion during whose window the input hardly moved |
| ADS mode | The state in which the V/OCT output uses the ADS1115 value |
| FAST mode | The state in which the V/OCT output uses the coarse value |
| Conversion window | The interval from the start to the end of one ADS1115 conversion (the time between RDY falling edges) |
| Frame | One unit sent by the PIC over UART, holding the values of all 8 channels (chapter 7) |
| Unipolar input (UNI) | One of the CV inputs (WT CV, FM CV). Interprets 0–5 V as upward modulation from 0 V (0 to +5 V). 0 V when unplugged (2.5) |
| Bipolar input (BI) | One of the CV inputs (WT CV, FM CV). **It accepts 0–5 V** and interprets it as ±2.5 V modulation around 2.5 V. It is not an input for ± voltages. 2.5 V when unplugged (2.5) |
| Sent value `raw` | The value the PIC sends for each input: the 12-bit ADC average × 4, **14 bits (0–16380)** (7.2, 7.3). All conversions on the RP2350 work on this value |
| Zero point | The sent value treated as "no modulation" for each CV input. Measured with the input unplugged and saved (9.4) |

---

## 2. Hardware

### 2.1 Power

```
AE 5V bus ──┬──────────────── ADS1115 VDD / V/OCT buffer (MCP6001) V+ / I2C pull-ups
            │
            ├──────────────── PIC16F18426 VDD (= top of the knobs, 2.5)
            │
            ├──────────────── Output buffer (MCP6001) V+ (`wtvco_audio_process.md` §1.3)
            │
            ├──────────────── XC6206 (3.3 V LDO) ── PCM5102A
            │
            └──|>|── RP2350-Zero 5V pin    (Schottky diode)
```

- **The module is meant to run from the AE bus.** The ADS1115, V/OCT buffer, PIC, PCM5102A and output buffer are all powered from the AE bus,
  so **running from USB alone is not supported** (nothing but the RP2350 would run, so there is no degraded mode for it either).
- **USB is used together with the AE bus supply, for monitoring, calibration commands (chapter 9) and flashing.**
- **Schottky diode**: keeps the PC's VBUS and the AE 5 V bus from fighting while both are connected.
  **The RP2350-Zero's 5V pin (header pin 23, net VBUS on the schematic) is tied directly to the USB-C VBUS with no diode**
  (according to Waveshare's published schematic; the only part on VBUS is the ESD5451N ESD protection). **The external Schottky diode is mandatory.**
  The 3.3 V is made from VBUS by an LDO (ME6217C33M5G, CE tied to VIN). A drop of about 0.3 V is no problem.
- If the AE bus is switched off while USB is plugged in, the RP2350 alone keeps running and keeps sending I2S to the unpowered PCM5102A.
  The PCM5102A's digital inputs (BCK, LRCK, DIN) are failsafe inputs (Failsafe LVCMOS; datasheet chapter 7, note 1 of the pin function table), so it is not damaged,
  but do not leave it in this state for long. (The 3.9 V absolute maximum rating (datasheet 8.1) is for DVDD = 3.3 V and is no basis for the unpowered case.)

### 2.2 V/OCT input circuit

```
                                            +-----------+
V/OCT IN --[ 1kΩ ]--+----------------------| ADS1115   |
  (jack)            |                       | AIN1      |
                    |                       +-----------+
                    +--[ 10MΩ ]-- GND
                    |
                    +--[ 1nF ]-- GND
                    |
                    |   +------------------+
                    +---| MCP6001          |--[ 10kΩ ]--+-- ADC1 (RP2350 GP27)
                        | voltage follower |            |
                        +------------------+          [ 15kΩ ]
                                                        |
                                                       GND
```

- **1 kΩ (right after the jack)**: short-circuit protection and input current limiting.
- **1 nF**: stabilises the ADS1115 input. The time constant with the 1 kΩ is 1 µs, much shorter than the ADS1115 conversion window (about 1.16 ms) and the coarse ADC's raw sample (2 µs).
- **10 MΩ**: pull-down. Pulls the node towards GND when unplugged. **It does not reach 0 V** (the ADS1115 input pulls towards about 0.7 V, so the node settles at about 0.2–0.3 V; chapter 6).
  A lower value would increase the V/OCT error through the divider it forms with the source's output impedance (about 1 kΩ) plus the 1 kΩ protection resistor, and since that error changes with every source module it could not be calibrated out.
  With 10 MΩ the pull towards GND is about 0.02% (about 1.2 cent at 5 V), and a 1 kΩ change in the source impedance causes about 0.6 cent, which calibration can absorb.
- **MCP6001** (5 V single supply, rail-to-rail in and out, single): buffer that isolates the node from the coarse ADC side. V+ = 5 V, V− = GND, 0.1 µF right at the supply pin.
  - The output buffer (`wtvco_audio_process.md` §1.3) is also an MCP6001, a separate IC from this buffer.
  - If an MCP6002 (dual) is used, wire the unused half as a voltage follower with its non-inverting input tied to GND.
  - Hanging the divider directly on the node would turn the spread in the source module's output impedance into an absolute error on the ADS1115 side, so **the divider always goes after the buffer**.
  - **Do not use zero-drift (chopper) op-amps** (they emit pulse currents at the input and disturb the node).
- **10 kΩ / 15 kΩ divider**: ratio 0.6. A 5 V input becomes 3.0 V, leaving margin to the ADC limit (IOVDD = 3.3 V).
  The upper 10 kΩ also isolates the follower from capacitive load. The Thevenin resistance is 6 kΩ, low enough for the ADC's sampling.
  There is no capacitor on the ADC pin.
- **The RP2350's ADC pins are not 5 V tolerant.** Their input limit is set by IOVDD (3.3 V).

### 2.3 ADS1115 configuration

| Item | Value |
|---|---|
| Role | V/OCT1 |
| I2C bus | I2C0 (GP8 = SDA, GP9 = SCL) |
| ADDR pin | GND (address 0x48, datasheet Table 5) |
| Input | AIN1 only (pin 5, matching the board wiring) |
| Operation | Continuous conversion, ALERT/RDY used as conversion-ready (RDY = GP10) |
| Supply | **5 V** |

- One device on the bus.
- **It runs from 5 V** so that it can take V/OCT (0–5 V) without a divider. The analog inputs must not exceed VDD + 0.3 V (datasheet absolute maximum ratings, note).
  Even in the ±6.144 V range, only 0–VDD can actually be measured, and the 0–5 V V/OCT fits in it.
- AIN0, AIN2 and AIN3 are unused (tied to GND). Do not overdrive unused inputs (datasheet "MULTIPLEXER": overdriving an unused input can affect conversions on other inputs).

### 2.4 I2C and RDY pull-ups

- SDA/SCL are **pulled up to 5 V with 4.7 kΩ**.
  - The RP2350's GPIOs (FT pads, datasheet 14.8.2.1) tolerate inputs **up to 5.5 V while IOVDD is present (2.5 V or more)**.
    **With IOVDD = 0 V only up to 3.63 V** is allowed.
  - I2C is open-drain and the RP2350 never drives High, so the pin never outputs 5 V.
  - It satisfies both the ADS1115's (5 V supply) VIH = 0.7 × VDD = 3.5 V (datasheet electrical characteristics) and the RP2350's VIH = 2.0 V. **No level shifter is needed.**
  - **The conditions also hold at power-up.** The RP2350's 3.3 V is made from the same AE 5 V bus (through the Schottky diode and the RP2350-Zero's LDO)
    and rises together with the bus. By the time the bus exceeds 3.63 V, IOVDD is already around 3 V, satisfying the 2.5 V condition.
    - FT pads draw almost no current, so **limiting the current with series or pull-up resistors does not lower the pin voltage**.
      The safety argument rests on "rising together", not on current limiting.
    - The 5 V bus and the RP2350-Zero's 3V3 pin were observed together on an oscilloscope at power-up and power-down, and the conditions held (13.3).
  - This argument applies to every pin pulled up to or receiving 5 V (I2C, RDY, UART RX).
- ALERT/RDY is an open-drain output. It is **pulled up to 3.3 V with 10 kΩ** (5 V would also work).
- The clock is 400 kHz (the ADS1115's Fast mode limit, datasheet Table 1). The ADS1115 does not stretch the clock.

### 2.5 Circuits around the PIC

#### Knobs (4)

```
PIC VDD ──+──────────────+──────────────+──────────────+
          |              |              |              |
        [10kB]         [10kB]         [10kB]         [10kB]
          |              |              |              |
    FREQ1 COARSE     FREQ1 FINE      WT1 KNOB       FM1 DEPTH     → PIC analog inputs (100nF on each wiper)
          |              |              |              |
GND ──────+──────────────+──────────────+──────────────+
```

- The top of the knobs is taken from **the same node as the PIC's VDD pin**, and the ADC reference is VREF+ = VDD.
  This makes the reading **ratiometric**, so variations of the 5 V bus do not appear in it.
- The 100 nF on each wiper supplies charge to the ADC's sampling capacitor and filters noise.
  **FREQ1 COARSE may use 1 µF** (even less noise; the response gets slower, which is fine for a slow control).
- Use a good-quality potentiometer with little crackle for FREQ1 COARSE (1 LSB ≈ 1.76 cent, so wiper noise shows up directly as pitch).
- 0.1 µF + 0.01 µF (datasheet 39.4.2 note: the condition for the HFINTOSC accuracy) + 10 µF right at the PIC's supply pins.

#### WT CV (unipolar and bipolar inputs)

AE Modular CVs are unipolar 0–5 V, so with a single input the WT could only be modulated upwards.
Therefore WT CV has two inputs (pin sockets on AE).

| Input | Accepts | Interpretation (modulation) | Unplugged |
|---|---|---|---|
| WT1 CV UNI (unipolar) | 0–5 V | From 0 V. 0 to +1.0 (upwards only) | 0 V by the pull-down → no modulation |
| WT1 CV BI (bipolar) | 0–5 V | Around 2.5 V. −0.5 to +0.5 (both ways) | 2.5 V by the bias → no modulation |

- **Both inputs have the same sensitivity** (1 V = WT 0.2).
- **Both can be used at once.** They are summed internally (8.4). For example, an ADSR into UNI and an LFO into BI give a combined ADSR + LFO modulation.
- Two inputs instead of a switch: the PIC has ADC pins to spare, and an extra pin socket takes less room and costs less than a switch.
- **The result depends on where you plug in.** A 0 V-based CV such as an ADSR in BI shifts the WT by −0.5 at rest (0 V).
  An LFO in UNI only swings upwards. The panel labels must make UNI and BI distinguishable.

```
WT UNI IN --[ 1kΩ ]--+-- PIC analog input (RC3)
 (pin)               |
                     +--[ 100kΩ ]-- GND
                     |
                     +--[ 100nF ]-- GND      ← right at the PIC pin
```

```
                          PIC VDD (same node as the top of the knobs)
                             |
                          [ 200kΩ ]
                             |
WT BI IN --[ 1kΩ ]--+--------+-- PIC analog input (RC4)
 (pin)              |
                    +--[ 200kΩ ]-- GND
                    |
                    +--[ 100nF ]-- GND       ← right at the PIC pin
```

- The PIC runs from 5 V, so **no divider is needed**. The 1 kΩ is for protection.
- **The 100 kΩ on UNI** is a pull-down that brings the input to 0 V when unplugged.
- **The 2 × 200 kΩ on BI** split the PIC's VDD in half as a bias (Thevenin equivalent 2.5 V / 100 kΩ).
  - It is made from the same supply as the ADC reference (VREF+ = VDD), so it is ratiometric: unplugged, it always reads the ADC mid-scale (about 2048 in 12 bits, about 8190 as a sent value) whatever the 5 V bus does.
    This matches the 2.5 V of the theoretical conversion (8.4), so **unplugged, the modulation is 0 regardless of the bus voltage**.
  - Use two 1% resistors of the same value. The resistor mismatch (up to about 25 mV) is cancelled by the zero-point calibration (9.4), but this keeps the mismatch small even before calibration.
- **Error from the input loading**: the divider formed by the source's output impedance (about 1 kΩ) plus the 1 kΩ protection and the 100 kΩ Thevenin resistance gives a gain error of about 2%.
  UNI shrinks around 0 V and BI around 2.5 V, so **BI's mid-point (no modulation) does not move**. It can be absorbed by calibration on the RP2350 (9.4, optional).
- **100 nF (right at the PIC pin)**: supplies charge to the ADC's sampling capacitor and filters noise and aliasing.
  - The datasheet recommends an analog source impedance of **10 kΩ or less** (DS40001985C 32.3, figure 32-4, note 3 of equation 32-1).
    The WT CV inputs are 100 kΩ when unplugged and still about 2 kΩ + α when connected, so the charge for sampling comes from the capacitor at the pin.
  - **Charge-sharing error**: the sample-and-hold capacitor CHOLD = 28 pF (figure 32-4) is not discharged between conversions (note 2 of equation 32-1),
    so in the first acquisition after switching channels, charge is shared between CHOLD (holding the previous channel's voltage) and the capacitor.
    Even in the worst case (5 V difference) this is 5 V × 28 pF / 100 nF ≈ **1.4 mV (about 1.1 LSB)**. From the second conversion of a burst average on it is negligible. No problem for a low-accuracy input.
  - **Leakage error**: the 10 kΩ recommendation is also there to meet the pin leakage specification (note 3 of equation 32-1).
    DC leakage cannot be supplied by a capacitor, so it was calculated. The I/O port input leakage IIL is typically ±5 nA, at most ±125 nA (chapter 39, D340).
    Unplugged (100 kΩ) the worst case is 12.5 mV (WT 0.0025), typically 0.5 mV. Connected (about 2 kΩ) the worst case is 0.25 mV. **Negligible.**
  - **Bandwidth**: connected, the time constant is about 2 kΩ × 100 nF = 200 µs (fc ≈ 0.8 kHz). This also acts as an anti-aliasing filter for the PIC's sampling (one frame per 1 ms).
    Unplugged it is 100 kΩ × 100 nF = 10 ms, which is fine because the input is static.
  - Use low-leakage ceramic capacitors (datasheet 34.9: parts on analog inputs must have low leakage).
    Place them as close to the PIC pins as possible, with a short return to the PIC's VSS.
- WT CV is an absolute voltage from another module, so it does pick up errors from VDD (5 V bus) variations. Accepted for a low-accuracy input.
- The offsets when unplugged (BI bias resistor mismatch, the PIC ADC's offset, pin leakage) are cancelled by the **zero-point calibration (9.4)**.

#### FM CV (unipolar and bipolar inputs) and FM DEPTH

Inputs for frequency modulation (**exponential FM = pitch modulation**).

- **Only for slow modulation by LFOs, envelopes and the like.** The CV is read through the PIC every 1 ms and the input RC has fc ≈ 0.8 kHz,
  so audio-rate FM (modulation that creates overtones, as in FM synthesis) is out of scope.
- **The input arrangement, circuit and unplugged behaviour are the same as for WT CV.** A second set of the UNI and BI circuits above (1 kΩ, 100 kΩ pull-down / 2 × 200 kΩ bias, 100 nF at the pin) is used.
  UNI = RC5, BI = RA4 (2.7).
- Both inputs have the same sensitivity. The summed voltage `fm_v` (−2.5 to +7.5 V) is multiplied by the sensitivity set with FM DEPTH to give the pitch change (8.6).
- The **FM DEPTH knob** sets the pitch change per volt in the range **0–2400 cent/V**. The curve is non-linear (square; 8.3).
- The modulation range at maximum DEPTH is **−6000 to +18000 cent (−5 to +15 oct)**. The audio side clamps the pitch to C0 – just below C11.
- **CV errors appear in the pitch multiplied by DEPTH.** 1 LSB (1.22 mV) is about 2.9 cent at maximum DEPTH.
  Unplugged offsets (e.g. the BI bias resistor mismatch of up to about 25 mV is about 60 cent at maximum DEPTH) are **cancelled by the zero-point calibration (9.4)**. For FM CV it is effectively required.
- The accuracy is **accepted as low**. Even with DEPTH set to 1200 cent/V, accurate 1 V/oct tracking like V/OCT is not expected.

#### UART (PIC → RP2350, one way)

```
PIC TX (RA5) --[ 4.7kΩ ]-- RP2350 GP1 (UART0 RX)
```

- The PIC's TX is 5 V, but the RP2350's GPIOs are 5 V tolerant, so they can be connected directly (same conditions as 2.4).
  The PIC's TX starts driving High only after the PIC firmware has initialised the UART, by which time the RP2350's 3.3 V is certainly up.
- **The 4.7 kΩ series resistor is there for protection** (limiting the short-circuit current if GP1 were set as an output by mistake, for example).
  It is not needed to meet the FT pad's 5 V tolerance conditions (2.4).
  (Together with the RP2350's pin capacitance the time constant is under 100 ns, much shorter than one bit at 1 Mbps = 1 µs.)
- **GP1's internal pull-up is enabled** (firmware). Without the PIC (8.2) nothing is connected to GP1,
  and with the default internal pull-down the RX line would sit Low and produce framing errors continuously.
  With the PIC fitted, its TX drives High, so this has no effect. No external pull-up is needed.
- There is no signal from the RP2350 to the PIC.

#### Programming and MCLR

- **The PIC is programmed off the board (on a breadboard or similar) and then fitted into the socket on the board** (procedure in `../firmware/README.md`).
  There is no ICSP header on the board.
  - Nothing external is connected to RA0 (ICSPDAT) or RA1 (ICSPCLK) (2.7). An ICSP header could be added later.
  - The PIC on the board sits **in a DIP socket** (it is removed and reinserted every time it is reprogrammed).
- **RA3 (MCLR) is pulled up to VDD with 10 kΩ.** The configuration is MCLRE = 1 (MCLR enabled).

### 2.6 RP2350-Zero pin assignment

| GPIO | Signal | Notes |
|---|---|---|
| GP1 | UART | UART0 RX. From the PIC's TX (RA5) through 4.7 kΩ. Internal pull-up (2.5) |
| GP8 | V/OCT_I2C_SDA | I2C0 SDA. ADS1115 |
| GP9 | V/OCT_I2C_SCL | I2C0 SCL |
| GP10 | V/OCT_RDY | ADS1115 ALERT/RDY. Interrupt on the falling edge |
| GP12 | DAC_DIN | PCM5102A (audio side). PIO OUT pin |
| GP13 | DAC_LRCK | The audio side's PIO program drives LRCK and BCK as two consecutive side-set pins (**BCK = LRCK + 1**; the side-set base is LRCK) |
| GP14 | DAC_BCK | |
| GP16 | (do not use) | On-board RGB LED |
| GP27 | V/OCT_COARSE | ADC1. V/OCT (coarse). Connected to **the node after the 10 kΩ / 15 kΩ divider on the buffer output** (2.2; 3.0 V max) |
| Others | Unused | Output Low |

- **The UART from the PIC is received on UART0** (RX = GP1). arduino-pico's `Serial1` is not used; UART0 is initialised with pico-sdk's `hardware_uart`
  and its receive FIFO is read directly from the core1 loop (no interrupt; 8.1).
  `Serial1` assigns the TX pin (GP0) to the UART even when only receiving, so it is not used. GP0 is driven Low like the other unused pins.
- In the left-hand row (GP29, 28, 27, 26, 15, 14), the pins next to the ADC (GP27), GP28 and GP26, are unused.
  The same row does carry the I2S BCK (GP14, 6.144 MHz), but its contribution to the coarse ADC noise turned out to be small (13.1).
- The ADC pin (GP27) is used with the pad's input buffer disabled (IE = 0) and output disabled (OD = 1) (datasheet 12.4).
- **On the RP2350-Zero, ADC_AVDD (pin 44) is tied directly, without a filter, to the 3V3 net shared with IOVDD** (according to Waveshare's published schematic;
  the only decoupling is the 100 nF shared with IOVDD). The coarse ADC's reference is therefore the LDO's 3.3 V itself, but its scale is absorbed by the
  relative calibration against the ADS1115 (9.3), and the coarse ADC noise was within limits (13.1).

### 2.7 PIC16F18426 pin assignment (14 pins)

| Pin | Name | Signal | Notes |
|---|---|---|---|
| 1 | VDD | 5 V (AE bus) | Same node as the top of the knobs |
| 2 | RA5 | UART | EUSART TX assigned with PPS (RA5PPS = 0x0F, TX1/CK1). To the RP2350's GP1 through 4.7 kΩ |
| 3 | RA4 | FM_BI | Analog input ANA4. FM CV (bipolar) |
| 4 | RA3 / MCLR | MCLR | Pulled up to VDD with 10 kΩ (2.5) |
| 5 | RC5 | FM_UNI | Analog input ANC5. FM CV (unipolar) |
| 6 | RC4 | WT_BI | Analog input ANC4. WT CV (bipolar) |
| 7 | RC3 | WT_UNI | Analog input ANC3. WT CV (unipolar) |
| 8 | RC2 | FM_DEPTH | Analog input ANC2. FM DEPTH knob |
| 9 | RC1 | FREQ_FINE | Analog input ANC1. FREQ FINE knob |
| 10 | RC0 | FREQ_COARSE | Analog input ANC0. FREQ COARSE knob |
| 11 | RA2 | WT_KNOB | Analog input ANA2. WT knob |
| 12 | RA1 / ICSPCLK | Unused | Output Low |
| 13 | RA0 / ICSPDAT | (test) | Outputs High while converting (for measuring the conversion time in 7.2). No other function |
| 14 | VSS | GND | |

- All 8 analog inputs are on pins that have ADC channels (datasheet ADPCH list).
- TX can be routed to any port with PPS (datasheet Table 16-3).
- TX (RA5, pin 2) sits between VDD (pin 1) and FM_BI (RA4, pin 3). Do not omit the 100 nF at the FM_BI pin (2.5).

---

## 3. RP2350 internal ADC (coarse V/OCT reading)

### 3.1 Settings

| Item | Setting | Notes |
|---|---|---|
| Clock | 48 MHz, CLKDIV = 0 | Fastest, 500 kSPS (96 cycles per conversion = 2 µs) |
| Channel | AINSEL = 1 (ADC1 = GP27) fixed, **no round robin** (RROBIN = 0) | Only V/OCT1 is read, at 500 kSPS |
| FIFO | Enabled, DREQ enabled, SHIFT = 0 (keep 12 bits) | |
| Transfer | DMA into a ring buffer (16 bit × 1024, a power of 2). **Ring wrap on the write side**, **runs forever** | Continuous sampling without the CPU. 1024 samples is about 2 ms |

- **Sampling always runs at a constant rate.** It is never stopped or slowed down depending on the state.
- The DMA ring's write position is itself the time axis (**2 µs per sample**).
- **The ring buffer is aligned to its own size in bytes** (16 bit × 1024 = 2048 bytes → a 2048-byte boundary). The DMA ring wrap only folds the low address bits,
  so a misaligned buffer would be written past its end.
- **It runs forever the same way as the audio side's output ring**, using the RP2350 DMA's ENDLESS mode (`dma_encode_endless_transfer_count()` in the pico-sdk
  bundled with arduino-pico 6.1.0; `wtvco_audio_process.md` §1.11).

### 3.2 Pre-processing

core1 follows the DMA ring and computes two kinds of average.

| Value | How | Update | Use |
|---|---|---|---|
| Coarse sample `p` | Average of each block of 16 raw samples | **Every 32 µs** (31.25 kHz) | Conversion-window statistics (stillness check, `ref`) |
| Moving average `s` | Moving average of the last 16 raw samples | **Every raw sample (every 2 µs)** | **Jump detection and the FAST-mode output** (chapter 5) |

- `s` is computed as "add the new sample to the sum and subtract the sample from 16 samples ago". That older sample is still in the DMA ring, so no extra buffer is needed.
- The window lengths are equal, so `s` and `p` have the same noise. `s` exists to detect a jump without waiting for the next `p` block boundary (32 µs).
- On each loop pass core1 processes all the raw samples the DMA has added since the previous pass (a pass is within 10 µs, so about 5 samples each time; 10.1).
- A raw sample with the FIFO ERR bit (conversion error) set is **replaced by the previous value**, so as not to disturb the moving-average window.
- **If core1 falls behind and the DMA laps it by a full ring or more, the difference of the write positions alone cannot detect it** (the difference wraps around the ring length).
  Left undetected, every window would be closed about one ring late, and the ADS1115 results would no longer match their windows.
  So when more than 1.92 ms (shorter than one ring = 2.048 ms) has passed since the previous pass, it is also treated as an overrun,
  and the moving average and the window-boundary queue (4.3) are rebuilt.

Input-referred size of one coarse-sample LSB (12-bit raw value):
3.3 V / 4096 / 0.6 ≈ 1.34 mV ≈ **1.6 cent**. Averaging 16 samples reduces random noise to about 1/4.

---

## 4. ADS1115 settings

### 4.1 Register settings

| Item | Setting | Reason (datasheet) |
|---|---|---|
| MUX | AIN1 single-ended, fixed (MUX[2:0] = 101: AINP = AIN1, AINN = GND) | The MUX never switches, so continuous conversion can be used |
| PGA | FSR ±6.144 V (PGA[2:0] = 000) | Required to take 0–5 V without a divider. 1 LSB = 187.5 µV ≈ 0.22 cent |
| MODE | Continuous conversion (0) | |
| DR | 860 SPS (111) | Nominal conversion period 1.163 ms. Internal oscillator, so **±10%** (datasheet "Data rate variation") → 1.057–1.292 ms |
| COMP_POL | 0 (active Low) | RDY is detected on the falling edge |
| COMP_QUE | Anything but 11 (e.g. 00) | 11 disables the comparator = RDY output |
| Hi_thresh | 0x8000 (MSB = 1) | **Required to enable the RDY function** (datasheet "CONVERSION READY PIN") |
| Lo_thresh | 0x0000 (MSB = 0) | **Same. Forget it and no interrupt ever arrives** |

- With COMP_QUE = 00 the Config register value is **0x50E0** (OS = 0, MUX = 101, PGA = 000, MODE = 0, DR = 111, COMP_MODE/POL/LAT = 0, COMP_QUE = 00).
- After writing the configuration, **set the pointer register back to Conversion (0x00)**.
  From then on a read is just the address plus 2 bytes (about 70 µs at 400 kHz, 4.4). To avoid stalling core1 during that time, reads are non-blocking (4.4).
- In continuous mode, ALERT/RDY gives **a Low pulse of about 8 µs per conversion** (datasheet "CONVERSION READY PIN"). Its falling edge is used.
- **Do not assume a fixed conversion period.** It varies by ±10%, so always use RDY as the reference.

### 4.2 Conversion window

- RDY(n) falling edge to RDY(n+1) falling edge = the window of conversion n+1.
- The ADS1115 returns the average of the input over this interval (ΔΣ + sinc filter, whose response has zeros at integer multiples of the data rate; datasheet figure 21).
- So checking whether the coarse value moved during this interval tells whether the conversion result can be trusted.

### 4.3 Matching RDY to the sample stream

- The RDY GPIO interrupt handler **only records the DMA write position at that moment**.
  The I2C read happens outside the handler (in the core1 loop). **The interrupt is registered on core1** (core0 is dedicated to audio; 10.1).
- The recorded position is the window boundary. Window statistics are computed from the coarse samples split at this boundary.
  Even if the read is somewhat late, the boundary time stays exact.
- Interrupt latency (a few µs) is much smaller than one coarse sample (32 µs) and can be ignored.
- In the implementation, the RDY interrupt pushes a pair "DMA write position, RDY sequence number" into a queue, and the window is closed when the coarse-ADC processing reaches that position.
  The ADS1115 read result carries the same RDY number and is matched to the closed window by that number. This prevents mixing up results and windows.

### 4.4 Non-blocking read

arduino-pico's `Wire` does not return until the read is complete. During that time jump detection stops and the V/OCT latency grows.
So **`Wire` is not used. pico-sdk's `hardware_i2c` (the DW_apb_i2c registers) is used directly, and the I2C FIFO lets the read proceed without waiting.**

Bus time of one read (400 kHz, datasheet figure 30 "Read Word" without the pointer write):

```
START + address 9 clocks + data 9 clocks × 2 + STOP ≈ 28 SCL ≈ 70 µs
```

1. At initialisation, set the target address (`IC_TAR` = 0x48) and set the ADS1115 pointer back to Conversion (4.1).
2. At a window boundary, write **two read commands** to `IC_DATA_CMD` (STOP on the second). They just go into the TX FIFO, so this returns immediately.
3. On later loop passes check `IC_RXFLR`, and **if both bytes are there, take them** (about 70 µs later). If not, do nothing and move on.
4. Aborts such as NACK are detected through TX_ABRT in `IC_RAW_INTR_STAT`, and that conversion is discarded (cleared with `IC_CLR_TX_ABRT`; 11.2).
5. Raw-sample processing (moving average, jump detection, output) never stops during a read. The statistics of the closed window are kept and evaluated when the result arrives (5.3).

- If the result has not arrived by the next RDY, that conversion is discarded (11.2).
- At most two pairs of read commands are queued, and the receive side **always takes bytes in pairs of two** (if a newer pair is waiting, the older one is discarded).
  Never drain the receive FIFO halfway: the byte pairs get out of step and, for example, 3.00 V reads as −5.60 V.

---

## 5. Sensor-fusion algorithm (V/OCT)

### 5.1 Principle

Decisions are always made by **comparing coarse values with each other**. Comparing the ADS1115 value with the coarse value directly would add
the calibration errors of both (offset, gain) to the threshold and force a large threshold.

### 5.2 State and variables

| Variable | Meaning |
|---|---|
| `state` | `ADS` or `FAST` |
| `p` | Coarse sample (updated every 32 µs). For window statistics |
| `s` | Moving average (updated every raw sample = every 2 µs). For jump detection and the FAST-mode output |
| `ref` | Average of `p` over the last clean conversion window. Reference for jump detection |
| `offset` | `to_coarse_scale(ads_val) - ref`. Correction used in FAST mode |
| `ads_val` | Latest clean ADS1115 value |
| `win_min` / `win_max` / `win_sum` / `win_cnt` | Statistics of `p` over the current conversion window. Reset at every RDY |
| `window_discarded` | Flag to discard this window (coarse ADC FIFO ERR, DMA overrun, etc.; 11.2) |
| `closed` | Copy of the closed window's statistics (`min` / `max` / `sum` / `cnt` / `discarded`). Kept while waiting for the I2C result |
| `i2c_pending` | Waiting for an I2C read result (4.4) |
| `jump_cnt` | Consecutive jump detections (**in raw samples**) |
| `dirty_cnt` | Consecutive non-clean windows while in ADS mode (**in windows**). Prevents getting stuck (5.3, 5.5) |

### 5.3 Pseudo-code

```c
// --- every raw sample (2 µs; the core1 loop processes the DMA's new samples in a batch) ---
s = moving_average_16(raw);                // 3.2

if (state == ADS) {
    if (abs(s - ref) > T_JUMP) {
        if (++jump_cnt >= JUMP_CONFIRM) {  // 17 raw samples = 34 µs. Rejects single spikes (5.4)
            state = FAST;
            jump_cnt = 0;
        }
    } else {
        jump_cnt = 0;
    }
}

// --- every coarse sample (16 raw samples = 32 µs) ---
win_min = min(win_min, p);
win_max = max(win_max, p);
win_sum += p;
win_cnt++;

// --- output (every core1 loop pass; within 10 µs) ---
out_raw = (state == ADS) ? ads_val : to_ads_scale(s + offset);
out = smooth(out_raw);                     // see 5.5
voct_cent = clamp(to_cent(out), 0, 6000);  // to the shared variable (10.2). Out-of-range values are clamped (chapter 6)

// --- window boundary (the raw-sample stream reaches the position recorded at RDY) ---
closed = { win_min, win_max, win_sum, win_cnt, window_discarded };
i2c_start_read();                          // 4.4. Only queues the commands in the FIFO and returns at once
i2c_pending = true;
win_min = +INF;  win_max = -INF;  win_sum = 0;  win_cnt = 0;   // do not carry p over from the previous window
window_discarded = false;

// Getting-stuck prevention (below). Decided from the coarse statistics alone, without waiting for the I2C result
if (state == ADS && !closed.discarded && closed.cnt > 0) {
    if ((closed.max - closed.min) >= T_STILL) {
        if (++dirty_cnt >= DIRTY_CONFIRM) {    // input that keeps moving slowly
            state = FAST;                      // switch with smoothing (5.5)
            dirty_cnt = 0;
        }
    } else {
        dirty_cnt = 0;
    }
}

// --- when the I2C result is complete (4.4; about 70 µs after the boundary) ---
ads_result = i2c_take_result();
i2c_pending = false;
cand_ref = closed.sum / closed.cnt;
if (!closed.discarded
    && closed.cnt > 0
    && (closed.max - closed.min) < T_STILL
    && abs(s - cand_ref) <= T_JUMP) {      // the input did not move while waiting for the result (below)
    ads_val = ads_result;
    ref     = cand_ref;
    offset  = to_coarse_scale(ads_result) - ref;
    state   = ADS;                         // returning is smoothed
    dirty_cnt = 0;
    calib_collect_pair(ads_result, ref);   // for the automatic relative calibration (9.3)
}
```

- There are about 36 coarse samples per window (1.163 ms / 32 µs; 33–40 with the data-rate spread).
- **Watch for a jump while waiting for the result.** If the note changes during the roughly 70 µs between the window closing and the I2C result arriving,
  the result (the previous note's value) would put the output back into ADS mode. To prevent this, when the result is accepted it is checked that
  `s` is not more than `T_JUMP` away from the closed window's average. If it is, the result is discarded
  (jump detection runs in parallel, so the switch to FAST mode still happens).
- **Prevent getting stuck in ADS mode.** When a window is not clean (its range is `T_STILL` or more) but `s` is still within `T_JUMP` of `ref`,
  neither jump detection nor the ADS1115 update works, and the output freezes at an old `ads_val`.
  - Example 1: the start of a slow glide. With `T_STILL` = 5 cent, any movement faster than about 4 cent/ms (one octave in about 0.3 s or less)
    freezes the output until the offset reaches `T_JUMP`.
  - Example 2: fast wobble or hum with an amplitude below `T_JUMP`. The output stays frozen.
  - So **when `DIRTY_CONFIRM` consecutive non-clean windows occur in ADS mode, switch to FAST mode.**
    FAST mode follows with the coarse value, and once the input stops, the next clean conversion returns to ADS mode.
  - Discarded windows (`discarded`) are not counted (and do not reset the count either).
- **Results that are far off are not used.** If the ADS1115 value is 200 cent or more away from the value converted from the coarse reading, the result is used neither for the output nor for the relative calibration (9.3)
  (`CFG_FUSION_PLAUSIBLE_CENT`). This guards against values corrupted by communication errors and the like.
  The rule in 5.1, "decide by comparing coarse values", concerns jump detection with its small thresholds. This threshold is far larger than the uncalibrated error (up to about 100 cent), so there is no conflict.

### 5.4 Constants

| Constant | Value | Rationale |
|---|---|---|
| `T_STILL` | **8 LSB (about 13 cent)** | In LSBs of the coarse sample `p` (theoretical 1.34 mV ≈ 1.6 cent). Margin over the range of `p` within a window at rest (at most 5.6 LSB; 13.1) |
| `T_JUMP` | **10 LSB (about 16 cent)** | `T_JUMP > T_STILL` is required. At rest there were no false triggers at thresholds of 5 LSB and above (13.1). Smaller values detect small pitch changes faster (10.4) |
| `MA_LEN` | 16 raw samples (32 µs) | Window length of the moving average `s`. Same as the number of samples averaged into `p` |
| `JUMP_CONFIRM` | **17 raw samples (34 µs)** | **`MA_LEN` + 1**. A single-sample spike can move the moving average for at most `MA_LEN` samples, so one more than that means a single spike never confirms a jump |
| `DIRTY_CONFIRM` | **2 windows** (about 2.3 ms) | After this many consecutive non-clean windows in ADS mode, switch to FAST (5.3). With 1, a single noisy window would enter FAST |
| `SMOOTH_MS` | **4 ms** | Time constant for removing the step at a mode change (5.5) |

### 5.5 Mode changes

- **How the step is removed**: the output step at the moment of switching (output before the switch − target after the switch) is kept as a "glide" and added to the target.
  The glide decays exponentially to 0 with a first-order time constant of `SMOOTH_MS`. The target keeps following `s` (FAST) or `ads_val` (ADS) during the decay.
- **ADS → FAST**: switch at once (speed is the point). The `offset` correction keeps the step small.
  Because `JUMP_CONFIRM` > `MA_LEN`, the moving-average window is already filled with post-change values when FAST is entered.
  **The output never passes through intermediate values; it goes straight to the new pitch.**
- **ADS → FAST (via `DIRTY_CONFIRM`)**: not urgent, so it is smoothed over `SMOOTH_MS`.
  The difference at this point (the old `ads_val` vs. the coarse value) is below `T_JUMP`. The target keeps following `s` during smoothing.
- **FAST → ADS**: a small step (a few cent) from the residual of `offset`. Smoothed over `SMOOTH_MS`.
- **Time spent in FAST** (for a single step of the input that then stays still; from the datasheet conversion period 1.057–1.292 ms):

  | Part | Shortest | Longest |
  |---|---|---|
  | Rest of the window containing the change | about 0 | 1.292 ms |
  | One following clean window | 1.057 ms | 1.292 ms |
  | I2C read | 0.07 ms | 0.07 ms |
  | **Until back in ADS mode** | **about 1.13 ms** | **about 2.65 ms** |

  After that the output is smoothed towards the ADS value over `SMOOTH_MS`.
- During glides and vibrato the windows are never clean, so the output enters FAST mode and stays there
  (fast movement enters through jump detection, slow movement through `DIRTY_CONFIRM`; 5.3).
  Once the input stops, the next clean conversion returns to ADS mode.
  Movement slow enough to keep the windows clean (less than `T_STILL` per window) is followed conversion by conversion in ADS mode.
- `offset` is renewed at every clean conversion (about every 1.16 ms).
  So slow drift of ADC_AVDD leaves almost no error in FAST mode.

### 5.6 Scale

- The internal representation is the **ADS1115 scale** (1 LSB = 187.5 µV).
- Coarse values are converted with `to_ads_scale()`. Its coefficients come from the automatic relative calibration in 9.3.
  The theoretical starting values are gain ≈ 7.16 (ADS LSB / coarse LSB = 1.343 mV / 0.1875 mV, with ADC_AVDD = 3.3 V and divider ratio 0.6) and offset 0.
- The ADS1115 absolute calibration (9.2) `V = a × code + b` is applied once, just before writing `voct_cent`.
  Inside the fusion (thresholds, `offset`, relative calibration) everything stays in raw ADS1115 units, so the absolute calibration does not affect it.
- V/OCT is linear at 1 V/oct, so **no logarithm is needed**. `cent = volt × 1200`.
- The value is passed to the audio side as a `float` in cent (chapter 10). The RP2350 has a single-precision FPU.

---

## 6. Unplugged V/OCT

**There is no mechanism to detect an unplugged input.** The node is only pulled down with 10 MΩ (2.2),
and the firmware does not distinguish plugged from unplugged; it always runs the same processing (fusion, output).
**V/OCT is rarely left unplugged, and not reaching 0 V when unplugged is accepted.**

- **Unplugged, it does not read 0 V but about 0.2–0.3 V (about 240–360 cent ≈ 3 semitones).**
  - In the equivalent circuit of the ADS1115 input (datasheet figure 26), the common-mode impedance ZCM goes not to GND but to **about 0.7 V**.
    In single-ended mode in the ±6.144 V range (AINN = GND), the node sees ZCM 10 MΩ to 0.7 V and the differential impedance ZDIFF 22 MΩ (Table 2) to GND.
  - With the 10 MΩ pull-down added, it settles at the following voltage (typical values).

    ```
    V = 0.7V × (1/10MΩ) ÷ (1/10MΩ + 1/22MΩ + 1/10MΩ) ≈ 0.29V ≈ 340cent
    ```

  - ZCM, ZDIFF and 0.7 V are all typical values; they vary between parts and with temperature. **The unplugged pitch is neither accurate nor stable.**
  - A smaller pull-down does not solve it (even 1 MΩ leaves about 60 mV and makes the error from the source's output impedance 10 times larger; 2.2).
- **Effect**: unplugged, the module sounds about 3 semitones above the pitch set with the knobs. Plugging in a cable that outputs 0 V brings it down by that amount.
  The 0–6000 cent clamp (5.3) does not help with this offset (the clamp only limits out-of-range values).
- **No effect when connected.** The current pulled towards 0.7 V (up to about 70 nA) gives only about 0.14 mV (about 0.2 cent) across the source's output impedance plus the protection resistor (about 2 kΩ).
- The measured unplugged reading was 0.2747 V (about 330 cent), as calculated (13.1).
- **The output `voct_cent` is clamped to 0–6000 cent** (5.3). Even if the error at a 0 V input goes negative, it just sticks at 0 and does no harm.
  The range of the shared variable (10.2) and the audio side's assumption (`wtvco_audio_process.md` §1.5) match this.
- **Right after unplugging**, the node keeps the previous voltage and settles to the voltage above over some tens of ms through (10 MΩ ∥ ADS1115 input) × 1 nF (τ ≈ 4 ms).
  The pitch moves like a short glide. Accepted.
- **A patch cable plugged in at one end only** (plugged into V/OCT with the other end floating) becomes an antenna on a high-impedance node
  and can pick up hum that wobbles the pitch. Unplugging the cable fixes it. Accepted.
- When plugging or unplugging moves the input a lot, FAST mode is entered as usual and ADS mode returns once the input is still. No special handling is needed.

---

## 7. PIC firmware

The PIC **only reads raw values and sends them**. It holds no calibration and receives nothing.
Calibration, scaling and smoothing are all done on the RP2350 (chapter 8).

### 7.1 Clock

- HFINTOSC 32 MHz (Fosc = 32 MHz).
- Its accuracy is **±2% over 0–60 °C** (±3% over 60–85 °C) per datasheet figure 39-6. No OSCTUNE trimming is done (see 7.4).
- 0.1 µF and 0.01 µF in parallel right at the supply pins (datasheet 39.4.2 note; a precondition for this accuracy).

### 7.2 ADC (ADCC)

| Item | Setting |
|---|---|
| Resolution | 12 bit |
| Reference | VREF+ = VDD, VREF− = VSS |
| Conversion clock | Fosc/32 (TAD = 1 µs) |
| Mode | Burst average (MD = 011; ADRPT / ADCRS set the number of averaged conversions) |
| Averages and right shift | See the table below. All inputs are read every frame |
| Order | **DEPTH → KNOB → FINE → COARSE → WT CV UNI → WT CV BI → FM CV UNI → FM CV BI** (below) |
| Acquisition time | A few µs or more after each channel switch (short is fine thanks to the capacitors on the wipers and CV inputs) |

**Why this order**

- **The four CVs come last, back to back**, so they are the freshest values when the frame is sent (10.4).
- **COARSE is placed so that it does not overlap the UART transmission.** A frame starts to be sent right after the CV conversions, so the transmission (about 180 µs, 7.4)
  overlaps the first conversions of the next frame (conversions take about 0.81–0.91 ms, and the transmission ends at about 0.99–1.09 ms).
  COARSE is the most noise-sensitive (1 LSB ≈ 1.76 cent), so it avoids conversions during transmission.
- With DEPTH, KNOB and FINE first (16 conversions in total, about 230–260 µs), COARSE is converted after the previous frame's transmission has finished.
  DEPTH and KNOB are low to medium accuracy and FINE is 1 LSB ≈ 0.05 cent, so overlapping the transmission does not matter for them.

**Averages and right shift (ADRPT / ADCRS)**

| Input | Averages (ADRPT) | ADCRS | Value sent (ADFLTR) | Why this many |
|---|---|---|---|---|
| FREQ1 COARSE | 16 | 2 | sum / 4 = average × 4 | The strong IIR on the RP2350 (75 ms) averages over tens of frames, so this is enough |
| FREQ1 FINE | 8 | 1 | sum / 2 = average × 4 | Same (IIR 20 ms) |
| WT1 KNOB | 4 | 0 | sum = average × 4 | Medium accuracy. IIR follows |
| FM1 DEPTH | 4 | 0 | sum = average × 4 | Low accuracy. IIR follows |
| WT1 CV UNI / BI | 4 / 4 | 0 | sum = average × 4 | Low accuracy |
| FM1 CV UNI / BI | 8 / 8 | 1 | sum / 2 = average × 4 | Noise is multiplied by DEPTH in the pitch, so more than WT CV |
| **Total** | **56** | | | |

- ADCRS is chosen to match the number of averages so that **every input is on the same "average × 4" scale (0–16380)**.
  The extra resolution of the average beyond 12 bits is kept and sent as 14 bits (7.3).
- The relatively small number of averages for the knobs does not hurt stability. How far the noise drops depends not on the PIC-side averaging but on
  the total number of samples inside the RP2350's IIR time constant (e.g. COARSE: 16 × 75 frames = 1200 samples).
  The PIC-side averaging is mainly an anti-aliasing step before decimating to 1 kHz.

**Conversion time within one frame** (datasheet chapter 39, ADC characteristics):

| Item | Value | Source |
|---|---|---|
| Conversion time TCNV | 14 TAD + 2 TCY = 14.25 µs (TAD = 1 µs, TCY = 125 ns) | AD22 |
| Acquisition time TACQ | 2 µs (typ.) | AD23 |
| Per conversion | 14.25 µs (no acquisition) to 16.25 µs (acquisition every time) | |
| 56 in total | **about 0.81 ms (acquisition only at channel switches) to about 0.91 ms (every time)** | |

- This **fits in one frame period (1 ms) with about 10% to spare**. The ADC and the transmit timer both derive from Fosc, so the HFINTOSC error does not change their ratio.
- The measured conversion time was about 0.90 ms, fitting in 1 ms with about 10% to spare (RA0 is High while converting, observed on an oscilloscope; 13.2).
  This matches the upper end of the calculation, so there is little room to add averages or channels.
- The averaged value is sent with 14 bits (average × 4; 7.3). What to do with the low bits is up to the RP2350.
- The EUSART sends in the background, so transmission can run in parallel with conversions.

### 7.3 Frame format

Each value is **14 bits** (the sent value: average × 4, 0–16380; 7.2). For synchronisation, **only the first byte has bit 7 = 1; all others have bit 7 = 0**.

| Byte | Content |
|---|---|
| 0 | Header: `0x80 \| seq` (seq = 0–127, incrementing) |
| 1, 2 | WT1 CV UNI: upper 7 bits, lower 7 bits |
| 3, 4 | WT1 CV BI |
| 5, 6 | FM1 CV UNI |
| 7, 8 | FM1 CV BI |
| 9, 10 | FREQ1 COARSE |
| 11, 12 | FREQ1 FINE |
| 13, 14 | WT1 KNOB |
| 15, 16 | FM1 DEPTH |
| 17 | Checksum: lower 7 bits of the sum of bytes 1–16 |

- 1 frame = 18 bytes.
- Value reconstruction: `raw = (upper byte << 7) | lower byte` (bit 7 of each byte is 0).
- **The send period is 1 ms (1 kHz)**, kept constant with a timer (0.98–1.02 ms with the HFINTOSC error).
- The RP2350 uses seq to count lost frames.

### 7.4 UART settings

| Item | Setting |
|---|---|
| Format | 8N1, TX only |
| Baud rate | 1 Mbps (BRG16 = 1, BRGH = 1, SPBRG = 7; 32 MHz / (4 × 8) = 1 MHz exactly) |
| Time to send one frame | 18 bytes × 10 bits = 180 bit times = **about 180 µs** |

- The division is exact, so the baud-rate generator has no rounding error. The only error source is the HFINTOSC accuracy (±2%, 7.1).
- The receiving RP2350 is crystal-based and makes 1 Mbps exactly (clk_peri 150 MHz / (16 × 1 MHz) = 9.375 = integer 9 + fraction 24/64).
- A ±2% offset is within the tolerance of 8N1. The measured period error was +0.03 to +0.19%, and no framing or checksum errors occurred (13.2).
  OSCTUNE trimming is not needed.
- Transmission is interrupt-driven, one byte at a time on the EUSART interrupt, and the next frame's conversions continue during it.

---

## 8. PIC reception and value conversion on the RP2350

### 8.1 Reception

- UART0, RX = GP1 (no TX pin assigned; 2.6), 1 Mbps, 8N1.
  It is initialised with pico-sdk's `hardware_uart`, and the receive FIFO is read directly from the core1 loop (no interrupt). This is light, and GP0 is not taken by the UART as it would be with `Serial1`.
- A byte with bit 7 = 1 marks the start of a frame; when 18 bytes are in, the checksum is verified.
- Frames with a bad checksum, a bit-7 = 1 byte in the middle and so on are **discarded entirely**.
- Per-byte error flags (framing, break, overrun), checksum errors and gaps in seq are each counted and shown by the USB console's `pic` command.
- Reception runs every 100 µs (10.1). Frames come every 1 ms and the receive FIFO (32 bytes) holds about 320 µs at 1 Mbps, so nothing is lost by this decimation.

### 8.2 When frames stop

| Time since the last good frame | Handling |
|---|---|
| Up to `T_PIC_HOLD` (100 ms) | **Hold the last values** |
| After `T_PIC_HOLD` | **Treat as 0** (table below) |

| Value | Output when treated as 0 |
|---|---|
| `coarse_cent` | **3600 cent (C3)**. COARSE alone becomes C3 rather than 0, to avoid dropping to C0 and producing a very low tone |
| `fine_cent` | 0 cent (as if the knob were centred) |
| `wt_cv` | 0.0 (no modulation from either input: UNI as 0 V, BI as 2.5 V. **Do not treat BI as raw value 0**, which would mean −0.5) |
| `wt_knob` | 0.0 |
| `fm_cent` | 0 cent (no modulation from either FM CV input, same as DEPTH 0. Do not treat BI as raw value 0) |

- Right after start-up there are no "last values", so everything is treated as 0 until the first good frame arrives.
- When good frames arrive again, normal operation resumes directly (through the smoothing).
- Thanks to this, **everything except the knobs (V/OCT, audio output) runs and can be tested without the PIC fitted.**
  It then plays at C3 + V/OCT (about 3 semitones higher if V/OCT is also unplugged, per chapter 6).

### 8.3 Knob conversion

| Knob | Conversion | Output |
|---|---|---|
| FREQ1 COARSE | `coarse_cent = raw / 16380 × 7200` | 0–7200 cent (C0–C6) |
| FREQ1 FINE | `fine_cent = (raw / 16380 − 0.5) × 200` | −100 to +100 cent |
| WT1 KNOB | `raw / 16380` | 0.0–1.0 |
| FM1 DEPTH | `k = raw / 16380`, `fm_depth = 2400 × k²` | 0–2400 cent/V (square curve) |

- Ratiometric, so **no calibration is needed**.
- Out-of-range values are clamped to each range.

**FREQ1 COARSE (stability first)**

One ADC LSB ≈ 1.76 cent (one sent-value LSB ≈ 0.44 cent), so a 1-LSB wobble of the ADC is already audible as a pitch wobble. Processing order:

1. A **strong first-order IIR** (time constant 75 ms) removes the noise.
2. A **hysteresis (dead band)** is applied: the output is updated only when it differs from the input by more than `COARSE_HYST` (4.5 cent ≈ 10 LSB of the sent value).
   - The dead band leaves an error of a few cent, which FINE is meant to correct.
   - While the knob is being turned it moves a lot, so the dead band is not noticeable.
3. On each update the output goes through smoothing (10.3), so the steps are not audible.

**FREQ1 FINE**

One ADC LSB ≈ 0.05 cent (0.012 cent per sent-value LSB) is fine enough that wobble rarely matters. A first-order IIR (20 ms) and a small hysteresis (0.1 cent) are enough.

**WT1 KNOB (medium accuracy)**

A first-order IIR (15 ms) and a larger hysteresis (0.004, where the full WT range is 1.0).

**FM1 DEPTH (low accuracy)**

- A linear curve would squeeze the 10–50 cent/V often used for vibrato into the bottom 2% of the knob, so a **square curve** is used.

  | Knob position k | DEPTH |
  |---|---|
  | 0.1 | 24 cent/V |
  | 0.3 | 216 cent/V |
  | 0.5 | 600 cent/V |
  | about 0.71 | 1200 cent/V |
  | 1.0 | 2400 cent/V |

- The IIR and hysteresis are applied to `k` (before the curve). A first-order IIR (20 ms) and a small hysteresis (0.001 on k).
- One sent-value LSB changes DEPTH by at most about 0.3 cent/V (near k = 1).

**How the time constants and dead bands were chosen**

The values above were chosen from the measured noise (13.2) and by listening. The IIR is updated every frame (1 ms) as `y += a (x − y)`, `a = 1 − exp(−1 ms / τ)`.
The USB console's `tune` command changes them without flashing (in RAM only), for trying other values.

### 8.4 Common CV conversion (WT CV, FM CV)

For both WT CV and FM CV, the UNI and BI sent values are converted to voltages (modulation amounts) with the formulas below and summed. **Both inputs have the same sensitivity.**

```
v_uni = (raw_uni − zero_uni) × gain_uni       // V. Modulation from 0 V
v_bi  = (raw_bi  − zero_bi ) × gain_bi        // V. Modulation from 2.5 V
cv_v  = clamp(v_uni + v_bi, −2.5, +7.5)       // V
```

| Coefficient | Meaning | Default (uncalibrated) | Determined by |
|---|---|---|---|
| `zero_uni` / `zero_bi` | Zero point (sent value treated as no modulation) | UNI: 0, BI: 8190 (mid-scale = 2.5 V) | Zero-point calibration (9.4) |
| `gain_uni` / `gain_bi` | Volts per sent-value LSB | 5 V / 16380 (≈ 0.305 mV) | Gain calibration (9.4, optional) |

- There is one set of coefficients for each of the four inputs (WT CV UNI, BI, FM CV UNI, BI).
- **Individual inputs are not clamped**, to keep BI's negative side, and because clipping the noise on one side when unplugged would shift its average.
  Only the summed value is limited to −2.5 to +7.5 V.
- The roughly 2% gain error from input loading would be absorbed by the gain calibration (2.5).
- Unplugged, UNI sits at 0 V (pull-down) and BI at 2.5 V (bias); after subtracting the zero points both give no modulation.
- BI's bias (VDD / 2) is the mid-scale of the sent value, so even with the default zero point (8190) the unplugged value is almost 0 regardless of the bus voltage.
  The remaining offsets (resistor mismatch, ADC offset, leakage) are cancelled by the zero-point calibration.

### 8.5 WT CV conversion

1. Sum WT CV UNI and BI as in 8.4 to get `cv_v_wt` (−2.5 to +7.5 V).
2. Normalise: `wt_cv = cv_v_wt / 5.0` (1 V = 0.2, range −0.5 to +1.5).
3. Smooth with a first-order IIR (time constant 2 ms, fc ≈ 80 Hz; removes the 1 ms steps and noise without dulling medium-speed modulation).
4. **Do not clamp here.** Clamping to 0–1 would remove BI's negative modulation.
   The clamp is applied once, at the last stage on the audio side (10.3).

- With both unplugged, `wt_cv = 0`.
- The audio side combines them as `wt = clamp(wt_knob + wt_cv, 0, 1)` (10.3).

### 8.6 FM CV conversion

1. Sum FM CV UNI and BI as in 8.4 to get `fm_v` (−2.5 to +7.5 V).
2. Smooth `fm_v` with a first-order IIR (the same 2 ms as WT CV). The noise is multiplied by DEPTH in the pitch,
   but with DEPTH centred and CV unplugged the wobble is ±0.3 cent, which is inaudible (13.2).
3. **`fm_cent = fm_depth × fm_v`** (`fm_depth` from 8.3; 0–2400 cent/V). Range −6000 to +18000 cent.
4. **Do not clamp here.** The pitch is clamped on the audio side (10.3).

- With both unplugged, `fm_cent = 0`. Also 0 when DEPTH = 0.
- **For slow modulation only** (2.5). The accuracy is accepted as low; accurate 1 V/oct tracking is not expected.
- The audio side adds `fm_cent` to the pitch sum (1.3, 10.3).

---

## 9. Calibration

### 9.1 Overview

The RP2350-Zero's USB CDC talks directly to a terminal on a PC (the USB console; 10.1).
Commands are processed in the core1 loop and do not stop the sound. **There is no calibration on the PIC side.**

| Command | Purpose |
|---|---|
| `calmode on` / `off` | Enter / leave calibration mode. The commands below work only in calibration mode |
| `vcal <mV>` | Enter the measured voltage applied to V/OCT and record one point (9.2) |
| `vcal fit` / `clear` / `list` | Compute the coefficients from the recorded points / forget the points / list them |
| `czero` | Measure the CV inputs' zero points (9.4) |
| `calsave` | Save all calibration values to flash (9.5) |
| `calshow` / `caldefault` | Show the current values / go back to the theoretical values (not saved) |

### 9.2 ADS1115 absolute calibration (manual, with a DMM)

The reference voltage source need not be accurate. **It only has to be stable** (the value measured with the DMM is what gets entered).

1. Enter calibration mode with `calmode on`.
2. Apply a voltage to the V/OCT jack. Shorting the jack to GND gives the 0 V point.
3. Measure that voltage with the DMM.
4. Enter the measured value (mV) with `vcal <mV>`. The RP2350 averages 0.5 s of ADS1115 values and records the point.
   If the value wobbles during that time (the input is not stable), the point is discarded and an error is shown.
5. Repeat for 2 or more points (as far apart as possible, e.g. 1 V and 4 V).
6. `vcal fit` computes gain and offset by least squares. The result is rejected if there are fewer than 2 points, if the points are not at least 1 V apart,
   or if the gain is 2% or more off the theoretical value (most likely a misreading or a typing mistake).
7. Save with `calsave` (9.5) and check with `calshow`.

For reference, the ADS1115's own errors are gain error typ. 0.01%, max 0.15%, and single-ended offset typ. ±3 LSB (datasheet electrical characteristics, at FS = ±2.048 V).
The maximum gain error of 0.15% is about 9 cent at 5 V (6000 cent), so calibration is required.

### 9.3 Coarse ADC ↔ ADS1115 relative calibration (automatic)

In ADS mode, pairs of `ads_result` and `ref` for the same voltage are constantly available.
Accumulating them and running least squares **keeps the conversion coefficients (gain and offset of `to_ads_scale`) up to date without any external equipment.**

- **Collecting pairs**: one pair (average of the window's coarse values, ADS1115 value) for every clean conversion accepted by the fusion (5.3).
- **Bins**: accuracy suffers if the voltages are lopsided, so the coarse range (0–4095) is split into 8 bins and pairs are averaged per bin
  (plain average up to 4096 pairs, then an exponential average with weight 1/4096).
- **Solving**: each bin with 64 or more pairs counts as one point for least squares.
  If the outermost bins are at least 600 LSB (about 1.3 V) apart, gain and offset are updated; otherwise only the offset.
  A gain more than ±20% off the theoretical value is rejected. It is recomputed every 64 pairs (about every 74 ms).
- The result is updated in RAM continuously. **It is not saved to flash automatically** (9.5).
- Right after start-up the previously saved values are used, or the theoretical values (5.6) if there are none.
- On the hardware it converged in about 15 seconds (13.1).

### 9.4 CV input (WT CV, FM CV) calibration

#### Zero-point calibration (recommended; no DMM needed)

The sent value with the input unplugged is measured and saved as the "no modulation" point (`zero_*`, 8.4).
This cancels the unplugged offsets from the BI bias resistor mismatch (up to about 25 mV), the PIC ADC's offset and the pin leakage.
For FM CV the offset is multiplied by DEPTH in the pitch (25 mV ≈ 60 cent at maximum DEPTH), so **it is effectively required for FM CV**. WT CV is done in the same step.

1. Enter calibration mode with `calmode on`.
2. **Unplug everything from all CV inputs (WT CV UNI, BI, FM CV UNI, BI).** V/OCT and the knobs do not matter.
3. Send `czero`. The RP2350 averages the sent values from the PIC for 1 second (about 1000 frames) per input.
4. Plausibility check: if UNI is ±400 LSB (≈ ±120 mV) or more away from 0, or BI from 8190,
   that input is not accepted and an error is shown (something may be plugged in, or there is a wiring fault).
5. The four measured values and their differences from the defaults (in mV) are shown.
6. Save with `calsave` (9.5).

- Redo it after replacing the board or the PIC. It shifts a little with temperature, which is accepted for a low-accuracy input.
- UNI sits near 0 V because of the pull-down, so if the ADC offset is negative the sent value sticks at 0 and does not appear in the zero point (a few LSB; accepted).
- The leakage offset appears only when unplugged and almost disappears with the low impedance of a connected source.
  If the zero point includes leakage, a connected input is offset by that amount in the opposite direction (typ. 0.5 mV ≈ 1.2 cent at maximum DEPTH, worst case 12.5 mV). Accepted.

#### Gain calibration (optional, with a DMM)

With the same procedure as 9.2, apply 2 voltages (e.g. 0.2 V and 4.8 V) to each input and determine the gain (`gain_*`) **per input**.

- Gain = difference of the 2 voltages / difference of the sent values. The zero point comes from the zero-point calibration and is not determined here
  (BI can also be computed from the difference of 2 points alone, so no 2.5 V reference is needed).
- The RP2350 computes it from the sent values received from the PIC.
- **The current firmware has no command for gain calibration.** The gain runs at the theoretical value (5 V / 16380).
  For a low-accuracy input the roughly 2% gain error (2.5) is accepted. The saved data has fields for the gain, so a command can be added later.

### 9.5 Saving

The RP2350 has no EEPROM, so values are saved to flash (arduino-pico's EEPROM library; one flash sector).

- **While flash is erased or written, XIP stops and both cores wait. Doing it while playing interrupts the sound.**
- **Before saving, the audio side is asked to mute** (`audio_mute_req` / `audio_mute_ack` in 10.2).
  The audio output uses DMA reading a ring in SRAM continuously, so saving without muting would repeat the rest of the ring and produce a buzz.
  1. Set `audio_mute_req = 1`.
  2. Wait for `audio_mute_ack == 1` (the audio side fades out and fills the ring with 0). If it does not come within 50 ms, cancel the save and show an error.
  3. Write the flash.
  4. Set `audio_mute_req = 0` (the audio side returns to normal).
- Saving happens **only when a command is run in calibration mode** (results of the manual and zero-point calibrations, and the current automatic relative calibration).
- There is **no** automatic saving during normal operation.

| Item | Content |
|---|---|
| ADS1115 | Gain, offset (manual calibration) |
| Coarse ADC | Gain and offset to the ADS1115 scale (automatic relative calibration) |
| WT1 CV UNI, BI | Zero point per input (zero-point calibration), gain (currently theoretical) |
| FM1 CV UNI, BI | Zero point per input (zero-point calibration), gain (currently theoretical) |
| Integrity | Identifier and checksum (FNV-1a); if corrupted, fall back to the theoretical defaults |

---

## 10. Interface to the audio side (core0)

### 10.1 Core split

| Core | Role |
|---|---|
| core0 | Sound generation, I2S output (PCM5102A; own PIO + DMA ring). **Dedicated to this loop; no other processing or interrupts** |
| core1 | Coarse ADC + DMA, I2C (ADS1115), fusion, PIC reception, knob / WT CV / FM CV conversion, calibration commands |

**Timing constraints on the core1 loop**

- **Keep one loop pass within 10 µs.** The V/OCT output is updated once per pass, so the length of a pass adds directly to the latency.
- No blocking operations: `Wire` (4.4), `delay()` and the like.
- **Decimate the work.** V/OCT (coarse ADC, ADS1115, fusion) is processed every pass. PIC reception and value conversion run every 100 µs, the USB console every 250 µs,
  and at most one command is executed per pass. Heavy work (least squares and so on) is split into slices over several passes.
- **The USB console does not use arduino-pico's `Serial` on core1.** Every `Serial` method waits for the USB lock and then runs `tud_task()`,
  which stalled core1 for up to 2.5 ms. Instead, the USB lock is only used "if it can be taken", and the CDC FIFOs are read and written directly
  (`tud_task()` is left to arduino-pico, which runs it on core0 every 1 ms).
- **core1's SysTick interrupt is disabled** (`core1_disable_systick = true`). arduino-pico installs it only for `rp2040.getCycleCount()`,
  and it is not needed within the 10 µs budget.
- The input section's interrupts (RDY GPIO interrupt, DMA completion, etc.) are **registered on core1**. core0 is dedicated to audio, and being stopped by an interrupt makes the output underrun (`wtvco_audio_process.md` §1.11).
- Measured: 97% of passes within 5 µs, 99.55% within 10 µs (13.1).

### 10.2 Shared variables

core1 writes and core0 reads (except `audio_mute_ack`, which goes the other way). All are single 32-bit variables (`volatile float` / `volatile uint32_t`),
each read or written in one access (no locking needed).

| Variable | Type | Unit, range | Updated |
|---|---|---|---|
| `voct_cent` | float | 0–6000 cent (clamped. Not 0 when unplugged; chapter 6) | Every core1 loop pass (within 10 µs) |
| `coarse_cent` | float | 0–7200 cent (from C0) | On update (when the hysteresis is exceeded) |
| `fine_cent` | float | −100 to +100 cent | Every 1 ms |
| `wt_cv` | float | −0.5 to +1.5 (UNI + BI summed, **not clamped**; 8.5) | Every 1 ms |
| `wt_knob` | float | 0.0–1.0 | Every 1 ms |
| `fm_cent` | float | −6000 to +18000 cent (FM CV × DEPTH, **not clamped**; 8.6) | Every 1 ms |
| `status` | uint32_t | Bit flags (FAST/ADS mode, ADS degraded, PIC lost, etc.) | Any time |
| `audio_mute_req` | uint32_t | 0 / 1. Mute request before a flash save (9.5) | Only when saving |
| `audio_mute_ack` | uint32_t | 0 / 1. **Written by core0, read by core1.** Mute acknowledgement | Only when saving |

### 10.3 How the audio side uses them

- The pitch is computed with the formula in 1.3.
- The WT position is `wt = clamp(wt_knob + wt_cv, 0, 1)` (KNOB is the base, CV the modulation; UNI upwards, BI both ways).
  **The clamp is applied here, once** (the input section passes the values unclamped; 8.5, 8.6). Details are on the audio side.
- For FM, `fm_cent` is added to the pitch sum (1.3). The pitch is clamped on the audio side.
  `fm_cent` is smoothed **separately** from the knob pitch (COARSE + FINE), so that the long knob time constant does not dull LFO modulation.
- `coarse_cent`, `fine_cent`, `wt_knob`, `wt_cv` and `fm_cent` change in steps, so **interpolate them per sample or pass them through a first-order low-pass before use**
  (used as is, they cause zipper noise).
- The audio side **reads `voct_cent` every sample** (`wtvco_audio_process.md` §1.5).
  The V/OCT response is therefore not tied to the audio block length.

### 10.4 Latency budget

**V/OCT (from note-on until core0 has the value)**

For pitch changes of a semitone or more.

| Part | Worst | Typical | Basis |
|---|---|---|---|
| Raw sample (every 2 µs) | 2 µs | 1 µs | 500 kSPS, no round robin (3.1) |
| Until the moving average `s` exceeds `T_JUMP` | 6 µs | 2 µs | Below |
| `JUMP_CONFIRM` 17 raw samples | 34 µs | 34 µs | 5.4 |
| One core1 loop pass (output update interval) | 10 µs | 5 µs | 10.1 |
| **Total** | **about 52 µs** | **about 42 µs** | |

- The time until the moving average exceeds `T_JUMP` depends on the size of the change. With `T_JUMP` = 15 cent,
  a semitone (100 cent) takes 3 raw samples (6 µs), and two semitones or more take 1–2 raw samples.
  Smaller changes take longer (e.g. 20 cent takes 13 raw samples = 26 µs).
- The input circuit's time constants (1 kΩ × 1 nF = 1 µs; the divider after the buffer is 6 kΩ, low enough for ADC sampling) are negligible compared with the table.

The audio side (output ring, PCM5102A, output buffer) adds at most about 167 µs, typically about 162 µs, so from note-on to the audio output (OUT1) the
**total is at most about 219 µs, typically about 204 µs** (`wtvco_audio_process.md` §1.11). Against the 500 µs requirement this leaves at least about 281 µs.
The measured value was about 210 µs (13.1).

**Return to high accuracy (ADS mode) for V/OCT**

As in the table in 5.5, the output returns to the ADS1115 value about 1.13–2.65 ms after a step, then is smoothed over `SMOOTH_MS`.
Accuracy is coarse (a few to a dozen or so cent) during that time, but there is no large pitch error.

**WT CV and FM CV (from an input change to the shared variable)**

Values for the configuration in 7.2 (frame period 1 ms, 56 conversions).

| Part | Worst | Typical | Basis |
|---|---|---|---|
| Until the next CV conversion | 1.02 ms | 0.5 ms | Frame period 1 ms (HFINTOSC ±2%). CVs are converted once per frame |
| CV burst conversions (WT UNI 4 + BI 4 + FM UNI 8 + BI 8) | 0.39 ms | 0.35 ms | 7.2 (14.25–16.25 µs × 24). Worst for WT CV UNI, converted first |
| UART transmission (18 bytes) | 0.18 ms | 0.18 ms | 7.4. The four CVs are converted last and sent right after |
| RP2350 reception → shared variable | 0.01 ms | 0.005 ms | One core1 loop pass |
| **Total** | **about 1.6 ms** | **about 1.0 ms** | + input RC (τ ≈ 0.2 ms, 2.5) + first-order IIR (8.5, 8.6) |

The audio side adds the control period (≤ 167 µs) + output (about 167 µs) + smoothing (2 ms). Plenty for medium-speed requirements.

**Knobs**

Same path as the CVs, longer because they are converted earlier (worst total about 2.1 ms for DEPTH, converted first, and about 1.9 ms for COARSE).
The actual response is set by the IIR time constants (COARSE 75 ms etc.; 8.3); the path delay is negligible.

---

## 11. Start-up and error handling

### 11.1 Start-up sequence (core1)

1. Configure the coarse ADC and start continuous sampling into the DMA ring (first get to a state that can produce sound)
2. Read the calibration values from flash. Defaults if the checksum does not match
3. Enable GP1's internal pull-up (2.5) and start receiving on UART0 (`hardware_uart`, RX = GP1; 2.6, 8.1). Until the first good frame, the PIC values are the defaults in 8.2 (COARSE = C3)
4. Initialise I2C0 with `hardware_i2c` (not `Wire`; 4.4) and write Config, Hi_thresh and Lo_thresh to the ADS1115
   (blocking writes are fine at start-up only)
5. Read back the Config and check it (**exclude the OS bit (bit 15), which reflects the conversion status**; datasheet Table 9). Set the pointer back to Conversion
6. Enable the RDY (GP10) GPIO interrupt **from core1**
7. Run in **FAST mode** until the first clean conversion arrives

### 11.2 Error handling

| Event | Response |
|---|---|
| ADS1115 does not respond (at start-up) | **Fall back to the coarse ADC alone** and set a flag in `status`. The sound continues. Retry the initialisation every second |
| No RDY for 50 ms | Reinitialise the ADS1115. If that fails, fall back and retry every second |
| I2C bus hang (SDA stuck Low) | Toggle SCL 9 times to release it → STOP → reinitialise |
| The read misses the next RDY | Discard that conversion. Also reset the window statistics and move on to the next window |
| The I2C read is aborted (TX_ABRT, NACK, etc.) | Discard that conversion (4.4). Clear `TX_ABRT` and wait for the next conversion. If it persists, handle it like "no RDY" |
| The input moved while waiting for the I2C result | Discard that result (5.3). Jump detection runs in parallel |
| Coarse ADC FIFO ERR / DMA overrun | Replace the sample with the previous value (3.2). Discard any window whose statistics are broken |
| A PIC frame is corrupted | Discard that frame (8.1). Count the error |
| PIC frames stop | As in 8.2 (hold the last values → treat as 0) |

**Keeping the sound going even when degraded is the top priority.** Nothing in the implementation may make it go silent.

- **Reinitialisation does not stall core1.** The four writes (Config, Hi_thresh, Lo_thresh, pointer) are only queued in the I2C FIFO
  (the configuration is read back only at start-up).
- While degraded or reinitialising, the fusion is put in FAST mode and runs on the coarse ADC. The conversion results right after a reinitialisation are not trusted, so the first two are discarded.
- On the hardware, recovery about 51 ms after RDY stopped, and AE bus power-off → fallback → automatic recovery when power returned, were both confirmed (13.3).

---

## 12. Known error sources

| Source | Size | Handling |
|---|---|---|
| Spread of the source module's output impedance (V/OCT) | About 0.01–0.02% against the ADS1115 input impedance (10 MΩ common-mode, ±6.144 V range) | Negligible (under 1 cent) |
| ADS1115 gain error and offset | Max 0.15% (≈ 9 cent at 5 V), typ. ±3 LSB | Absorbed by the manual calibration (9.2) |
| Pull towards GND by the 10 MΩ pull-down | About 0.02% | Absorbed by calibration |
| Unplugged V/OCT reading | About 0.2–0.3 V (about 240–360 cent), because the ADS1115 input pulls towards about 0.7 V. Varies between parts and with temperature | **Accepted** (hardly ever used unplugged). The effect when connected is about 0.2 cent, negligible. Measured 0.2747 V (chapter 6, 13.1) |
| Noise and drift of the coarse ADC reference (ADC_AVDD) | ADC_AVDD tied directly to 3V3 without a filter (2.6). Standard deviation of `p` 0.63–0.78 LSB (13.1) | Affects FAST mode only. Drift is removed by the `offset` updates |
| Coarse ADC divider resistor error | Gain error of the resistor tolerance | Absorbed by the automatic relative calibration |
| ADS1115 data-rate spread | ±10% | Harmless, because everything is referenced to RDY |
| Absolute accuracy in FAST mode | A few to a dozen or so cent | Only for about 1.1–2.7 ms after a step (5.5). Lasts while the input keeps moving, e.g. during a glide. Accepted |
| Variation of the knob supply (5 V bus) | — | Cancelled because the knobs are ratiometric |
| FREQ1 COARSE resolution and noise | One ADC LSB ≈ 1.76 cent (the 14-bit sent value has 0.44-cent steps) | PIC-side averaging, strong IIR, hysteresis (8.3). The few cent left are corrected with FINE |
| FREQ1 COARSE wiper noise | Depends on the potentiometer | Use a good part. Increase the wiper capacitor (2.5) |
| FREQ1 FINE resolution | One ADC LSB ≈ 0.05 cent | No problem |
| 5 V bus variation for the CVs (WT, FM) | The bus variation ratio | Accepted for low-accuracy inputs |
| Gain error of the CVs (WT, FM) from input loading (UNI pull-down, BI bias) | About 2% (UNI around 0 V, BI around 2.5 V) | Would be absorbed by the gain calibration (optional). BI's mid-point does not move |
| CV BI bias resistor mismatch | Up to about 25 mV with two 1% parts (WT 0.005; FM about 60 cent at maximum DEPTH) | Cancelled by the zero-point calibration (9.4) |
| PIC pin leakage (CV unplugged) | Worst 12.5 mV, typ. 0.5 mV | Cancelled by the zero-point calibration. When connected, about the same remains in the opposite direction (typ. 1.2 cent for FM; 9.4) |
| Charge sharing with CHOLD (CV) | Worst about 1.4 mV (first conversion after a channel switch) | Kept small by the 100 nF at the pin. Negligible (2.5) |
| FM CV resolution and noise | One ADC LSB ≈ 2.9 cent, one sent-value LSB ≈ 0.73 cent (maximum DEPTH) | PIC-side averaging (8 conversions) and IIR (8.6). Accepted as low accuracy |
| FM sensitivity (DEPTH) accuracy | About 2% gain error + bus variation + knob position | Accepted as low accuracy. Accurate 1 V/oct tracking is not expected |

---

## 13. Verification results

Measurements on two prototype units and results of simulations and tests on a PC.
Values are from the first unit unless stated otherwise.

### 13.1 V/OCT

| Item | Result |
|---|---|
| Coarse ADC | 500 kSPS. Standard deviation of the coarse sample `p` (16 averaged) 0.63–0.78 LSB (1.0–1.3 cent). With I2S stopped it is 0.48 LSB, so the contribution of BCK (GP14) is small |
| Coarse ADC vs. ADS1115 | Coarse ADC reading (theoretical conversion) ≈ 0.979–0.982 × V + about 20 mV. Absorbed by the automatic relative calibration (9.3). Per-bin residuals ±2–4 cent (non-linearity of the RP2350 ADC) |
| Convergence of the relative calibration | Converged in about 15 seconds; the error of the FAST-mode correction went from 76 cent (theoretical values) to ±4 cent |
| ADS1115 | About 860 conversions/s. RDY period 1148–1171 µs (differs between parts). Noise with a source connected: standard deviation 0.13 cent, under 1 cent peak-to-peak |
| ADS1115 absolute calibration (9.2) | Calibrated at about 1 V and about 4 V. Gain vs. theoretical: unit 1 +0.084%, unit 2 +0.130%. After calibration a 4.001 V input read 4801.04 cent (theoretical 4801.2 cent) |
| Unplugged V/OCT (chapter 6) | 0.2747 V on the ADS1115 (about 330 cent), 0.288 V on the coarse ADC (theoretical conversion). As calculated (about 0.29 V). Before the ADS1115 is configured (its power-on default) it is about 0.02 V. Unplugged ADS1115 noise: standard deviation 2.3 LSB (0.5 cent), 15 LSB (3.4 cent) peak-to-peak, larger than with a source connected (0.58 LSB) because the node is high-impedance |
| Basis for `T_STILL` / `T_JUMP` (5.4) | Unplugged (near 0 V), 26,000 windows of 36 `p` each had a range of at most 5.6 LSB. Simulated jump detection gave 0 false triggers at thresholds of 5 LSB and above (7 in 30 s at 4 LSB). Noise is about 15% larger at mid-range voltages. Hence `T_STILL` = 8 LSB, `T_JUMP` = 10 LSB |
| False triggers at static voltages | 1.04 V / 2.51 V / 3.99 V for 30 s each (about 26,000 windows). No switches to FAST by jump detection, 0–1 non-clean windows, stayed in ADS mode. `voct_cent` varied by 0.2–0.5 cent. Unit 2 also 0 at 1 / 2.5 / 4 V |
| Fusion response (simulation) | A single-sample +500 LSB spike does not enter FAST. Steps of a semitone or more reach the new pitch in 40 µs. An ±8 cent, 400 Hz wobble (below `T_JUMP`, but it makes windows dirty) moves to FAST and does not get stuck (5.3) |
| Return to ADS mode | About 1.8 ms after a step (within the 1.13–2.65 ms estimated in 5.5) |
| Note-on → OUT1 | **About 210 µs**, measured on an oscilloscope triggered by the moment the CV source switched steps (estimate at most 219 µs, requirement within 500 µs; 10.4) |
| One core1 loop pass | 97% within 5 µs, 99.55% within 10 µs, maximum about 30 µs (passes that do not run a console command) |

### 13.2 PIC, knobs, CV

| Item | Result |
|---|---|
| PIC conversion time per frame (7.2) | Measured with RA0 High while converting, on an oscilloscope. Conversion about 0.90 ms, spare 95–100 µs, period 1 ms. Matches the upper end of the calculation (0.91 ms) |
| PIC frame period and UART (7.4) | 1000.3–1001.9 frames/s (HFINTOSC error +0.03 to +0.19%). No checksum, sync or framing errors in 30 s. Receive FIFO overruns on the RP2350 happen only when a heavy USB console command stops core1 for 320 µs or more (not during normal playing) |
| Knob noise (centred, 12 s) | Peak-to-peak of the sent value: COARSE 1 LSB, FINE 4 LSB, KNOB 2 LSB, DEPTH 4 LSB. The converted COARSE, FINE and KNOB did not move at all in 12 seconds (8.3) |
| CV zero points (9.4) | Unit 1: WT UNI 25.1, WT BI 8148.0, FM UNI 24.0, FM BI 8157.5 (offsets from the defaults +7.7 / −12.8 / +7.3 / −9.9 mV). Unit 2: 24.4, 8150.8, 24.0, 8163.7. After calibration, with CVs unplugged, `wt_cv` = 0.000 and `fm_cent` = −0.02 cent |
| FM wobble | ±0.3 cent with DEPTH centred and CV unplugged |
| CV charge sharing (2.5) | Unplugged WT UNI moves by 0–56 LSB (up to 17 mV) depending on the voltage of COARSE, converted just before it. This is the estimate in 2.5 (1.4 mV per conversion) adding up over the 10 ms unplugged time constant. About 0.003 in WT, and it disappears when a cable is plugged in |
| PIC frames stopping (8.2) | Simulation confirmed hold → defaults → recovery. On the hardware, switching the PIC's power off and on again also recovered automatically |

### 13.3 Power and fault handling

| Item | Result |
|---|---|
| Power-up and power-down (2.4) | The 5 V bus and the RP2350's 3V3 observed together on an oscilloscope. At power-up IOVDD was 2.5 V or more when the bus crossed 3.63 V; at power-down the bus was below 3.63 V when IOVDD dropped below 2.5 V. Both satisfy the conditions (USB not connected) |
| AE bus power-off with USB plugged in (2.1) | The RP2350 kept running, detected the loss of the ADS1115 and the PIC, and fell back. It recovered automatically when power returned |
| ADS1115 RDY stopped (11.2) | With a test command that stops RDY, it reinitialised and recovered after about 51 ms |
| Flash save (9.5) | Writing takes 25–34 ms. The sound stops during it and about 30 PIC frames are lost (as expected) |
| Recovery from a stuck I2C bus (11.2) | The bus-release code ran during the AE bus power-off test. A test holding SDA Low has not been done, because the board has no SDA test point |

### 13.4 Listening

The knob time constants and dead bands (8.3), the CV smoothing (8.5, 8.6), the fusion transitions (5.5),
and the audio side's smoothing and mipmap boundaries (`wtvco_audio_process.md` §1.7, §1.10) were checked by playing, found free of artefacts, and kept at their defaults.
To try other values, use the USB console's `tune` command (it changes them in RAM only; nothing is saved).

### 13.5 Host-side tests

The tests in `firmware/wtvco/tests/` run on a PC (`run_tests.sh`). The three related to the input section all pass.

| Test | Content |
|---|---|
| `test_fusion` | Simulation of the fusion (chapter 5). It models the coarse ADC (500 kSPS, with the measured gain error, offset and noise), ADS1115 windows whose results arrive 70 µs late, and a core1 loop that publishes the output every 10 µs. Checks the state transitions and output for static input, steps, single spikes, glitches, glides, hum, vibrato, and a step while an I2C result is pending |
| `test_calib` | The automatic relative calibration (9.3). Per-bin collection, switching between offset-only and gain-plus-offset updates, error over 0–5 V |
| `test_pic_input` | Conversion of the PIC values (8.2–8.6). Hold and defaults when frames stop, knob curves, IIR and dead bands, CV summing and zero points |
