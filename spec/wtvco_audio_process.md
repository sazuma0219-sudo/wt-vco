# WT VCO technical specification: audio processing

English | [日本語](wtvco_audio_process.ja.md)

This document describes the design of the **audio processing** of WT VCO, a wavetable VCO module for AE Modular (RP2350-Zero, **1 channel**), the reasons behind it, and the verification results.

- **The agreements with the input section (shared variables, core split, pins) are written in `wtvco_input_spec.md` (CV and knob inputs).**
- The latency budget (§1.11) is based on the datasheet values of the ADS1115 (TI SBAS444B) and the PCM5102A (TI SLAS859C).
- The schematic and BOM are in `../hardware/`, the firmware is in `../firmware/`.

---

## 1.1 Scope

### Covered here
- A **1-channel** wavetable oscillator running on the RP2350 (RP2350-Zero)
- Audio output to the PCM5102A over I2S (**own PIO program + DMA ring**; §1.11) and the output buffer after it (§1.3)
- Reading the **shared variables** written by the input section (core1) to set the pitch and wavetable position
- Offline generator scripts for the wavetable and the exp2 LUT

### Not covered here
- The input section (coarse ADC + DMA, ADS1115, sensor fusion, PIC reception, knob / WT CV / FM CV conversion, calibration) and the PIC firmware
  → `wtvco_input_spec.md`

### Features it does not have
- Envelopes, filters and other features beyond the oscillator
- Cross-fading at mipmap boundaries (the step is ultrasonic, so it is not addressed; §1.7)

---

## 1.2 Development environment

| Item | Value |
|---|---|
| IDE | Arduino IDE / Arduino CLI |
| Board package | **arduino-pico (Earle Philhower) 6.1.0** (not the official Arduino Mbed core) |
| Board | **RP2350-Zero** (Waveshare; FQBN `rp2040:rp2040:waveshare_rp2350_zero`) |
| I2S output | **Own PIO program + DMA ring** (arduino-pico's `I2S` library is not used; §1.11) |
| MCU | RP2350 (Cortex-M33 ×2, **single-precision FPU**, 150 MHz default, 520 KB SRAM) |
| DAC | PCM5102A (no SCK needed; the internal PLL makes the system clock from BCK) |

### Number formats

The RP2350 has a single-precision FPU, and the shared variables from the input section are also `float` (`wtvco_input_spec.md` 10.2).
They are used as follows.

| Processing | Format | Reason |
|---|---|---|
| Control-rate processing (reading the shared variables, combining, smoothing, knob pitch) | **float** | The inputs are float. A few dozen operations per control period, so the cost is negligible |
| Reading V/OCT (every sample) | float → Q16.16 conversion, **once only** | `voct_cent × (65536/1200)`: one multiply and an integer conversion, a few cycles on the FPU (§1.5) |
| pitch → phase_inc | **Q16.16 + exp2 LUT** (fixed point) | Faster than `exp2f()`, and the LUT is also more accurate |
| Audio path (phase, table lookup, interpolation, output gain) | **Fixed point (integer)** | A phase accumulator is inherently an integer |

**Inside the per-sample loop no float is used, except for that one V/OCT conversion.**

---

## 1.3 Audio specification

| Item | Value | Notes |
|---|---|---|
| Sample rate | **96 kHz** | Leaves margin against aliasing |
| I2S bit depth | **32 bit** | |
| Audio channels | **1** (OUT1) | |
| I2S frame | 32 bit × 2 (L / R) | The PCM5102A's I2S is always a stereo frame. **OUT1 goes on L. R is not used but gets the same value** (below) |
| Control period | **16 samples** (≈ 167 µs, 6 kHz) | Update period of the knobs and WT. **V/OCT is per sample** (§1.5) |
| Output method | **DMA ring of 64 frames, write lead of 8 frames (≈ 83 µs)** | Keeps the output buffering delay minimal (§1.11) |
| Output level | **−3.4 dBFS** (22154 in Q15) | The sine (frame 0) is 3 Vpp (±1.5 V around GND) at the PCM5102A output. Frames with larger peaks reach up to about 3.56 Vpp. The output buffer centres it on 2.5 V for OUT1 (sine 1–4 V, at most about 0.72–4.28 V). See below |
| PCM5102A interpolation filter | **Low latency (FLT = High)** | Group delay 3.5 tS = 36.5 µs. §1.11 |

### L / R

- OUT1 is the PCM5102A's **OUTL** (pin 6) through the output buffer (below).
- **OUTR (pin 7) is not used. Leave it open; do not connect it anywhere. Do not tie it to GND.**
  - OUTR is not an input but a line-driver output with a GND-centred signal. Tying it to GND would short the output
    (minimum load 1 kΩ; datasheet 8.3).
  - The datasheet says nothing about unused output channels. It is an output pin, so leaving it open with no load is fine.
  - There is only one output buffer, for OUT1. If a monitor is needed, split OUT1.
- R gets the same value as L. It costs almost nothing, and the OUTR pin can be used as a test point.
- The PCM5102A's zero-data detection works only when **both L and R are 0** (datasheet 9.3.2.3).
  Writing the same value to R does not make it enter analog mute during normal sound.

### Output level

The PCM5102A's full-scale output is nominally 2.1 Vrms in the datasheet (about 5.94 Vpp for a sine).
Measured, **−3 dBFS gave about 4 Vpp (about 5.65 Vpp full scale)**, so this measured value is used as the basis, and
**the sine (frame 0) is set to 3 Vpp (±1.5 V, GND-centred) at the PCM5102A output**. The output buffer (below) has a gain of −1 and only shifts the
DC level to 2.5 V without changing the amplitude. OUT1 is then **1–4 V (centred on 2.5 V, 3 Vpp)** for the sine.

**Why the reference is the sine, not full scale**: the wavetable is normalised with one factor for all frames (§1.7),
so each frame has a different peak. The sine (frame 0) peaks at only 25735 (−2.1 dBFS), so setting "3 Vpp at full scale" would give
only 2.36 Vpp for the sine (as measured, matching the calculation). So the level is set by the sine, which is the reference for the ear.
Frame 15, with the largest peak (30576), is about 3.56 Vpp (about 0.72–4.28 V at OUT1), within 0–5 V and the MCP6001's output range.

```
gain = 3 / (5.65 × 25735/32768) = 0.6761  →  0.6761 × 32768 = 22154   (−3.4 dBFS)
```

For another target (sine Vpp): `CFG_OUT_GAIN_Q15 = 32768 × target_Vpp / (5.65 × 0.7854)`

- The gain is applied to the oscillator output (after interpolation), and the same value is written to L and R.
- The wavetable is generated within full scale, so nothing clips after the gain.
- If the circuit around the PCM5102A changes, measure the full-scale voltage again and revise the value.
- The output buffer's gain is set by the ratio of two resistors (up to ±2% with 1% parts). To set 3 Vpp exactly, measure at OUT1 and revise the value.

### Output buffer (hardware; an assumption of the audio side)

The PCM5102A output is GND-centred (±1.5 V) and swings negative. AE Modular signals are 0–5 V, so
**an inverting buffer shifts it to be centred on 2.5 V.**

```
                                         +---[ 100kΩ ]---+
                                         |               |
                                         +---[ 47pF ]----+
                                         |               |
OUTL --[ 470Ω ]--+--[ 10µF ]--[ 100kΩ ]--+--|-\          |
(PCM5102A)       |                          |  >---------+--[ 470Ω ]-- OUT1
             [ 2.2nF ]             +2.5V ---|+/
                 |
                GND                 MCP6001 (V+ = AE 5V bus, V- = GND)
```

There is only one output buffer, for OUT1 (OUTR is not connected), so **a single MCP6001 is used**.
It is a separate IC from the V/OCT buffer (`wtvco_input_spec.md` 2.2). 0.1 µF right at the supply pin.

| Part | Value | Role, characteristics |
|---|---|---|
| 470 Ω + 2.2 nF | τ ≈ 1.0 µs (fc ≈ 154 kHz) | The PCM5102A's recommended output filter (datasheet 8.5 test conditions, figure 33) |
| 10 µF + 100 kΩ | fc ≈ 0.16 Hz (τ = 1 s) | AC coupling. Removes DC from the GND-centred signal. No attenuation even at C0 (16.35 Hz) |
| 100 kΩ / 100 kΩ | Gain **−1** (inverting) | Amplitude unchanged (3 Vpp for the sine). The +2.5 V on the non-inverting input becomes the output's centre |
| 47 pF ∥ 100 kΩ | fc ≈ 33.9 kHz (τ = 4.7 µs) | Low-pass in the feedback. About −1.3 dB at 20 kHz, about −3 dB at C11 (33.5 kHz) |
| MCP6001 | V+ = AE 5 V bus, rail-to-rail, single | The output, at most about 0.72–4.28 V, is within the supply range. 3.56 Vpp at 33 kHz needs a slew rate of about 0.37 V/µs, within the MCP6001's 0.6 V/µs |
| 470 Ω (output) | — | Short-circuit protection and isolation from capacitive loads |

- **The polarity is inverted.** For an oscillator this makes no audible difference. If it ever has to match the polarity of another signal,
  invert the gain (make `CFG_OUT_GAIN_Q15` negative).
- **Noise on the +2.5 V reference appears at the output about doubled in the audio band** (the noise gain of an inverting amplifier is 2).
  Decouple +2.5 V well.
- The 10 µF has 2.5 V DC across it (positive on the op-amp side). Mind the orientation if a polarised capacitor is used.
- **Right after power-up, until the 10 µF has charged (τ = 1 s, a few seconds), OUT1's DC level moves from the 5 V side down to 2.5 V.**
  It stays within 0–5 V, so it does no harm.
- The 47 pF low-pass (τ = 4.7 µs) is included in the latency budget (§1.11).

### I2S pins and clock

| GPIO | Signal |
|---|---|
| GP14 | DAC_BCK |
| GP13 | DAC_LRCK |
| GP12 | DAC_DIN |

> **Pins**: the own PIO program drives LRCK and BCK as two consecutive side-set pins
> (**side-set base = GP13 = LRCK (bit 0), GP14 = BCK (bit 1)**; BCK = LRCK + 1), and DATA as the OUT pin (GP12).

> **Note**: UART0 (RX = GP1) is used by the input section to receive from the PIC (`wtvco_input_spec.md` 2.6). The audio side does not touch it.

> **Clock**: 1 frame = 64 BCK (32 bit × 2). BCK = 64 fS = **6.144 MHz**.
> This is a combination for which the PCM5102A's BCK PLL can make its internal clock (datasheet Table 11: BCK 64 fS = 6.144 MHz at 96 kHz).
> With the PIO program written as 2 instructions per BCK,
> the divider is `150 MHz / (96000 × 64 × 2)` = **12.20703125 = 12 + 53/256**, which the 8-bit fractional divider **represents exactly**.
> The average sample rate is exactly 96000 Hz, but the fractional division puts a few ns of jitter on BCK (within what the PCM5102A's internal PLL absorbs).
> The system clock stays at the default 150 MHz.

---

## 1.4 External parameters (values received from the input section)

**Shared variables written by the input section (core1) and read by the audio side (core0). Defined in `wtvco_input_spec.md` 10.2.**

| Variable | Type | Unit, range | Update by the input section |
|---|---|---|---|
| `voct_cent` | float | 0–6000 cent (clamped by the input section. Not 0 when unplugged but about 240–360 cent; `wtvco_input_spec.md` chapter 6) | Every core1 loop pass (≤ 10 µs) |
| `coarse_cent` | float | 0–7200 cent (from C0) | Only when the hysteresis is exceeded (in steps) |
| `fine_cent` | float | −100 to +100 cent | Every 1 ms |
| `wt_cv` | float | −0.5 to +1.5 (UNI + BI summed, not clamped) | Every 1 ms |
| `wt_knob` | float | 0.0–1.0 | Every 1 ms |
| `fm_cent` | float | −6000 to +18000 cent (FM CV × DEPTH, not clamped) | Every 1 ms |
| `status` | uint32_t | Bit flags | Any time |
| `audio_mute_req` | uint32_t | 0 / 1. core1 → core0. Mute request before a flash save | Only when saving |
| `audio_mute_ack` | uint32_t | 0 / 1. **core0 → core1**. Mute acknowledgement | Only when saving |

- **Six values** set the sound (plus `status`).
- All are aligned single 32-bit variables. **Each is read or written in one access, so no locking is needed.**
- The input section is responsible for its defaults (when the PIC is lost, right after start-up, etc.; `wtvco_input_spec.md` 8.2).
  Example: without the PIC, `coarse_cent = 3600` (C3) and the rest 0. **The audio side has no defaults of its own and uses the shared variables as they are.**
- The audio side does not need to look at `status` (if it does, only for debug display; **never to decide to stop the sound**;
  `wtvco_input_spec.md` 11.2, "keep the sound going even when degraded").
- `audio_mute_req` / `audio_mute_ack` are a handshake for flash saving (§1.12, `wtvco_input_spec.md` 9.5).
  **This request is the only thing that stops the sound.**

### Separation of the input layer

**The audio side knows only the shared variables. It knows nothing about where the values come from (ADS1115 / coarse ADC / PIC / stub).**

- The shared variables are declared in one header (`shared_params.h`), included by both the audio side and the input section.
- Changing the source of the values **requires no change to the audio code**. The audio side can also be tested on its own with fixed values.
- The USB console's test commands (`coarse`, `voct`, `ksrc stub`, `vsrc stub`, etc.) also use this to write the shared variables directly.
  Afterwards, `default` gives control back to the input section.
- The audio side never touches the RP2350's internal ADC or the knob GPIOs (GP27 is the V/OCT coarse ADC; `wtvco_input_spec.md` 2.6).

---

## 1.5 Internal parameters

### How the pitch is set

Following the formula in `wtvco_input_spec.md` 1.3:

```
f = f_C0 × 2^((coarse_cent + fine_cent + voct_cent + fm_cent) / 1200)
f_C0 = 440 × 2^(−57/12) = 16.351597831 Hz   (A4 = 440 Hz, scientific pitch notation)
```

Internally the pitch **stays as octaves above C0 (logarithmic) and is combined by addition. The exponential is taken only once, at the end.**
The knob part and FM are computed at the control period, **the V/OCT part every sample**.

```
// --- control period (every 16 samples) ---
knob_cent = smooth_pitch(coarse_cent + fine_cent)       // §1.10. V/OCT not included
knob_q    = (int32_t)lrintf(knob_cent × 65536 / 1200)   // Q16.16 (octaves above C0)
fm_c      = smooth_fm(fm_cent)                          // §1.10. Smoothed separately from the knob pitch
fm_q      = (int32_t)lrintf(fm_c × 65536 / 1200)        // Q16.16. Can be negative
ctl_q     = knob_q + fm_q                               // summed once per control period

// --- every sample ---
voct_q    = (int32_t)(voct_cent × (65536.0f / 1200))    // the shared variable is read every sample
pitch_q   = clamp(ctl_q + voct_q, PITCH_MIN_Q, PITCH_MAX_Q) // signed comparison
phase_inc = exp2_lut(pitch_q)                           // ≈10 cycles
level     = pitch_q >> 16                               // mipmap level 0–10
```

- Reading V/OCT every sample means a V/OCT change takes effect from the next sample, without waiting for the control period (§1.11).
- `voct_cent` is clamped to 0–6000 by the input section and is never negative, so the truncation of the integer conversion (at most 1 LSB ≈ 0.018 cent) is no problem.
- The mipmap level is also decided per sample. Level switching at sample granularity is fine.

| Item | Type | Value / notes |
|---|---|---|
| pitch | **Q16.16** (octaves above C0) | 1 LSB ≈ 0.018 cent |
| PITCH_MIN | 0.0 | = C0 (16.35 Hz) |
| PITCH_MAX | **11.0 − 1 LSB** (`0x000AFFFF`) | ≈ just below C11 (33.49 kHz). See below |
| Phase accumulator | **uint32_t** | 96000 / 2³² ≈ 0.00002 Hz |
| WT position | Q16.16 | Integer part = frame number, fraction = cross-fade ratio |
| Intermediate values | int32_t | |

Precision of the float → Q16.16 conversion: each value is at most about 18000 cent, where single precision resolves about 0.002 cent. No problem.
Overflow of the sum: the total is −6100 to +31300 cent (about −5 to +26 oct), about ±2²¹ in Q16.16. Well within int32_t.

### Range and clamping (`wtvco_input_spec.md` 1.3)

The possible range from the input combinations is −100 to 13300 cent without FM and **−6100 to +31300 cent with FM**.

| End | Handling |
|---|---|
| Lower | **Clamped at C0**. Without FM this only matters with COARSE = 0 and FINE turned down. Deep downward FM sticks at C0 (by design) |
| Upper | **Clamped just below C11** (`CFG_PITCH_MAX_Q`). Deep upward FM sticks there (by design). Keeping the integer part at 10 or less keeps the mipmap level and the exp2 LUT shift within 0–10 |

Near the upper end, because of the Nyquist frequency (48 kHz), there is room for almost no overtones, and the top level is a sine (§1.7).
If there is no need to produce sounds above the audible range, `CFG_PITCH_MAX_Q` may be lowered (e.g. C9 = 9.0 − 1 LSB).
**The integer part of `PITCH_MAX_Q` must not exceed 10** (the LUT scaling and the number of table levels assume it).

### Frequency reference

| pitch | Note | Frequency |
|---|---|---|
| 0.0 | C0 | 16.352 Hz |
| 3.0 | C3 | 130.81 Hz |
| 6.0 | C6 | 1046.50 Hz |
| 11.0 − 1 LSB | ≈ C11 | ≈ 33488 Hz |

---

## 1.6 exp2 LUT

### Principle

```
phase_inc = f × 2³² / 96000
          = (f_C0 × 2³² / 96000) × 2^pitch
          = K × 2^pitch,   K = 731558.1034
```

Split with the laws of exponents:

```
2^pitch = 2^(integer part) × 2^(fraction)
           ↑shift            ↑table lookup
```

**The integer part is just a bit shift, so the table only needs the fraction (one octave).**

### Specification

| Item | Value |
|---|---|
| Entries | 1024 **+ 1 sentinel** (1025 in total, 4100 bytes) |
| Type | uint32_t |
| Stored value | `round(K × 2¹⁰ × 2^(i/1024))`, i = 0–1024 |
| Index | Upper 10 bits of the pitch fraction |
| Interpolation | Linear, with the lower 6 bits |
| Error (LUT value, before the shift) | ≈ 0.0001 cent (worst case in the estimate 0.000101 cent) |
| Cost | ≈ 10 cycles |

### Why the stored values are multiplied by 2¹⁰

**The shift for the top octave (integer part of PITCH_MAX = 10 → 2¹⁰) is multiplied in before storing.**

- The rounding error becomes negligible, and the error is set only by the interpolation error (≈ 0.0001 cent)
- The run-time shift is **always a right shift** (`10 − integer part`), so there is no branch
- There is no left shift, so no overflow either

### Sentinel entry

So that `LUT[i+1]` can be read unconditionally even for i = 1023, **one entry is added at the end**,
holding `K × 2¹⁰ × 2` (= `K × 2¹¹`). This removes the wrap check.
It is the same idea as "copy the first sample to the end" in the wavetable.

### Procedure

```
1. Take the integer part of pitch        → 0–10
2. Look up the table with the upper 10 bits of the fraction
3. Interpolate linearly with the neighbour (sentinel included) using the lower 6 bits
4. Shift right by (10 − integer part) bits (rounded)
```

K is included in the stored values, so no multiplication is needed at run time.

### Overflow check

The largest table value is the sentinel `K × 2¹¹` = 1,498,230,996 (31 bits, < 2³¹).
The difference × fraction (6 bits) in the interpolation also stays within uint32_t. Only right shifts follow, so the range is never exceeded.
The largest phase_inc (≈ C11) is also below 2³¹ = below Nyquist.

### Remaining error

phase_inc is finally an integer in uint32, so at pitch 0.0 (C0, phase_inc ≈ 731558)
a quantisation of **±0.5 LSB ≈ 0.0012 cent** remains. This is a floor set by the phase-accumulator width and has nothing to do with the LUT accuracy.
The worst case over the whole range in the estimate is 0.00126 cent.

---

## 1.7 Wavetable

### Three axes

| Axis | Meaning | Value |
|---|---|---|
| ① Frame length | Samples per cycle | 2048 (longest level) |
| ② Frames | Number of waveforms the WT moves through | **16** |
| ③ Mipmap levels | Band-limited versions per pitch range | **11** (for pitch integer parts 0–10) |

### Mipmaps

Playing all the overtones at high pitches would fold the components above Nyquist back down.
**A version with fewer overtones is prepared for each octave and selected by the integer part of the pitch.**
The base is C0, so every level boundary falls on a C.

| Level | pitch range | Notes | Frequency | Max harmonic | Table length |
|---|---|---|---|---|---|
| 0 | 0.0–1.0 | C0–C1 | 16.35–32.70 Hz | 1023 (length limit) | 2048 |
| 1 | 1.0–2.0 | C1–C2 | 32.70–65.41 Hz | 733 | 2048 |
| 2 | 2.0–3.0 | C2–C3 | 65.41–130.81 Hz | 366 | 2048 |
| 3 | 3.0–4.0 | C3–C4 | 130.81–261.63 Hz | 183 | 2048 |
| 4 | 4.0–5.0 | C4–C5 | 261.63–523.25 Hz | 91 | 1024 |
| 5 | 5.0–6.0 | C5–C6 | 523.25–1046.5 Hz | 45 | 512 |
| 6 | 6.0–7.0 | C6–C7 | 1046.5–2093.0 Hz | 22 | 256 |
| 7 | 7.0–8.0 | C7–C8 | 2093.0–4186.0 Hz | 11 | 256 |
| 8 | 8.0–9.0 | C8–C9 | 4186.0–8372.0 Hz | 5 | 256 |
| 9 | 9.0–10.0 | C9–C10 | 8372.0–16744 Hz | 2 | 256 |
| 10 | 10.0–11.0 | C10–C11 | 16744–33488 Hz | 1 (sine) | 256 |

Generation rules:

```
max harmonic = min( floor(48000 / level top frequency), max table length/2 − 1 )     top frequency = f_C0 × 2^(Level+1)
table length = smallest power of 2 ≥ 8 × max harmonic (min 256, max 2048)
```

- Level 10 can hold only the fundamental, so **every frame is the same sine** (moving the WT does not change the timbre; by design).
- Levels 9 and up are at the top of the audible range or ultrasonic.
- **The max harmonic of level 0 is 1023** (= 2048/2 − 1). In a 2048-sample table, harmonic 1024 sits exactly at Nyquist and cannot represent a sine component, so it is left out.

### Why the table length is at least "8 × max harmonic"

The tables are read with linear interpolation (§1.8). Linear interpolation creates **image components of harmonic h at (N − h) and so on**, for table length N.
If N is barely 2 × the max harmonic, the image of the top harmonic is attenuated by only about −14 dB, exceeds Nyquist and folds back into the audible range
(a muddiness not in integer ratio to the fundamental).

Estimate with a band-limited sawtooth (96 kHz, largest spurious component):

| Level | Fundamental | N = 2 × max harmonic or more (min 32) | N = 8 × max harmonic or more (min 256) |
|---|---|---|---|
| 3 | 255 Hz | N = 512: −59 dBc | N = 2048: −86 dBc |
| 4 | 515 Hz | N = 256: −53 dBc | N = 1024: −83 dBc |
| 5 | 1030 Hz | N = 128: −47 dBc | N = 512: −77 dBc |
| 6 | 2050 Hz | N = 64: −42 dBc | N = 256: −68 dBc |
| 7 | 4100 Hz | N = 32: −36 dBc (9.9 kHz) | N = 256: −75 dBc |

- With 8× or more, spurious components are about −68 dBc or below at every level. The droop of the top harmonic (high-frequency loss from interpolation) also shrinks from about −4 dB to within −0.6 dB.
- Levels 0–2 hit the length limit (2048) and do not reach 8×, but their top harmonics are small to begin with
  (−51 dB or less for a sawtooth) and their images fall mostly in the ultrasonic range, so it is not a problem.

### Memory

```
Per frame: 2048×4 + 1024 + 512 + 256×5 = 11008 samples
(11019 samples including the +1 copy for each table)
11019 × 2 bytes = 22,038 bytes/frame
22,038 × 16 frames ≈ 353 KB
```

**It is copied from flash (4 MB on the RP2350-Zero) to RAM at start-up and used from there** (`CFG_TABLES_IN_RAM = 1`; the exp2 LUT too).
It fits in the 520 KB RAM (while one note plays, only 2 frames × 1 level are read, at most about 8 KB).

- **Why RAM**: in flash (XIP), core0's table reads and core1's code in flash compete for the XIP cache.
  As a result core1's loop stalled for 60–100 µs and core0 underran more often. With the tables in RAM, core0 stalls longer than 20 µs dropped from 446 to 47,
  and its load from 12–14% to about 10.5%.
- For the same reason, core0's ring-following loop and control-period processing, and core1's coarse-ADC processing, fusion and input loop run from RAM (`RAM_FUNC`).
  Functions in RAM do not call library functions in flash (e.g. rounding is computed inline instead of calling `lrintf`).
- With `CFG_TABLES_IN_RAM = 0` it runs with the tables in flash (with the performance penalty above).

### Storage format

- Type: `int16_t`
- Each table is stored **one sample longer**, with the first sample copied to the end
  (so that `i+1` in the interpolation needs no wrap handling)
- No DC component
- Peaks are normalised to within full scale (with one factor for all frames and levels, so the volume does not change between levels)

### Level selection

The integer part of the pitch is used directly as the level index. **Nothing is done about switching levels at octave boundaries.**

- At the moment a level switches, **only overtones in the ultrasonic 24–48 kHz range change**.
  Just before a boundary (top of level L) overtones reach about 48 kHz, just after (bottom of level L+1) about 24 kHz,
  so the switch removes what lies in between (because it runs at 96 kHz and every level is built for a 48 kHz Nyquist).
- So the audible timbre does not change, and even FM sweeping back and forth across a boundary should not produce an audible step.
- Intermodulation in a later non-linear stage (e.g. an overdriven VCF) or a click at the switch could in theory be audible.
  Sweeping across boundaries with FM on the hardware did not produce an audible step (§1.13).

---

## 1.8 Interpolation

**Four table lookups** per sample.

```
idx       = phase >> (32 - LOG2_LEN)
phaseFrac = (phase << LOG2_LEN) >> 24        // 8 bit

a = lerp(tbl[f  ][idx], tbl[f  ][idx+1], phaseFrac)
b = lerp(tbl[f+1][idx], tbl[f+1][idx+1], phaseFrac)
s = lerp(a, b, frameFrac)                     // 8 bit
```

WT → frame position: `wtPos = WT × 15` (16 frames = 15 intervals).
At `WT = 1.0`, `f = 15` and `f+1` would be out of range, so either **round to `f = 14` with frameFrac at maximum**,
or clamp `wtPos` to `15.0 − 1 LSB`.

---

## 1.9 Combining the WT position (`wtvco_input_spec.md` 10.3)

```
wt = clamp(wt_knob + wt_cv, 0.0, 1.0)
```

- **The KNOB is the base position, and the CV is added to it as modulation.**
- `wt_cv` is the sum of the unipolar (UNI) and bipolar (BI) inputs computed by the input section, in the range **−0.5 to +1.5** (`wtvco_input_spec.md` 2.5, 8.4, 8.5).

  | Input | Accepts | Modulation |
  |---|---|---|
  | UNI | 0–5 V (from 0 V) | 0 to +1.0 (upwards only) |
  | BI | 0–5 V (around 2.5 V) | −0.5 to +0.5 (both ways) |

  Both inputs have a sensitivity of 1 V = 0.2.
- **The clamp is applied here, once.** The input section passes the values unclamped, so whatever the sum overshoots sticks at 0.0 / 1.0 here.
- With the CVs unplugged, `wt_cv = 0` (UNI sits at 0 V by the pull-down and BI at 2.5 V by the bias, per the input section), so the KNOB alone sets the position.
- An AE Modular LFO (0–5 V, centred on 2.5 V) into BI swings ±0.5 around the KNOB position. With the KNOB centred it sweeps the whole range (0–1).
- 0 V-based CVs such as an ADSR go into UNI. Using both gives combined modulation such as ADSR + LFO.
- Combining happens at the control period; after smoothing (§1.10) the result is converted to the Q16.16 WT position.

---

## 1.10 Parameter smoothing

The input section already smooths, but the shared variables still change **in steps**, every 1 ms or only when a hysteresis is exceeded
(`wtvco_input_spec.md` 10.3). Used as is, they would cause zipper noise, so the audio side applies a first-order low-pass at the control rate (6 kHz).

| Parameter | Time constant | Notes |
|---|---|---|
| `coarse_cent + fine_cent` (knob pitch) | **5 ms** | COARSE arrives in steps of 4–5 cent or more as the hysteresis is exceeded. This joins them smoothly |
| `voct_cent` | **None** | See below |
| WT (after combining `wt_knob + wt_cv`) | **2 ms** | Just enough to remove the 1 ms steps. Does not dull medium-speed WT CV modulation |
| `fm_cent` | **2 ms** | Separate from the knob pitch. 5 ms (fc ≈ 32 Hz) would dull faster LFOs |

The time constants are in `config.h`. They were checked by playing, found free of artefacts and kept at the values above (§1.13).
The USB console's `tune` command changes them without flashing (in RAM only), for trying other values.

### Why V/OCT is not smoothed

- V/OCT must **follow a note-on within 500 µs** (`wtvco_input_spec.md` 1.4).
  A 2 ms first-order low-pass alone would take 2 ms to reach 63%, failing the requirement.
- The steps at mode changes (FAST ↔ ADS) are already smoothed by the input section with `SMOOTH_MS` (`wtvco_input_spec.md` 5.5).
- So the audio side **reads `voct_cent` every sample and uses it as is** (§1.5).

---

## 1.11 Latency and I2S output

### The V/OCT requirement

From note-on to audio output **within 500 µs** (`wtvco_input_spec.md` 1.4).
The input section uses at most about 52 µs, typically about 42 µs (`wtvco_input_spec.md` 10.4).

### Overview of the measures

| Measure | Content | Where | Effect (worst case) |
|---|---|---|---|
| A | Read V/OCT every sample (§1.5) | Audio side | Wait for the control period 167 µs → 1 frame (10.4 µs) |
| B | Own PIO + DMA ring for I2S, writing N frames ahead of the read position (below) | Audio side | Output queue limited to N + 2 FIFO frames (about 104 µs with N = 8) |
| C | Non-blocking I2C read of the ADS1115 | Input section | core1 processing delay about 70 µs → 10 µs or less (`wtvco_input_spec.md` 4.4) |
| D | Jump detection on the per-raw-sample moving average | Input section | Waiting for coarse-sample boundaries + confirmation about 100 µs → about 40 µs (`wtvco_input_spec.md` 5.3) |

### I2S output: PIO + DMA ring (measure B)

arduino-pico's `I2S` library keeps a queue of buffers, and in steady state the queue is always full,
so the whole queue length (several hundred µs) becomes delay. Instead, **the DMA reads a ring in an endless loop, and
core0 keeps topping it up to "N frames ahead of the DMA read position"**.
It is the same structure as the input section's coarse ADC (a DMA ring that core1 follows), used in the output direction.

```
          core0 writes (up to N frames ahead of the read position)
                ↓
  [ ring of 64 frames (512 bytes, aligned to 512 bytes) ] ──DMA (endless)──→ PIO TX FIFO ──→ I2S (GP12/13/14)
                ↑
          DMA read position (from READ_ADDR)
```

| Item | Value | Notes |
|---|---|---|
| Ring length | **64 frames** (1 frame = L/R 32 bits each = 8 bytes, 512 bytes total) | `CFG_OUT_RING_FRAMES`. Power of 2. **Aligned to its own size in bytes** (for the DMA ring wrap) |
| Write lead N | **8 frames (≈ 83 µs)** | `CFG_OUT_LEAD_FRAMES`. Chosen from measurements (below). Upper limit 31 frames |
| DMA | Ring wrap on the read side (2⁹ bytes), paced by the PIO TX DREQ, **runs forever** | Uses the RP2350 DMA's endless (ENDLESS) transfer-count mode (`dma_encode_endless_transfer_count()` in the pico-sdk bundled with arduino-pico 6.1.0) |
| PIO TX FIFO | **Not joined** (4 words = 2 frames) | The FIFO is also delay, so it is kept shallow |

**core0 loop (pseudo-code)**

```c
for (;;) {
    rd   = dma_read_frame_index();                 // (READ_ADDR - ring_base) / 8
    lead = (wr - rd) & (RING - 1);
    if (lead > RING / 2) {                         // the read overtook the write = underrun
        underrun_cnt++;
        wr = (rd + LEAD) & (RING - 1);             // realign (an old frame plays briefly)
        continue;
    }
    min_lead = min(min_lead, lead);                // margin monitoring (below)
    while (((wr - rd) & (RING - 1)) < LEAD) {
        if (++ctl_cnt == CTRL_PERIOD) { control_update(); ctl_cnt = 0; }  // control period
        render_frame(&ring[wr]);                   // generate 1 sample → gain → write the same value to L/R
        wr = (wr + 1) & (RING - 1);
    }
}
```

- core0 is dedicated to this loop with nothing else in it (busy-waiting is fine).
- At start-up the ring is filled with 0 before the DMA and PIO start, and writing begins at `wr = rd + N`.
- **`min_lead` (the smallest lead observed) is recorded.** The closer it gets to 0, the closer an underrun is.
  The USB console's `show` command shows it together with `underrun_cnt`.
- For the underrun test `lead > RING / 2` to work, the normal lead must always be at most RING / 2 = 32 frames.
  **N is 31 or less** (which also matches the upper limit from the budget below).

**How N was chosen**

- N must be larger than "the longest time core0 can be stopped by interrupts and the like".
- In arduino-pico, the USB (TinyUSB) processing (`tud_task()` every 1 ms on core0, and USB interrupts) runs on core0.
- Measured: without USB (when playing) core0's longest stall was 37 µs, and **with N = 8 (83 µs) there were 0 underruns** (§1.13).
  With USB connected it stalls for up to 131 µs, and underruns occur occasionally, e.g. when connecting or opening/closing the port.
  USB is only used for calibration and the like, so this is accepted and N = 8 was chosen.
- If it is not enough, increase N (N = 16 gives 167 µs; the upper limit is 31 frames from the budget below).
- The input section's interrupts (RDY GPIO interrupt, DMA, etc.) are registered on core1 so that they never reach core0 (`wtvco_input_spec.md` 10.1).

### PCM5102A delay and settings (from datasheet SLAS859C)

The PCM5102A's interpolation filter is set to **Low latency (IIR) with the FLT pin (pin 11) High** (datasheet 9.3.4.1, Table 3).

| Item | Low latency (FLT = High, used) | Normal (FLT = Low) | Source |
|---|---|---|---|
| Group delay | **3.5 tS = 36.5 µs** (96 kHz) | 20–22 tS = 208–229 µs | Electrical characteristics, Tables 4, 7, 8 |
| Pass band | 0.47 fS (45.1 kHz) | 0.45 fS (43.2 kHz) | Electrical characteristics |
| Pass-band ripple | ±0.0001 dB | ±0.02 dB | Tables 4, 7 |
| Stop band / attenuation | 0.55 fS (52.8 kHz) and above / −52 dB | 0.55 fS and above / −60 dB | Tables 4, 7 |

- 1 tS = 1/fS = 10.42 µs (96 kHz).
- 96 kHz is detected inside the PCM5102A as "double rate", and the interpolation becomes the x4 type instead of x8,
  but the Low-latency group delay is 3.5 tS for both x8 and x4 (Tables 7, 8).
- With the Normal filter, the group delay alone is up to 229 µs, which would eat a large part of the budget. **FLT = High is required.**
- The stop band starts at 52.8 kHz, so the difference in attenuation only affects **ultrasonic image components**, not the audio band.
- Being IIR, the phase is not linear (the group delay varies with frequency). The phase of high overtones shifts slightly and the waveform may look different,
  but for an oscillator this is not audible.
- The recommended output filter (470 Ω + 2.2 nF, datasheet 8.5 test conditions) has a time constant of about 1.0 µs, negligible.
- The output buffer after it (§1.3) is a low-pass with a 4.7 µs time constant (100 kΩ ∥ 47 pF in the feedback). It is included in the budget.
- The datasheet gives no delay for the ΔΣ modulator and the charge-pump output stage; they are assumed to be much smaller than the filter delay above.

### Budget (V/OCT: note-on → audio output)

| Part | Worst | Typical | Basis |
|---|---|---|---|
| Input section (note-on → shared variable) | 52 µs | 42 µs | `wtvco_input_spec.md` 10.4 (including measures C and D) |
| Wait until the shared variable is read | 10.4 µs | 5.2 µs | Measure A. core0 generates one sample per frame (1/96 kHz) and reads it each time |
| Output ring (lead N) | 83.3 µs | 83.3 µs | Measure B. N = 8 frames × 10.42 µs |
| PIO TX FIFO | 20.8 µs | 20.8 µs | Not joined, 4 words = 2 frames |
| I2S transmission (1 frame) | 10.4 µs | 10.4 µs | 64 BCK ÷ 6.144 MHz |
| PCM5102A interpolation filter | 36.5 µs | 36.5 µs | 3.5 tS (FLT = High) |
| Output RC filter | 1.0 µs | 1.0 µs | 470 Ω × 2.2 nF |
| Output buffer | 4.7 µs | 4.7 µs | 100 kΩ × 47 pF (§1.3) |
| **Total** | **about 219 µs** | **about 204 µs** | **Within 500 µs, with at least about 281 µs to spare** |

- The audio side alone is **at most about 167 µs, typically about 162 µs** (rows 2 onwards in the table, up to OUT1).
- The input-section values are for pitch changes of a semitone or more. Smaller changes take a little longer to detect (`wtvco_input_spec.md` 10.4).
- Everything except N adds up to at most about 136 µs. Even N = 16 gives a total of about 303 µs, within 500 µs.
- **The upper limit of N is 31 frames** (total about 459 µs, leaving about 40 µs for variation; it also matches the condition N ≤ 31 for the ring's underrun test).
  If an N above this were ever needed, deal first with the interrupts that stop core0.

### Hardware requirements (PCM5102A pin settings)

| Pin | Setting | Reason (datasheet) |
|---|---|---|
| FLT (11) | **High (3.3 V)** | Low-latency filter (9.3.4.1) |
| FMT (16) | Low | I2S format (9.3.2.2) |
| DEMP (10) | Low | De-emphasis off |
| XSMT (17) | High (tied to AVDD, or externally controlled) | Soft mute released (9.3.3) |
| SCK (12) | **GND** | When SCK stays Low for 16 LRCK or more, the internal PLL makes the clock from BCK (9.3.5.3) |

Many ready-made PCM5102A modules set FLT, FMT, DEMP and XSMT with solder jumpers, so
**check that FLT is High on the module you use**.

### Other PCM5102A behaviour (beyond latency)

- **Zero-data detection**: when both L and R are 0 for 1024 LRCK (about 10.7 ms at 96 kHz), it enters analog mute (datasheet 9.3.2.3).
  An oscillator always produces sound, so this normally never happens, but it does during the mute for a flash save (§1.12).
  After the mute is released the sound comes back normally.
- **Clock errors**: if the LRCK–BCK relationship breaks for 4 LRCK or more, the output goes to bipolar zero and it resynchronises (datasheet 9.3.2.1).
  The PIO and DMA never stop, so this normally does not happen.
- **Start-up**: it takes about 4 ms from the arrival of the I2S clocks until the internal reset is released (datasheet figure 41). That is the brief silence right after start-up.

### WT CV and knobs

At most about 1.6 ms in the input section (PIC frame period + conversion + transmission; `wtvco_input_spec.md` 10.4) + the input section's smoothing.
On the audio side, the control period (≤ 167 µs) + output (about 167 µs) + 2–5 ms of smoothing. Plenty for medium-speed requirements.

---

## 1.12 Core split and CPU / memory estimates

### Core split (`wtvco_input_spec.md` 10.1)

| Core | Role | Entry point in arduino-pico |
|---|---|---|
| **core0** | **Sound generation, writing the output ring** (dedicated to this loop) | `setup()` / `loop()` |
| **core1** | Input processing (`input.*`), USB console (`console.*`) | `setup1()` / `loop1()` |

> **Note**: in arduino-pico, USB (TinyUSB) processing runs on core0. USB console commands are handled in the core1 loop,
> but the USB processing itself stops core0, so underruns occasionally occur while USB is connected (§1.11, §1.13).

### Mute during a flash save (handshake)

While the calibration values are written to flash, XIP stops and both cores wait (`wtvco_input_spec.md` 9.5).
Meanwhile **the DMA and PIO keep reading the ring in SRAM**, so without precautions the 64 frames left in the ring
repeat and make **a 1.5 kHz (96 kHz / 64) buzz**. To avoid this, the ring is silenced before saving.

```
core1 (save command received)       core0 (audio)
  audio_mute_req = 1  ─────────→  fade out (about 2 ms)
                                    write 0 over the whole ring (64 frames or more)
                      ←─────────  audio_mute_ack = 1
  write flash (both cores stopped;
   the DMA keeps sending the ring's 0s)
  audio_mute_req = 0  ─────────→  ack = 0, realign the write position, fade in over 2 ms and resume
```

- core1 waits for `audio_mute_ack` before saving. If it does not come within 50 ms, the save is cancelled and an error is shown.
- The 2 ms fade-in on release prevents a click.
- During the mute the PCM5102A enters analog mute through zero-data detection (§1.11). The sound comes back after release.
- Saving only happens in calibration mode, so the silence during it is accepted.

### CPU estimate

```
150 MHz ÷ 96 kHz = 1562 cycles/sample
Interpolation for one voice (4 table lookups + 3 lerps) ≈ 80 cycles
Gain and L/R write ≈ 10 cycles
Per-sample V/OCT conversion + exp2 LUT ≈ 20 cycles
Control-rate processing (float): a few hundred cycles per control period → a few dozen cycles per sample
→ load about 8–12%
```

core0 busy-waits following the output ring, so its load is measured as "time spent generating frames ÷ elapsed time".

### Memory

- RAM 520 KB: wavetable ≈ 353 KB + exp2 LUT 4 KB (§1.7; copied from flash at start-up), state variables, the output ring (512 bytes), the input section's DMA ring (16 bit × 1024).
  About 72% in total (§1.13)
- Flash 4 MB: the program, and the initial data of the wavetable and the exp2 LUT

---

## 1.13 Verification results

Measurements on two prototype units and results of tests on a PC. The input section's verification results are in `wtvco_input_spec.md` chapter 13.

### Measurements

| Item | Estimate | Measured |
|---|---|---|
| Sample rate | 96000 Hz | **96000.0 Hz** (measured from the DMA read position; the divider 12 + 53/256 is exact) |
| Control rate | 6000 Hz | 6000 Hz (every 16 samples) |
| core0 (audio) load | about 8–12% | **10.2–11.6%** (tables in RAM; 12–14% with the tables in flash). About 18% during heavy USB traffic (including the USB interrupt time). Time spent generating frames ÷ elapsed time |
| Flash | about 357 KB + α | Program 76,160 bytes. The initial data of the wavetable and LUT kept in RAM (about 357 KB) is also in flash |
| RAM (global variables) | — | 378,064 bytes (72% of 512 KB), of which the wavetable is 352,608 bytes and the exp2 LUT 4,100 bytes. 15,860 bytes with the tables in flash |
| Underruns with output lead N = 8 | 0 | **0** in 53 seconds without USB (longest core0 stall 37 µs). With USB connected core0 stalls for up to 68–131 µs, and an underrun occurs occasionally, e.g. when connecting or opening/closing the port |
| `min_lead` (N = 8) | > 0 | 1–5 frames with USB connected, 0–2 frames right after connecting |
| V/OCT step → OUT1 (audio side only) | 157–167 µs | **About 160 µs** (`voct_cent` stepped directly, with a GPIO toggled at that moment as the oscilloscope trigger) |
| Note-on → OUT1 (including the input section) | at most 219 µs | **About 210 µs** (oscilloscope triggered by the moment the CV source switched steps; requirement within 500 µs) |
| Output level (§1.3) | 3 Vpp sine | 3 Vpp confirmed for the sine (frame 0). PCM5102A full scale 5.65 Vpp (the same when measured again on the second board) |
| Output waveform | — | A 1.5 kHz sine and every frame at C3 checked on an oscilloscope at OUT1. No noticeable noise |

### Underruns during a flash save (only with USB connected)

Isolated in a single session with the USB port kept open.

| Operation | Underruns |
|---|---|
| Idle for 10 s | 0 |
| Mute handshake only (`mute 100`) | 0 |
| `calsave` (flash write 31 ms) | 2 |

No sound is produced during the mute, so the likely cause is that the USB processing held off during the flash write runs in a burst on core0 right after it is released (and the XIP cache is empty).
Saving is only possible from the USB console, so this cannot happen while playing (without USB). Right after release the sound is fading in and the amplitude is small, so nothing is done about it.

### Listening

The smoothing time constants (§1.10) and the mipmap boundaries (§1.7, sweeping across them with FM) were checked by playing; there were no artefacts or audible steps. The initial values were kept.

### Host-side tests

The tests in `firmware/wtvco/tests/` run on a PC (`run_tests.sh`). The three related to the audio side all pass.

| Test | Content |
|---|---|
| `test_pitch` | exp2 LUT against `exp2()` (error before the shift under 0.001 cent over the whole range, sentinel value, final phase_inc error under 0.002 cent, monotonic, below Nyquist). Pitch-to-frequency mapping (C0 16.352 Hz / C3 130.81 Hz / C6 1046.5 Hz). Conversion of the cent sum to Q16.16 and clamping: extreme input combinations (COARSE 0 + FINE −100, COARSE 7200 + FINE +100 + V/OCT 6000, COARSE 0 + FM −6000, everything at maximum + FM +18000) stick at the lower and upper ends without overflow. Mipmap level never outside 0–10. Length, max harmonic and layout of the generated tables match §1.7 |
| `test_oscillator` | WT position to frame and interpolation ratio (no out-of-range read even at WT = 1.0). No out-of-range table reads at any level. Interpolation error. Samples after the output gain do not clip for any frame or level. The oscillation period matches the pitch |
| `test_control` | WT combination `clamp(wt_knob + wt_cv, 0, 1)` (sticking at the ends, monotonicity, range). Smoothing responds with the specified time constants |
