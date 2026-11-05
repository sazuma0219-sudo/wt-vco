// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Compile-time configuration (wtvco_audio_process.md).
#pragma once
#include <stdint.h>

// Functions on the real-time paths run from RAM (no XIP cache misses). Host tests build with -DHOST_TEST.
#if defined(HOST_TEST)
#define RAM_FUNC(f) f
#else
#include "pico.h"
#define RAM_FUNC(f) __not_in_flash_func(f)
#endif

// ---- Audio --------------------------------------------------------------
#define CFG_SAMPLE_RATE      96000      // Hz (§1.3). Must match tools/gen_*.py
#define CFG_CTRL_PERIOD      16         // samples per control update (≈167 µs, 6 kHz)

// ---- Output ring (§1.11) ------------------------------------------------
#define CFG_OUT_RING_FRAMES  64         // frames (L+R, 8 bytes each). Power of 2
#define CFG_OUT_LEAD_FRAMES  8          // write lead N ahead of the DMA read position (≤ 31)

// ---- Output level (§1.3) ------------------------------------------------
// 32768 * target_Vpp / 5.65  (5.65 Vpp = measured PCM5102A full scale)
#define CFG_OUT_GAIN_Q15     22154      // −3.4 dBFS: sine frame (peak 25735) → 3 Vpp at OUTL (FS 5.65 Vpp measured; §1.3)

// ---- Table placement (§1.7) ---------------------------------------------
// 1: wavetable (≈353 KB) and exp2 LUT live in RAM, copied from flash at boot. Chosen because core0's
// table reads and core1's flash-resident code thrashed the shared XIP cache (core1 passes of 60–100 µs
// with almost no work, more core0 underruns; §1.7). 0: tables stay in flash (XIP).
#define CFG_TABLES_IN_RAM    1

// ---- Pitch range (§1.5) -------------------------------------------------
// Q16.16 octaves above C0. Integer part must not exceed 10 (exp2 LUT pre-shift, mipmap levels).
#define CFG_PITCH_MAX_Q      0x000AFFFF // just below C11

// ---- Smoothing time constants (§1.10), control-rate 1-pole lowpass -------
#define CFG_SMOOTH_KNOB_MS   5.0f       // coarse_cent + fine_cent
#define CFG_SMOOTH_FM_MS     2.0f       // fm_cent
#define CFG_SMOOTH_WT_MS     2.0f       // clamp(wt_knob + wt_cv, 0, 1)
// voct_cent is never smoothed.

// ---- V/OCT coarse ADC (wtvco_input_spec.md 2.2, 3) --------------------------
// Theoretical scale: the starting point of the automatic relative calibration (5.6, 9.3).
#define CFG_COARSE_VREF      3.3f       // ADC_AVDD
#define CFG_COARSE_DIV       0.6f       // 15k / (10k + 15k) after the MCP6001 follower

// ---- V/OCT fusion thresholds (wtvco_input_spec.md 5.4, 13.1) -------------
// Units: LSB of the 16-sample average of the coarse ADC (1 LSB ≈ 1.61 cent at the theoretical scale).
// Measured (input open, 0 V end, 26k software windows of 36 p): window range max 5.6 LSB,
// jump detector false triggers 0 for T ≥ 5 LSB (7 in 30 s at 4 LSB). Mid-range noise ≈ 15 % higher.
#define CFG_T_STILL_LSB      8          // ≈ 13 cent: 1.4 x the largest static window range
#define CFG_T_JUMP_LSB       10         // ≈ 16 cent: 2 x the false-trigger-free threshold, > T_STILL
#define CFG_FUSION_SMOOTH_MS 4.0f       // SMOOTH_MS (5.4, 5.5): FAST->ADS and DIRTY transitions
#define CFG_FUSION_PLAUSIBLE_CENT 200.0f  // ADS result rejected if this far from the coarse reading
static_assert(CFG_T_JUMP_LSB > CFG_T_STILL_LSB, "T_JUMP must exceed T_STILL (5.4)");

// Threshold study instrumentation in vin_coarse ("noise" command). Off in normal builds: it costs
// core1 loop time on every raw sample.
#define CFG_VIN_DIAG         0

// Console diagnostics (fcal, ads stop, i2s on/off, unplugged-USB monitor, fusion sequence dump).
// Only console commands; nothing runs unless typed. 0 removes them from the build.
#ifndef CFG_DIAG
#define CFG_DIAG             1
#endif

// ---- USB console (development / calibration) ----------------------------
// Toggled at each V/OCT step for latency measurement (wtvco_audio_process.md §1.13). RP2350 pins
// cannot be probed on this board. For a measurement set it to 1: GP1 is the PIC UART net, reachable on
// headers H6 / H12, but use it ONLY with the PIC removed (it would fight the PIC's TX through 4.7 kΩ).
// Default GP6 is unconnected, so the PIC can stay in its socket.
#define CFG_STUB_TRIG_PIN    6

// ---- Static checks ------------------------------------------------------
static_assert((CFG_OUT_RING_FRAMES & (CFG_OUT_RING_FRAMES - 1)) == 0, "CFG_OUT_RING_FRAMES must be a power of 2");
static_assert(CFG_OUT_LEAD_FRAMES >= 1 && CFG_OUT_LEAD_FRAMES <= 31, "CFG_OUT_LEAD_FRAMES must be 1..31");
static_assert(CFG_OUT_LEAD_FRAMES < CFG_OUT_RING_FRAMES / 2, "lead must stay below RING/2 for underrun detection");
static_assert(CFG_PITCH_MAX_Q > 0 && (CFG_PITCH_MAX_Q >> 16) <= 10, "CFG_PITCH_MAX_Q integer part must be 0..10");
static_assert(CFG_OUT_GAIN_Q15 >= -32768 && CFG_OUT_GAIN_Q15 <= 32767, "CFG_OUT_GAIN_Q15 must fit Q15");
