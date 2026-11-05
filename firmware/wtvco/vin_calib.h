// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Automatic relative calibration coarse ADC <-> ADS1115 (wtvco_input_spec.md 9.3).
// Every clean conversion gives a pair (coarse average of the window, ADS1115 code) for the SAME
// voltage, so the coarse scale can be fitted to the ADS1115 without any external reference.
//   - pairs are sorted into CAL_BINS voltage ranges; each bin keeps a running mean, so a voltage that is
//     held for a long time does not outweigh the others (9.3: collect from several ranges)
//   - fit: least squares over the bin means (each bin one point); with only one usable bin just the offset
//   - result: ads = gain * coarse_avg + offset (the fusion's to_ads_scale), RAM only, not saved (9.5)
// Pure logic (host-testable).
#pragma once
#include <stdint.h>

#define CAL_BINS          8          // over the coarse range 0 .. 4095
#define CAL_BIN_MIN_N     64         // pairs before a bin counts (≈ 74 ms of a held voltage)
#define CAL_BIN_MAX_N     4096       // after this the bin mean becomes a moving average (follows drift)
#define CAL_MIN_SPAN_LSB  600.0f     // coarse LSB between the outer bins before the gain is fitted (≈ 1.3 V)

struct CalBin { float x, y; uint32_t n; };   // running means: x = coarse avg LSB, y = ADS code

struct CalState {
    CalBin   bin[CAL_BINS];
    float    gain, offset;           // current result
    float    gain_theory;
    uint32_t pairs;                  // pairs collected
    uint32_t fits;                   // successful fits (gain + offset)
    uint32_t offset_only;            // offset-only updates
    uint8_t  bins_used;              // bins with >= CAL_BIN_MIN_N in the last solve
};

void calib_init(float gain_theory, float offset);
void calib_set_start(float gain, float offset);  // saved values (9.5): used until the first fit
void calib_add(float coarse_avg_lsb, int16_t ads_code);   // one clean pair
bool calib_solve();                  // recompute gain/offset; true if they changed
const CalState &calib_state();
