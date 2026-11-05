// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Wavetable oscillator: phase accumulator, mipmap/frame lookup, interpolation (§1.7, §1.8).
// Everything here runs per sample and is integer only.
#pragma once
#include <stdint.h>
#include "config.h"
#include "wavetable.h"

// WT position, Q16.16: integer part = frame (0 .. WT_NUM_FRAMES - 2), fraction = crossfade to the next frame.
// Clamped to (WT_NUM_FRAMES - 1) - 1 LSB so that frame + 1 is always a valid frame (WT = 1.0 -> f = 14, frac max).
#define OSC_WT_MAX_Q ((uint32_t)(WT_NUM_FRAMES - 1) * 65536u - 1u)

// Control rate: WT 0.0 .. 1.0 (already clamped) -> Q16.16 frame position.
static inline uint32_t osc_wt_to_q(float wt) {
    float p = wt * (float)(WT_NUM_FRAMES - 1) * 65536.0f;
    if (!(p > 0.0f)) return 0;                     // also catches NaN
    if (p >= (float)OSC_WT_MAX_Q) return OSC_WT_MAX_Q;
    return (uint32_t)p;
}

static inline int32_t osc_lerp8(int32_t a, int32_t b, int32_t frac8) {
    return a + (((b - a) * frac8) >> 8);
}

// One sample (int16 range) at the current phase, then advance the phase.
//   level: mipmap level 0 .. WT_NUM_LEVELS - 1 (= pitch integer part)
//   wt_q:  0 .. OSC_WT_MAX_Q
static inline int32_t osc_render(uint32_t *phase, uint32_t phase_inc, uint32_t level, uint32_t wt_q) {
    uint32_t f      = wt_q >> 16;
    int32_t  ffrac  = (int32_t)((wt_q >> 8) & 0xFFu);
    uint32_t lg     = WT_LEVEL_LOG2LEN[level];
    const int16_t *t0 = &WT_DATA[f][WT_LEVEL_OFFSET[level]];
    const int16_t *t1 = t0 + WT_FRAME_STRIDE;      // same level, next frame

    uint32_t ph     = *phase;
    uint32_t idx    = ph >> (32 - lg);
    int32_t  pfrac  = (int32_t)((ph << lg) >> 24); // 8 bit
    int32_t  a      = osc_lerp8(t0[idx], t0[idx + 1], pfrac);   // idx + 1 <= len: wrap sample
    int32_t  b      = osc_lerp8(t1[idx], t1[idx + 1], pfrac);
    *phase = ph + phase_inc;
    return osc_lerp8(a, b, ffrac);
}
