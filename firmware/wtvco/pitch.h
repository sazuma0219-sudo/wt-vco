// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Pitch: cent -> Q16.16 octaves above C0, clamp, exp2 LUT -> phase increment (§1.5, §1.6).
//
// Q16.16 pitch: integer part = octave (0..10 after clamp) = mipmap level,
//               fraction     = position inside the octave.
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "exp2_lut.h"

#define PITCH_MIN_Q   0
#define PITCH_MAX_Q   CFG_PITCH_MAX_Q
#define PITCH_CENT_TO_Q (65536.0f / 1200.0f)

static_assert(EXP2_LUT_SAMPLE_RATE == CFG_SAMPLE_RATE, "exp2_lut.h was generated for another sample rate");
static_assert(EXP2_LUT_PRESHIFT == 10, "exp2 LUT pre-shift must match the max pitch integer part");

// Control rate: rounded float -> Q16.16 (may be negative, e.g. FM or FINE).
// Rounds half away from zero inline (no libm call, so the caller can run from RAM).
static inline int32_t pitch_cent_to_q(float cent) {
    float q = cent * PITCH_CENT_TO_Q;
    return (int32_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
}

// Per sample: voct_cent (0..6000, already clamped by the input side) -> Q16.16.
// One multiply and a truncating conversion; truncation error ≤ 1 LSB ≈ 0.018 cent.
static inline int32_t pitch_voct_to_q(float voct_cent) {
    return (int32_t)(voct_cent * PITCH_CENT_TO_Q);
}

// Signed clamp to [C0, CFG_PITCH_MAX_Q].
static inline int32_t pitch_clamp(int32_t q) {
    if (q < PITCH_MIN_Q) return PITCH_MIN_Q;
    if (q > PITCH_MAX_Q) return PITCH_MAX_Q;
    return q;
}

// Mipmap level (= octave) of a clamped pitch: 0..10.
static inline uint32_t pitch_level(int32_t clamped_q) {
    return (uint32_t)clamped_q >> 16;
}

// 2^frac scaled by K * 2^10, before the octave shift.
// Index = top 10 bits of the fraction, linear interpolation with the low 6 bits.
static inline uint32_t exp2_lut_interp(uint32_t frac16) {
    uint32_t idx = frac16 >> 6;
    uint32_t f   = frac16 & 63u;
    uint32_t a   = EXP2_LUT[idx];
    uint32_t b   = EXP2_LUT[idx + 1];          // sentinel makes idx = 1023 safe
    return a + (((b - a) * f) >> 6);
}

// Phase increment for a clamped pitch (0 .. CFG_PITCH_MAX_Q). Always a right shift, rounded.
static inline uint32_t exp2_phase_inc(int32_t clamped_q) {
    uint32_t v  = exp2_lut_interp((uint32_t)clamped_q & 0xFFFFu);
    uint32_t sh = EXP2_LUT_PRESHIFT - ((uint32_t)clamped_q >> 16);
    return (v + ((1u << sh) >> 1)) >> sh;
}
