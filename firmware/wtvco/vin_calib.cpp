// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "vin_calib.h"
#include "config.h"

static CalState s;

void calib_init(float gain_theory, float offset) {
    s = CalState{};
    s.gain = s.gain_theory = gain_theory;
    s.offset = offset;
}

void calib_set_start(float gain, float offset) {
    s.gain = gain;
    s.offset = offset;
}

void RAM_FUNC(calib_add)(float x, int16_t code) {
    int b = (int)(x * CAL_BINS / 4096.0f);
    if (b < 0) b = 0;
    if (b >= CAL_BINS) b = CAL_BINS - 1;
    CalBin &k = s.bin[b];
    if (k.n < CAL_BIN_MAX_N) k.n++;
    float w = 1.0f / (float)k.n;               // exact mean up to MAX_N, then a 1/MAX_N moving average
    k.x += (x - k.x) * w;
    k.y += ((float)code - k.y) * w;
    s.pairs++;
}

bool calib_solve() {
    // Each populated bin is one point.
    float sx = 0, sy = 0, sxx = 0, sxy = 0, xmin = 1e9f, xmax = -1e9f;
    int m = 0;
    for (int i = 0; i < CAL_BINS; i++) {
        const CalBin &k = s.bin[i];
        if (k.n < CAL_BIN_MIN_N) continue;
        sx += k.x; sy += k.y; sxx += k.x * k.x; sxy += k.x * k.y;
        if (k.x < xmin) xmin = k.x;
        if (k.x > xmax) xmax = k.x;
        m++;
    }
    s.bins_used = (uint8_t)m;
    if (m == 0) return false;
    float g = s.gain, o;
    if (m >= 2 && xmax - xmin >= CAL_MIN_SPAN_LSB) {
        float den = m * sxx - sx * sx;
        if (den <= 0) return false;
        g = (m * sxy - sx * sy) / den;
        // Sanity: the divider and ADC_AVDD tolerances are a few %, never 20 %.
        if (g < s.gain_theory * 0.8f || g > s.gain_theory * 1.2f) return false;
        o = (sy - g * sx) / m;
        s.fits++;
    } else {
        o = (sy - g * sx) / m;                   // keep the gain, match the offset at the known points
        s.offset_only++;
    }
    bool changed = g != s.gain || o != s.offset;
    s.gain = g;
    s.offset = o;
    return changed;
}

const CalState &calib_state() { return s; }
