// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side tests for the automatic relative calibration (wtvco_input_spec.md 9.3).
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "../config.h"
#include "../vin_calib.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const double ADS_LSB = 187.5e-6, COARSE_V_PER_LSB = 3.3 / 4096 / 0.6;
static const double GAIN_THEORY = COARSE_V_PER_LSB / ADS_LSB;
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static double urand() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static double gauss() { double u = urand() + 1e-12, v = urand(); return sqrt(-2 * log(u)) * cos(6.283185307179586 * v); }

// Board model: coarse reading [V] = g * V + o. Pair noise: coarse window mean sd ~0.12 LSB, ADS sd 0.6 LSB.
static void feed(double v, int n, double g, double o) {
    for (int i = 0; i < n; i++) {
        double x = (g * v + o) / COARSE_V_PER_LSB + 0.12 * gauss();
        double y = v / ADS_LSB + 0.6 * gauss();
        calib_add((float)x, (int16_t)lround(y));
    }
}

// Worst error over 0..5 V, in cent, of ads predicted from the coarse value with the fitted calibration.
static double worst_cent(double g, double o) {
    const CalState &c = calib_state();
    double worst = 0;
    for (double v = 0; v <= 5.0; v += 0.01) {
        double x = (g * v + o) / COARSE_V_PER_LSB;
        double pred_v = (c.gain * x + c.offset) * ADS_LSB;
        worst = fmax(worst, fabs(pred_v - v) * 1200);
    }
    return worst;
}

int main() {
    const double g = 0.979, o = 0.021;     // measured on the board (wtvco_input_spec.md 13.1)
    printf("[uncalibrated (theory)]\n");
    calib_init((float)GAIN_THEORY, 0);
    printf("  worst error 0..5 V: %.1f cent\n", worst_cent(g, o));

    printf("[one held voltage (2 V): offset only]\n");
    calib_init((float)GAIN_THEORY, 0);
    feed(2.0, 2000, g, o);
    calib_solve();
    double e1 = worst_cent(g, o);
    double at2 = fabs(((calib_state().gain * ((g * 2 + o) / COARSE_V_PER_LSB) + calib_state().offset) * ADS_LSB - 2.0) * 1200);
    printf("  bins %u, fits %u, offset-only %u; error at 2 V %.2f cent, worst 0..5 V %.1f cent\n",
           calib_state().bins_used, calib_state().fits, calib_state().offset_only, at2, e1);
    CHECK(calib_state().fits == 0 && calib_state().offset_only == 1, "expected an offset-only update");
    CHECK(at2 < 0.5, "offset at the held voltage: %.2f cent", at2);

    printf("[staircase 0..4 V, 5 levels, as the sample CV]\n");
    calib_init((float)GAIN_THEORY, 0);
    for (int rep = 0; rep < 20; rep++)
        for (int k = 0; k <= 4; k++) feed(k * 1.0 + 0.03, 860, g, o);   // 1 s per step
    calib_solve();
    double e2 = worst_cent(g, o);
    printf("  bins %u, gain %.4f (ideal %.4f), offset %.1f ADS LSB; worst error 0..5 V %.2f cent\n",
           calib_state().bins_used, calib_state().gain, GAIN_THEORY / g, calib_state().offset, e2);
    CHECK(calib_state().fits == 1, "no gain fit");
    CHECK(e2 < 1.0, "calibrated error %.2f cent", e2);

    printf("[drift: the coarse offset moves by +5 mV, the staircase continues]\n");
    for (int rep = 0; rep < 40; rep++)
        for (int k = 0; k <= 4; k++) feed(k * 1.0 + 0.03, 860, g, o + 0.005);
    calib_solve();
    double e3 = worst_cent(g, o + 0.005);
    printf("  worst error 0..5 V after drift: %.2f cent\n", e3);
    CHECK(e3 < 1.0, "did not follow the drift: %.2f cent", e3);

    printf("[a gain far outside the tolerance is rejected]\n");
    calib_init((float)GAIN_THEORY, 0);
    for (int k = 0; k <= 4; k++) feed(k * 1.0, 500, 0.5, 0);
    bool ch = calib_solve();
    printf("  solve changed=%d, gain %.4f\n", ch, calib_state().gain);
    CHECK(!ch && fabs(calib_state().gain - GAIN_THEORY) < 1e-3, "absurd gain accepted");

    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
