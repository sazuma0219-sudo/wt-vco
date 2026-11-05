// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side tests for pitch / exp2 LUT / wavetable layout (wtvco_audio_process.md §1.13).
// Build & run: tests/run_tests.sh
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "../config.h"
#include "../pitch.h"
#include "../wavetable.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static double f_c0() { return 440.0 * pow(2.0, -57.0 / 12.0); }
static double K()    { return f_c0() * 4294967296.0 / CFG_SAMPLE_RATE; }
static double cents(double a, double b) { return 1200.0 * log2(a / b); }

// Mirrors the audio path: control-rate (knob + fm) + per-sample voct, then clamp.
static int32_t pitch_from_cents(float coarse, float fine, float voct, float fm, int32_t *unclamped) {
    int32_t knob_q = pitch_cent_to_q(coarse + fine);
    int32_t fm_q   = pitch_cent_to_q(fm);
    int32_t ctl_q  = knob_q + fm_q;
    int32_t voct_q = pitch_voct_to_q(voct);
    if (unclamped) *unclamped = ctl_q + voct_q;
    return pitch_clamp(ctl_q + voct_q);
}

static void test_lut_preshift() {
    printf("[exp2 LUT, before shift] all 65536 fractions\n");
    double worst = 0;
    for (uint32_t frac = 0; frac < 65536; frac++) {
        double exact = K() * 1024.0 * pow(2.0, frac / 65536.0);
        double e = fabs(cents(exp2_lut_interp(frac), exact));
        if (e > worst) worst = e;
    }
    printf("  worst = %.6f cent (limit 0.001)\n", worst);
    CHECK(worst < 0.001, "LUT error %.6f cent", worst);
    CHECK(EXP2_LUT[EXP2_LUT_SIZE] == (uint32_t)llround(K() * 2048.0), "sentinel != K*2^11");
}

static void test_phase_inc_full_range() {
    printf("[phase_inc] every Q16.16 pitch 0 .. CFG_PITCH_MAX_Q (0x%08X)\n", CFG_PITCH_MAX_Q);
    double worst = 0; int32_t worst_q = 0; uint32_t max_inc = 0;
    for (int32_t q = 0; q <= CFG_PITCH_MAX_Q; q++) {
        uint32_t inc = exp2_phase_inc(q);
        double exact = K() * pow(2.0, q / 65536.0);
        double e = fabs(cents(inc, exact));
        if (e > worst) { worst = e; worst_q = q; }
        if (inc > max_inc) max_inc = inc;
    }
    printf("  worst = %.6f cent at pitch %.5f (limit 0.002)\n", worst, worst_q / 65536.0);
    printf("  max phase_inc = %u (%.1f Hz), must be < 2^31\n", max_inc, max_inc * (double)CFG_SAMPLE_RATE / 4294967296.0);
    CHECK(worst < 0.002, "phase_inc error %.6f cent", worst);
    CHECK(max_inc < 0x80000000u, "phase_inc reaches Nyquist");
    // Monotonic across octave boundaries (shift changes there).
    for (int oct = 1; oct <= 10; oct++) {
        int32_t q = oct << 16;
        CHECK(exp2_phase_inc(q) > exp2_phase_inc(q - 1), "not monotonic at octave %d", oct);
    }
}

static void test_reference_pitches() {
    printf("[pitch -> frequency]\n");
    struct { int oct; const char *name; double spec; } ref[] = {
        {0, "C0", 16.352}, {3, "C3", 130.81}, {6, "C6", 1046.50},
    };
    for (auto &r : ref) {
        double f = exp2_phase_inc(r.oct << 16) * (double)CFG_SAMPLE_RATE / 4294967296.0;
        double ideal = 440.0 * pow(2.0, (12.0 * r.oct - 57.0) / 12.0);
        printf("  %s: %.4f Hz (ideal %.4f, spec %.3f)\n", r.name, f, ideal, r.spec);
        CHECK(fabs(cents(f, ideal)) < 0.002, "%s off by %.5f cent", r.name, cents(f, ideal));
        CHECK(fabs(f - r.spec) < 0.01, "%s %.4f differs from spec table %.3f", r.name, f, r.spec);
    }
    double fmax = exp2_phase_inc(CFG_PITCH_MAX_Q) * (double)CFG_SAMPLE_RATE / 4294967296.0;
    printf("  max (0x%08X): %.1f Hz\n", CFG_PITCH_MAX_Q, fmax);
}

static void test_cent_conversion_and_clamp() {
    printf("[cent -> Q16.16, clamp]\n");
    CHECK(pitch_cent_to_q(1200.0f) == 65536, "1200 cent != 1 octave");
    CHECK(pitch_cent_to_q(3600.0f) == (3 << 16), "3600 cent != C3");
    CHECK(pitch_cent_to_q(-100.0f) == -5461, "-100 cent -> %d", pitch_cent_to_q(-100.0f));
    CHECK(pitch_voct_to_q(6000.0f) == 5 * 65536, "voct 6000 -> %d", pitch_voct_to_q(6000.0f));
    // V/OCT conversion error on a 0.01 cent grid: truncation (0..1 LSB) plus the single-precision
    // rounding of the product (float ulp at 5 oct = 1/64 LSB ≈ 0.0003 cent), so it may dip slightly below 0.
    double worst = 0, lowest = 0;
    for (int i = 0; i <= 600000; i++) {
        float c = i * 0.01f;
        double e = c * 65536.0 / 1200.0 - pitch_voct_to_q(c);
        if (e > worst) worst = e;
        if (e < lowest) lowest = e;
    }
    printf("  voct conversion error = %.4f .. %.4f LSB (%.4f .. %.4f cent)\n",
           lowest, worst, lowest * 1200.0 / 65536.0, worst * 1200.0 / 65536.0);
    CHECK(worst <= 1.02 && lowest >= -0.02, "voct conversion %.4f..%.4f LSB", lowest, worst);

    struct { const char *name; float coarse, fine, voct, fm; int32_t expect; } cases[] = {
        {"COARSE 0 + FINE -100",                         0, -100,    0,      0, PITCH_MIN_Q},
        {"COARSE 7200 + FINE +100 + VOCT 6000",       7200,  100, 6000,      0, PITCH_MAX_Q},
        {"COARSE 0 + FM -6000",                          0,    0,    0,  -6000, PITCH_MIN_Q},
        {"COARSE 7200 + FINE +100 + VOCT 6000 + FM +18000", 7200, 100, 6000, 18000, PITCH_MAX_Q},
        {"COARSE 0 + FINE -100 + FM -6000 (min sum)",    0, -100,    0,  -6000, PITCH_MIN_Q},
        {"COARSE 3600 (C3, PIC default)",             3600,    0,    0,      0, 3 << 16},
        {"COARSE 3600 + VOCT 1200",                   3600,    0, 1200,      0, 4 << 16},
    };
    for (auto &c : cases) {
        int32_t raw;
        int32_t q = pitch_from_cents(c.coarse, c.fine, c.voct, c.fm, &raw);
        double raw_expect = (c.coarse + c.fine + c.voct + c.fm) * 65536.0 / 1200.0;
        printf("  %-50s sum %9d (%.2f oct) -> %8d level %u\n", c.name, raw, raw / 65536.0, q, pitch_level(q));
        CHECK(fabs(raw - raw_expect) <= 2.0, "%s: unclamped sum %d, expected %.1f (overflow?)", c.name, raw, raw_expect);
        CHECK(q == c.expect, "%s: got %d, expected %d", c.name, q, c.expect);
        CHECK(pitch_level(q) <= 10, "%s: level %u", c.name, pitch_level(q));
    }
}

static void test_levels() {
    printf("[mipmap level] clamp() output over the full input range\n");
    uint32_t lo = 99, hi = 0;
    for (int32_t raw = -(8 << 16); raw <= (30 << 16); raw += 97) {
        uint32_t l = pitch_level(pitch_clamp(raw));
        if (l < lo) lo = l;
        if (l > hi) hi = l;
    }
    printf("  level range = %u .. %u\n", lo, hi);
    CHECK(lo == 0 && hi == 10, "level range %u..%u", lo, hi);
}

static void test_wavetable_layout() {
    printf("[wavetable.h] lengths / max harmonics vs §1.7 table\n");
    static const uint16_t spec_maxh[11] = {1023, 733, 366, 183, 91, 45, 22, 11, 5, 2, 1};
    static const uint16_t spec_len[11]  = {2048, 2048, 2048, 2048, 1024, 512, 256, 256, 256, 256, 256};
    CHECK(WT_NUM_FRAMES == 16 && WT_NUM_LEVELS == 11, "frames/levels");
    uint32_t off = 0;
    for (int l = 0; l < WT_NUM_LEVELS; l++) {
        uint32_t len = 1u << WT_LEVEL_LOG2LEN[l];
        CHECK(len == spec_len[l], "level %d len %u != %u", l, len, spec_len[l]);
        CHECK(WT_LEVEL_MAXH[l] == spec_maxh[l], "level %d maxh %u != %u", l, WT_LEVEL_MAXH[l], spec_maxh[l]);
        CHECK(WT_LEVEL_OFFSET[l] == off, "level %d offset", l);
        off += len + 1;
        for (int f = 0; f < WT_NUM_FRAMES; f++) {
            const int16_t *t = &WT_DATA[f][WT_LEVEL_OFFSET[l]];
            CHECK(t[len] == t[0], "frame %d level %d wrap sample", f, l);
        }
    }
    CHECK(off == WT_FRAME_STRIDE, "stride %u != %u", (unsigned)WT_FRAME_STRIDE, off);
    printf("  stride = %u samples, %u bytes total\n", (unsigned)WT_FRAME_STRIDE,
           (unsigned)(sizeof(WT_DATA)));
}

int main() {
    test_lut_preshift();
    test_phase_inc_full_range();
    test_reference_pitches();
    test_cent_conversion_and_clamp();
    test_levels();
    test_wavetable_layout();
    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
