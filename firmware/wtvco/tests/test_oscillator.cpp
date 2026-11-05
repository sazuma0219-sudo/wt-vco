// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side tests for the oscillator (wtvco_audio_process.md §1.13).
// Build & run: tests/run_tests.sh
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "../config.h"
#include "../pitch.h"
#include "../oscillator.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng_state = 12345;
static uint32_t rnd() { rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5; return rng_state; }

static void test_wt_position() {
    printf("[WT -> frame position]\n");
    struct { float wt; uint32_t f; uint32_t frac8; } c[] = {
        {0.0f, 0, 0}, {0.5f, 7, 128}, {1.0f, 14, 255}, {1.5f, 14, 255}, {-0.5f, 0, 0}, {NAN, 0, 0},
    };
    for (auto &x : c) {
        uint32_t q = osc_wt_to_q(x.wt);
        printf("  wt %5.2f -> 0x%06X  frame %u frac8 %u\n", x.wt, q, q >> 16, (q >> 8) & 0xFF);
        CHECK((q >> 16) == x.f && ((q >> 8) & 0xFF) == x.frac8, "wt %f -> frame %u frac %u", x.wt, q >> 16, (q >> 8) & 0xFF);
        CHECK((q >> 16) + 1 < WT_NUM_FRAMES, "frame + 1 out of range for wt %f", x.wt);
    }
}

static void test_index_range() {
    printf("[table index range] max phase at every level, max WT\n");
    for (uint32_t l = 0; l < WT_NUM_LEVELS; l++) {
        uint32_t lg = WT_LEVEL_LOG2LEN[l];
        uint32_t len = 1u << lg;
        uint32_t idx_max = 0xFFFFFFFFu >> (32 - lg);
        CHECK(idx_max + 1 == len, "level %u: idx+1 = %u, table has %u+1 samples", l, idx_max + 1, len);
        uint32_t f = OSC_WT_MAX_Q >> 16;
        // Last sample read: frame f+1, offset + idx + 1. Must stay inside WT_DATA.
        size_t last = (size_t)(f + 1) * WT_FRAME_STRIDE + WT_LEVEL_OFFSET[l] + idx_max + 1;
        CHECK(last < (size_t)WT_NUM_FRAMES * WT_FRAME_STRIDE, "level %u reads past WT_DATA", l);
        CHECK(WT_LEVEL_OFFSET[l] + idx_max + 1 < (l + 1 < WT_NUM_LEVELS ? WT_LEVEL_OFFSET[l + 1] : WT_FRAME_STRIDE),
              "level %u reads into the next level", l);
        uint32_t ph = 0xFFFFFFFFu;
        int32_t s = osc_render(&ph, 0, l, OSC_WT_MAX_Q);
        CHECK(s >= -32768 && s <= 32767, "level %u sample %d", l, s);
    }
}

// Reference with the same 8-bit fractions but exact arithmetic.
static double ref_sample(uint32_t phase, uint32_t level, uint32_t wt_q) {
    uint32_t lg = WT_LEVEL_LOG2LEN[level];
    uint32_t f = wt_q >> 16, idx = phase >> (32 - lg);
    double pf = ((phase << lg) >> 24) / 256.0, ff = ((wt_q >> 8) & 0xFF) / 256.0;
    const int16_t *t0 = &WT_DATA[f][WT_LEVEL_OFFSET[level]], *t1 = t0 + WT_FRAME_STRIDE;
    double a = t0[idx] + (t0[idx + 1] - t0[idx]) * pf;
    double b = t1[idx] + (t1[idx + 1] - t1[idx]) * pf;
    return a + (b - a) * ff;
}

static void test_interpolation() {
    printf("[interpolation] 1e6 random phase / level / WT against exact arithmetic\n");
    double worst = 0;
    for (int i = 0; i < 1000000; i++) {
        uint32_t ph = rnd(), l = rnd() % WT_NUM_LEVELS, wq = rnd() % (OSC_WT_MAX_Q + 1);
        uint32_t p = ph;
        double e = fabs(osc_render(&p, 0, l, wq) - ref_sample(ph, l, wq));
        if (e > worst) worst = e;
    }
    printf("  worst = %.3f LSB (3 truncating lerps, limit 3)\n", worst);
    CHECK(worst <= 3.0, "interpolation error %.3f LSB", worst);
}

static void test_gain_no_clip() {
    printf("[output gain] peak of every table x CFG_OUT_GAIN_Q15 (%d)\n", CFG_OUT_GAIN_Q15);
    int32_t peak = 0;
    for (int f = 0; f < WT_NUM_FRAMES; f++)
        for (uint32_t i = 0; i < WT_FRAME_STRIDE; i++) {
            int32_t v = abs(WT_DATA[f][i]);
            if (v > peak) peak = v;
        }
    int32_t out_pos = (peak * CFG_OUT_GAIN_Q15) >> 15, out_neg = (-peak * CFG_OUT_GAIN_Q15) >> 15;
    printf("  table peak %d -> output %d / %d (int16 range, 32-bit word %08X)\n",
           peak, out_pos, out_neg, (uint32_t)out_pos << 16);
    // Linear interpolation never leaves the range of its end points, so the table peak bounds the output.
    CHECK(out_pos <= 32767 && out_neg >= -32768, "output clips");
}

static void test_frequency() {
    printf("[render] 1 s at C3 / WT 0.5: count rising zero crossings\n");
    int32_t pitch_q = pitch_clamp(pitch_cent_to_q(3600.0f));
    uint32_t inc = exp2_phase_inc(pitch_q), lvl = pitch_level(pitch_q), wq = osc_wt_to_q(0.5f);
    uint32_t ph = 0, crossings = 0;
    int32_t prev = 0, peak = 0;
    for (int n = 0; n < CFG_SAMPLE_RATE; n++) {
        uint32_t before = ph;
        int32_t s = osc_render(&ph, inc, lvl, wq);
        if (ph < before) crossings++;           // phase wraps once per period
        if (abs(s) > peak) peak = abs(s);
        prev = s;
    }
    (void)prev;
    printf("  level %u, %u periods/s (C3 = 130.81 Hz), peak %d\n", lvl, crossings, peak);
    CHECK(lvl == 3, "level %u", lvl);
    CHECK(crossings == 130 || crossings == 131, "periods %u", crossings);
}

int main() {
    test_wt_position();
    test_index_range();
    test_interpolation();
    test_gain_no_clip();
    test_frequency();
    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
