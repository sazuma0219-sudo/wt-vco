// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side tests for the control-rate processing (wtvco_audio_process.md §1.13).
// Build & run: tests/run_tests.sh
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "../config.h"
#include "../control.h"
#include "../oscillator.h"
#include "../pitch.h"
#include "../shared_params.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const float CTRL_HZ = (float)CFG_SAMPLE_RATE / CFG_CTRL_PERIOD;

static void test_wt_combine() {
    printf("[WT = clamp(wt_knob + wt_cv, 0, 1)]\n");
    struct { float k, cv, expect; } c[] = {
        {1.0f, 1.5f, 1.0f}, {0.0f, -0.5f, 0.0f}, {0.5f, -0.5f, 0.0f}, {0.5f, 0.5f, 1.0f},
        {0.5f, 0.0f, 0.5f}, {0.3f, 0.2f, 0.5f}, {0.0f, 1.5f, 1.0f}, {1.0f, -0.5f, 0.5f},
    };
    for (auto &x : c) {
        float w = control_wt_combine(x.k, x.cv);
        printf("  knob %4.1f + cv %4.1f -> %.3f\n", x.k, x.cv, w);
        CHECK(fabsf(w - x.expect) < 1e-6f, "knob %f cv %f -> %f, expected %f", x.k, x.cv, w, x.expect);
    }
    // KNOB 0.5 with CV -0.5 .. +0.5 sweeps the whole 0 .. 1 range monotonically.
    float prev = -1.0f, lo = 2, hi = -1;
    for (int i = 0; i <= 100; i++) {
        float w = control_wt_combine(0.5f, -0.5f + i * 0.01f);
        CHECK(w >= prev, "not monotonic at step %d", i);
        prev = w;
        if (w < lo) lo = w;
        if (w > hi) hi = w;
    }
    printf("  knob 0.5, cv -0.5..+0.5 -> %.3f .. %.3f\n", lo, hi);
    CHECK(lo == 0.0f && fabsf(hi - 1.0f) < 1e-6f, "range %f..%f", lo, hi);
}

// Steps the shared variable and measures how many control ticks the smoothed value needs to reach 63 %.
static void test_time_constants() {
    printf("[smoothing] 63 %% step response at %.0f Hz control rate\n", CTRL_HZ);
    ControlOut out;

    // Knob pitch: coarse 3600 -> 4800 (+1 oct).
    g_shared.coarse_cent = 3600; g_shared.fine_cent = 0; g_shared.fm_cent = 0;
    g_shared.wt_knob = 0; g_shared.wt_cv = 0; g_shared.voct_cent = 0;
    control_init(&out);
    CHECK(out.ctl_q == (3 << 16), "init knob %d", out.ctl_q);
    g_shared.coarse_cent = 4800;
    int n = 0;
    while (out.ctl_q < (3 << 16) + (int32_t)(0.632 * 65536) && n < 10000) { control_update(&out); n++; }
    printf("  knob : %.2f ms (CFG %.1f ms)\n", n * 1000.0 / CTRL_HZ, CFG_SMOOTH_KNOB_MS);
    CHECK(fabs(n * 1000.0 / CTRL_HZ - CFG_SMOOTH_KNOB_MS) < 0.3, "knob tau");

    // FM: 0 -> +1200, separate and faster.
    g_shared.coarse_cent = 3600;
    control_init(&out);
    g_shared.fm_cent = 1200;
    n = 0;
    while (out.ctl_q < (3 << 16) + (int32_t)(0.632 * 65536) && n < 10000) { control_update(&out); n++; }
    printf("  fm   : %.2f ms (CFG %.1f ms)\n", n * 1000.0 / CTRL_HZ, CFG_SMOOTH_FM_MS);
    CHECK(fabs(n * 1000.0 / CTRL_HZ - CFG_SMOOTH_FM_MS) < 0.3, "fm tau");

    // WT: 0 -> 1 (after combine).
    g_shared.fm_cent = 0;
    control_init(&out);
    g_shared.wt_knob = 1.0f;
    n = 0;
    while (out.wt_q < (uint32_t)(0.632 * OSC_WT_MAX_Q) && n < 10000) { control_update(&out); n++; }
    printf("  wt   : %.2f ms (CFG %.1f ms)\n", n * 1000.0 / CTRL_HZ, CFG_SMOOTH_WT_MS);
    CHECK(fabs(n * 1000.0 / CTRL_HZ - CFG_SMOOTH_WT_MS) < 0.3, "wt tau");
    for (int i = 0; i < 1000; i++) control_update(&out);
    CHECK(out.wt_q == OSC_WT_MAX_Q, "wt settles at max (%u)", out.wt_q);

    // Extreme combination: everything at the top, FM +18000 -> ctl_q fits int32 and clamps to max with V/OCT.
    g_shared.coarse_cent = 7200; g_shared.fine_cent = 100; g_shared.fm_cent = 18000;
    control_init(&out);
    int32_t p = pitch_clamp(out.ctl_q + pitch_voct_to_q(6000));
    printf("  extreme: ctl_q %d (%.2f oct) -> clamped %d\n", out.ctl_q, out.ctl_q / 65536.0, p);
    CHECK(p == PITCH_MAX_Q, "extreme clamp");
    g_shared.coarse_cent = 0; g_shared.fine_cent = -100; g_shared.fm_cent = -6000;
    control_init(&out);
    p = pitch_clamp(out.ctl_q + pitch_voct_to_q(0));
    CHECK(p == PITCH_MIN_Q, "extreme low clamp");

    // Runtime change ('tune'): the new knob tau applies at once and survives control_init.
    g_shared.coarse_cent = 3600; g_shared.fine_cent = 0; g_shared.fm_cent = 0;
    control_init(&out);
    control_set_smoothing(10.0f, CFG_SMOOTH_FM_MS, CFG_SMOOTH_WT_MS);
    g_shared.coarse_cent = 4800;
    n = 0;
    while (out.ctl_q < (3 << 16) + (int32_t)(0.632 * 65536) && n < 10000) { control_update(&out); n++; }
    printf("  knob after set_smoothing(10 ms): %.2f ms\n", n * 1000.0 / CTRL_HZ);
    CHECK(fabs(n * 1000.0 / CTRL_HZ - 10.0) < 0.3, "retuned knob tau");
    float k, f, w;
    control_get_smoothing(&k, &f, &w);
    CHECK(k == 10.0f && f == CFG_SMOOTH_FM_MS && w == CFG_SMOOTH_WT_MS, "get_smoothing");
    control_set_smoothing(CFG_SMOOTH_KNOB_MS, CFG_SMOOTH_FM_MS, CFG_SMOOTH_WT_MS);
}

int main() {
    test_wt_combine();
    test_time_constants();
    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
