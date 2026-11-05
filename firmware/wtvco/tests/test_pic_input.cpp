// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side tests for the PIC value conversion (wtvco_input_spec.md 8.2 - 8.6).
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "../pic_input.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

enum { WT_UNI, WT_BI, FM_UNI, FM_BI, COARSE, FINE, KNOB, DEPTH };
static const uint16_t FULL = 16380;
static uint16_t volts(float v) { return (uint16_t)lroundf(v / 5.0f * FULL); }

// Feed the same frame for 'ms' milliseconds; returns the output after the last one.
static PicInputOut run(const uint16_t raw[8], uint32_t &t, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) { pic_input_frame(raw, t); t++; }
    return pic_input_update(t);
}

static void fresh() { PicInputConfig c; pic_input_default_config(&c); pic_input_init(c); }

int main() {
    uint32_t t = 1000;
    printf("[no frame yet -> 8.2 defaults]\n");
    fresh();
    PicInputOut o = pic_input_update(t);
    printf("  coarse %.0f fine %.0f knob %.2f wt_cv %.2f fm %.0f ok=%d\n", o.coarse_cent, o.fine_cent, o.wt_knob, o.wt_cv, o.fm_cent, o.pic_ok);
    CHECK(!o.pic_ok && o.coarse_cent == 3600 && o.fine_cent == 0 && o.wt_knob == 0 && o.wt_cv == 0 && o.fm_cent == 0, "defaults");

    printf("[all knobs centred, CVs unplugged (UNI 0 V, BI 2.5 V)]\n");
    uint16_t raw[8] = {0, 8190, 0, 8190, 8190, 8190, 8190, 8190};
    o = run(raw, t, 1000);
    printf("  coarse %.1f fine %.2f knob %.3f wt_cv %.4f fm %.2f\n", o.coarse_cent, o.fine_cent, o.wt_knob, o.wt_cv, o.fm_cent);
    CHECK(o.pic_ok, "ok");
    CHECK(fabsf(o.coarse_cent - 3600) < 5, "coarse mid %.1f", o.coarse_cent);
    CHECK(fabsf(o.fine_cent) < 0.2f, "fine mid %.2f", o.fine_cent);
    CHECK(fabsf(o.wt_knob - 0.5f) < 0.005f, "knob mid %.3f", o.wt_knob);
    CHECK(fabsf(o.wt_cv) < 1e-3f && fabsf(o.fm_cent) < 0.5f, "unplugged CVs must be 0 (BI mid-scale)");

    printf("[ranges: knobs fully up / down]\n");
    uint16_t up[8] = {0, 8190, 0, 8190, FULL, FULL, FULL, FULL};
    o = run(up, t, 2000);
    printf("  up:   coarse %.1f fine %.2f knob %.3f\n", o.coarse_cent, o.fine_cent, o.wt_knob);
    CHECK(fabsf(o.coarse_cent - 7200) < 5 && fabsf(o.fine_cent - 100) < 0.2f && fabsf(o.wt_knob - 1) < 0.005f, "top");
    uint16_t dn[8] = {0, 8190, 0, 8190, 0, 0, 0, 0};
    o = run(dn, t, 2000);
    printf("  down: coarse %.1f fine %.2f knob %.3f\n", o.coarse_cent, o.fine_cent, o.wt_knob);
    CHECK(o.coarse_cent < 5 && fabsf(o.fine_cent + 100) < 0.2f && o.wt_knob < 0.005f, "bottom");

    printf("[COARSE dead band: ±2 raw LSB of jitter does not move the output]\n");
    fresh();
    uint16_t c[8] = {0, 8190, 0, 8190, 8000, 8190, 8190, 0};
    o = run(c, t, 1000);
    float base = o.coarse_cent, lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < 5000; i++) {
        c[COARSE] = (uint16_t)(8000 + ((i * 7919) % 5) - 2);   // ±2 raw LSB (±0.9 cent) jitter
        pic_input_frame(c, t++);
        float v = pic_input_update(t).coarse_cent;
        lo = fminf(lo, v); hi = fmaxf(hi, v);
    }
    printf("  jitter ±2 raw LSB -> output range %.3f cent (base %.1f)\n", hi - lo, base);
    CHECK(hi - lo < 0.01f, "coarse output moves with jitter: %.3f cent", hi - lo);
    c[COARSE] = 8100;                                            // +44 cent: must follow
    o = run(c, t, 2000);
    printf("  step +100 raw LSB -> %.1f cent (expected %.1f within the dead band)\n", o.coarse_cent, 8100.0f / FULL * 7200);
    CHECK(fabsf(o.coarse_cent - 8100.0f / FULL * 7200) <= 4.6f, "coarse did not follow a real move");

    printf("[WT CV: UNI 5 V -> +1.0, BI 0 V -> -0.5, both -> +0.5; not clamped]\n");
    fresh();
    uint16_t w[8] = {volts(5), 8190, 0, 8190, 8190, 8190, 8190, 0};
    o = run(w, t, 100);   CHECK(fabsf(o.wt_cv - 1.0f) < 0.01f, "UNI 5 V: %.3f", o.wt_cv);
    w[WT_UNI] = 0; w[WT_BI] = 0;
    o = run(w, t, 100);   CHECK(fabsf(o.wt_cv + 0.5f) < 0.01f, "BI 0 V: %.3f", o.wt_cv);
    w[WT_UNI] = volts(5); w[WT_BI] = FULL;
    o = run(w, t, 100);   CHECK(fabsf(o.wt_cv - 1.5f) < 0.01f, "UNI 5 V + BI 5 V: %.3f", o.wt_cv);
    printf("  UNI 5 V + BI 5 V -> %.3f (the audio side clamps WT once, 10.3)\n", o.wt_cv);

    printf("[FM: DEPTH curve and CV]\n");
    fresh();
    uint16_t f[8] = {0, 8190, volts(1.0f), 8190, 8190, 8190, 8190, FULL};
    o = run(f, t, 500);
    printf("  DEPTH max, FM UNI 1 V -> %.1f cent (2400)\n", o.fm_cent);
    CHECK(fabsf(o.fm_cent - 2400) < 10, "depth max");
    f[DEPTH] = (uint16_t)lroundf(0.5f * FULL);
    o = run(f, t, 500);
    printf("  DEPTH 0.5, FM UNI 1 V -> %.1f cent (600)\n", o.fm_cent);
    CHECK(fabsf(o.fm_cent - 600) < 5, "depth half");
    f[FM_UNI] = volts(5); f[FM_BI] = FULL; f[DEPTH] = FULL;
    o = run(f, t, 500);
    printf("  DEPTH max, UNI 5 V + BI 5 V -> %.0f cent (+18000, not clamped)\n", o.fm_cent);
    CHECK(fabsf(o.fm_cent - 18000) < 50, "fm max");
    f[FM_UNI] = 0; f[FM_BI] = 0;
    o = run(f, t, 500);
    CHECK(fabsf(o.fm_cent + 6000) < 50, "fm min %.0f", o.fm_cent);
    f[DEPTH] = 0; f[FM_UNI] = volts(5);
    o = run(f, t, 500);
    CHECK(fabsf(o.fm_cent) < 0.5f, "depth 0 -> no FM (%.2f)", o.fm_cent);

    printf("[frame loss: hold 100 ms, then defaults; recovery]\n");
    fresh();
    uint16_t k[8] = {0, 8190, 0, 8190, 12000, 12000, 12000, 0};
    o = run(k, t, 500);
    float held = o.coarse_cent;
    o = pic_input_update(t + 90);
    CHECK(o.pic_ok && o.coarse_cent == held, "should hold at 90 ms");
    o = pic_input_update(t + 150);
    printf("  after 150 ms without frames: coarse %.0f ok=%d\n", o.coarse_cent, o.pic_ok);
    CHECK(!o.pic_ok && o.coarse_cent == 3600 && o.wt_knob == 0, "defaults after the hold time");
    t += 150;
    o = run(k, t, 1);
    printf("  first frame back: coarse %.0f ok=%d (smoothers restart from the new value)\n", o.coarse_cent, o.pic_ok);
    CHECK(o.pic_ok && fabsf(o.coarse_cent - 12000.0f / FULL * 7200) < 1, "recovery");

    printf("[retune while running: state kept, new time constant takes effect]\n");
    fresh();
    uint16_t r_lo[8] = {0, 8190, 0, 8190, 4000, 8190, 8190, 8190};
    uint16_t r_hi[8] = {0, 8190, 0, 8190, 12000, 8190, 8190, 8190};
    o = run(r_lo, t, 1000);
    float before = o.coarse_cent;
    PicInputConfig c0 = pic_input_config();
    pic_input_retune(10, 0, c0.fine_tau_ms, c0.fine_hyst_cent, c0.knob_tau_ms, c0.knob_hyst,
                     c0.depth_tau_ms, c0.depth_hyst, c0.cv_tau_ms);
    o = pic_input_update(t);
    CHECK(o.coarse_cent == before, "retune must not move the output (%.2f -> %.2f)", before, o.coarse_cent);
    CHECK(pic_input_config().coarse_tau_ms == 10 && pic_input_config().zero[1] == c0.zero[1], "config updated, CV cal kept");
    o = run(r_hi, t, 10);                          // 10 ms = 1 tau -> 63 % of the step
    float target = 12000.0f / FULL * 7200, frac = (o.coarse_cent - before) / (target - before);
    printf("  after 1 tau (10 ms): %.0f %% of the step\n", frac * 100);
    CHECK(fabsf(frac - 0.63f) < 0.05f, "tau 10 ms: expected ~63 %%, got %.0f %%", frac * 100);

    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
