// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Host-side simulation of the V/OCT fusion (wtvco_input_spec.md 5).
// A synthetic V/OCT voltage is sampled like the hardware does it: coarse ADC at 500 kSPS (with the gain /
// offset error measured on the board and realistic noise), ADS1115 windows of 1.15 ms whose result
// arrives 70 µs after RDY, and a core1 loop that publishes the output every 10 µs.
// Build & run: tests/run_tests.sh
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../config.h"
#include "../vin_fusion.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---- board model -------------------------------------------------------------------------------
// Coarse ADC error vs the ADS1115. "board" = measured on the board (uncalibrated, 13.1); "calibrated" = after 9.3.
static double COARSE_GAIN_ERR = 0.979, COARSE_OFS_V = 0.021;
static const char *g_model = "board";
static const double COARSE_RAW_SD = 2.6;      // raw LSB -> p sd ≈ 0.65 LSB (measured)
static const double ADS_SD = 0.6;             // ADS LSB (measured with a source connected)
static const double ADS_LSB = 187.5e-6;
static const double CENT_PER_ADS = ADS_LSB * 1200.0;
static const int    RAW_PER_WIN = 575;        // 1.15 ms / 2 µs
static const int    READ_DELAY = 35;          // 70 µs
static const int    OUT_EVERY = 5;            // core1 loop ≈ 10 µs
static uint32_t     g_corrupt_seq = 0;        // window whose ADS result is corrupted (0 = none)

static uint64_t rng = 88172645463325252ull;
static double urand() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static double gauss() { double u = urand() + 1e-12, v = urand(); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }

struct SimResult {
    double max_err_cent_after_settle;   // |out - V| after the given settle time
    double first_t_within;              // first time (µs) the output was within 'tol' of the final value
    double last_fast_t;                 // last time (µs) the state was FAST
    FusionStats st;
};

// v(t_us) -> volts. spike_at: raw sample index that gets a +spike.
static SimResult run(double dur_us, std::function<double(double)> v, double settle_from_us, double tol_cent,
                     double final_v, long spike_at = -1, double spike_lsb = 0,
                     std::function<void(double, double, bool)> probe = nullptr) {
    FusionConfig cfg;
    cfg.t_still_sum = CFG_T_STILL_LSB * 16;
    cfg.t_jump_sum  = CFG_T_JUMP_LSB * 16;
    cfg.gain        = (3.3 / 4096.0 / 0.6) / ADS_LSB;     // theoretical scale, uncalibrated
    cfg.offset      = 0.0f;
    cfg.smooth_us   = 4000.0f;
    cfg.plausible_ads = 200.0f / (ADS_LSB * 1200.0);
    fusion_init(cfg);

    uint32_t ring[16] = {0}, s_sum = 0, p_sum = 0;
    int pn = 0;
    double win_acc = 0; int win_n = 0; uint32_t seq = 0;
    long pending_at = -1; int16_t pending_code = 0; uint32_t pending_seq = 0;
    SimResult r{0, -1, -1, {}};
    long n_total = (long)(dur_us / 2.0);
    for (long n = 0; n < n_total; n++) {
        double t = n * 2.0, vv = v(t);
        double cv = COARSE_GAIN_ERR * vv + COARSE_OFS_V;
        double code = cv * 0.6 / 3.3 * 4096.0 + COARSE_RAW_SD * gauss();
        if (n == spike_at) code += spike_lsb;
        uint32_t raw = (uint32_t)fmax(0, fmin(4095, lround(code)));
        s_sum += raw - ring[n & 15];
        ring[n & 15] = raw;
        if (n >= 16) fusion_sample(s_sum);
        p_sum += raw;
        if (++pn == 16) { fusion_block(p_sum); p_sum = 0; pn = 0; }

        win_acc += vv; win_n++;
        if (win_n == RAW_PER_WIN) {                      // RDY: window closes, conversion = window mean
            seq++;
            fusion_window_close(seq);
            pending_code = (int16_t)lround(win_acc / win_n / ADS_LSB + ADS_SD * gauss());
            if (seq == g_corrupt_seq) pending_code = (int16_t)(pending_code - 1300);   // glitch
            pending_seq = seq;
            pending_at = n + READ_DELAY;
            win_acc = 0; win_n = 0;
        }
        if (n == pending_at) fusion_ads_result(pending_code, pending_seq, s_sum);

        if (n % OUT_EVERY == 0 && n >= 16) {
            double out = fusion_output(s_sum, OUT_EVERY * 2) * CENT_PER_ADS;
            double err = out - vv * 1200.0;
            if (probe) probe(t, out, fusion_is_fast());
            if (t >= settle_from_us && fabs(err) > r.max_err_cent_after_settle) r.max_err_cent_after_settle = fabs(err);
            if (r.first_t_within < 0 && t >= settle_from_us - 1e9 && fabs(out - final_v * 1200.0) < tol_cent && t > 0)
                r.first_t_within = t;
            if (fusion_is_fast()) r.last_fast_t = t;
        }
    }
    r.st = fusion_stats();
    return r;
}

static void print_stats(const FusionStats &s) {
    printf("    windows %u  ads_updates %u  to_fast jump %u / dirty %u  to_ads %u  rej unclean %u moved %u seq %u disc %u\n",
           s.windows, s.ads_updates, s.to_fast_jump, s.to_fast_dirty, s.to_ads, s.rej_unclean, s.rej_moved,
           s.rej_seq, s.rej_discarded);
}

static void test_static() {
    printf("[static 1.000 V, 100 ms]\n");
    SimResult r = run(100000, [](double) { return 1.0; }, 10000, 1.0, 1.0);
    printf("  max error after 10 ms: %.3f cent, last FAST at %.2f ms\n", r.max_err_cent_after_settle, r.last_fast_t / 1000);
    print_stats(r.st);
    CHECK(r.st.to_fast_jump == 0 && r.st.to_fast_dirty == 0, "false FAST transitions on a static input");
    CHECK(r.max_err_cent_after_settle < 1.0, "static error %.3f cent", r.max_err_cent_after_settle);
    CHECK(r.last_fast_t < 5000, "did not lock to ADS within 5 ms (last FAST %.0f us)", r.last_fast_t);
}

// FAST mode error budget: the coarse gain error over the jump (FAST uses the offset learned at the
// old voltage) plus noise. Uncalibrated board: 2.1 % of the jump; calibrated: noise only.
static double fast_budget(double v0, double v1) {
    double gain_err = fabs(1.0 - COARSE_GAIN_ERR) + 0.001;
    return gain_err * fabs(v1 - v0) * 1200.0 + 8.0;
}

static void test_step(double v0, double v1, const char *name) {
    printf("[%s, step %s: %.3f V -> %.3f V at 50 ms]\n", g_model, name, v0, v1);
    const double T0 = 50000;
    const double budget = fast_budget(v0, v1);
    double t_reached = -1, worst_fast_err = 0, t_ads_back = -1, worst_from_old = 1e9;
    bool prev_fast = false;
    SimResult r = run(T0 + 30000, [&](double t) { return t < T0 ? v0 : v1; }, T0 + 25000, 1.0, v1, -1, 0,
        [&](double t, double out, bool fast) {
            if (t < T0) { prev_fast = fast; return; }
            double err = out - v1 * 1200.0;
            if (t_reached < 0 && fabs(err) < budget) t_reached = t - T0;
            if (t - T0 > 100) {
                if (fast && fabs(err) > worst_fast_err) worst_fast_err = fabs(err);
                worst_from_old = fmin(worst_from_old, fabs(out - v0 * 1200.0));
            }
            if (prev_fast && !fast && t_ads_back < 0) t_ads_back = t - T0;
            prev_fast = fast;
        });
    printf("  within %.0f cent (FAST budget) after %.0f us; FAST error %.1f cent; never back near the old pitch\n"
           "  (closest %.0f cent); back to ADS after %.2f ms; error 25 ms after the step %.3f cent\n",
           budget, t_reached, worst_fast_err, worst_from_old, t_ads_back / 1000, r.max_err_cent_after_settle);
    print_stats(r.st);
    CHECK(t_reached >= 0 && t_reached <= 60, "%s: took %.0f us to reach the new pitch", name, t_reached);
    CHECK(worst_fast_err < budget, "%s: FAST error %.1f > budget %.1f cent", name, worst_fast_err, budget);
    CHECK(worst_from_old > fabs(v1 - v0) * 1200 * 0.9, "%s: output went back toward the old pitch", name);
    CHECK(t_ads_back > 0 && t_ads_back < 2700, "%s: back to ADS after %.0f us", name, t_ads_back);
    CHECK(r.max_err_cent_after_settle < 1.0, "%s: settled error %.3f cent", name, r.max_err_cent_after_settle);
}

static void test_glitch() {
    printf("[one ADS result 1300 LSB (290 cent) off, as seen after an ADS restart]\n");
    g_corrupt_seq = 40;                       // ≈ 46 ms, well after the start-up glide has settled
    SimResult r = run(80000, [](double) { return 2.0; }, 30000, 1.0, 2.0);
    g_corrupt_seq = 0;
    printf("  max error after 30 ms: %.3f cent, implausible rejected %u\n", r.max_err_cent_after_settle, r.st.rej_implausible);
    CHECK(r.st.rej_implausible == 1, "glitch not rejected");
    CHECK(r.max_err_cent_after_settle < 1.0, "glitch reached the output: %.2f cent", r.max_err_cent_after_settle);
}

static void test_spike() {
    printf("[single raw-sample spike +500 LSB on a static input]\n");
    SimResult r = run(60000, [](double) { return 2.0; }, 10000, 1.0, 2.0, 25000, 500);
    print_stats(r.st);
    CHECK(r.st.to_fast_jump == 0, "a single spike triggered FAST");
}

// Glide from 1 V by 'dv' volts over 'ms' milliseconds, then hold.
static void test_glide(double dv, double ms) {
    double rate = dv * 1200 / ms;
    printf("[glide 1 V -> %.2f V over %.0f ms (%.0f cent/ms), then hold]\n", 1.0 + dv, ms, rate);
    auto v = [=](double t) { double a = (t - 20000) / (ms * 1000); return 1.0 + dv * fmin(1.0, fmax(0.0, a)); };
    double worst = 0;
    SimResult r = run(20000 + ms * 1000 + 40000, v, 10000, 1.0, 1.0 + dv, -1, 0, [&](double t, double out, bool) {
        if (t > 10000) worst = fmax(worst, fabs(out - v(t) * 1200));
    });
    // Allowed lag: in ADS mode the output is one conversion (~1.2 ms) behind, in FAST the coarse gain error.
    double budget = rate * 1.5 + fabs(1.0 - COARSE_GAIN_ERR) * dv * 1200 + 10;
    printf("  worst tracking error during and after the glide: %.1f cent (budget %.0f)\n", worst, budget);
    print_stats(r.st);
    CHECK(worst < budget, "glide stranded: %.1f cent behind", worst);
}

// Fast wobble with an amplitude below T_JUMP (5.3 example 2: hum / fast small vibrato). Windows are
// unclean but |s - ref| never exceeds T_JUMP: only DIRTY_CONFIRM can get the output out of ADS mode.
static void test_hum() {
    printf("[wobble ±8 cent at 400 Hz around 2 V (|s-ref| < T_JUMP, windows unclean)]\n");
    auto v = [](double t) { return 2.0 + (8.0 / 1200) * sin(2 * M_PI * 400 * t / 1e6); };
    double worst = 0;
    SimResult r = run(200000, v, 20000, 1.0, 2.0, -1, 0, [&](double t, double out, bool) {
        if (t > 50000) worst = fmax(worst, fabs(out - v(t) * 1200));
    });
    printf("  worst tracking error: %.1f cent\n", worst);
    print_stats(r.st);
    CHECK(r.st.to_fast_dirty >= 1, "DIRTY_CONFIRM never moved the output to FAST");
    // While windows are sometimes clean the state alternates and ADS mode holds a window mean for up to
    // ~2 windows, so the error can reach the wobble's peak-to-peak (16 cent) plus noise. Not stranded = within that.
    CHECK(worst < 16 + 6, "wobble not followed: %.1f cent", worst);
}

static void test_vibrato() {
    printf("[vibrato ±10 cent at 6 Hz (below T_JUMP) around 2 V]\n");
    double worst = 0;
    SimResult r = run(500000, [](double t) { return 2.0 + (10.0 / 1200) * sin(2 * M_PI * 6 * t / 1e6); },
        20000, 1.0, 2.0, -1, 0, [&](double t, double out, bool) {
            double vv = 2.0 + (10.0 / 1200) * sin(2 * M_PI * 6 * t / 1e6);
            if (t > 50000) worst = fmax(worst, fabs(out - vv * 1200));
        });
    printf("  worst tracking error: %.1f cent\n", worst);
    print_stats(r.st);
    CHECK(worst < 12, "vibrato not followed: %.1f cent", worst);
}

static void test_step_during_read() {
    printf("[step right after an RDY, before its result arrives]\n");
    // RDY #n at sample 575*n - 1; put the step 10 samples after RDY #80 so the read is still pending.
    const double t_step = (575.0 * 80 - 1 + 10) * 2.0;
    double stale = 0;
    SimResult r = run(t_step + 10000, [&](double t) { return t < t_step ? 1.0 : 3.0; }, t_step + 6000, 1.0, 3.0, -1, 0,
        [&](double t, double out, bool) {
            if (t > t_step + 200) stale = fmax(stale, fabs(out - 3.0 * 1200));
        });
    print_stats(r.st);
    printf("  worst error from 200 us after the step: %.1f cent\n", stale);
    CHECK(r.st.rej_moved >= 1, "the pending result was not rejected");
    CHECK(stale < fast_budget(1.0, 3.0), "stale value used after the step (%.1f cent, budget %.0f)", stale,
          fast_budget(1.0, 3.0));
}

int main() {
    for (int m = 0; m < 2; m++) {
        if (m == 1) { COARSE_GAIN_ERR = 1.0; COARSE_OFS_V = 0.0; g_model = "calibrated"; printf("\n=== calibrated coarse ADC ===\n"); }
        else printf("=== board model: coarse gain -2.1 %%, offset +21 mV (uncalibrated) ===\n");
        test_static();
        test_step(1.0, 2.0, "+1 oct");
        test_step(3.0, 3.0 + 1.0 / 12, "+1 semitone");
        test_step(4.0, 0.25, "-3.75 V");
        test_spike();
        test_glitch();
        test_glide(0.5, 300);     // 2 cent/ms: windows stay clean, ADS follows
        test_glide(1.0, 60);      // 20 cent/ms: unclean windows -> DIRTY_CONFIRM -> FAST
        test_vibrato();
        test_hum();
        test_step_during_read();
    }
    printf(g_fail ? "\n%d FAILURE(S)\n" : "\nALL PASSED\n", g_fail);
    return g_fail ? 1 : 0;
}
