// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "pic_input.h"
#include <math.h>

// Smoothing at the frame rate (1 kHz): y += a (x - y), a = 1 - exp(-1 ms / tau).
struct Iir {
    float y = 0, a = 1;
    bool  init = false;
    void  set_tau(float tau_ms) { a = tau_ms > 0 ? 1.0f - expf(-1.0f / tau_ms) : 1.0f; }
    float step(float x) { if (!init) { y = x; init = true; } else y += a * (x - y); return y; }
};

// Dead band (8.3): the output only moves when the input is more than 'h' away from it.
struct Hyst {
    float out = 0, h = 0;
    bool  init = false;
    float step(float x) {
        if (!init) { out = x; init = true; }
        else if (x > out + h) out = x - h;
        else if (x < out - h) out = x + h;
        return out;
    }
};

static PicInputConfig s_cfg;
static Iir   s_coarse, s_fine, s_knob, s_depth, s_wt, s_fm;
static Hyst  h_coarse, h_fine, h_knob, h_depth;
static PicInputOut s_out;
static PicInputOut s_live;                  // last values computed from frames
static bool     s_have_frame = false;
static uint32_t s_last_ms = 0;

void pic_input_default_config(PicInputConfig *c) {
    // Values from 8.3, 8.5, 8.6 (chosen from measured noise and by ear, 13.2 / 13.4; try others with 'tune').
    c->coarse_tau_ms = 75;  c->coarse_hyst_cent = 4.5f;    // 1 ADC LSB ≈ 1.76 cent; ≈ 10 raw LSB
    c->fine_tau_ms   = 20;  c->fine_hyst_cent   = 0.1f;
    c->knob_tau_ms   = 15;  c->knob_hyst        = 0.004f;  // "large" hysteresis for a mid-precision knob
    c->depth_tau_ms  = 20;  c->depth_hyst       = 0.001f;
    c->cv_tau_ms     = 2;                                  // fc ≈ 80 Hz: mid-speed CV (WT / slow FM)
    c->hold_ms       = 100;
    const float g = 5.0f / PICIN_RAW_MAX;                  // theory: 0.305 mV per raw LSB (8.4)
    c->zero[0] = 0;    c->gain[0] = g;                     // WT UNI: 0 V
    c->zero[1] = 8190; c->gain[1] = g;                     // WT BI: mid-scale = 2.5 V
    c->zero[2] = 0;    c->gain[2] = g;                     // FM UNI
    c->zero[3] = 8190; c->gain[3] = g;                     // FM BI
}

static void defaults(PicInputOut *o) {         // 8.2 "regarded as 0"
    o->coarse_cent = 3600.0f;                  // C3, not C0
    o->fine_cent = 0.0f;
    o->wt_knob = 0.0f;
    o->wt_cv = 0.0f;                           // BI is NOT raw 0 (that would be -0.5)
    o->fm_cent = 0.0f;
}

void pic_input_init(const PicInputConfig &c) {
    s_cfg = c;
    s_coarse = Iir(); s_fine = Iir(); s_knob = Iir(); s_depth = Iir(); s_wt = Iir(); s_fm = Iir();
    s_coarse.set_tau(c.coarse_tau_ms); s_fine.set_tau(c.fine_tau_ms); s_knob.set_tau(c.knob_tau_ms);
    s_depth.set_tau(c.depth_tau_ms);   s_wt.set_tau(c.cv_tau_ms);     s_fm.set_tau(c.cv_tau_ms);
    h_coarse = Hyst(); h_fine = Hyst(); h_knob = Hyst(); h_depth = Hyst();
    h_coarse.h = c.coarse_hyst_cent; h_fine.h = c.fine_hyst_cent; h_knob.h = c.knob_hyst; h_depth.h = c.depth_hyst;
    s_have_frame = false;
    defaults(&s_out);
    s_out.pic_ok = false;
    s_live = s_out;
}

void pic_input_set_cv(const float zero[4], const float gain[4]) {
    for (int i = 0; i < 4; i++) { s_cfg.zero[i] = zero[i]; s_cfg.gain[i] = gain[i]; }
}

const PicInputConfig &pic_input_config() { return s_cfg; }

void pic_input_retune(float coarse_tau_ms, float coarse_hyst_cent, float fine_tau_ms, float fine_hyst_cent,
                      float knob_tau_ms, float knob_hyst, float depth_tau_ms, float depth_hyst, float cv_tau_ms) {
    s_cfg.coarse_tau_ms = coarse_tau_ms; s_cfg.coarse_hyst_cent = coarse_hyst_cent;
    s_cfg.fine_tau_ms   = fine_tau_ms;   s_cfg.fine_hyst_cent   = fine_hyst_cent;
    s_cfg.knob_tau_ms   = knob_tau_ms;   s_cfg.knob_hyst        = knob_hyst;
    s_cfg.depth_tau_ms  = depth_tau_ms;  s_cfg.depth_hyst       = depth_hyst;
    s_cfg.cv_tau_ms     = cv_tau_ms;
    // Filter states are kept: only the coefficients and dead bands change (no jump while playing).
    s_coarse.set_tau(coarse_tau_ms); s_fine.set_tau(fine_tau_ms); s_knob.set_tau(knob_tau_ms);
    s_depth.set_tau(depth_tau_ms);   s_wt.set_tau(cv_tau_ms);     s_fm.set_tau(cv_tau_ms);
    h_coarse.h = coarse_hyst_cent; h_fine.h = fine_hyst_cent; h_knob.h = knob_hyst; h_depth.h = depth_hyst;
}

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void pic_input_frame(const uint16_t raw[8], uint32_t now_ms) {
    enum { WT_UNI, WT_BI, FM_UNI, FM_BI, COARSE, FINE, KNOB, DEPTH };
    // When frames come back after a loss, restart the smoothers from the new values (8.2: back to normal,
    // the audio side smooths the step).
    if (!s_have_frame || now_ms - s_last_ms > s_cfg.hold_ms) {
        s_coarse.init = s_fine.init = s_knob.init = s_depth.init = s_wt.init = s_fm.init = false;
        h_coarse.init = h_fine.init = h_knob.init = h_depth.init = false;
    }
    s_have_frame = true;
    s_last_ms = now_ms;

    // 8.3 knobs (ratiometric: no calibration). IIR first, then the dead band.
    float coarse = clampf(raw[COARSE] / PICIN_RAW_MAX, 0, 1) * 7200.0f;
    s_live.coarse_cent = clampf(h_coarse.step(s_coarse.step(coarse)), 0, 7200);
    float fine = (clampf(raw[FINE] / PICIN_RAW_MAX, 0, 1) - 0.5f) * 200.0f;
    s_live.fine_cent = clampf(h_fine.step(s_fine.step(fine)), -100, 100);
    s_live.wt_knob = clampf(h_knob.step(s_knob.step(clampf(raw[KNOB] / PICIN_RAW_MAX, 0, 1))), 0, 1);
    float k = clampf(h_depth.step(s_depth.step(clampf(raw[DEPTH] / PICIN_RAW_MAX, 0, 1))), 0, 1);
    float fm_depth = 2400.0f * k * k;                               // cent/V, square curve

    // 8.4 CVs: UNI + BI per destination, clamped only after summing.
    float v[4];
    for (int i = 0; i < 4; i++) v[i] = ((float)raw[i] - s_cfg.zero[i]) * s_cfg.gain[i];
    float wt_v = clampf(v[0] + v[1], -2.5f, 7.5f);
    float fm_v = clampf(v[2] + v[3], -2.5f, 7.5f);
    s_live.wt_cv = s_wt.step(wt_v / 5.0f);                          // 8.5: 1 V = 0.2, not clamped
    s_live.fm_cent = fm_depth * s_fm.step(fm_v);                    // 8.6: not clamped
    s_live.pic_ok = true;
}

const PicInputOut &pic_input_update(uint32_t now_ms) {
    if (s_have_frame && now_ms - s_last_ms <= s_cfg.hold_ms) {
        s_out = s_live;                        // live, or holding the last values (< T_PIC_HOLD)
    } else {
        defaults(&s_out);                      // 8.2: no frame yet, or lost for longer than T_PIC_HOLD
        s_out.pic_ok = false;
    }
    return s_out;
}
