// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "control.h"

#include <math.h>
#include "config.h"
#include "oscillator.h"
#include "pitch.h"
#include "shared_params.h"

#define CTRL_RATE_HZ ((float)CFG_SAMPLE_RATE / (float)CFG_CTRL_PERIOD)   // 6 kHz

struct Smoother {
    float y;
    float a;
    float step(float x) { y += a * (x - y); return y; }
};

static Smoother s_knob, s_fm, s_wt;
static float s_tau_knob = CFG_SMOOTH_KNOB_MS, s_tau_fm = CFG_SMOOTH_FM_MS, s_tau_wt = CFG_SMOOTH_WT_MS;

float control_smooth_coef(float tau_ms) {
    if (tau_ms <= 0.0f) return 1.0f;
    return 1.0f - expf(-1000.0f / (tau_ms * CTRL_RATE_HZ));
}

float RAM_FUNC(control_wt_combine)(float wt_knob, float wt_cv) {
    float wt = wt_knob + wt_cv;
    if (!(wt > 0.0f)) return 0.0f;    // also catches NaN
    if (wt > 1.0f) return 1.0f;
    return wt;
}

static void outputs(ControlOut *out, float knob, float fm, float wt) {
    out->ctl_q = pitch_cent_to_q(knob) + pitch_cent_to_q(fm);
    out->wt_q  = osc_wt_to_q(wt);
}

void control_init(ControlOut *out) {
    s_knob = {g_shared.coarse_cent + g_shared.fine_cent, control_smooth_coef(s_tau_knob)};
    s_fm   = {g_shared.fm_cent,                           control_smooth_coef(s_tau_fm)};
    s_wt   = {control_wt_combine(g_shared.wt_knob, g_shared.wt_cv), control_smooth_coef(s_tau_wt)};
    outputs(out, s_knob.y, s_fm.y, s_wt.y);
}

// Called from core1 (console) while core0 runs control_update: each coefficient is one aligned 32-bit
// store, so core0 sees either the old or the new value.
void control_set_smoothing(float knob_ms, float fm_ms, float wt_ms) {
    s_tau_knob = knob_ms; s_tau_fm = fm_ms; s_tau_wt = wt_ms;
    s_knob.a = control_smooth_coef(knob_ms);
    s_fm.a   = control_smooth_coef(fm_ms);
    s_wt.a   = control_smooth_coef(wt_ms);
}

void control_get_smoothing(float *knob_ms, float *fm_ms, float *wt_ms) {
    *knob_ms = s_tau_knob; *fm_ms = s_tau_fm; *wt_ms = s_tau_wt;
}

void RAM_FUNC(control_update)(ControlOut *out) {
    float knob = s_knob.step(g_shared.coarse_cent + g_shared.fine_cent);
    float fm   = s_fm.step(g_shared.fm_cent);
    float wt   = s_wt.step(control_wt_combine(g_shared.wt_knob, g_shared.wt_cv));
    outputs(out, knob, fm, wt);
}
