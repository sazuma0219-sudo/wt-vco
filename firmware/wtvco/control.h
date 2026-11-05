// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Control-rate processing (every CFG_CTRL_PERIOD samples): reads the shared variables, combines and
// smooths the knob pitch, FM and WT position (§1.5, §1.9, §1.10). Float is fine here.
// V/OCT is NOT handled here; the audio loop reads it every sample.
#pragma once
#include <stdint.h>

struct ControlOut {
    int32_t  ctl_q;     // Q16.16 octaves above C0: smoothed knob pitch + smoothed FM (unclamped, may be negative)
    uint32_t wt_q;      // Q16.16 frame position, 0 .. OSC_WT_MAX_Q
};

// Starts the smoothers at the current inputs (no glide at power-up).
void control_init(ControlOut *out);
void control_update(ControlOut *out);

// wt = clamp(wt_knob + wt_cv, 0, 1)  (§1.9). The only place WT is clamped.
float control_wt_combine(float wt_knob, float wt_cv);

// 1-pole lowpass coefficient for a time constant at the control rate.
float control_smooth_coef(float tau_ms);

// Smoothing time constants (§1.10), changeable while running (console 'tune', from core1).
// Start at CFG_SMOOTH_*_MS.
void control_set_smoothing(float knob_ms, float fm_ms, float wt_ms);
void control_get_smoothing(float *knob_ms, float *fm_ms, float *wt_ms);
