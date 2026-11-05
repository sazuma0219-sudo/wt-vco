// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// PIC values -> shared variables (wtvco_input_spec.md 8.2 - 8.6). Pure logic, host-testable.
// Called once per received frame (1 kHz) and once per loop pass for the frame-loss timeout.
#pragma once
#include <stdint.h>

#define PICIN_RAW_MAX   16380.0f        // "average x 4" full scale (7.2)

struct PicInputConfig {
    float coarse_tau_ms, coarse_hyst_cent;   // 8.3 COARSE: strong IIR + dead band
    float fine_tau_ms,   fine_hyst_cent;
    float knob_tau_ms,   knob_hyst;          // WT KNOB (0..1 units)
    float depth_tau_ms,  depth_hyst;         // FM DEPTH, on k before the square curve
    float cv_tau_ms;                         // WT CV and FM CV (8.5, 8.6)
    uint32_t hold_ms;                        // T_PIC_HOLD (8.2)
    // CV conversion per input (8.4): v = (raw - zero) * gain. Order: WT_UNI, WT_BI, FM_UNI, FM_BI.
    float zero[4];
    float gain[4];                           // V per raw LSB
};

struct PicInputOut {                         // what goes into the shared variables
    float coarse_cent, fine_cent, wt_knob, wt_cv, fm_cent;
    bool  pic_ok;                            // false: frames lost for longer than hold_ms -> defaults
};

void pic_input_default_config(PicInputConfig *c);
void pic_input_init(const PicInputConfig &c);
void pic_input_set_cv(const float zero[4], const float gain[4]);   // 9.4 zero / gain calibration
const PicInputConfig &pic_input_config();
// Changes the time constants and dead bands while running (console 'tune'); filter states are kept.
void pic_input_retune(float coarse_tau_ms, float coarse_hyst_cent, float fine_tau_ms, float fine_hyst_cent,
                      float knob_tau_ms, float knob_hyst, float depth_tau_ms, float depth_hyst, float cv_tau_ms);
// One good frame (raw values in pic_rx order: WT_UNI, WT_BI, FM_UNI, FM_BI, COARSE, FINE, KNOB, DEPTH).
void pic_input_frame(const uint16_t raw[8], uint32_t now_ms);
// Every loop pass: applies the timeout of 8.2. Returns the values to publish.
const PicInputOut &pic_input_update(uint32_t now_ms);
