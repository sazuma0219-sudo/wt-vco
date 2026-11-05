// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Input section on core1 (wtvco_input_spec.md): V/OCT fusion (coarse ADC + ADS1115), relative
// calibration, PIC frames -> knobs and CVs, status. Writes the shared variables every loop pass.
// Runs on its own; the USB console (console.h) only observes it and may override sources for tests.
#pragma once
#include <stdint.h>

// Where voct_cent comes from. FUSION is the product; the others are for tests (console).
enum InputVoctSrc { IN_VOCT_FUSION, IN_VOCT_ADC, IN_VOCT_ADS, IN_VOCT_EXTERNAL };

struct InputCtl {
    InputVoctSrc voct_src = IN_VOCT_FUSION;
    bool knobs_from_pic = true;   // false: coarse/fine/wt/fm are written by someone else (console)
    bool cal_auto = true;         // relative calibration updates the fusion scale (9.3)
};

void input_setup();               // core1, start-up order of 11.1
void input_loop();                // core1, one pass (budget 10 µs, 10.1)

InputCtl &input_ctl();
bool input_pic_ok();              // PIC frames arriving (8.2)
void input_cal_reset();           // relative calibration back to theory
void input_set_ads_observer(void (*fn)(int16_t code));   // every ADS1115 result (console statistics)
void input_set_pic_observer(void (*fn)(const uint16_t raw[8]));   // every good PIC frame (calibration)

// Calibration (9): current values, and applying a set (from flash or from the console).
struct CalData;
void input_get_cal(CalData *d);
void input_apply_cal(const CalData &d);
bool input_cal_loaded();          // values came from flash at start-up
