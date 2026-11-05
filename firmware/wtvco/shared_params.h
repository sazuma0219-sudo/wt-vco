// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Shared variables between the input section (core1) and the audio side (core0).
// Definition: wtvco_input_spec.md 10.2. Every member is a single aligned 32-bit value, read and written
// in one access, so no locking is needed. The audio side only reads these (except audio_mute_ack)
// and never knows where the values come from (input stub or the real input section).
#pragma once
#include <stdint.h>

struct SharedParams {
    volatile float    voct_cent;       // 0 .. 6000 cent (clamped by the input side). Read every sample.
    volatile float    coarse_cent;     // 0 .. 7200 cent (C0 based)
    volatile float    fine_cent;       // -100 .. +100 cent
    volatile float    wt_cv;           // -0.5 .. +1.5 (UNI + BI, not clamped)
    volatile float    wt_knob;         // 0.0 .. 1.0
    volatile float    fm_cent;         // -6000 .. +18000 cent (not clamped)
    volatile uint32_t status;          // bit flags (input side). The audio side never mutes on these.
    volatile uint32_t audio_mute_req;  // core1 -> core0: 1 = fade out and silence the ring (flash save)
    volatile uint32_t audio_mute_ack;  // core0 -> core1: 1 = ring is silent
};

// status bits (input side; the audio side never mutes on them)
#define STATUS_VOCT_FAST   (1u << 0)   // V/OCT is currently from the coarse ADC (FAST mode, 5)
#define STATUS_ADS_FAIL    (1u << 1)   // ADS1115 did not start: coarse ADC only (11.2)
#define STATUS_PIC_LOST    (1u << 2)   // no PIC frame for longer than T_PIC_HOLD: knob/CV defaults (8.2)

extern SharedParams g_shared;
