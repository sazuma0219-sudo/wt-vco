// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// core0 audio loop: chases the DMA read position and keeps CFG_OUT_LEAD_FRAMES frames written ahead (§1.11).
#pragma once
#include <stdint.h>

// Statistics, read by core1 (single 32-bit values).
extern volatile uint32_t audio_underrun_cnt;    // times the writer fell behind the DMA
extern volatile uint32_t audio_min_lead;        // smallest lead (frames) seen since the last reset
extern volatile uint32_t audio_min_lead_reset;  // core1 sets 1, core0 resets min_lead / max_gap and clears it
extern volatile uint32_t audio_max_gap_us;      // longest time between two passes of the loop (= longest stall of core0)
extern volatile uint32_t audio_stall_cnt;       // passes that took longer than AUDIO_STALL_US (since boot)
#define AUDIO_STALL_US 20
extern volatile uint32_t audio_busy_us;         // cumulative µs spent rendering (wraps). Load = Δbusy / Δtime

// Starts I2S output. Returns false if PIO/DMA could not be claimed.
bool audio_begin();

// Never returns. Call from loop() on core0.
void audio_run();
