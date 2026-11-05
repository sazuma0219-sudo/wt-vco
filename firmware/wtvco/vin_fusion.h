// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// V/OCT sensor fusion: coarse ADC (fast) + ADS1115 (precise) (wtvco_input_spec.md 5).
// Pure logic, no hardware access: vin_coarse / ads1115 feed it, host tests drive it with synthetic data.
//
// Units
//   coarse:  s_sum / p_sum are SUMS of 16 12-bit samples ("sum16"), as produced by vin_coarse
//   ads:     ADS1115 codes (1 LSB = 187.5 µV). The fused output is in ADS LSB (5.6).
//   to_ads_scale(coarse average) = gain * (sum16 / 16) + offset      (relative calibration, 9.3)
#pragma once
#include <stdint.h>

#define FUS_MA_LEN          16
#define FUS_JUMP_CONFIRM    (FUS_MA_LEN + 1)   // 5.4: 17 raw samples
#define FUS_DIRTY_CONFIRM   2                  // 5.4: windows

struct FusionConfig {
    uint32_t t_still_sum;      // T_STILL in sum16 units
    uint32_t t_jump_sum;       // T_JUMP  in sum16 units
    float    gain;             // ADS LSB per coarse LSB (theory ≈ 7.16)
    float    offset;           // ADS LSB
    float    smooth_us;        // SMOOTH_MS in µs (5.5)
    float    plausible_ads;    // max |ADS - to_ads_scale(coarse)| (ADS LSB) for a result to be believed
};

struct FusionStats {
    uint32_t to_fast_jump;     // ADS -> FAST by the jump detector (immediate)
    uint32_t to_fast_dirty;    // ADS -> FAST after DIRTY_CONFIRM unclean windows (smoothed)
    uint32_t to_ads;           // FAST/ADS -> ADS on a clean conversion (smoothed)
    uint32_t ads_updates;      // clean conversions accepted (includes staying in ADS)
    uint32_t rej_unclean;      // result dropped: window range >= T_STILL
    uint32_t rej_moved;        // result dropped: input moved while the read was pending
    uint32_t rej_discarded;    // result dropped: window marked discarded (ERR / overrun)
    uint32_t rej_seq;          // result dropped: no matching closed window (core1 lagging)
    uint32_t rej_implausible;  // result dropped: far from the coarse reading (I2C glitch, restart)
    uint32_t windows;          // windows closed
    uint32_t last_closed_seq;  // diagnostics: RDY number of the last closed window
    uint32_t last_result_seq;  // diagnostics: RDY number of the last result offered
};

void fusion_init(const FusionConfig &cfg);
void fusion_set_cal(float gain, float offset);
void fusion_set_smooth_us(float us);           // SMOOTH_MS while running (console 'tune'); > 0
float fusion_smooth_us();

// Called by the coarse path, in sample order.
void fusion_sample(uint32_t s_sum);            // every raw sample (2 µs)
void fusion_block(uint32_t p_sum);             // every 16 raw samples (32 µs)
void fusion_window_close(uint32_t seq);        // at an ADS1115 RDY boundary (4.3), seq = RDY number
void fusion_mark_discard();                    // current window is unusable (FIFO ERR, ring overrun)

// Called when the ADS1115 result of conversion 'seq' arrives (≈70 µs after its RDY).
void fusion_ads_result(int16_t code, uint32_t seq, uint32_t s_sum_now);

// Fused value in ADS LSB, with the 5.5 transition smoothing. Call once per loop pass.
float fusion_output(uint32_t s_sum_now, uint32_t dt_us);

// The (coarse average, ADS code) pair of the last accepted clean conversion, once (9.3 calibration).
bool fusion_take_pair(float *coarse_avg_lsb, int16_t *ads_code);

// The ADS1115 is not delivering (11.2): leave ADS mode so the output follows the coarse ADC instead of
// holding a stale value. Smoothed like a DIRTY transition.
void fusion_force_fast();

bool fusion_is_fast();
const FusionStats &fusion_stats();
float fusion_offset_sum16();                   // FAST-mode correction (sum16 units), for diagnostics
