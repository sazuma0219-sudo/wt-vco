// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// wtvco_input_spec.md 5.3 / 5.5, kept as close to the pseudo code as possible.
#include "vin_fusion.h"
#include "config.h"

enum FusState { ST_ADS, ST_FAST };

struct WinStats {
    uint32_t min, max, sum, cnt;
    bool     discarded;
    uint32_t seq;
    bool     valid;
};

static FusionConfig s_cfg;
static FusionStats  s_st;
static FusState     s_state = ST_FAST;          // 11.1 step 7: FAST until the first clean conversion
static uint32_t     s_ref = 0;                  // mean p of the last clean window (sum16)
static bool         s_have_ref = false;
static float        s_offset = 0.0f;            // to_coarse_scale(ads_val) - ref (sum16)
static int16_t      s_ads_val = 0;
static uint32_t     s_jump_cnt = 0;
static uint32_t     s_dirty_cnt = 0;
static WinStats     s_win;                      // window being filled
static WinStats     s_closed;                   // last closed window, waiting for its result
static float        s_glide = 0.0f;             // smoothing: output = target + glide, glide -> 0
static bool         s_glide_pending = false;    // smoothed switch happened; glide is set at the next output
static float        s_last_out = 0.0f;
static bool         s_have_out = false;
static bool         s_pair_ready = false;       // calibration pair from the last accepted conversion
static float        s_pair_x = 0.0f;
static int16_t      s_pair_y = 0;

static void win_reset(WinStats &w) {
    w.min = UINT32_MAX; w.max = 0; w.sum = 0; w.cnt = 0; w.discarded = false; w.valid = false;
}

void fusion_init(const FusionConfig &cfg) {
    s_cfg = cfg;
    s_st = FusionStats{};
    s_state = ST_FAST;
    s_have_ref = false;
    s_offset = 0.0f;
    s_jump_cnt = s_dirty_cnt = 0;
    win_reset(s_win);
    win_reset(s_closed);
    s_glide = 0.0f;
    s_glide_pending = false;
    s_have_out = false;
}

void fusion_set_cal(float gain, float offset) { s_cfg.gain = gain; s_cfg.offset = offset; }
void fusion_set_smooth_us(float us) { s_cfg.smooth_us = us; }
float fusion_smooth_us() { return s_cfg.smooth_us; }

static inline float to_ads_scale(float sum16) { return s_cfg.gain * (sum16 / FUS_MA_LEN) + s_cfg.offset; }
static inline float to_coarse_sum16(float ads) { return (ads - s_cfg.offset) / s_cfg.gain * FUS_MA_LEN; }

// Target value of the current state (ADS LSB), before smoothing.
static inline float target(uint32_t s_sum) {
    return s_state == ST_ADS ? (float)s_ads_val : to_ads_scale((float)s_sum + s_offset);
}

// Switch state. smooth = keep the output continuous and let it glide to the new target (5.5).
// The glide is anchored at the next fusion_output() call, where the current s is known; during the
// glide the target keeps following s (5.5: "補間中も目標値は s に追従").
static void switch_to(FusState st, bool smooth) {
    s_state = st;
    if (smooth) {
        s_glide_pending = s_have_out;
    } else {
        s_glide = 0.0f;
        s_glide_pending = false;
    }
}

void RAM_FUNC(fusion_sample)(uint32_t s_sum) {
    if (s_state != ST_ADS || !s_have_ref) return;
    uint32_t dev = s_sum > s_ref ? s_sum - s_ref : s_ref - s_sum;
    if (dev > s_cfg.t_jump_sum) {
        if (++s_jump_cnt >= FUS_JUMP_CONFIRM) {    // 34 µs: single spikes cannot confirm (5.4)
            s_jump_cnt = 0;
            s_st.to_fast_jump++;
            switch_to(ST_FAST, false);             // immediate: the speed is the point (5.5)
        }
    } else {
        s_jump_cnt = 0;
    }
}

void RAM_FUNC(fusion_block)(uint32_t p_sum) {
    if (p_sum < s_win.min) s_win.min = p_sum;
    if (p_sum > s_win.max) s_win.max = p_sum;
    s_win.sum += p_sum;
    s_win.cnt++;
}

void fusion_mark_discard() { s_win.discarded = true; }

void RAM_FUNC(fusion_window_close)(uint32_t seq) {
    s_closed = s_win;
    s_closed.seq = seq;
    s_closed.valid = true;
    s_st.last_closed_seq = seq;
    win_reset(s_win);
    s_st.windows++;

    // Stranded-in-ADS guard (5.3): judged from the coarse statistics alone, without the result.
    if (s_state == ST_ADS && !s_closed.discarded && s_closed.cnt > 0) {
        if (s_closed.max - s_closed.min >= s_cfg.t_still_sum) {
            if (++s_dirty_cnt >= FUS_DIRTY_CONFIRM) {
                s_dirty_cnt = 0;
                s_st.to_fast_dirty++;
                switch_to(ST_FAST, true);
            }
        } else {
            s_dirty_cnt = 0;
        }
    }
}

void RAM_FUNC(fusion_ads_result)(int16_t code, uint32_t seq, uint32_t s_sum_now) {
    s_st.last_result_seq = seq;
    if (!s_closed.valid || s_closed.seq != seq) { s_st.rej_seq++; return; }
    s_closed.valid = false;
    if (s_closed.discarded || s_closed.cnt == 0) { s_st.rej_discarded++; return; }
    if (s_closed.max - s_closed.min >= s_cfg.t_still_sum) { s_st.rej_unclean++; return; }
    uint32_t cand_ref = s_closed.sum / s_closed.cnt;
    uint32_t dev = s_sum_now > cand_ref ? s_sum_now - cand_ref : cand_ref - s_sum_now;
    if (dev > s_cfg.t_jump_sum) { s_st.rej_moved++; return; }   // moved while the read was pending
    // Gross plausibility against the coarse ADC (a loose bound, far above any calibration error): a
    // corrupt or restart value must neither drive the output nor enter the relative calibration.
    float diff = (float)code - to_ads_scale((float)cand_ref);
    if (diff > s_cfg.plausible_ads || diff < -s_cfg.plausible_ads) { s_st.rej_implausible++; return; }

    s_ads_val  = code;
    s_ref      = cand_ref;
    s_have_ref = true;
    s_offset   = to_coarse_sum16((float)code) - (float)cand_ref;
    s_dirty_cnt = 0;
    s_jump_cnt  = 0;
    s_st.ads_updates++;
    if (s_state != ST_ADS) {
        s_st.to_ads++;
        switch_to(ST_ADS, true);                   // small residual step, smoothed (5.5)
    }
    s_pair_x = (float)cand_ref / FUS_MA_LEN;  // calib_collect_pair (9.3): same voltage, both converters
    s_pair_y = code;
    s_pair_ready = true;
}

float RAM_FUNC(fusion_output)(uint32_t s_sum_now, uint32_t dt_us) {
    if (s_glide_pending) {
        s_glide = s_last_out - target(s_sum_now);
        s_glide_pending = false;
        dt_us = 0;                                  // start the glide from the previous output
    }
    if (s_glide != 0.0f) {
        float k = (float)dt_us / s_cfg.smooth_us;   // 1st-order decay, dt << SMOOTH_MS
        if (k >= 1.0f) s_glide = 0.0f;
        else {
            s_glide -= s_glide * k;
            if (s_glide > -0.05f && s_glide < 0.05f) s_glide = 0.0f;
        }
    }
    s_last_out = target(s_sum_now) + s_glide;
    s_have_out = true;
    return s_last_out;
}

bool fusion_take_pair(float *x, int16_t *y) {
    if (!s_pair_ready) return false;
    s_pair_ready = false;
    *x = s_pair_x;
    *y = s_pair_y;
    return true;
}

void fusion_force_fast() {
    if (s_state == ST_ADS) { s_st.to_fast_dirty++; switch_to(ST_FAST, true); }
}

bool fusion_is_fast() { return s_state == ST_FAST; }
const FusionStats &fusion_stats() { return s_st; }
float fusion_offset_sum16() { return s_offset; }
