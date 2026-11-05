// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "audio.h"

#include <Arduino.h>
#include "config.h"
#include "control.h"
#include "i2s_out.h"
#include "oscillator.h"
#include "pitch.h"
#include "shared_params.h"

volatile uint32_t audio_underrun_cnt   = 0;
volatile uint32_t audio_min_lead       = CFG_OUT_RING_FRAMES;
volatile uint32_t audio_min_lead_reset = 0;
volatile uint32_t audio_max_gap_us     = 0;
volatile uint32_t audio_stall_cnt      = 0;
volatile uint32_t audio_busy_us        = 0;

// Mute for flash saves (§1.12): fade out, zero the whole ring, ack, wait, resync, fade in.
#define FADE_SAMPLES   ((CFG_SAMPLE_RATE * 2) / 1000)            // 2 ms
#define FADE_UNITY     65536u
#define FADE_STEP      (FADE_UNITY / FADE_SAMPLES)

enum MuteState : uint32_t { MS_RUN, MS_FADE_OUT, MS_MUTED };

static ControlOut s_ctl;
static uint32_t   s_phase;
static uint32_t   s_fade = FADE_UNITY;        // Q16 output fade, FADE_UNITY = full level
static MuteState  s_mute = MS_RUN;

bool audio_begin() {
    control_init(&s_ctl);
    return i2s_out_begin();
}

// One output frame: per-sample V/OCT -> pitch -> phase_inc / level, oscillator, gain, L = R.
static inline void render_frame(uint32_t frame) {
    int32_t  voct_q    = pitch_voct_to_q(g_shared.voct_cent);      // the only float op per sample
    int32_t  pitch_q   = pitch_clamp(s_ctl.ctl_q + voct_q);
    uint32_t phase_inc = exp2_phase_inc(pitch_q);
    int32_t  s         = osc_render(&s_phase, phase_inc, pitch_level(pitch_q), s_ctl.wt_q);
    int32_t  out       = (s * CFG_OUT_GAIN_Q15) >> 15;
    if (s_fade != FADE_UNITY) out = (int32_t)(((int64_t)out * s_fade) >> 16);
    i2s_out_write_frame(frame, out);
}

// Advances the fade by one sample. Returns true when a fade-out has reached silence.
static inline bool fade_step() {
    if (s_mute == MS_FADE_OUT) {
        s_fade = s_fade > FADE_STEP ? s_fade - FADE_STEP : 0;
        return s_fade == 0;
    }
    if (s_fade != FADE_UNITY) s_fade = s_fade + FADE_STEP < FADE_UNITY ? s_fade + FADE_STEP : FADE_UNITY;
    return false;
}

// Runs from RAM so that XIP cache misses on code cannot stall the loop.
void __not_in_flash_func(audio_run)() {
    uint32_t wr      = (i2s_out_read_frame() + CFG_OUT_LEAD_FRAMES) & I2S_RING_MASK;
    uint32_t ctl_cnt = 0;
    uint32_t min_lead = CFG_OUT_RING_FRAMES;
    uint32_t max_gap  = 0;
    uint32_t t_prev   = time_us_32();

    for (;;) {
        // Stall monitor: time since the previous pass (a pass renders at most a few frames, ~1 µs each).
        uint32_t t_now = time_us_32();
        uint32_t gap   = t_now - t_prev;
        t_prev = t_now;
        if (gap > max_gap) { max_gap = gap; audio_max_gap_us = gap; }
        if (gap > AUDIO_STALL_US) audio_stall_cnt = audio_stall_cnt + 1;

        if (s_mute == MS_MUTED) {
            // The DMA keeps replaying the zeroed ring. Wait for the request to drop, then resync.
            if (g_shared.audio_mute_req) { t_prev = time_us_32(); continue; }
            g_shared.audio_mute_ack = 0;
            s_mute   = MS_RUN;                          // s_fade = 0 -> fades back in
            wr       = (i2s_out_read_frame() + CFG_OUT_LEAD_FRAMES) & I2S_RING_MASK;
            min_lead = CFG_OUT_RING_FRAMES;
            t_prev   = time_us_32();
            continue;
        }

        uint32_t rd   = i2s_out_read_frame();
        uint32_t lead = (wr - rd) & I2S_RING_MASK;
        if (lead > CFG_OUT_RING_FRAMES / 2) {           // the DMA overtook the writer
            audio_underrun_cnt = audio_underrun_cnt + 1;
            wr = (rd + CFG_OUT_LEAD_FRAMES) & I2S_RING_MASK;
            continue;
        }
        if (audio_min_lead_reset) {
            min_lead = CFG_OUT_RING_FRAMES;
            max_gap  = 0;
            audio_max_gap_us = 0;
            audio_min_lead_reset = 0;
        }
        if (lead < min_lead) {
            min_lead = lead;
            audio_min_lead = lead;
        }
        if (lead >= CFG_OUT_LEAD_FRAMES) continue;      // nothing to render yet (busy wait)

        // Load measurement: the µs timer is coarser than one frame, but the start of a batch is
        // uncorrelated with the timer ticks, so the summed differences are unbiased. Only batches
        // that render are timed, so the busy-wait passes add no overhead to the figure.
        uint32_t t_busy = time_us_32();
        while (((wr - rd) & I2S_RING_MASK) < CFG_OUT_LEAD_FRAMES) {
            if (++ctl_cnt == CFG_CTRL_PERIOD) {
                control_update(&s_ctl);
                ctl_cnt = 0;
                if (s_mute == MS_RUN && g_shared.audio_mute_req) s_mute = MS_FADE_OUT;
            }
            bool silent = fade_step();
            render_frame(wr);
            wr = (wr + 1) & I2S_RING_MASK;
            if (silent) {
                for (uint32_t i = 0; i < 2 * CFG_OUT_RING_FRAMES; i++) i2s_out_ring[i] = 0;
                s_mute = MS_MUTED;
                g_shared.audio_mute_ack = 1;
                break;
            }
        }
        audio_busy_us = audio_busy_us + (time_us_32() - t_busy);
    }
}
