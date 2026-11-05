// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "input.h"

#include <Arduino.h>
#include "ads1115.h"
#include "config.h"
#include "hardware/gpio.h"
#include "calib_store.h"
#include "pic_input.h"
#include "pic_rx.h"
#include "shared_params.h"
#include "vin_calib.h"
#include "vin_coarse.h"
#include "vin_fusion.h"

static InputCtl s_ctl;
static bool     s_pic_ok = false;
static void   (*s_ads_observer)(int16_t) = nullptr;
static void   (*s_pic_observer)(const uint16_t *) = nullptr;
// ADS1115 absolute calibration (9.2): volts = a * code + b. Applied to the fused value at the output.
static float    s_ads_a = ADS_LSB_V, s_ads_b = 0.0f;
static CalData  s_cal;                          // last applied set (CV values, for input_get_cal)
static bool     s_cal_loaded = false;

InputCtl &input_ctl() { return s_ctl; }
bool input_pic_ok() { return s_pic_ok; }
void input_set_ads_observer(void (*fn)(int16_t)) { s_ads_observer = fn; }
void input_set_pic_observer(void (*fn)(const uint16_t *)) { s_pic_observer = fn; }
bool input_cal_loaded() { return s_cal_loaded; }

void input_apply_cal(const CalData &d) {
    s_cal = d;
    s_ads_a = d.ads_a;
    s_ads_b = d.ads_b;
    calib_set_start(d.rel_gain, d.rel_offset);
    fusion_set_cal(d.rel_gain, d.rel_offset);
    pic_input_set_cv(d.cv_zero, d.cv_gain);
}

void input_get_cal(CalData *d) {
    *d = s_cal;
    d->ads_a = s_ads_a;
    d->ads_b = s_ads_b;
    d->rel_gain = calib_state().gain;            // the live relative calibration (9.3)
    d->rel_offset = calib_state().offset;
}

static inline float clamp_voct(float c) {        // 5.3: voct_cent is clamped to 0 .. 6000 cent
    return c < 0.0f ? 0.0f : (c > 6000.0f ? 6000.0f : c);
}

static inline float ads_to_cent(float code) {     // ADS LSB -> cent with the absolute calibration (9.2)
    return (s_ads_a * code + s_ads_b) * 1200.0f;
}

static float theory_gain() {                     // ADS LSB per coarse LSB, before calibration (5.6)
    return (CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV) / ADS_LSB_V;
}

void input_cal_reset() {
    calib_init(theory_gain(), 0.0f);
    fusion_set_cal(calib_state().gain, calib_state().offset);
}

// Unused pins are driven low (2.6). GP0 is UART0 TX, which is not used (PIC -> RP2350 only).
// GP16 is the on-board RGB LED and is left alone. GP6 may be the console's test trigger.
static void park_unused_pins() {
    static const uint8_t unused[] = {0, 2, 3, 4, 5, 7, 11, 15, 26, 28, 29};
    for (uint8_t p : unused) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_OUT);
        gpio_put(p, 0);
    }
}

void input_setup() {
    park_unused_pins();

    FusionConfig fc;
    fc.t_still_sum   = CFG_T_STILL_LSB * FUS_MA_LEN;
    fc.t_jump_sum    = CFG_T_JUMP_LSB * FUS_MA_LEN;
    fc.gain          = theory_gain();
    fc.offset        = 0.0f;
    fc.smooth_us     = CFG_FUSION_SMOOTH_MS * 1000.0f;
    fc.plausible_ads = CFG_FUSION_PLAUSIBLE_CENT / (ADS_LSB_V * 1200.0f);
    fusion_init(fc);                             // FAST until the first clean conversion (11.1 step 7)

    // 11.1 step 1: coarse ADC + DMA first, so a pitch exists as early as possible.
    if (!vin_coarse_begin()) s_ctl.voct_src = IN_VOCT_EXTERNAL;   // no DMA channel (never seen)
    // 11.1 step 2: calibration values from flash (theory defaults if missing / corrupt, 9.5).
    calib_init(fc.gain, fc.offset);
    CalData cd;
    s_cal_loaded = calstore_read(&cd);
    // 11.1 step 3: PIC UART with the pull-up; until the first frame the knobs read the 8.2 defaults.
    pic_rx_begin();
    PicInputConfig pc;
    pic_input_default_config(&pc);
    pic_input_init(pc);
    input_apply_cal(cd);
    // 11.1 steps 4 - 6: ADS1115 setup + read-back, then RDY interrupt on this core. Without the
    // ADS1115 the input runs degraded and retries every second (11.2).
    ads_begin();
    ads_start_irq();
}

void RAM_FUNC(input_loop)() {
    static uint32_t t_prev = 0;
    uint32_t now_us = time_us_32();
    uint32_t dt = t_prev ? now_us - t_prev : 0;
    t_prev = now_us;

    // ---- V/OCT ------------------------------------------------------------------------------
    vin_coarse_poll();                           // raw samples -> s / p, windows -> fusion (3, 5)
    ads_poll();                                  // RDY -> non-blocking read, fault handling (4, 11.2)
    if (!ads_stats().ok) fusion_force_fast();    // degraded: follow the coarse ADC

    uint32_t s_now = vin_coarse_stats().s_sum;
    int16_t code;
    uint32_t seq;
    if (ads_take_result(&code, &seq)) {
        if (s_ads_observer) s_ads_observer(code);
        fusion_ads_result(code, seq, s_now);
        float px;
        int16_t py;
        if (fusion_take_pair(&px, &py)) {        // clean conversion: relative calibration (9.3)
            calib_add(px, py);
            static uint32_t since = 0;
            if (s_ctl.cal_auto && ++since >= 64) {   // ≈ every 74 ms of clean conversions
                since = 0;
                if (calib_solve()) fusion_set_cal(calib_state().gain, calib_state().offset);
            }
        }
        if (s_ctl.voct_src == IN_VOCT_ADS) g_shared.voct_cent = clamp_voct(ads_to_cent(code));
    }
    float fused = fusion_output(s_now, dt);      // keeps the 5.5 smoothing running whatever the source
    if (s_ctl.voct_src == IN_VOCT_FUSION)   g_shared.voct_cent = clamp_voct(ads_to_cent(fused));
    else if (s_ctl.voct_src == IN_VOCT_ADC) g_shared.voct_cent = clamp_voct(vin_coarse_cent());

    // ---- PIC: knobs and CVs (8), every 100 µs ------------------------------------------------
    // Frames arrive every 1 ms and the UART FIFO holds 320 µs of bytes, so this part does not need
    // every pass; running it every pass cost the V/OCT path several µs (10.1 budget).
    static uint32_t t_pic = 0;
    if (now_us - t_pic < 100u) return;
    t_pic = now_us;
    pic_rx_poll();
    uint16_t raw[PIC_N_VALUES];
    uint32_t now_ms = millis();
    if (pic_rx_take(raw)) {
        pic_input_frame(raw, now_ms);
        if (s_pic_observer) s_pic_observer(raw);
    }
    const PicInputOut &po = pic_input_update(now_ms);   // hold / defaults on frame loss (8.2)
    s_pic_ok = po.pic_ok;
    if (s_ctl.knobs_from_pic) {
        g_shared.coarse_cent = po.coarse_cent;
        g_shared.fine_cent   = po.fine_cent;
        g_shared.wt_knob     = po.wt_knob;
        g_shared.wt_cv       = po.wt_cv;
        g_shared.fm_cent     = po.fm_cent;
    }

    g_shared.status = (fusion_is_fast() ? STATUS_VOCT_FAST : 0u) | (ads_stats().ok ? 0u : STATUS_ADS_FAIL) |
                      (s_pic_ok ? 0u : STATUS_PIC_LOST);
}
