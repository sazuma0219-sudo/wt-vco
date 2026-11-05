// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "console.h"

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "ads1115.h"
#include "audio.h"
#include "calib_store.h"
#include "config.h"
#include "control.h"
#include "input.h"
#include "pic_input.h"
#include "pic_rx.h"
#include "shared_params.h"
#include "tusb.h"
#include "USB.h"            // arduino-pico core: USB.mutex
#include "vin_calib.h"
#include "vin_coarse.h"
#include "vin_fusion.h"
#if CFG_DIAG
#include "hardware/pio.h"
#include "i2s_out.h"
#endif

static inline float coarse_lsb_cent() { return CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV * 1200.0f; }
static inline float clamp_voct(float c) { return c < 0.0f ? 0.0f : (c > 6000.0f ? 6000.0f : c); }

static const char *vsrc_name() {
    switch (input_ctl().voct_src) {
    case IN_VOCT_FUSION: return "fusion";
    case IN_VOCT_ADC:    return "adc";
    case IN_VOCT_ADS:    return "ads";
    default:             return "stub";
    }
}

// ---------------------------------------------------------------- output --
// Output never waits: text goes into a queue that out_flush() drains as far as USB accepts.
static char     s_out[2048];
static uint32_t s_out_head = 0, s_out_tail = 0;   // head = write, tail = read

static void out_str(const char *s) {
    while (*s) {
        uint32_t next = (s_out_head + 1) % sizeof s_out;
        if (next == s_out_tail) return;            // queue full: drop the rest
        s_out[s_out_head] = *s++;
        s_out_head = next;
    }
}

static void outf(const char *fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    out_str(buf);
}

// USB CDC access without blocking core1.
// arduino-pico's Serial methods take USB.mutex with a *blocking* enter and then run a full tud_task()
// on every call (available, read, availableForWrite, write), which held core1 for up to ~2.5 ms.
// Here we only try the mutex (skip this pass if core0's USB IRQ owns it) and touch the CDC FIFOs
// directly; tud_task() keeps running from arduino-pico's 1 ms IRQ on core0.
static bool usb_try_lock() {
    uint32_t owner;
    return mutex_try_enter(&USB.mutex, &owner);
}
static void usb_unlock() { mutex_exit(&USB.mutex); }

static void out_flush() {
    if (s_out_tail == s_out_head || !usb_try_lock()) return;
    uint32_t room = tud_cdc_write_available();
    bool wrote = false;
    while (room > 0 && s_out_tail != s_out_head) {
        uint32_t end = s_out_head > s_out_tail ? s_out_head : sizeof s_out;
        uint32_t n = end - s_out_tail;
        if (n > room) n = room;
        n = tud_cdc_write(&s_out[s_out_tail], n);
        if (!n) break;
        s_out_tail = (s_out_tail + n) % sizeof s_out;
        room -= n;
        wrote = true;
    }
    if (wrote) tud_cdc_write_flush();
    usb_unlock();
}

// ---------------------------------------------------------------- core1 pass timing (10.1) --
static uint32_t s_loop_max_us = 0;          // passes without a command
static uint32_t s_cmd_max_us = 0;           // passes that ran a command
static uint32_t s_steady_max_us = 0;        // passes away from commands (and their ADC backlog)
static uint32_t s_hist[6];                  // steady passes: ≤2, ≤5, ≤10, ≤20, ≤50, >50 µs

static void timing_add(uint32_t dt, bool ran_cmd) {
    if (!ran_cmd && dt > s_loop_max_us) s_loop_max_us = dt;
    if (ran_cmd && dt > s_cmd_max_us) s_cmd_max_us = dt;
    static uint32_t skip = 0;
    if (ran_cmd) skip = 3;
    else if (skip) skip--;
    else {
        if (dt > s_steady_max_us) s_steady_max_us = dt;
        s_hist[dt <= 2 ? 0 : dt <= 5 ? 1 : dt <= 10 ? 2 : dt <= 20 ? 3 : dt <= 50 ? 4 : 5]++;
    }
}

// ---------------------------------------------------------------- V/OCT step test (latency) --
static bool     s_step_on = false;
static float    s_step_a = 0, s_step_b = 1200;
static uint32_t s_step_half_us = 250000;
static uint32_t s_step_t0 = 0;
static bool     s_step_phase = false;

static void step_service() {
    if (!s_step_on) return;
    uint32_t now = time_us_32();
    if (now - s_step_t0 < s_step_half_us) return;
    s_step_t0 += s_step_half_us;
    s_step_phase = !s_step_phase;
    gpio_put(CFG_STUB_TRIG_PIN, s_step_phase);    // trigger edge and value change back to back
    g_shared.voct_cent = s_step_phase ? s_step_b : s_step_a;
}

// ---------------------------------------------------------------- mute handshake test (9.5) --
enum MuteTest { MT_IDLE, MT_WAIT_ACK, MT_HOLD };
static MuteTest s_mt = MT_IDLE;
static uint32_t s_mt_t0 = 0, s_mt_ack_us = 0, s_mt_hold_ms = 500;

static void mute_service() {
    uint32_t now = time_us_32();
    switch (s_mt) {
    case MT_IDLE:
        break;
    case MT_WAIT_ACK:
        if (g_shared.audio_mute_ack) {
            s_mt_ack_us = now - s_mt_t0;
            s_mt_t0 = now;
            s_mt = MT_HOLD;
        } else if (now - s_mt_t0 > 50000u) {
            g_shared.audio_mute_req = 0;
            s_mt = MT_IDLE;
            out_str("mute: NO ACK within 50 ms -> aborted (a real save would be cancelled)\r\n");
        }
        break;
    case MT_HOLD:                                   // stands in for the flash write
        if (now - s_mt_t0 >= s_mt_hold_ms * 1000u) {
            g_shared.audio_mute_req = 0;
            s_mt = MT_IDLE;
            outf("mute: ack after %lu us, held %lu ms, released\r\n",
                 (unsigned long)s_mt_ack_us, (unsigned long)s_mt_hold_ms);
        }
        break;
    }
}

// ---------------------------------------------------------------- core0 load --
static float s_load_pct = 0;

static void load_service() {
    static uint32_t t0 = 0, b0 = 0;
    uint32_t now = time_us_32();
    if (now - t0 < 1000000u) return;
    uint32_t b = audio_busy_us;
    s_load_pct = 100.0f * (float)(b - b0) / (float)(now - t0);
    t0 = now; b0 = b;
}

#if CFG_DIAG
// ---------------------------------------------------------------- stall monitor while USB is unplugged --
// Every 100 ms the audio loop's worst stall and underruns are collected. Windows that end while USB
// is not mounted (or suspended: no VBUS sensing on this board) count as "unplugged", except the last
// two before the next mount (re-enumeration may already have started in them).
#define MON_WINDOW_MS 100
static uint32_t s_mon_t = 0, s_mon_under_prev = 0;
static uint32_t s_hist_gap[2] = {0, 0}, s_hist_under[2] = {0, 0}, s_hist_n = 0;
static uint32_t s_off_windows = 0, s_off_max_gap = 0, s_off_underruns = 0;
static bool     s_monitor = false;

static void monitor_service() {
    if (!s_monitor) return;
    uint32_t now = millis();
    if (now - s_mon_t < MON_WINDOW_MS) return;
    s_mon_t = now;
    uint32_t gap = audio_max_gap_us, under = audio_underrun_cnt;
    audio_min_lead_reset = 1;
    uint32_t dunder = under - s_mon_under_prev;
    s_mon_under_prev = under;
    if (!tud_mounted() || tud_suspended()) {
        if (s_hist_n == 2) {
            s_off_windows++;
            if (s_hist_gap[0] > s_off_max_gap) s_off_max_gap = s_hist_gap[0];
            s_off_underruns += s_hist_under[0];
            s_hist_gap[0] = s_hist_gap[1]; s_hist_under[0] = s_hist_under[1];
            s_hist_n = 1;
        }
        s_hist_gap[s_hist_n] = gap; s_hist_under[s_hist_n] = dunder; s_hist_n++;
    } else {
        s_hist_n = 0;
    }
}

static void i2s_enable(bool on) {
    const I2sOutInfo &inf = i2s_out_info();
    pio_sm_set_enabled(PIO_INSTANCE(inf.pio_index), inf.sm, on);
}
#endif

// ---------------------------------------------------------------- calibration (9) --
// calmode on -> vcal <mV> (several points, DMM reading) -> vcal fit; czero with all CVs unplugged;
// calsave writes everything to flash behind the audio mute handshake.
static bool s_calmode = false;

#define VCAL_MAX_POINTS 8
#define VCAL_COLLECT_N  430                       // ≈ 0.5 s of ADS1115 results
static float    s_vp_code[VCAL_MAX_POINTS], s_vp_volt[VCAL_MAX_POINTS];
static uint32_t s_vp_n = 0;
static bool     s_vc_on = false;                  // collecting ADS results for one point
static float    s_vc_volt = 0;
static uint32_t s_vc_n = 0;
static int64_t  s_vc_sum = 0, s_vc_sumsq = 0;

#define CZERO_COLLECT_N 1000                      // ≈ 1 s of PIC frames (9.4)
static bool     s_cz_on = false;
static uint32_t s_cz_n = 0;
static uint64_t s_cz_sum[4];

static void pic_observer(const uint16_t *raw) {
    if (!s_cz_on || s_cz_n >= CZERO_COLLECT_N) return;
    for (int i = 0; i < 4; i++) s_cz_sum[i] += raw[i];   // WT_UNI, WT_BI, FM_UNI, FM_BI
    s_cz_n++;
}

// ---------------------------------------------------------------- ADS1115 statistics --
static uint32_t a_n = 0;
static int32_t  a_min = INT32_MAX, a_max = INT32_MIN;
static int64_t  a_sum = 0, a_sumsq = 0;
static void ads_observer(int16_t c) {
    a_n++; a_sum += c; a_sumsq += (int64_t)c * c;
    if (c < a_min) a_min = c;
    if (c > a_max) a_max = c;
    if (s_vc_on && s_vc_n < VCAL_COLLECT_N) { s_vc_n++; s_vc_sum += c; s_vc_sumsq += (int64_t)c * c; }
}

// ---------------------------------------------------------------- reports --
static void show() {
    outf("coarse=%.2f fine=%.2f voct=%.2f wtknob=%.3f wtcv=%.3f fm=%.2f status=0x%08lx\r\n",
         g_shared.coarse_cent, g_shared.fine_cent, g_shared.voct_cent, g_shared.wt_knob,
         g_shared.wt_cv, g_shared.fm_cent, (unsigned long)g_shared.status);
    outf("underrun=%lu min_lead=%lu/%d max_gap=%lu us stalls>%dus=%lu mute_req=%lu ack=%lu step=%s\r\n",
         (unsigned long)audio_underrun_cnt, (unsigned long)audio_min_lead, CFG_OUT_LEAD_FRAMES,
         (unsigned long)audio_max_gap_us, AUDIO_STALL_US, (unsigned long)audio_stall_cnt,
         (unsigned long)g_shared.audio_mute_req, (unsigned long)g_shared.audio_mute_ack,
         s_step_on ? "on" : "off");
    outf("core0 load=%.1f %% (rendering, last 1 s)\r\n", s_load_pct);
}

static void adc_report() {
    static uint32_t t0 = 0, n0 = 0;
    const VinStats &st = vin_coarse_stats();
    uint32_t now = time_us_32(), n = st.samples;
    float rate = t0 ? (float)(n - n0) * 1e6f / (float)(now - t0) : 0.0f;
    t0 = now; n0 = n;
    VinBlockStats b = vin_coarse_take_block_stats();
    outf("adc: %.0f S/s  raw=%lu  s=%.2f LSB (%.1f cent, %.4f V)  err=%lu overrun=%lu  vsrc=%s\r\n",
         rate, (unsigned long)st.last_raw, st.s_sum / 16.0f, vin_coarse_cent(),
         vin_coarse_cent() / 1200.0f, (unsigned long)st.err_cnt, (unsigned long)st.overrun_cnt, vsrc_name());
    outf("  p (16-avg) over %lu blocks: mean=%.2f  sd=%.3f  p-p=%.2f LSB  (sd %.2f cent, p-p %.2f cent)\r\n",
         (unsigned long)b.n, b.mean_sum / 16.0, b.sd_sum / 16.0, (b.max_sum - b.min_sum) / 16.0,
         b.sd_sum / 16.0 * coarse_lsb_cent(), (b.max_sum - b.min_sum) / 16.0 * coarse_lsb_cent());
    outf("  core1 pass max=%lu us (no command), command pass max=%lu us\r\n",
         (unsigned long)s_loop_max_us, (unsigned long)s_cmd_max_us);
    uint32_t tot = 0;
    for (auto h : s_hist) tot += h;
    if (!tot) tot = 1;
    outf("  steady passes: max=%lu us  <=2:%.2f%% <=5:%.2f%% <=10:%.3f%% <=20:%.4f%% <=50:%.4f%% >50:%lu passes\r\n",
         (unsigned long)s_steady_max_us, 100.0 * s_hist[0] / tot, 100.0 * s_hist[1] / tot,
         100.0 * s_hist[2] / tot, 100.0 * s_hist[3] / tot, 100.0 * s_hist[4] / tot, (unsigned long)s_hist[5]);
    s_loop_max_us = 0; s_cmd_max_us = 0; s_steady_max_us = 0;
    for (auto &h : s_hist) h = 0;
}

#if CFG_DIAG && CFG_VIN_DIAG
static void noise_report() {
    static uint32_t t0 = 0;
    uint32_t now = time_us_32();
    float secs = t0 ? (now - t0) / 1e6f : 0.0f;
    t0 = now;
    VinDiag g = vin_coarse_take_diag();
    const float pct[4] = {0.99f, 0.999f, 0.9999f, 1.0f};
    float pv[4] = {0, 0, 0, 0};
    for (int q = 0; q < 4; q++) {
        uint32_t need = (uint32_t)(pct[q] * g.windows + 0.5f), acc = 0;
        for (int b = 0; b < VIN_DIAG_RANGE_BINS; b++) {
            acc += g.range_hist[b];
            if (acc >= need) { pv[q] = (b + 1) * 0.5f; break; }
        }
    }
    outf("noise: %.1f s, %lu windows of %d p.  window range (LSB): 99%%<=%.1f 99.9%%<=%.1f 99.99%%<=%.1f max=%.2f (%.1f cent)\r\n",
         secs, (unsigned long)g.windows, VIN_DIAG_WIN_BLOCKS, pv[0], pv[1], pv[2],
         g.range_max_sum / 16.0f, g.range_max_sum / 16.0f * coarse_lsb_cent());
    out_str("  jump triggers (|s-ref| > T for 17 samples):");
    for (int k = 0; k < VIN_DIAG_NT; k++)
        outf(" T=%lu(%.1fc):%lu", (unsigned long)g.t_lsb[k], g.t_lsb[k] * coarse_lsb_cent(), (unsigned long)g.triggers[k]);
    out_str("\r\n");
}
#endif

static void pic_report() {
    static PicStats prev{};
    static uint32_t t0 = 0;
    PicStats st = pic_rx_stats();
    uint32_t now = millis();
    float secs = t0 ? (now - t0) / 1000.0f : 0.0f;
    t0 = now;
    uint32_t fr = st.frames - prev.frames;
    outf("pic: %lu frames in %.2f s (%.1f /s), last %lu ms ago  errors: checksum %lu sync %lu framing %lu overrun %lu break %lu seq-gap %lu\r\n",
         (unsigned long)fr, secs, secs > 0 ? fr / secs : 0.0f, (unsigned long)(now - st.last_frame_ms),
         (unsigned long)(st.checksum_err - prev.checksum_err), (unsigned long)(st.sync_err - prev.sync_err),
         (unsigned long)(st.framing_err - prev.framing_err), (unsigned long)(st.overrun_err - prev.overrun_err),
         (unsigned long)(st.break_err - prev.break_err), (unsigned long)(st.seq_gaps - prev.seq_gaps));
    static const char *const name[PIC_N_VALUES] = {"WT_UNI", "WT_BI", "FM_UNI", "FM_BI", "COARSE", "FINE", "KNOB", "DEPTH"};
    const uint16_t *v = pic_rx_last();
    for (int i = 0; i < PIC_N_VALUES; i++)
        outf("  %-6s %5u  (%.3f V of VDD 5 V)\r\n", name[i], v[i], v[i] * 5.0f / 16380.0f);
    outf("  -> %s  coarse=%.2f fine=%.2f wtknob=%.3f wtcv=%.3f fm=%.1f  (ksrc %s)\r\n",
         input_pic_ok() ? "PIC ok" : "PIC LOST (8.2 defaults)", g_shared.coarse_cent, g_shared.fine_cent,
         g_shared.wt_knob, g_shared.wt_cv, g_shared.fm_cent, input_ctl().knobs_from_pic ? "pic" : "stub");
    prev = st;
}

static void cal_report() {
    const CalState &c = calib_state();
    const float v_per_coarse = CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV;
    // The fit expressed the other way round, as the coarse reading of a true voltage: coarse = g*V + o.
    float g = v_per_coarse / (c.gain * ADS_LSB_V);
    float o_mv = -c.offset * ADS_LSB_V * g * 1000.0f;
    outf("cal: %s  pairs=%lu fits=%lu offset-only=%lu bins used=%u\r\n", input_ctl().cal_auto ? "auto" : "PAUSED",
         (unsigned long)c.pairs, (unsigned long)c.fits, (unsigned long)c.offset_only, c.bins_used);
    outf("  gain %.4f ADS/coarse LSB (theory %.4f), offset %.1f ADS LSB  ->  coarse reads %.4f * V %+.1f mV\r\n",
         c.gain, c.gain_theory, c.offset, g, o_mv);
    for (int i = 0; i < CAL_BINS; i++) {
        if (!c.bin[i].n) continue;
        float resid = (c.gain * c.bin[i].x + c.offset - c.bin[i].y) * ADS_LSB_V * 1200.0f;
        outf("  bin %d: n=%4lu  %.4f V  coarse %.2f LSB  residual %+.2f cent\r\n", i, (unsigned long)c.bin[i].n,
             c.bin[i].y * ADS_LSB_V, c.bin[i].x, resid);
    }
}

static void fus_report() {
    static FusionStats prev{};
    FusionStats st = fusion_stats();
    outf("fus: state=%s voct=%.2f cent  FAST offset=%.2f coarse LSB (%.1f cent)  vsrc=%s\r\n",
         fusion_is_fast() ? "FAST" : "ADS", g_shared.voct_cent, fusion_offset_sum16() / 16.0f,
         fusion_offset_sum16() / 16.0f * coarse_lsb_cent(), vsrc_name());
    outf("  windows %lu  ads_updates %lu  ->FAST jump %lu dirty %lu  ->ADS %lu  rejected: unclean %lu moved %lu seq %lu discarded %lu implausible %lu\r\n",
         (unsigned long)(st.windows - prev.windows), (unsigned long)(st.ads_updates - prev.ads_updates),
         (unsigned long)(st.to_fast_jump - prev.to_fast_jump), (unsigned long)(st.to_fast_dirty - prev.to_fast_dirty),
         (unsigned long)(st.to_ads - prev.to_ads), (unsigned long)(st.rej_unclean - prev.rej_unclean),
         (unsigned long)(st.rej_moved - prev.rej_moved), (unsigned long)(st.rej_seq - prev.rej_seq),
         (unsigned long)(st.rej_discarded - prev.rej_discarded),
         (unsigned long)(st.rej_implausible - prev.rej_implausible));
#if CFG_DIAG
    VinBqDebug q = vin_coarse_bq_debug();
    outf("  seq: rdy=%lu closed=%lu result=%lu  boundary queue head=%lu tail=%lu next idx=%lu seq=%lu  rd=%lu wr=%lu\r\n",
         (unsigned long)ads_stats().rdy_cnt, (unsigned long)st.last_closed_seq, (unsigned long)st.last_result_seq,
         (unsigned long)q.head, (unsigned long)q.tail, (unsigned long)q.next_idx, (unsigned long)q.next_seq,
         (unsigned long)q.rd, (unsigned long)vin_coarse_write_index());
#endif
    prev = st;
}

static void ads_report() {
    static uint32_t t0 = 0, r0 = 0;
    const AdsStats &st = ads_stats();
    uint32_t now = time_us_32();
    float rate = t0 ? (st.result_cnt - r0) * 1e6f / (now - t0) : 0.0f;
    t0 = now; r0 = st.result_cnt;
    outf("ads: %s  RDY timeouts %lu, reinit %lu (failed %lu), bus releases %lu, skipped after restart %lu\r\n",
         st.ok ? "running" : "DEGRADED (coarse ADC only, retrying every second)",
         (unsigned long)st.rdy_timeout_cnt, (unsigned long)st.reinit_cnt, (unsigned long)st.reinit_fail_cnt,
         (unsigned long)st.bus_release_cnt, (unsigned long)st.skipped_cnt);
    if (!st.ok) return;
    const float cent_per = ADS_LSB_V * 1200.0f;
    outf("ads: %.1f res/s  code=%d (%.4f V, %.1f cent)  rdy=%lu late=%lu abort=%lu  vsrc=%s\r\n",
         rate, st.last_code, st.last_code * ADS_LSB_V, st.last_code * cent_per,
         (unsigned long)st.rdy_cnt, (unsigned long)st.late_cnt, (unsigned long)st.abort_cnt, vsrc_name());
    outf("  RDY period %lu..%lu us, RDY->result max %lu us\r\n",
         (unsigned long)(st.rdy_min_us == UINT32_MAX ? 0 : st.rdy_min_us), (unsigned long)st.rdy_max_us,
         (unsigned long)st.read_max_us);
    if (a_n) {
        double mean = (double)a_sum / a_n, var = (double)a_sumsq / a_n - mean * mean;
        double sd = var > 0 ? sqrt(var) : 0;
        outf("  %lu results: mean=%.2f sd=%.2f p-p=%ld LSB  (sd %.2f cent, p-p %.2f cent)\r\n",
             (unsigned long)a_n, mean, sd, (long)(a_max - a_min), sd * cent_per, (a_max - a_min) * cent_per);
    }
    a_n = 0; a_sum = 0; a_sumsq = 0; a_min = INT32_MAX; a_max = INT32_MIN;
    ads_reset_minmax();
}

static void cal_show() {
    CalData d;
    input_get_cal(&d);
    outf("calibration (%s at start-up, mode %s):\r\n", input_cal_loaded() ? "loaded from flash" : "theory defaults",
         s_calmode ? "ON" : "off");
    outf("  ADS1115:  V = %.9f * code %+.6f   (theory %.9f * code)\r\n", d.ads_a, d.ads_b, ADS_LSB_V);
    outf("  coarse:   ads = %.5f * coarse %+.2f\r\n", d.rel_gain, d.rel_offset);
    static const char *const nm[4] = {"WT UNI", "WT BI ", "FM UNI", "FM BI "};
    for (int i = 0; i < 4; i++)
        outf("  %s:   zero %.1f (default %s)  gain %.4f mV/LSB\r\n", nm[i], d.cv_zero[i], (i & 1) ? "8190" : "0",
             d.cv_gain[i] * 1000.0f);
    outf("  ADS points collected: %lu\r\n", (unsigned long)s_vp_n);
    for (uint32_t i = 0; i < s_vp_n; i++)
        outf("    %lu: %.4f V  code %.2f\r\n", (unsigned long)i, s_vp_volt[i], s_vp_code[i]);
}

static void vcal_fit() {
    if (s_vp_n < 2) { out_str("vcal fit: need at least 2 points\r\n"); return; }
    double sx = 0, sy = 0, sxx = 0, sxy = 0, xmin = 1e9, xmax = -1e9;
    for (uint32_t i = 0; i < s_vp_n; i++) {
        sx += s_vp_code[i]; sy += s_vp_volt[i];
        sxx += (double)s_vp_code[i] * s_vp_code[i]; sxy += (double)s_vp_code[i] * s_vp_volt[i];
        if (s_vp_code[i] < xmin) xmin = s_vp_code[i];
        if (s_vp_code[i] > xmax) xmax = s_vp_code[i];
    }
    if ((xmax - xmin) * ADS_LSB_V < 1.0) { out_str("vcal fit: points must span at least 1 V (e.g. 0.2 V and 4.8 V)\r\n"); return; }
    double n = s_vp_n, den = n * sxx - sx * sx;
    double a = (n * sxy - sx * sy) / den, b = (sy - a * sx) / n;
    if (a < ADS_LSB_V * 0.98 || a > ADS_LSB_V * 1.02) {
        outf("vcal fit: gain %.9f is more than 2 %% off the theory - check the readings; not applied\r\n", a);
        return;
    }
    CalData d;
    input_get_cal(&d);
    d.ads_a = (float)a;
    d.ads_b = (float)b;
    input_apply_cal(d);
    outf("vcal fit: V = %.9f * code %+.6f  (gain %+.3f %% vs theory, offset %+.2f mV) applied, not saved\r\n",
         a, b, (a / ADS_LSB_V - 1.0) * 100.0, b * 1000.0);
    for (uint32_t i = 0; i < s_vp_n; i++)
        outf("  point %lu: %.4f V  residual %+.2f cent\r\n", (unsigned long)i, s_vp_volt[i],
             (a * s_vp_code[i] + b - s_vp_volt[i]) * 1200.0);
}

// Finishes measurements started by commands (called from the service part of console_loop).
static void calib_service() {
    if (s_vc_on && s_vc_n >= VCAL_COLLECT_N) {
        s_vc_on = false;
        double mean = (double)s_vc_sum / s_vc_n, var = (double)s_vc_sumsq / s_vc_n - mean * mean;
        double sd = var > 0 ? sqrt(var) : 0;
        if (sd > 5.0) {
            outf("vcal: reading not steady (sd %.1f LSB) - point discarded\r\n", sd);
        } else if (s_vp_n >= VCAL_MAX_POINTS) {
            out_str("vcal: point list full ('vcal clear')\r\n");
        } else {
            s_vp_code[s_vp_n] = (float)mean;
            s_vp_volt[s_vp_n] = s_vc_volt;
            s_vp_n++;
            outf("vcal: point %lu  %.4f V  code %.2f (sd %.2f)  theory says %.4f V\r\n", (unsigned long)(s_vp_n - 1),
                 s_vc_volt, mean, sd, mean * ADS_LSB_V);
        }
    }
    if (s_cz_on && s_cz_n >= CZERO_COLLECT_N) {
        s_cz_on = false;
        CalData d;
        input_get_cal(&d);
        static const char *const nm[4] = {"WT UNI", "WT BI", "FM UNI", "FM BI"};
        static const float def[4] = {0.0f, 8190.0f, 0.0f, 8190.0f};
        int applied = 0;
        for (int i = 0; i < 4; i++) {
            float z = (float)s_cz_sum[i] / s_cz_n;
            if (fabsf(z - def[i]) > 400.0f) {          // 9.4: something plugged in / wiring fault
                outf("czero: %s reads %.1f, more than 400 LSB from %.0f - NOT applied\r\n", nm[i], z, def[i]);
                continue;
            }
            outf("czero: %s zero %.1f (default %.0f, %+.1f mV)\r\n", nm[i], z, def[i], (z - def[i]) * d.cv_gain[i] * 1000.0f);
            d.cv_zero[i] = z;
            applied++;
        }
        input_apply_cal(d);
        outf("czero: %d of 4 applied, not saved ('calsave')\r\n", applied);
    }
}

// ---- save behind the mute handshake (9.5) ----
enum SaveState { SV_IDLE, SV_WAIT_ACK };
static SaveState s_sv = SV_IDLE;
static uint32_t  s_sv_t0 = 0;

static void save_service() {
    if (s_sv != SV_WAIT_ACK) return;
    if (g_shared.audio_mute_ack) {
        CalData d;
        input_get_cal(&d);
        uint32_t t0 = millis();
        bool ok = calstore_write(d);               // both cores stall here; the DMA plays the silent ring
        uint32_t dt = millis() - t0;
        g_shared.audio_mute_req = 0;
        s_sv = SV_IDLE;
        outf("calsave: %s (flash write %lu ms), audio released\r\n", ok ? "saved" : "FAILED", (unsigned long)dt);
    } else if (millis() - s_sv_t0 > 50) {
        g_shared.audio_mute_req = 0;
        s_sv = SV_IDLE;
        out_str("calsave: no mute ack within 50 ms - save cancelled\r\n");
    }
}

static void help() {
    out_str("monitor:\r\n"
            "  show (s)             shared variables, audio statistics, core0 load\r\n"
            "  adc / ads / fus / pic / cal   coarse ADC, ADS1115, fusion, PIC link, relative calibration\r\n"
            "  watch                toggle a 1 s audio status line;  resetlead\r\n"
            "test overrides:\r\n"
            "  coarse <0..7200>  fine <-100..100>  wtknob <0..1>  wtcv <-0.5..1.5>  fm <-6000..18000>  (-> ksrc stub)\r\n"
            "  voct <0..6000>       (-> vsrc stub)\r\n"
            "  ksrc pic|stub        knobs and CVs from the PIC (default) or from the commands above\r\n"
            "  vsrc fusion|adc|ads|stub   V/OCT from the fusion (default), one converter alone, or 'voct'\r\n"
            "  step <a> <b> <ms>    V/OCT alternates a/b every <ms>, trigger on GP");
    outf("%d;  step off\r\n", CFG_STUB_TRIG_PIN);
    out_str("  mute [hold_ms]       mute handshake test (9.5)\r\n"
            "  cal auto|reset       resume / restart the relative calibration\r\n"
            "  default              vsrc fusion, ksrc pic, 8.2 defaults\r\n"
            "listening tests:\r\n"
            "  tune                 time constants / dead bands (RAM only, not saved)\r\n"
            "  tune <name> [value]  show / set one;  tune reset   back to the compiled values\r\n"
            "calibration (9):\r\n"
            "  calmode on|off       enter / leave calibration mode (needed for the commands below)\r\n"
            "  vcal <mV>            V/OCT held at a DMM-measured voltage: record a point (0.5 s)\r\n"
            "  vcal fit|clear|list  fit V = a*code + b over the points / forget them / show them\r\n"
            "  czero                all CV inputs unplugged: measure the zero points (1 s)\r\n"
            "  calsave              save everything to flash (audio is muted briefly)\r\n"
            "  calshow / caldefault show the values / back to theory (not saved)\r\n");
#if CFG_DIAG
    out_str("diagnostics (CFG_DIAG):\r\n"
            "  fcal <g> <o_mV> | fcal off   manual coarse scale (pauses 'cal auto')\r\n"
            "  ads stop             stop the ADS1115 conversions (watchdog test)\r\n"
            "  i2s on|off           stop the I2S clocks (noise test; audio stops)\r\n"
            "  monitor / offline    stalls while USB is unplugged\r\n"
#if CFG_VIN_DIAG
            "  noise                threshold study (CFG_VIN_DIAG)\r\n"
#endif
            );
#endif
}

// ------------------------------------------------------------------- tune --
// Time constants and dead bands for listening tests (pic_input 8.3 - 8.6, control §1.10, fusion 5.5).
// RAM only: not saved, a restart brings back the compiled values.
enum TuneId { TU_COARSE_TAU, TU_COARSE_HYST, TU_FINE_TAU, TU_FINE_HYST, TU_KNOB_TAU, TU_KNOB_HYST,
              TU_DEPTH_TAU, TU_DEPTH_HYST, TU_CV_TAU, TU_A_KNOB, TU_A_FM, TU_A_WT, TU_FUSION, TU_N };

struct TuneDef { const char *name, *unit; float lo, hi; const char *what; };
static const TuneDef k_tune[TU_N] = {
    {"coarse_tau",  "ms",   0, 500,  "COARSE knob IIR (PIC, 1 kHz)"},
    {"coarse_hyst", "cent", 0, 50,   "COARSE dead band"},
    {"fine_tau",    "ms",   0, 500,  "FINE knob IIR"},
    {"fine_hyst",   "cent", 0, 5,    "FINE dead band"},
    {"knob_tau",    "ms",   0, 500,  "WT knob IIR"},
    {"knob_hyst",   "",     0, 0.05f, "WT knob dead band (0..1 units)"},
    {"depth_tau",   "ms",   0, 500,  "FM DEPTH IIR"},
    {"depth_hyst",  "",     0, 0.05f, "FM DEPTH dead band (k units)"},
    {"cv_tau",      "ms",   0, 50,   "WT CV / FM CV IIR"},
    {"a_knob",      "ms",   0, 50,   "audio side: coarse + fine (6 kHz)"},
    {"a_fm",        "ms",   0, 50,   "audio side: FM"},
    {"a_wt",        "ms",   0, 50,   "audio side: WT position"},
    {"fusion",      "ms",   0.1f, 50, "fusion FAST <-> ADS transition"},
};

static void tune_get(float v[TU_N]) {
    const PicInputConfig &c = pic_input_config();
    v[TU_COARSE_TAU] = c.coarse_tau_ms; v[TU_COARSE_HYST] = c.coarse_hyst_cent;
    v[TU_FINE_TAU]   = c.fine_tau_ms;   v[TU_FINE_HYST]   = c.fine_hyst_cent;
    v[TU_KNOB_TAU]   = c.knob_tau_ms;   v[TU_KNOB_HYST]   = c.knob_hyst;
    v[TU_DEPTH_TAU]  = c.depth_tau_ms;  v[TU_DEPTH_HYST]  = c.depth_hyst;
    v[TU_CV_TAU]     = c.cv_tau_ms;
    control_get_smoothing(&v[TU_A_KNOB], &v[TU_A_FM], &v[TU_A_WT]);
    v[TU_FUSION]     = fusion_smooth_us() / 1000.0f;
}

static void tune_set(const float v[TU_N]) {
    pic_input_retune(v[TU_COARSE_TAU], v[TU_COARSE_HYST], v[TU_FINE_TAU], v[TU_FINE_HYST],
                     v[TU_KNOB_TAU], v[TU_KNOB_HYST], v[TU_DEPTH_TAU], v[TU_DEPTH_HYST], v[TU_CV_TAU]);
    control_set_smoothing(v[TU_A_KNOB], v[TU_A_FM], v[TU_A_WT]);
    fusion_set_smooth_us(v[TU_FUSION] * 1000.0f);
}

static void tune_defaults(float v[TU_N]) {
    PicInputConfig c;
    pic_input_default_config(&c);
    v[TU_COARSE_TAU] = c.coarse_tau_ms; v[TU_COARSE_HYST] = c.coarse_hyst_cent;
    v[TU_FINE_TAU]   = c.fine_tau_ms;   v[TU_FINE_HYST]   = c.fine_hyst_cent;
    v[TU_KNOB_TAU]   = c.knob_tau_ms;   v[TU_KNOB_HYST]   = c.knob_hyst;
    v[TU_DEPTH_TAU]  = c.depth_tau_ms;  v[TU_DEPTH_HYST]  = c.depth_hyst;
    v[TU_CV_TAU]     = c.cv_tau_ms;
    v[TU_A_KNOB] = CFG_SMOOTH_KNOB_MS; v[TU_A_FM] = CFG_SMOOTH_FM_MS; v[TU_A_WT] = CFG_SMOOTH_WT_MS;
    v[TU_FUSION] = CFG_FUSION_SMOOTH_MS;
}

static void tune_show() {
    float v[TU_N], d[TU_N];
    tune_get(v);
    tune_defaults(d);
    out_str("tune (RAM only, not saved):\r\n");
    for (int i = 0; i < TU_N; i++)
        outf("  %-12s %8.4g %-4s (default %g)%s  %s\r\n", k_tune[i].name, v[i], k_tune[i].unit, d[i],
             v[i] != d[i] ? " *" : "  ", k_tune[i].what);
}

static void tune_cmd(const char *a1, const char *a2) {
    float v[TU_N];
    if (!a1) { tune_show(); return; }
    if (!strcmp(a1, "reset")) { tune_defaults(v); tune_set(v); out_str("tune: compiled values restored\r\n"); return; }
    for (int i = 0; i < TU_N; i++) {
        if (strcmp(a1, k_tune[i].name)) continue;
        tune_get(v);
        if (!a2) { outf("%s = %g %s\r\n", k_tune[i].name, v[i], k_tune[i].unit); return; }
        char *end;
        float x = strtof(a2, &end);
        if (end == a2 || !(x >= k_tune[i].lo && x <= k_tune[i].hi)) {
            outf("%s: value must be %g .. %g\r\n", k_tune[i].name, k_tune[i].lo, k_tune[i].hi);
            return;
        }
        v[i] = x;
        tune_set(v);
        outf("%s = %g %s\r\n", k_tune[i].name, x, k_tune[i].unit);
        return;
    }
    outf("tune: unknown name '%s' (tune lists them)\r\n", a1);
}

static bool set_param(volatile float *dst, const char *name, const char *arg, float lo, float hi) {
    char *end;
    float v = strtof(arg, &end);
    if (end == arg || v < lo || v > hi) {
        outf("%s: value must be %g .. %g\r\n", name, lo, hi);
        return false;
    }
    *dst = v;
    outf("%s = %g\r\n", name, v);
    return true;
}

static void defaults() {
    g_shared.coarse_cent = 3600.0f;     // wtvco_input_spec.md 8.2
    g_shared.fine_cent   = 0.0f;
    g_shared.voct_cent   = 0.0f;
    g_shared.wt_cv       = 0.0f;
    g_shared.wt_knob     = 0.0f;
    g_shared.fm_cent     = 0.0f;
}

static bool s_watch = false;

// ---------------------------------------------------------------- commands --
static void command(char *line) {
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *a1 = strtok(nullptr, " \t");
    char *a2 = strtok(nullptr, " \t");
    char *a3 = strtok(nullptr, " \t");
    const char *arg = a1 ? a1 : "";
    InputCtl &ctl = input_ctl();

    if      (!strcmp(cmd, "show") || !strcmp(cmd, "s")) show();
    else if (!strcmp(cmd, "adc"))    adc_report();
    else if (!strcmp(cmd, "fus"))    fus_report();
    else if (!strcmp(cmd, "pic"))    pic_report();
    else if (!strcmp(cmd, "ads")) {
#if CFG_DIAG
        if (a1 && !strcmp(a1, "stop")) { ads_debug_stop(); out_str("ads: forced to single-shot (RDY stops) -> watchdog test\r\n"); return; }
#endif
        ads_report();
    }
    else if (!strcmp(cmd, "cal")) {
        if (a1 && !strcmp(a1, "auto"))       ctl.cal_auto = true;
        else if (a1 && !strcmp(a1, "reset")) input_cal_reset();
        cal_report();
    }
    else if (!strcmp(cmd, "coarse")) { ctl.knobs_from_pic = false; set_param(&g_shared.coarse_cent, "coarse", arg, 0, 7200); }
    else if (!strcmp(cmd, "fine"))   { ctl.knobs_from_pic = false; set_param(&g_shared.fine_cent,   "fine",   arg, -100, 100); }
    else if (!strcmp(cmd, "wtknob")) { ctl.knobs_from_pic = false; set_param(&g_shared.wt_knob,     "wtknob", arg, 0, 1); }
    else if (!strcmp(cmd, "wtcv"))   { ctl.knobs_from_pic = false; set_param(&g_shared.wt_cv,       "wtcv",   arg, -0.5f, 1.5f); }
    else if (!strcmp(cmd, "fm"))     { ctl.knobs_from_pic = false; set_param(&g_shared.fm_cent,     "fm",     arg, -6000, 18000); }
    else if (!strcmp(cmd, "voct"))   { ctl.voct_src = IN_VOCT_EXTERNAL; set_param(&g_shared.voct_cent, "voct", arg, 0, 6000); }
    else if (!strcmp(cmd, "ksrc")) {
        if (a1 && !strcmp(a1, "pic"))       ctl.knobs_from_pic = true;
        else if (a1 && !strcmp(a1, "stub")) ctl.knobs_from_pic = false;
        outf("ksrc %s\r\n", ctl.knobs_from_pic ? "pic" : "stub");
    }
    else if (!strcmp(cmd, "vsrc")) {
        if (a1 && !strcmp(a1, "fusion"))    { s_step_on = false; ctl.voct_src = IN_VOCT_FUSION; }
        else if (a1 && !strcmp(a1, "adc"))  { s_step_on = false; ctl.voct_src = IN_VOCT_ADC; }
        else if (a1 && !strcmp(a1, "ads"))  { s_step_on = false; ctl.voct_src = IN_VOCT_ADS; }
        else if (a1 && !strcmp(a1, "stub")) { ctl.voct_src = IN_VOCT_EXTERNAL; }
        outf("vsrc %s\r\n", vsrc_name());
    }
    else if (!strcmp(cmd, "step")) {
        if (a1 && !strcmp(a1, "off")) { s_step_on = false; out_str("step off\r\n"); return; }
        if (!a1 || !a2 || !a3) { out_str("usage: step <a> <b> <ms>\r\n"); return; }
        float a = strtof(a1, nullptr), b = strtof(a2, nullptr);
        long ms = strtol(a3, nullptr, 10);
        if (a < 0 || a > 6000 || b < 0 || b > 6000 || ms < 1 || ms > 10000) {
            out_str("step: a, b in 0..6000, ms in 1..10000\r\n");
            return;
        }
        ctl.voct_src = IN_VOCT_EXTERNAL;
        s_step_a = a; s_step_b = b; s_step_half_us = (uint32_t)ms * 1000u;
        s_step_phase = false;
        gpio_put(CFG_STUB_TRIG_PIN, 0);
        g_shared.voct_cent = a;
        s_step_t0 = time_us_32();
        s_step_on = true;
        outf("step %g <-> %g every %ld ms, trigger GP%d\r\n", a, b, ms, CFG_STUB_TRIG_PIN);
    }
    else if (!strcmp(cmd, "mute")) {
        if (s_mt != MT_IDLE) { out_str("mute: already running\r\n"); return; }
        s_mt_hold_ms = a1 ? (uint32_t)strtoul(a1, nullptr, 10) : 500u;
        g_shared.audio_mute_req = 1;
        s_mt_t0 = time_us_32();
        s_mt = MT_WAIT_ACK;
        outf("mute: req = 1, holding %lu ms after ack\r\n", (unsigned long)s_mt_hold_ms);
    }
    else if (!strcmp(cmd, "default")) {
        s_step_on = false; ctl.voct_src = IN_VOCT_FUSION; ctl.knobs_from_pic = true; defaults();
        out_str("defaults (vsrc fusion, ksrc pic)\r\n");
    }
    else if (!strcmp(cmd, "calmode")) {
        if (a1) s_calmode = !strcmp(a1, "on");
        outf("calmode %s\r\n", s_calmode ? "ON (vcal / czero / calsave enabled)" : "off");
    }
    else if (!strcmp(cmd, "calshow")) cal_show();
    else if (!strcmp(cmd, "caldefault")) {
        CalData d;
        calstore_defaults(&d);
        input_apply_cal(d);
        input_cal_reset();
        out_str("caldefault: theory values applied (not saved)\r\n");
    }
    else if (!strcmp(cmd, "vcal") || !strcmp(cmd, "czero") || !strcmp(cmd, "calsave")) {
        if (!s_calmode) { out_str("calibration mode is off ('calmode on')\r\n"); return; }
        if (!strcmp(cmd, "czero")) {
            s_cz_on = true; s_cz_n = 0;
            for (auto &x : s_cz_sum) x = 0;
            out_str("czero: measuring 1 s - keep all CV inputs unplugged\r\n");
        } else if (!strcmp(cmd, "calsave")) {
            if (s_sv != SV_IDLE || s_mt != MT_IDLE) { out_str("calsave: busy\r\n"); return; }
            g_shared.audio_mute_req = 1;
            s_sv_t0 = millis();
            s_sv = SV_WAIT_ACK;
            out_str("calsave: muting audio...\r\n");
        } else if (a1 && !strcmp(a1, "fit"))   vcal_fit();
        else if (a1 && !strcmp(a1, "clear"))   { s_vp_n = 0; out_str("vcal: points cleared\r\n"); }
        else if (a1 && !strcmp(a1, "list"))    cal_show();
        else if (a1) {
            float mv = strtof(a1, nullptr);
            if (mv < -100 || mv > 5500) { out_str("vcal: <mV> must be -100 .. 5500\r\n"); return; }
            if (!ads_stats().ok) { out_str("vcal: ADS1115 not running\r\n"); return; }
            s_vc_volt = mv / 1000.0f; s_vc_n = 0; s_vc_sum = 0; s_vc_sumsq = 0; s_vc_on = true;
            outf("vcal: measuring %.4f V for 0.5 s - keep the input steady\r\n", s_vc_volt);
        } else out_str("usage: vcal <mV> | vcal fit | vcal clear | vcal list\r\n");
    }
    else if (!strcmp(cmd, "resetlead")) { audio_min_lead_reset = 1; out_str("min_lead reset\r\n"); }
    else if (!strcmp(cmd, "watch"))  { s_watch = !s_watch; outf("watch %s\r\n", s_watch ? "on" : "off"); }
#if CFG_DIAG
    else if (!strcmp(cmd, "fcal")) {
        // Manual coarse scale: coarse reading [V] = g * true [V] + o  ->  "fcal <g> <o_mV>", "fcal off" = theory.
        const float theory = (CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV) / ADS_LSB_V;
        float g = 1.0f, o_mv = 0.0f;
        if (a1 && strcmp(a1, "off")) { g = strtof(a1, nullptr); o_mv = a2 ? strtof(a2, nullptr) : 0.0f; }
        if (!(g > 0.5f && g < 1.5f)) { out_str("fcal: gain must be 0.5 .. 1.5\r\n"); return; }
        float gain = theory / g, ofs = -(o_mv / 1000.0f / g) / ADS_LSB_V;
        fusion_set_cal(gain, ofs);
        ctl.cal_auto = false;
        outf("fcal: coarse = %.4f * V %+.1f mV -> fusion gain %.4f, offset %.1f ADS LSB ('cal auto' resumes)\r\n",
             g, o_mv, gain, ofs);
    }
    else if (!strcmp(cmd, "i2s")) {
        bool on = !(a1 && !strcmp(a1, "off"));
        i2s_enable(on);
        outf("i2s %s\r\n", on ? "on" : "off (audio stops)");
    }
    else if (!strcmp(cmd, "monitor")) {
        s_monitor = !s_monitor;
        s_off_windows = s_off_max_gap = s_off_underruns = 0; s_hist_n = 0;
        s_mon_under_prev = audio_underrun_cnt; s_mon_t = millis();
        outf("monitor %s (unplug USB, wait, plug back, then 'offline')\r\n", s_monitor ? "on" : "off");
    }
    else if (!strcmp(cmd, "offline")) {
        outf("unplugged: %lu windows (%.1f s), max_gap=%lu us, underruns=%lu\r\n",
             (unsigned long)s_off_windows, s_off_windows * MON_WINDOW_MS / 1000.0f,
             (unsigned long)s_off_max_gap, (unsigned long)s_off_underruns);
    }
#if CFG_VIN_DIAG
    else if (!strcmp(cmd, "noise"))  noise_report();
#endif
#endif
    else if (!strcmp(cmd, "tune"))   tune_cmd(a1, a2);
    else if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) help();
    else outf("unknown command '%s' (help)\r\n", cmd);
}

// At most ONE command per pass: commands are slow (report formatting ≈ 1–2 ms), and two in one pass
// once stalled core1 for 2.8 ms, longer than the coarse-ADC ring. Unread bytes wait in s_rx.
static char     s_line[64];
static uint32_t s_line_len = 0;
static char     s_rx[64];
static uint32_t s_rx_len = 0, s_rx_pos = 0;

static bool read_commands() {
    if (s_rx_pos >= s_rx_len) {
        s_rx_len = s_rx_pos = 0;
        if (usb_try_lock()) {
            if (tud_cdc_available()) s_rx_len = tud_cdc_read(s_rx, sizeof s_rx);
            usb_unlock();
        }
    }
    while (s_rx_pos < s_rx_len) {
        char c = s_rx[s_rx_pos++];
        if (c == '\r' || c == '\n') {
            if (s_line_len) {
                s_line[s_line_len] = 0;
                command(s_line);
                s_line_len = 0;
                return true;
            }
        } else if (s_line_len < sizeof s_line - 1) {
            s_line[s_line_len++] = c;
        }
    }
    return false;
}

// ---------------------------------------------------------------- entry points --
void console_setup() {
    gpio_init(CFG_STUB_TRIG_PIN);
    gpio_set_dir(CFG_STUB_TRIG_PIN, GPIO_OUT);
    gpio_put(CFG_STUB_TRIG_PIN, 0);
    input_set_ads_observer(ads_observer);
    input_set_pic_observer(pic_observer);
    Serial.begin(115200);
    if (!ads_stats().ok) out_str("ADS1115: no response / config mismatch -> coarse ADC only\r\n");
}

void console_loop(uint32_t pass_start_us) {
    step_service();                                 // timing-critical test signal: every pass
    // Everything else at most every 250 µs: the USB try-lock and CDC calls cost a few µs per pass,
    // which pushed the core1 pass time from ≤5 µs to 5-10 µs.
    static uint32_t t_svc = 0;
    if (pass_start_us - t_svc < 250u && s_rx_pos >= s_rx_len) {
        timing_add(time_us_32() - pass_start_us, false);
        return;
    }
    t_svc = pass_start_us;
    mute_service();
    calib_service();
    save_service();
    load_service();
#if CFG_DIAG
    monitor_service();
#endif
    bool ran_cmd = read_commands();
    if (s_watch) {
        static uint32_t t = 0;
        if (millis() - t >= 1000) {
            t = millis();
            outf("underrun=%lu min_lead=%lu/%d max_gap=%lu us stalls=%lu\r\n", (unsigned long)audio_underrun_cnt,
                 (unsigned long)audio_min_lead, CFG_OUT_LEAD_FRAMES, (unsigned long)audio_max_gap_us,
                 (unsigned long)audio_stall_cnt);
            audio_min_lead_reset = 1;
        }
    }
    out_flush();
    timing_add(time_us_32() - pass_start_us, ran_cmd);
}
