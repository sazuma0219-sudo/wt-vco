// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "vin_coarse.h"

#include <math.h>
#include "config.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "vin_fusion.h"

#define RING_BYTES (VIN_RING_LEN * 2)
static_assert((VIN_RING_LEN & (VIN_RING_LEN - 1)) == 0, "VIN_RING_LEN must be a power of 2");
static constexpr unsigned ring_bits(unsigned b) { return b <= 1 ? 0 : 1 + ring_bits(b >> 1); }

// DMA write-side ring wrap folds only the low address bits: align the buffer to its size (3.1).
alignas(RING_BYTES) static volatile uint16_t s_ring[VIN_RING_LEN];

static int       s_dma_ch = -1;

// ADS1115 window boundaries (4.3): the RDY IRQ pushes the ring index the DMA was about to write, plus
// the RDY number. When processing reaches that index, the samples before it form the closed window.
#define BQ_LEN 8
static volatile uint32_t s_bq_idx[BQ_LEN], s_bq_seq[BQ_LEN];
static volatile uint32_t s_bq_head = 0;      // written by the IRQ
static uint32_t          s_bq_tail = 0;      // read by the loop
static uint32_t  s_rd = 0;            // next ring index to process
static uint32_t  s_prev = 0;          // previous good raw value (ERR replacement)
static uint32_t  s_blk_sum = 0, s_blk_n = 0;
static VinStats  s_st;

// Block statistics accumulators (p).
static uint32_t  b_n = 0, b_min = UINT32_MAX, b_max = 0;
static uint64_t  b_sum = 0, b_sumsq = 0;      // integer: no soft/coprocessor double on the hot path

// ---- Threshold study (5.4, 13.1): emulate 5.3 on a static input ------------------------------
// Windows of VIN_DIAG_WIN_BLOCKS p (≈ one ADS1115 conversion) are closed in software. Each window's
// range (max - min of p) goes into a histogram (T_STILL), and its mean becomes ref. Every raw sample
// then runs the jump detector for several candidate T_JUMP values: |s - ref| > T for JUMP_CONFIRM
// consecutive samples counts as a trigger. On a static input every trigger is a false one.
// Not const: keeps the table in RAM (a const table in flash cost ~2 µs per sample through the XIP cache).
static uint32_t DIAG_T_LSB[VIN_DIAG_NT] = {1, 2, 3, 4, 5, 6, 8, 10};
static uint32_t d_win_n = 0, d_win_min = UINT32_MAX, d_win_max = 0, d_win_sum = 0;
static uint32_t d_ref_sum = 0;            // mean p of the last closed window (16 x LSB units, like s_sum)
static bool     d_have_ref = false;
static uint32_t d_consec[VIN_DIAG_NT];
static VinDiag  d;

static inline uint32_t dma_write_index() {
    uint32_t addr = dma_hw->ch[s_dma_ch].write_addr;
    return ((addr - (uint32_t)(uintptr_t)s_ring) >> 1) & (VIN_RING_LEN - 1);
}

bool vin_coarse_begin() {
    adc_init();
    adc_gpio_init(VIN_PIN);                                        // IE = 0, function NULL, no pulls
    hw_set_bits(&pads_bank0_hw->io[VIN_PIN], PADS_BANK0_GPIO0_OD_BITS);   // OD = 1 (3.1 / 2.6)
    adc_select_input(VIN_ADC_INPUT);
    adc_set_round_robin(0);
    adc_fifo_setup(true, true, 1, true /* ERR bit in FIFO */, false);
    adc_set_clkdiv(0);                                             // 96 cycles @ 48 MHz = 2 µs

    s_dma_ch = dma_claim_unused_channel(false);
    if (s_dma_ch < 0) return false;
    dma_channel_config c = dma_channel_get_default_config(s_dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true /* write side */, ring_bits(RING_BYTES));
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure(s_dma_ch, &c, (void *)s_ring, &adc_hw->fifo,
                          dma_encode_endless_transfer_count(), true);
    adc_run(true);

    // Prime the moving sum with the first MA_LEN samples.
    while (dma_write_index() < VIN_MA_LEN) { }
    s_st.s_sum = 0;
    for (uint32_t i = 0; i < VIN_MA_LEN; i++) {
        uint32_t v = s_ring[i] & 0x0FFFu;
        s_ring[i] = (uint16_t)v;
        s_st.s_sum += v;
        s_prev = v;
    }
    s_rd = VIN_MA_LEN;
    return true;
}

void RAM_FUNC(vin_coarse_poll)() {
    // The position difference alone cannot see a lag of more than one ring (it wraps), so the time
    // since the previous snapshot is checked as well. A missed wrap once delayed every later window
    // close by up to a full ring, and every ADS1115 result then arrived before its window closed.
    static uint32_t t_prev = 0;
    uint32_t now = time_us_32();
    uint32_t wr = dma_write_index();
    uint32_t n = (wr - s_rd) & (VIN_RING_LEN - 1);
    bool lagged = t_prev && (now - t_prev) > (VIN_RING_LEN - 4 * VIN_MA_LEN) * 2u;   // 2 µs per sample
    t_prev = now;
    if (lagged || n > VIN_RING_LEN - 4 * VIN_MA_LEN) {
        // Fell (almost) a full ring behind: the old samples needed by the moving sum are being
        // overwritten. Rebuild the sum from the newest MA_LEN samples.
        s_st.overrun_cnt++;
        fusion_mark_discard();                     // window statistics are broken (11.2)
        s_bq_tail = s_bq_head;                     // queued boundaries may lie in the skipped part
        s_st.s_sum = 0;
        for (uint32_t i = 0; i < VIN_MA_LEN; i++) {
            uint32_t v = s_ring[(wr - VIN_MA_LEN + i) & (VIN_RING_LEN - 1)] & 0x0FFFu;
            s_ring[(wr - VIN_MA_LEN + i) & (VIN_RING_LEN - 1)] = (uint16_t)v;
            s_st.s_sum += v;
            s_prev = v;
        }
        s_rd = wr;
        s_blk_sum = 0; s_blk_n = 0;
        return;
    }
    while (s_rd != wr) {
        if (s_bq_tail != s_bq_head && s_rd == s_bq_idx[s_bq_tail % BQ_LEN]) {
            fusion_window_close(s_bq_seq[s_bq_tail % BQ_LEN]);
            s_bq_tail++;
        }
        uint32_t raw = s_ring[s_rd];
        if (raw & 0x8000u) {                       // conversion error: keep the window intact
            raw = s_prev;
            s_st.err_cnt++;
            fusion_mark_discard();
        } else {
            raw &= 0x0FFFu;
        }
        s_ring[s_rd] = (uint16_t)raw;              // the moving sum subtracts it again 16 samples later
        uint32_t old = s_ring[(s_rd - VIN_MA_LEN) & (VIN_RING_LEN - 1)];
        s_st.s_sum = s_st.s_sum + raw - old;
        s_prev = raw;
        s_st.last_raw = raw;
        s_st.samples++;
        fusion_sample(s_st.s_sum);

        if (CFG_VIN_DIAG && d_have_ref) {          // jump detector emulation, every raw sample
            uint32_t s = s_st.s_sum;
            uint32_t dev = s > d_ref_sum ? s - d_ref_sum : d_ref_sum - s;
            for (int k = 0; k < VIN_DIAG_NT; k++) {
                if (dev > DIAG_T_LSB[k] * VIN_MA_LEN) {
                    if (++d_consec[k] >= VIN_DIAG_JUMP_CONFIRM) { d.triggers[k]++; d_consec[k] = 0; }
                } else {
                    d_consec[k] = 0;
                }
            }
        }

        s_blk_sum += raw;
        if (++s_blk_n == VIN_MA_LEN) {             // coarse sample p
            s_st.p_sum = s_blk_sum;
            fusion_block(s_blk_sum);
            b_n++;
            if (s_blk_sum < b_min) b_min = s_blk_sum;
            if (s_blk_sum > b_max) b_max = s_blk_sum;
            b_sum += s_blk_sum;
            b_sumsq += (uint64_t)s_blk_sum * s_blk_sum;

            if (CFG_VIN_DIAG) {
            if (s_blk_sum < d_win_min) d_win_min = s_blk_sum;
            if (s_blk_sum > d_win_max) d_win_max = s_blk_sum;
            d_win_sum += s_blk_sum;
            }
            if (CFG_VIN_DIAG && ++d_win_n == VIN_DIAG_WIN_BLOCKS) {  // window closed
                uint32_t range = d_win_max - d_win_min;          // 16 x LSB
                uint32_t bin = range / (VIN_MA_LEN / 2);         // 0.5 LSB bins
                if (bin >= VIN_DIAG_RANGE_BINS) bin = VIN_DIAG_RANGE_BINS - 1;
                d.range_hist[bin]++;
                if (range > d.range_max_sum) d.range_max_sum = range;
                d.windows++;
                d_ref_sum = d_win_sum / VIN_DIAG_WIN_BLOCKS;
                d_have_ref = true;
                d_win_n = 0; d_win_min = UINT32_MAX; d_win_max = 0; d_win_sum = 0;
            }
            s_blk_sum = 0; s_blk_n = 0;
        }
        s_rd = (s_rd + 1) & (VIN_RING_LEN - 1);
    }
}

const VinStats &vin_coarse_stats() { return s_st; }

VinBlockStats vin_coarse_take_block_stats() {
    VinBlockStats r;
    r.n = b_n;
    r.min_sum = b_n ? b_min : 0;
    r.max_sum = b_n ? b_max : 0;
    r.mean_sum = b_n ? (double)b_sum / b_n : 0;
    double var = b_n ? (double)b_sumsq / b_n - r.mean_sum * r.mean_sum : 0;
    r.sd_sum = var > 0 ? sqrt(var) : 0;
    b_n = 0; b_min = UINT32_MAX; b_max = 0; b_sum = 0; b_sumsq = 0;
    return r;
}

uint32_t RAM_FUNC(vin_coarse_write_index)() { return dma_write_index(); }

VinBqDebug vin_coarse_bq_debug() {
    VinBqDebug d;
    d.head = s_bq_head; d.tail = s_bq_tail; d.rd = s_rd;
    d.next_idx = s_bq_idx[s_bq_tail % BQ_LEN]; d.next_seq = s_bq_seq[s_bq_tail % BQ_LEN];
    return d;
}

void RAM_FUNC(vin_coarse_mark_boundary)(uint32_t seq) {
    uint32_t h = s_bq_head;
    if (h - s_bq_tail >= BQ_LEN) return;           // loop far behind: the overrun path will resync
    s_bq_idx[h % BQ_LEN] = dma_write_index();
    s_bq_seq[h % BQ_LEN] = seq;
    __dmb();
    s_bq_head = h + 1;
}

VinDiag vin_coarse_take_diag() {
    VinDiag r = d;
    for (int k = 0; k < VIN_DIAG_NT; k++) r.t_lsb[k] = DIAG_T_LSB[k];
    d = VinDiag{};
    return r;
}

float vin_coarse_cent() {
    // volt at the jack = (s/16) * VREF / 4096 / DIV ; cent = volt * 1200
    const float k = CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV * 1200.0f / VIN_MA_LEN;
    return (float)s_st.s_sum * k;
}
