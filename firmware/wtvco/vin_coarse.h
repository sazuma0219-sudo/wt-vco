// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// V/OCT coarse reading: RP2350 ADC (GP27 = ADC1) at 500 kSPS into an endless DMA ring (wtvco_input_spec.md 3).
// core1 chases the DMA write position and builds
//   s = moving average of the last 16 raw samples (updated every raw sample, 2 µs)
//   p = block average of 16 raw samples (every 32 µs)
// Both are kept as SUMS of 16 samples (16 x 12-bit LSB) to stay integer.
#pragma once
#include <stdint.h>

#define VIN_PIN          27                 // ADC1 (as wired; net V/OCT_COARSE)
#define VIN_ADC_INPUT    (VIN_PIN - 26)
#define VIN_RING_LEN     1024               // 16-bit samples, ≈ 2 ms. Power of 2
#define VIN_MA_LEN       16                 // MA_LEN (4.4 / 5.4)

struct VinStats {
    uint32_t samples;       // raw samples processed (since boot)
    uint32_t err_cnt;       // samples with the FIFO ERR bit (replaced by the previous value)
    uint32_t overrun_cnt;   // core1 fell more than a ring behind (resync)
    uint32_t last_raw;      // 12-bit
    uint32_t s_sum;         // current moving sum (16 x LSB)
    uint32_t p_sum;         // latest block sum (16 x LSB)
};

bool vin_coarse_begin();         // core1. ADC + DMA, free running from here on
uint32_t vin_coarse_write_index();   // DMA write position in the ring
void vin_coarse_mark_boundary(uint32_t seq);
struct VinBqDebug { uint32_t head, tail, next_idx, next_seq, rd; };
VinBqDebug vin_coarse_bq_debug();   // ADS1115 RDY IRQ: window boundary at the current DMA position
void vin_coarse_poll();          // core1 loop: process everything the DMA wrote since the last call
const VinStats &vin_coarse_stats();

// Block statistics (p) since the last call, then reset: for the noise measurement (T_STILL / T_JUMP, 5.4, 13.1).
struct VinBlockStats { uint32_t n; uint32_t min_sum, max_sum; double mean_sum, sd_sum; };
VinBlockStats vin_coarse_take_block_stats();

// Threshold study (5.4, 13.1): software windows + jump detector emulation on the live input.
#define VIN_DIAG_WIN_BLOCKS     36      // p per window ≈ 1.15 ms ≈ one ADS1115 conversion (5.3)
#define VIN_DIAG_JUMP_CONFIRM   17      // JUMP_CONFIRM = MA_LEN + 1 (5.4)
#define VIN_DIAG_NT             8       // candidate T_JUMP values
#define VIN_DIAG_RANGE_BINS     64      // window range histogram, 0.5 LSB bins (last = overflow)
struct VinDiag {
    uint32_t windows;
    uint32_t range_max_sum;                 // largest window range, 16 x LSB
    uint32_t range_hist[VIN_DIAG_RANGE_BINS];
    uint32_t t_lsb[VIN_DIAG_NT];            // candidate T_JUMP in LSB (of the 16-average)
    uint32_t triggers[VIN_DIAG_NT];         // detector triggers per candidate
};
VinDiag vin_coarse_take_diag();             // returns and resets

// Moving average s -> cent, theoretical scale (ADC_AVDD 3.3 V, divider 0.6); calibration comes later (9.3).
float vin_coarse_cent();
