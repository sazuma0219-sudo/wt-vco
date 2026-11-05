// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// I2S output: own PIO program + endless DMA ring (wtvco_audio_process.md §1.11).
//
//   ring (CFG_OUT_RING_FRAMES frames, aligned to its size) --DMA, read-side ring wrap, endless-->
//   PIO TX FIFO (not joined, 4 words = 2 frames) --> PCM5102A (GP14 BCK, GP13 LRCK, GP12 DATA)
//
// The DMA never stops; the writer chases the read position (i2s_out_read_frame()).
#pragma once
#include <stdint.h>
#include "config.h"

// Pins as wired on the board (schematic nets DAC_BCK / DAC_LRCK / DAC_DIN).
#define I2S_PIN_LRCK  13    // side-set base (bit 0)
#define I2S_PIN_BCK   14    // side-set bit 1 (must be LRCK + 1)
#define I2S_PIN_DATA  12
static_assert(I2S_PIN_BCK == I2S_PIN_LRCK + 1, "side-set needs LRCK, BCK on consecutive pins");

#define I2S_RING_MASK (CFG_OUT_RING_FRAMES - 1)

struct I2sOutInfo {
    uint32_t sys_hz;        // clk_sys at init
    uint32_t div_int;       // PIO clock divider = div_int + div_frac8 / 256
    uint32_t div_frac8;
    bool     exact;         // true if the divider gives exactly CFG_SAMPLE_RATE
    uint32_t pio_index;     // 0..2
    uint32_t sm;
    int      dma_ch;
};

// Two 32-bit words per frame: [2n] = L, [2n + 1] = R (left-justified signed samples).
extern volatile uint32_t i2s_out_ring[2 * CFG_OUT_RING_FRAMES];

// Zero the ring, start DMA and PIO. Returns false if no PIO state machine / DMA channel was free.
bool i2s_out_begin();

const I2sOutInfo &i2s_out_info();

// Frame index the DMA reads next (0 .. CFG_OUT_RING_FRAMES - 1).
uint32_t i2s_out_read_frame();

// Write one mono sample (int16 range after gain) to both L and R of a frame.
static inline void i2s_out_write_frame(uint32_t frame, int32_t sample16) {
    uint32_t w = (uint32_t)sample16 << 16;
    i2s_out_ring[2 * frame]     = w;
    i2s_out_ring[2 * frame + 1] = w;
}
