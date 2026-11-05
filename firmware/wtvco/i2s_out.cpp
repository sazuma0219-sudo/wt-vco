// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "i2s_out.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "i2s_out.pio.h"

#define RING_BYTES (2 * CFG_OUT_RING_FRAMES * 4)
static_assert((RING_BYTES & (RING_BYTES - 1)) == 0, "ring size must be a power of 2");

static constexpr uint ring_bits(uint bytes) { return bytes <= 1 ? 0 : 1 + ring_bits(bytes >> 1); }
static_assert(ring_bits(RING_BYTES) <= 15, "DMA ring wrap supports up to 2^15 bytes");

// DMA read-side ring wrap only folds the low address bits, so the buffer must be aligned to its size.
alignas(RING_BYTES) volatile uint32_t i2s_out_ring[2 * CFG_OUT_RING_FRAMES];

static I2sOutInfo s_info;
static uintptr_t  s_ring_base;
static volatile const void *s_read_addr_reg;

bool i2s_out_begin() {
    for (uint32_t i = 0; i < 2 * CFG_OUT_RING_FRAMES; i++) i2s_out_ring[i] = 0;
    s_ring_base = (uintptr_t)i2s_out_ring;

    // ---- PIO --------------------------------------------------------------
    PIO pio;
    uint sm, offset;
    if (!pio_claim_free_sm_and_add_program(&i2s_out_program, &pio, &sm, &offset)) return false;

    // 2 instructions per BCK, 64 BCK per frame -> PIO clock = 128 * fS.
    uint32_t sys = clock_get_hz(clk_sys);
    uint64_t den = (uint64_t)CFG_SAMPLE_RATE * 128u;
    uint64_t div256 = ((uint64_t)sys * 256u + den / 2) / den;
    s_info.sys_hz    = sys;
    s_info.div_int   = (uint32_t)(div256 >> 8);
    s_info.div_frac8 = (uint32_t)(div256 & 0xFF);
    s_info.exact     = (div256 * den == (uint64_t)sys * 256u);
    s_info.pio_index = PIO_NUM(pio);
    s_info.sm        = sm;

    pio_sm_config c = i2s_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, I2S_PIN_DATA, 1);
    sm_config_set_sideset_pins(&c, I2S_PIN_LRCK);
    sm_config_set_out_shift(&c, false /* shift left, MSB first */, true /* autopull */, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);       // keep the FIFO shallow (latency)
    sm_config_set_clkdiv_int_frac8(&c, s_info.div_int, (uint8_t)s_info.div_frac8);

    pio_gpio_init(pio, I2S_PIN_BCK);
    pio_gpio_init(pio, I2S_PIN_LRCK);
    pio_gpio_init(pio, I2S_PIN_DATA);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_PIN_LRCK, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_PIN_DATA, 1, true);
    pio_sm_init(pio, sm, offset + i2s_out_offset_entry_point, &c);

    // ---- DMA: endless, read-side ring over the whole buffer, paced by the PIO TX DREQ ----
    int ch = dma_claim_unused_channel(false);
    if (ch < 0) return false;
    s_info.dma_ch = ch;

    dma_channel_config d = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, false);
    channel_config_set_ring(&d, false /* read side */, ring_bits(RING_BYTES));
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, true));
    dma_channel_configure(ch, &d, &pio->txf[sm], (const void *)i2s_out_ring,
                          dma_encode_endless_transfer_count(), true);
    s_read_addr_reg = &dma_hw->ch[ch].read_addr;

    pio_sm_set_enabled(pio, sm, true);
    return true;
}

const I2sOutInfo &i2s_out_info() { return s_info; }

uint32_t i2s_out_read_frame() {
    uint32_t addr = *(volatile const uint32_t *)s_read_addr_reg;
    return ((addr - (uint32_t)s_ring_base) >> 3) & I2S_RING_MASK;
}
