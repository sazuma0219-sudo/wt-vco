// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "pic_rx.h"

#include <Arduino.h>
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#define FRAME_LEN 18

static uart_inst_t *const U = uart0;         // GP1 = UART0 RX
static PicStats s_st;
static uint8_t  s_buf[FRAME_LEN];
static uint32_t s_len = 0;                   // bytes collected; 0 = waiting for a header
static uint16_t s_val[PIC_N_VALUES];
static bool     s_new = false;
static int      s_last_seq = -1;

void pic_rx_begin() {
    uart_init(U, PIC_BAUD);                   // 8N1 by default
    uart_set_fifo_enabled(U, true);
    gpio_set_function(PIC_PIN_RX, GPIO_FUNC_UART);
    gpio_pull_up(PIC_PIN_RX);                 // idle high without a PIC (2.5)
}

static void frame_done() {
    uint8_t sum = 0;
    for (int i = 1; i <= 16; i++) sum += s_buf[i];
    if ((sum & 0x7F) != s_buf[17]) { s_st.checksum_err++; return; }
    for (int i = 0; i < PIC_N_VALUES; i++) s_val[i] = (uint16_t)(s_buf[1 + 2 * i] << 7 | s_buf[2 + 2 * i]);
    int seq = s_buf[0] & 0x7F;
    if (s_last_seq >= 0) s_st.seq_gaps += (uint32_t)((seq - s_last_seq - 1) & 0x7F);
    s_last_seq = seq;
    s_st.frames++;
    s_st.last_frame_ms = millis();
    s_new = true;
}

void RAM_FUNC(pic_rx_poll)() {
    uart_hw_t *hw = uart_get_hw(U);
    while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
        uint32_t dr = hw->dr;                 // data + error flags of this byte
        s_st.bytes++;
        if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_OE_BITS)) {
            if (dr & UART_UARTDR_FE_BITS) s_st.framing_err++;
            if (dr & UART_UARTDR_BE_BITS) s_st.break_err++;
            if (dr & UART_UARTDR_OE_BITS) s_st.overrun_err++;
            s_len = 0;                        // drop the frame in progress (8.1)
            continue;
        }
        uint8_t b = (uint8_t)dr;
        if (b & 0x80) {                       // header
            if (s_len) s_st.sync_err++;       // previous frame was incomplete
            s_buf[0] = b;
            s_len = 1;
        } else if (s_len) {
            s_buf[s_len++] = b;
            if (s_len == FRAME_LEN) { frame_done(); s_len = 0; }
        }
    }
}

bool pic_rx_take(uint16_t v[PIC_N_VALUES]) {
    if (!s_new) return false;
    s_new = false;
    for (int i = 0; i < PIC_N_VALUES; i++) v[i] = s_val[i];
    return true;
}

const PicStats &pic_rx_stats() { return s_st; }
const uint16_t *pic_rx_last() { return s_val; }
