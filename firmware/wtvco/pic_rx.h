// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// PIC frame receiver (wtvco_input_spec.md 7.3, 8.1). UART0 RX on GP1, 1 Mbps 8N1, polled from the
// core1 loop straight out of the hardware FIFO (no interrupt, no Serial1 object). GP0 (UART0 TX) is
// not used and stays a plain GPIO.
#pragma once
#include <stdint.h>

#define PIC_PIN_RX      1
#define PIC_BAUD        1000000
#define PIC_N_VALUES    8

// Value order in the frame (7.3)
enum { PIC_WT_UNI, PIC_WT_BI, PIC_FM_UNI, PIC_FM_BI, PIC_COARSE, PIC_FINE, PIC_KNOB, PIC_DEPTH };

struct PicStats {
    uint32_t frames;          // good frames
    uint32_t bytes;
    uint32_t checksum_err;    // complete frame, wrong checksum
    uint32_t sync_err;        // header byte (bit7 = 1) in the middle of a frame
    uint32_t framing_err;     // UART framing errors (stop bit)
    uint32_t overrun_err;     // UART FIFO overruns
    uint32_t break_err;
    uint32_t seq_gaps;        // frames missing according to the sequence number
    uint32_t last_frame_ms;   // millis() of the last good frame
};

void pic_rx_begin();
void pic_rx_poll();                         // core1 loop: drain the FIFO, assemble frames
bool pic_rx_take(uint16_t v[PIC_N_VALUES]);  // true once per new good frame (raw 0 .. 16380)
const PicStats &pic_rx_stats();
const uint16_t *pic_rx_last();              // latest good values (raw)
