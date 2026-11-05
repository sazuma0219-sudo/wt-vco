// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// ADS1115 on I2C0 (V/OCT precise reading, wtvco_input_spec.md 2.3, 4).
//   - continuous conversion, AIN1 single-ended, ±6.144 V, 860 SPS
//   - ALERT/RDY falling edge on GP10 -> core1 GPIO IRQ records the coarse-ADC ring position (window boundary)
//   - conversion results are read WITHOUT blocking: two read commands are queued in the I2C TX FIFO and the
//     bytes are collected on a later loop pass (4.4). Wire is not used.
// Pins as wired: SDA = GP8, SCL = GP9, RDY = GP10.
#pragma once
#include <stdint.h>

#define ADS_PIN_SDA   8
#define ADS_PIN_SCL   9
#define ADS_PIN_RDY   10
#define ADS_ADDR      0x48
#define ADS_I2C_HZ    400000
#define ADS_CONFIG    0x50E0     // OS=0 MUX=101(AIN1) PGA=000(±6.144V) MODE=0 DR=111(860SPS) COMP_QUE=00 (4.1)
#define ADS_LSB_V     0.0001875f // ±6.144 V / 32768

struct AdsStats {
    bool     ok;              // init succeeded (config read back)
    uint32_t rdy_cnt;         // RDY edges
    uint32_t result_cnt;      // results collected
    uint32_t late_cnt;        // a new RDY arrived while the previous read was still pending (discarded)
    uint32_t abort_cnt;       // TX_ABRT (NACK etc.)
    int16_t  last_code;       // latest conversion result
    uint32_t last_rdy_ring;   // coarse-ADC ring write index at the latest RDY (window boundary, 4.3)
    uint32_t rdy_min_us, rdy_max_us;   // RDY period range since the last stats reset
    uint32_t read_max_us;     // RDY -> result latency max since the last reset (≈70 µs expected)
    uint32_t rdy_timeout_cnt; // RDY stopped for 50 ms (11.2)
    uint32_t reinit_cnt;      // reinitialisations started
    uint32_t reinit_fail_cnt; // ... that failed (NACK / bus stuck): degraded, retried every second
    uint32_t bus_release_cnt; // SDA was stuck low: 9 SCL clocks + STOP
    uint32_t skipped_cnt;     // results dropped right after a (re)initialisation
};

bool ads_begin();             // core1. Blocking setup (allowed at start-up, 11.1)
void ads_start_irq();         // core1, after ads_begin (whatever it returned): RDY IRQ + fault handling
void ads_debug_stop();        // diagnostic: put the ADS1115 to single-shot so RDY stops
void ads_poll();              // core1 loop: start a read after RDY, collect it when the bytes are there
bool ads_take_result(int16_t *code, uint32_t *seq);   // true once per new result; seq = its RDY number
const AdsStats &ads_stats();
void ads_reset_minmax();
