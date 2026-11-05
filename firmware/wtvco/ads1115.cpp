// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "ads1115.h"

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "vin_coarse.h"

static i2c_inst_t *const I2C = i2c0;
static AdsStats s_st;

// Written by the RDY IRQ, consumed by ads_poll().
static volatile uint32_t s_rdy_seq = 0;      // incremented per RDY edge
static volatile uint32_t s_rdy_time = 0;     // time_us_32() at the latest edge
static uint32_t s_rdy_seen = 0;              // last sequence handled by ads_poll
static uint32_t s_prev_rdy_time = 0;

// Reads queued in the controller (2 command words each). Bytes always come back as whole pairs in
// order, so a late result is dropped by discarding its pair, never by draining the RX FIFO halfway
// (that misaligned the bytes: 3.00 V read as -5.60 V).
static uint32_t s_outstanding = 0;
static uint32_t s_q_seq[4];                   // RDY number of each queued read, oldest first
static uint32_t s_result_seq = 0;
static uint32_t s_pending_t0 = 0;             // RDY time of the newest queued read
#define ADS_MAX_OUTSTANDING 2
static bool     s_have_result = false;
static uint32_t s_skip = 0;                   // results to drop after (re)initialisation
static int16_t  s_result = 0;

static void RAM_FUNC(rdy_irq)() {
    if (gpio_get_irq_event_mask(ADS_PIN_RDY) & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(ADS_PIN_RDY, GPIO_IRQ_EDGE_FALL);
        s_rdy_time = time_us_32();
        uint32_t seq = s_rdy_seq + 1;
        s_st.last_rdy_ring = vin_coarse_write_index();
        vin_coarse_mark_boundary(seq);                   // window boundary on the coarse sample stream
        s_rdy_seq = seq;
    }
}

static bool write_reg(uint8_t reg, uint16_t v) {
    uint8_t b[3] = {reg, (uint8_t)(v >> 8), (uint8_t)v};
    return i2c_write_timeout_us(I2C, ADS_ADDR, b, 3, false, 5000) == 3;
}

static bool read_reg(uint8_t reg, uint16_t *v) {
    uint8_t b[2];
    if (i2c_write_timeout_us(I2C, ADS_ADDR, &reg, 1, true, 5000) != 1) return false;
    if (i2c_read_timeout_us(I2C, ADS_ADDR, b, 2, false, 5000) != 2) return false;
    *v = (uint16_t)(b[0] << 8 | b[1]);
    return true;
}

bool ads_begin() {
    i2c_init(I2C, ADS_I2C_HZ);
    gpio_set_function(ADS_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(ADS_PIN_SCL, GPIO_FUNC_I2C);
    gpio_disable_pulls(ADS_PIN_SDA);              // external 4.7 kΩ to 5 V (2.4)
    gpio_disable_pulls(ADS_PIN_SCL);

    gpio_init(ADS_PIN_RDY);
    gpio_set_dir(ADS_PIN_RDY, GPIO_IN);
    gpio_disable_pulls(ADS_PIN_RDY);              // external 10 kΩ to 3.3 V

    uint16_t cfg = 0;
    s_st.ok = write_reg(0x03, 0x8000)              // Hi_thresh MSB = 1  } RDY mode (4.1)
           && write_reg(0x02, 0x0000)              // Lo_thresh MSB = 0  }
           && write_reg(0x01, ADS_CONFIG)
           && read_reg(0x01, &cfg)                 // pointer is left at Config here
           && ((cfg & 0x7FFF) == (ADS_CONFIG & 0x7FFF));   // OS (bit 15) excluded (11.1 step 5)
    if (!s_st.ok) return false;
    // Point back at the Conversion register: every later read is just address + 2 bytes.
    uint8_t p = 0x00;
    if (i2c_write_timeout_us(I2C, ADS_ADDR, &p, 1, false, 5000) != 1) { s_st.ok = false; return false; }

    return true;
}

// ---- Fault handling (11.2), all non-blocking except the 9-clock bus release (~100 µs) -------------
#define ADS_RDY_TIMEOUT_US   50000u     // no RDY for 50 ms -> reinitialise
#define ADS_RETRY_US         1000000u   // while degraded: one attempt per second
#define ADS_REINIT_WAIT_US   3000u      // queued writes must finish within this

enum ReinitState { RI_IDLE, RI_WAIT };
static ReinitState s_ri = RI_IDLE;
static uint32_t    s_ri_t0 = 0;
static uint32_t    s_next_retry = 0;
static uint32_t    s_last_rdy_seen_us = 0;  // time of the last RDY handled (watchdog)

static void set_target() {
    i2c_hw_t *hw = I2C->hw;
    hw->enable = 0;
    hw->tar = ADS_ADDR;
    hw->enable = 1;
}

// SDA held low by a slave stuck mid-byte: clock SCL 9 times, then a STOP, then restart the controller.
static void bus_release() {
    gpio_set_function(ADS_PIN_SCL, GPIO_FUNC_SIO);
    gpio_set_function(ADS_PIN_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(ADS_PIN_SDA, GPIO_IN);       // released = pulled up externally (open drain style)
    gpio_put(ADS_PIN_SCL, 0);
    gpio_put(ADS_PIN_SDA, 0);
    for (int i = 0; i < 9 && !gpio_get(ADS_PIN_SDA); i++) {
        gpio_set_dir(ADS_PIN_SCL, GPIO_OUT);  busy_wait_us_32(5);   // SCL low
        gpio_set_dir(ADS_PIN_SCL, GPIO_IN);   busy_wait_us_32(5);   // SCL released (high)
    }
    gpio_set_dir(ADS_PIN_SDA, GPIO_OUT);  busy_wait_us_32(5);       // STOP: SDA low -> high, SCL high
    gpio_set_dir(ADS_PIN_SCL, GPIO_IN);   busy_wait_us_32(5);
    gpio_set_dir(ADS_PIN_SDA, GPIO_IN);   busy_wait_us_32(5);
    i2c_init(I2C, ADS_I2C_HZ);
    gpio_set_function(ADS_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(ADS_PIN_SCL, GPIO_FUNC_I2C);
    s_st.bus_release_cnt++;
}

// Queue Hi_thresh, Lo_thresh, Config and the pointer reset as four write transactions (4.1).
static void reinit_start(uint32_t now) {
    i2c_hw_t *hw = I2C->hw;
    s_outstanding = 0;
    s_st.ok = false;                          // no results until the device answers again
    s_st.reinit_cnt++;
    if (!gpio_get(ADS_PIN_SDA)) bus_release();
    (void)hw->clr_tx_abrt;
    set_target();
    while (hw->rxflr) (void)hw->data_cmd;
    const uint8_t seqs[4][3] = {{0x03, 0x80, 0x00}, {0x02, 0x00, 0x00},
                                {0x01, (uint8_t)(ADS_CONFIG >> 8), (uint8_t)ADS_CONFIG}, {0x00, 0, 0}};
    for (int t = 0; t < 4; t++) {
        int n = t == 3 ? 1 : 3;
        for (int i = 0; i < n; i++)
            hw->data_cmd = seqs[t][i] | (i == n - 1 ? I2C_IC_DATA_CMD_STOP_BITS : 0);
    }
    s_ri = RI_WAIT;
    s_ri_t0 = now;
}

static void reinit_fail(uint32_t now) {
    s_st.ok = false;                          // degraded: coarse ADC only (11.2)
    s_st.reinit_fail_cnt++;
    s_ri = RI_IDLE;
    s_next_retry = now + ADS_RETRY_US;
}

// Returns true while a reinitialisation is in progress (normal processing is skipped).
static bool fault_service(uint32_t now) {
    i2c_hw_t *hw = I2C->hw;
    if (s_ri == RI_WAIT) {
        if (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) {   // NACK: device absent / not ready
            (void)hw->clr_tx_abrt;
            reinit_fail(now);
        } else if ((hw->status & I2C_IC_STATUS_TFE_BITS) && !(hw->status & I2C_IC_STATUS_ACTIVITY_BITS)) {
            s_ri = RI_IDLE;                   // all writes acknowledged: conversions restart
            s_st.ok = true;
            s_skip = 2;                       // the first conversions after a restart are not trusted
            s_last_rdy_seen_us = now;         // give it a full timeout to deliver the first RDY
            s_rdy_seen = s_rdy_seq;
        } else if (now - s_ri_t0 > ADS_REINIT_WAIT_US) {
            reinit_fail(now);                 // bus stuck; the next attempt starts with a bus release
        }
        return true;
    }
    if (!s_st.ok) {
        if ((int32_t)(now - s_next_retry) >= 0) reinit_start(now);
        return true;
    }
    if (now - s_last_rdy_seen_us > ADS_RDY_TIMEOUT_US) {   // RDY stopped (11.2)
        s_st.rdy_timeout_cnt++;
        reinit_start(now);
        return true;
    }
    return false;
}

void ads_start_irq() {
    // RDY interrupt on core1 (this function runs on core1; the enable is per core). Installed even if
    // the first init failed, so a later reinitialisation needs no IRQ setup.
    s_st.rdy_min_us = UINT32_MAX;
    s_last_rdy_seen_us = time_us_32();
    s_next_retry = time_us_32() + ADS_RETRY_US;
    gpio_add_raw_irq_handler(ADS_PIN_RDY, rdy_irq);
    gpio_set_irq_enabled(ADS_PIN_RDY, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void ads_debug_stop() {
    // Diagnostic: switch the ADS1115 to single-shot (power-down) so RDY stops, to exercise the watchdog.
    uint8_t b[3] = {0x01, (uint8_t)((ADS_CONFIG | 0x0100) >> 8), (uint8_t)(ADS_CONFIG | 0x0100)};
    i2c_hw_t *hw = I2C->hw;
    s_outstanding = 0;
    for (int i = 0; i < 3; i++) hw->data_cmd = b[i] | (i == 2 ? I2C_IC_DATA_CMD_STOP_BITS : 0);
    hw->data_cmd = 0x00 | I2C_IC_DATA_CMD_STOP_BITS;   // pointer back to Conversion
}

void RAM_FUNC(ads_poll)() {
    uint32_t now = time_us_32();
    if (fault_service(now)) return;
    i2c_hw_t *hw = I2C->hw;

    uint32_t seq = s_rdy_seq;
    if (seq != s_rdy_seen) {
        uint32_t t = s_rdy_time;
        s_st.rdy_cnt += seq - s_rdy_seen;
        if (s_prev_rdy_time && seq - s_rdy_seen == 1) {
            uint32_t per = t - s_prev_rdy_time;
            if (per < s_st.rdy_min_us) s_st.rdy_min_us = per;
            if (per > s_st.rdy_max_us) s_st.rdy_max_us = per;
        }
        s_prev_rdy_time = t;
        s_rdy_seen = seq;
        s_last_rdy_seen_us = now;
        if (s_outstanding < ADS_MAX_OUTSTANDING) {
            // Queue two read commands; the controller sends START + address(R) itself, STOP after byte 2.
            hw->data_cmd = I2C_IC_DATA_CMD_CMD_BITS;
            hw->data_cmd = I2C_IC_DATA_CMD_CMD_BITS | I2C_IC_DATA_CMD_STOP_BITS;
            s_q_seq[s_outstanding] = seq;
            s_outstanding++;
            s_pending_t0 = t;
        } else {
            s_st.late_cnt++;                      // bus still busy with older reads: skip this one
        }
    }

    if (!s_outstanding) return;
    if (hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) {
        // NACK etc.: the controller flushed its TX FIFO. Wait for the bus to go idle (bounded: one
        // transaction is ~70 µs), then drop whatever partial data arrived and start clean (4.4, 11.2).
        (void)hw->clr_tx_abrt;
        uint32_t t0 = time_us_32();
        while ((hw->status & I2C_IC_STATUS_ACTIVITY_BITS) && time_us_32() - t0 < 200) { }
        while (hw->rxflr) (void)hw->data_cmd;
        s_st.abort_cnt++;
        s_outstanding = 0;
        return;
    }
    while (hw->rxflr >= 2 && s_outstanding) {
        uint8_t hi = (uint8_t)hw->data_cmd;
        uint8_t lo = (uint8_t)hw->data_cmd;
        uint32_t rseq = s_q_seq[0];
        for (uint32_t i = 1; i < s_outstanding; i++) s_q_seq[i - 1] = s_q_seq[i];
        s_outstanding--;
        if (s_outstanding) {                      // a newer read is queued: this pair is stale (11.2)
            s_st.late_cnt++;
            continue;
        }
        if (s_skip) { s_skip--; s_st.skipped_cnt++; continue; }
        s_result = (int16_t)(hi << 8 | lo);
        s_result_seq = rseq;
        s_have_result = true;
        s_st.result_cnt++;
        s_st.last_code = s_result;
        uint32_t lat = time_us_32() - s_pending_t0;
        if (lat > s_st.read_max_us) s_st.read_max_us = lat;
    }
}

bool ads_take_result(int16_t *code, uint32_t *seq) {
    if (!s_have_result) return false;
    s_have_result = false;
    *code = s_result;
    *seq = s_result_seq;
    return true;
}

const AdsStats &ads_stats() { return s_st; }

void ads_reset_minmax() {
    s_st.rdy_min_us = UINT32_MAX;
    s_st.rdy_max_us = 0;
    s_st.read_max_us = 0;
}
