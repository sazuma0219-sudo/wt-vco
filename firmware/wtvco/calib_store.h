// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Calibration values in flash (wtvco_input_spec.md 9.5), via arduino-pico's EEPROM emulation
// (one 4 KB flash sector). Writing stalls both cores for tens of ms: only call calstore_write()
// after the audio side has acknowledged a mute request (audio_mute_req / audio_mute_ack).
#pragma once
#include <stdint.h>

struct CalData {
    uint32_t magic;
    uint32_t version;
    // ADS1115 absolute calibration (9.2): volts = ads_a * code + ads_b
    float    ads_a, ads_b;
    // Coarse ADC -> ADS1115 relative calibration (9.3): ads = rel_gain * coarse_avg + rel_offset
    float    rel_gain, rel_offset;
    // CV inputs (9.4), order WT_UNI, WT_BI, FM_UNI, FM_BI: volts = (raw - zero) * gain
    float    cv_zero[4], cv_gain[4];
    uint32_t checksum;
};

void calstore_defaults(CalData *d);             // theory values (5.6, 8.4)
bool calstore_read(CalData *d);                 // false (and defaults) if missing or corrupt
bool calstore_write(const CalData &d);          // blocking flash write; see the note above
