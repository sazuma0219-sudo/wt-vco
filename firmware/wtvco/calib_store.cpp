// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "calib_store.h"

#include <EEPROM.h>
#include <string.h>
#include "ads1115.h"
#include "config.h"

#define CAL_MAGIC   0x57544356u   // "WTCV"
#define CAL_VERSION 1u

static uint32_t checksum(const CalData &d) {
    // FNV-1a over everything before the checksum field.
    const uint8_t *p = (const uint8_t *)&d;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(CalData, checksum); i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

void calstore_defaults(CalData *d) {
    memset(d, 0, sizeof *d);
    d->magic = CAL_MAGIC;
    d->version = CAL_VERSION;
    d->ads_a = ADS_LSB_V;
    d->ads_b = 0.0f;
    d->rel_gain = (CFG_COARSE_VREF / 4096.0f / CFG_COARSE_DIV) / ADS_LSB_V;
    d->rel_offset = 0.0f;
    const float g = 5.0f / 16380.0f;
    const float z[4] = {0.0f, 8190.0f, 0.0f, 8190.0f};
    for (int i = 0; i < 4; i++) { d->cv_zero[i] = z[i]; d->cv_gain[i] = g; }
    d->checksum = checksum(*d);
}

static bool s_begun = false;
static void begin() {
    if (!s_begun) { EEPROM.begin(sizeof(CalData)); s_begun = true; }
}

bool calstore_read(CalData *d) {
    begin();
    EEPROM.get(0, *d);
    bool ok = d->magic == CAL_MAGIC && d->version == CAL_VERSION && d->checksum == checksum(*d);
    // Sanity on top of the checksum: never start from absurd scales.
    ok = ok && d->ads_a > ADS_LSB_V * 0.9f && d->ads_a < ADS_LSB_V * 1.1f;
    if (!ok) calstore_defaults(d);
    return ok;
}

bool calstore_write(const CalData &src) {
    begin();
    CalData d = src;
    d.magic = CAL_MAGIC;
    d.version = CAL_VERSION;
    d.checksum = checksum(d);
    EEPROM.put(0, d);
    if (!EEPROM.commit()) return false;           // erase + program, other core idled meanwhile
    CalData back;
    EEPROM.get(0, back);
    return memcmp(&back, &d, sizeof d) == 0;
}
