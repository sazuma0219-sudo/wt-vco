// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Single definition of the wavetable. The oscillator itself is inline in oscillator.h.
// With CFG_TABLES_IN_RAM the table is placed in a RAM section (copied from flash at boot), so core0
// never competes with core1's code for the XIP cache (§1.7: the RAM option).
#include "config.h"
#if CFG_TABLES_IN_RAM && !defined(HOST_TEST)
#include "pico.h"
#define WT_DATA_ATTR __not_in_flash("wavetable")
#endif
#define WAVETABLE_IMPLEMENTATION
#include "oscillator.h"
