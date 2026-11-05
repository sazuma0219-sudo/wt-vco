// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// Single definition of the exp2 LUT. The pitch functions are inline in pitch.h.
// With CFG_TABLES_IN_RAM the LUT lives in RAM (see oscillator.cpp).
#include "config.h"
#if CFG_TABLES_IN_RAM && !defined(HOST_TEST)
#include "pico.h"
#define EXP2_LUT_ATTR __not_in_flash("exp2_lut")
#endif
#define EXP2_LUT_IMPLEMENTATION
#include "pitch.h"
