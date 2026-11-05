// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// USB CDC console on core1: monitoring, test overrides and (later) calibration commands.
// Never blocks core1: output is queued, USB is only touched with a try-lock, one command per pass.
// Type "help" for the command list.
#pragma once
#include <stdint.h>

void console_setup();                           // after input_setup()
void console_loop(uint32_t pass_start_us);      // after input_loop(); pass_start_us = time at the pass start
