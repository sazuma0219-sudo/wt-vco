// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
// RP2350 Wavetable VCO 1ch (wtvco_audio_process.md, wtvco_input_spec.md)
//   core0: audio (setup / loop) — nothing else runs here, no Serial, no delay()
//   core1: input section (input.*) + USB console (console.*), setup1 / loop1
// The audio side only reads the shared variables (shared_params.h); it does not know the input side.
// I2S pins: GP14 BCK, GP13 LRCK, GP12 DATA (as wired; see i2s_out.h).
#include "config.h"
#include "audio.h"
#include "console.h"
#include "input.h"
#include "hardware/timer.h"

static volatile bool g_audio_ok = false;

// arduino-pico installs a SysTick interrupt on core1 (every 2^24 cycles, only for rp2040.getCycleCount()).
// core1 has a 10 µs loop budget (wtvco_input_spec.md 10.1), so keep it free of interrupts we do not use.
bool core1_disable_systick = true;

void setup() {
    g_audio_ok = audio_begin();
}

void loop() {
    if (g_audio_ok) audio_run();   // never returns
}

// ---------------------------------------------------------------- core1 --
void setup1() {
    input_setup();
    console_setup();
}

void loop1() {
    uint32_t t0 = time_us_32();
    input_loop();
    console_loop(t0);
}
