// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Shota Azuma
#include "shared_params.h"

// Static initial values = the input side's defaults before the first PIC frame (wtvco_input_spec.md 8.2),
// so core0 has sane values even if it starts before core1 writes anything.
SharedParams g_shared = {
    /* voct_cent      */ 0.0f,
    /* coarse_cent    */ 3600.0f,   // C3
    /* fine_cent      */ 0.0f,
    /* wt_cv          */ 0.0f,
    /* wt_knob        */ 0.0f,
    /* fm_cent        */ 0.0f,
    /* status         */ 0,
    /* audio_mute_req */ 0,
    /* audio_mute_ack */ 0,
};
