#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Shota Azuma
# Build with XC8. Usage: build.sh [test]   (test = TEST_FIXED link-test build)
# Toolchain paths default to a Windows install of XC8 v3.10 + MPLAB X v6.35 (Git Bash);
# override them with the XC8 / DFP environment variables for other versions or OSes.
set -e
cd "$(dirname "$0")"
XC8="${XC8:-/c/Program Files/Microchip/xc8/v3.10/bin/xc8-cc.exe}"
DFP="${DFP:-/c/Program Files/Microchip/MPLABX/v6.35/packs/Microchip/PIC16F1xxxx_DFP/1.31.465}"
mkdir -p build
if [ "$1" = "test" ]; then
    "$XC8" -mcpu=16F18426 -mdfp="$DFP/xc8" -O2 -Wall -DTEST_FIXED=1 -o build/pic_wtvco_test.hex main.c
else
    "$XC8" -mcpu=16F18426 -mdfp="$DFP/xc8" -O2 -Wall -o build/pic_wtvco.hex main.c
fi
