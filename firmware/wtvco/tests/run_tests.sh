#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Shota Azuma
# Build and run the host-side tests (needs g++ on PATH).
set -e
cd "$(dirname "$0")"
mkdir -p build
CXX="g++ -std=c++17 -O2 -Wall -Wextra -DHOST_TEST"
$CXX -o build/test_pitch      test_pitch.cpp      ../pitch.cpp ../oscillator.cpp -lm
$CXX -o build/test_oscillator test_oscillator.cpp ../pitch.cpp ../oscillator.cpp -lm
$CXX -o build/test_control    test_control.cpp    ../pitch.cpp ../oscillator.cpp ../control.cpp ../shared_params.cpp -lm
$CXX -o build/test_fusion     test_fusion.cpp     ../vin_fusion.cpp -lm
$CXX -o build/test_calib      test_calib.cpp      ../vin_calib.cpp -lm
$CXX -o build/test_pic_input  test_pic_input.cpp  ../pic_input.cpp -lm
echo "=== test_pitch ===";      ./build/test_pitch
echo "=== test_oscillator ==="; ./build/test_oscillator
echo "=== test_control ===";    ./build/test_control
echo "=== test_fusion ===";     ./build/test_fusion
echo "=== test_calib ===";      ./build/test_calib
echo "=== test_pic_input ===";  ./build/test_pic_input
