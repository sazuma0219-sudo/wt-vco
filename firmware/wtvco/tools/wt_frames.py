# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Shota Azuma
"""Wavetable frame definitions (wtvco_audio_process.md §1.7).

This is the file to edit when you want different waveforms. gen_wavetable.py only calls
get_frames(), which must return NUM_FRAMES single-cycle waveforms of BASE_LEN samples each
(float, any amplitude; DC and band-limiting are handled by the generator).

Two sources are provided:
  - SOURCE = "morph": harmonic morph through KEYS (default: sine -> triangle -> saw -> square)
  - SOURCE = "wav":   a mono WAV containing NUM_FRAMES consecutive single cycles (Serum-style)
"""
import math
import wave

import numpy as np

NUM_FRAMES = 16
BASE_LEN = 2048                 # longest table length; frames are defined at this resolution
MAX_H = BASE_LEN // 2 - 1       # 1023: highest harmonic representable at BASE_LEN (Nyquist bin excluded)

SOURCE = "morph"
WAV_PATH = "frames.wav"         # used when SOURCE == "wav" (relative to this file)


# ---------------------------------------------------------------------------
# Harmonic spectra (sine coefficients b_h, waveform = sum b_h * sin(2*pi*h*t)).
# Index 0 is unused (DC). All keys share sine phase, so morphing between them is smooth.
# ---------------------------------------------------------------------------
def spec_sine():
    b = np.zeros(MAX_H + 1)
    b[1] = 1.0
    return b


def spec_triangle():
    b = np.zeros(MAX_H + 1)
    for h in range(1, MAX_H + 1, 2):
        b[h] = (8.0 / math.pi ** 2) * (-1.0) ** ((h - 1) // 2) / h ** 2
    return b


def spec_saw():
    b = np.zeros(MAX_H + 1)
    for h in range(1, MAX_H + 1):
        b[h] = (2.0 / math.pi) * (-1.0) ** (h + 1) / h
    return b


def spec_square():
    b = np.zeros(MAX_H + 1)
    for h in range(1, MAX_H + 1, 2):
        b[h] = (4.0 / math.pi) / h
    return b


# Frames are spread evenly across these keys: frame 0 = KEYS[0], frame 15 = KEYS[-1].
KEYS = [spec_sine, spec_triangle, spec_saw, spec_square]


def _synth(b):
    t = np.arange(BASE_LEN) / BASE_LEN
    x = np.zeros(BASE_LEN)
    for h in np.nonzero(b)[0]:
        x += b[h] * np.sin(2.0 * math.pi * h * t)
    return x


def frames_morph():
    keys = [k() for k in KEYS]
    frames = []
    for i in range(NUM_FRAMES):
        pos = i * (len(keys) - 1) / (NUM_FRAMES - 1)
        k0 = min(int(pos), len(keys) - 2)
        f = pos - k0
        frames.append(_synth((1.0 - f) * keys[k0] + f * keys[k0 + 1]))
    return frames


def frames_wav(path):
    with wave.open(path, "rb") as w:
        if w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise ValueError("WAV must be mono 16-bit")
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(float)
    cyc = len(data) // NUM_FRAMES
    if cyc * NUM_FRAMES != len(data):
        raise ValueError(f"WAV length {len(data)} is not a multiple of {NUM_FRAMES} frames")
    frames = []
    for i in range(NUM_FRAMES):
        # Resample one cycle to BASE_LEN in the frequency domain (exact for periodic signals).
        X = np.fft.rfft(data[i * cyc:(i + 1) * cyc])
        Y = np.zeros(BASE_LEN // 2 + 1, dtype=complex)
        n = min(len(X), MAX_H + 1)
        Y[:n] = X[:n]
        frames.append(np.fft.irfft(Y, n=BASE_LEN) * (BASE_LEN / cyc))
    return frames


def get_frames():
    if SOURCE == "morph":
        return frames_morph()
    if SOURCE == "wav":
        import os
        return frames_wav(os.path.join(os.path.dirname(os.path.abspath(__file__)), WAV_PATH))
    raise ValueError(f"unknown SOURCE {SOURCE!r}")
