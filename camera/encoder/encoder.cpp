/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * encoder.cpp - Video encoder class.
 */

#include <cstring>

#include "encoder.hpp"
#include "mjpeg_encoder.hpp"
#include "../core/video_options.hpp"

Encoder *Encoder::Create(VideoOptions const *options) {
    if (strcasecmp(options->codec.c_str(), "mjpeg") == 0) {
        return new MjpegEncoder(options);
    }
    throw std::runtime_error("Unrecognised codec " + options->codec);
}
