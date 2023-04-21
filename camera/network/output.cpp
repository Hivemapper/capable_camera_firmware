/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.cpp - video stream output base class
 */

#include <cinttypes>
#include <stdexcept>
#include <iostream>

#include "file_output.hpp"
#include "output.hpp"
#include "../core/video_options.hpp"

int64_t timestamp_now()
{
    auto duration = std::chrono::high_resolution_clock::now().time_since_epoch();
    return std::chrono::duration_cast< std::chrono::milliseconds >( duration ).count();
}

Output::Output(VideoOptions const *options)
        : options_(options), state_(WAITING_KEYFRAME), fp_timestamps_(nullptr), time_offset_(0), last_timestamp_(0)
{
    if (!options->save_pts.empty())
    {
        fp_timestamps_ = fopen(options->save_pts.c_str(), "w");
        if (!fp_timestamps_)
            throw std::runtime_error("Failed to open timestamp file " + options->save_pts);
        fprintf(fp_timestamps_, "frame,encode_ready,output_done\n");
    }

    start_time_ = timestamp_now();

    enable_ = !options->pause;
}

Output::~Output()
{
    if (fp_timestamps_)
        fclose(fp_timestamps_);
}

void Output::Signal()
{
    enable_ = !enable_;
}

bool Output::GetContinueRunningStatus()
{
    return enable_;
}

void Output::OutputReady(void *mem,
                         size_t size,
                         void *prevMem,
                         size_t prevSize,
                         void *exifMem,
                         size_t exifSize,
                         int64_t timestamp_us,
                         bool keyframe)
{
    int64_t ready_time = timestamp_now();

    // When output is enabled, we may have to wait for the next keyframe.
    uint32_t flags = keyframe ? FLAG_KEYFRAME : FLAG_NONE;
    if (!enable_)
        state_ = DISABLED;
    else if (state_ == DISABLED)
        state_ = WAITING_KEYFRAME;
    if (state_ == WAITING_KEYFRAME && keyframe)
        state_ = RUNNING, flags |= FLAG_RESTART;
    if (state_ != RUNNING)
        return;

    // Frig the timestamps to be continuous after a pause.
    if (flags & FLAG_RESTART)
    {
        time_offset_ = timestamp_us - last_timestamp_;
    }
    last_timestamp_ = timestamp_us - time_offset_;

    try{
        outputBuffer(mem,
                     size,
                     prevMem,
                     prevSize,
                     exifMem,
                     exifSize,
                     last_timestamp_,
                     flags);
    }
    catch(const std::exception& e){
        std::cout << "Exception!" << std::endl;
        std::cout << e.what() << std::endl;
        Signal();
    }

    int64_t done_time = timestamp_now();

    // Save timestamps to a file, if that was requested.
    if (fp_timestamps_) {
        fprintf(fp_timestamps_, "%" PRId64 ",%" PRId64 ",%" PRId64 "\n", last_timestamp_ / 1000,
                ready_time-start_time_, done_time-start_time_);
    }
}

void Output::outputBuffer(void *mem,
                          size_t size,
                          void *prevMem,
                          size_t prevSize,
                          void *exifMem,
                          size_t exifSize,
                          int64_t timestamp_us,
                          uint32_t flags)
{
    // DEPRECATED. Separate FileOutput class created to handle file output. If a null/empty
    // pipe is needed, then create an Output object and leave this empty.
}

Output *Output::Create(VideoOptions const *options)
{
        return new FileOutput(options);
}
