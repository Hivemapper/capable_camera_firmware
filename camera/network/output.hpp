/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * output.hpp - video stream output base class
 */

#pragma once

#include <cstdio>
#include <chrono>
#include <atomic>

#include "core/video_options.hpp"

class Output
{
public:
  static Output *Create(VideoOptions const *options);

  Output(VideoOptions const *options);
  virtual ~Output();
  virtual void Signal(); // a derived class might redefine what this means
  void OutputReady(void *mem,
                   size_t size,
                   void *prevMem,
                   size_t prevSize,
                   void *exifMem,
                   size_t exifSize,
                   int64_t timestamp_us,
                   bool keyframe);


protected:
  enum Flag
  {
    FLAG_NONE = 0,
    FLAG_KEYFRAME = 1,
    FLAG_RESTART = 2
  };
  virtual void outputBuffer(void *mem,
                            size_t size,
                            void *prevMem,
                            size_t prevSize,
                            void *exifMem,
                            size_t exifSize,
                            int64_t timestamp_us,
                            uint32_t flags);
  VideoOptions const *options_;
  bool GetContinueRunningStatus();

private:
  enum State
  {
    DISABLED = 0,
    WAITING_KEYFRAME = 1,
    RUNNING = 2
  };
  State state_;
  std::atomic<bool> enable_;
  FILE *fp_timestamps_;
  int64_t time_offset_;
  int64_t last_timestamp_;
  int64_t start_time_;
};
