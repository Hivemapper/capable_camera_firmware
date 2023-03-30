/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * thread.hpp - General thread class
 */

#pragma once

#include <chrono>
#include <thread>

class Thread
{
public:
    Thread(bool verbose, int debugLevel);
    ~Thread();

    bool IsRunning();
    void Start();
    void Stop();

    void SetInterval(std::chrono::microseconds time);

protected:
    virtual void ThreadFunc() = 0;

    const bool                  mVerbose;
    const int                   mDebugLevel;

private:
    void ThreadLoop();

    bool                        mStop;
    std::thread                 mThread;

    std::chrono::microseconds   mInterval;
};
