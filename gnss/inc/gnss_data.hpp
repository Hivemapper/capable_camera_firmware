/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * gnss_logger.hpp - Session manager for gnss communications
 */

#pragma once

#include <functional>

#include "gps.h"

#include "thread.hpp"

class GnssData : protected Thread
{
public:

    GnssData(bool verbose, int debugLevel, bool noFilter);
    ~GnssData();	
    
    void SetupGpsdConnect();
    void TeardownGpsdConnect();

    void StartStream();
    void StopStream();
    void SignalGnssLock();

    bool IsFixed();

    typedef std::function<void(gps_data_t&)> DataFunc;
    void SetLogFunc(DataFunc func);

protected:
    virtual void ThreadFunc() override;
    
    bool            mConnected;
    bool            mStreaming;
    bool            mNoFilter;
    int             mMode;

    gps_data_t      mGpsData;
    DataFunc        mDataFunc;
};
