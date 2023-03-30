/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu_data.hpp - Definition for IMU data collection class
 */

#pragma once

#include <functional>
#include <memory>

#include "imu.hpp"
#include "thread.hpp"

class ImuData : public Thread
{
public:
    using ImuPtr = std::shared_ptr<Imu>;
    ImuData(
        ImuPtr &imuPtr,
        unsigned int sampleInterval,
        bool verbose,
        int debugLevel
    );
    ~ImuData();

    enum DataStatus
    {
        AccelAvailable  = 1,
        GyroAvailable   = 2,
        MagAvailable    = 4,
        TempAvailable   = 8
    };

    struct Data
    {
        unsigned char   status;
        Imu::AxisValues accel;
        Imu::AxisValues gyro;
        Imu::AxisValues mag;
        float           temp;
        timespec        time;
    };
    
    typedef std::function<void(Data&)> DataFunc;
    void SetLogFunc(DataFunc func);

protected:
    virtual void ThreadFunc() override;

    std::shared_ptr<Imu>    mImuPtr;
    DataFunc                mDataFunc;
};
