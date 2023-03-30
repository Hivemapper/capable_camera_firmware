/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu_data.cpp - Implementation for IMU data collection class
 */

#include "imu_data.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

ImuData::ImuData(
    ImuPtr &imuPtr,
    unsigned int sampleInterval,
    bool verbose,
    int debugLevel
    ) :
    Thread(verbose, debugLevel),
    mImuPtr(imuPtr)
{
    // Set timeout output logging
    std::chrono::milliseconds interval{sampleInterval};
    SetInterval(interval);
}

ImuData::~ImuData() {};

void ImuData::SetLogFunc(DataFunc func)
{
    mDataFunc = func;
}

void ImuData::ThreadFunc()
{
    // Verify that the IMU driver has prepared the IMU and the data handling
    // function has been passed in.
    if ((mImuPtr->IsReady() == true) && mDataFunc)
    {
        // Prepare the object to pass data around
        bool success = false;
        Data newData;
        memset(&newData, 0, sizeof(Data));

        // Try to get accelerometer data, if we do update the flag
        success = mImuPtr->GetAccelValues(newData.accel);
        if (success == true)
        {
            newData.status |= DataStatus::AccelAvailable;
        }

        // Try to get gyroscope data, if we do update the flag
        success = mImuPtr->GetGyroValues(newData.gyro);
        if (success == true)
        {
            newData.status |= DataStatus::GyroAvailable;
        }

        // Try to get magnetometer data, if we do update the flag
        success = mImuPtr->GetMagValues(newData.mag);
        if (success == true)
        {
            newData.status |= DataStatus::MagAvailable;
        }

        // Try to get temp data, if we do update the flag
        success = mImuPtr->GetTempValue(newData.temp);
        if (success == true)
        {
            newData.status |= DataStatus::TempAvailable;
        }

        timespec_get(&newData.time, TIME_UTC);
        mDataFunc(newData);
    }
    else
    {
        if (mVerbose == true)
        {
            std::cout << "Imu device is not ready..." << std::endl;
        }
    }
}