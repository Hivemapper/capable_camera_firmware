/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu.hpp - Definition for IMU base class
 */

#pragma once

#include <array>
#include <memory>

#include "interface.hpp"

class Imu
{
public:
    Imu(Interface::IfacePtr &iface, bool verbose = false);
    ~Imu();

    bool            IsReady();

    virtual void    Init() = 0;
    virtual void    Reset() = 0;

    using AxisData = std::array<short, 3>;
    virtual bool    GetAccelData(AxisData &result) = 0;
    virtual bool    GetGyroData(AxisData &result) = 0;
    virtual bool    GetMagData(AxisData &result) = 0;
    virtual bool    GetTempData(short &result) = 0;

    using AxisValues = std::array<float, 3>;
    virtual bool    GetAccelValues(AxisValues &results) = 0;
    virtual bool    GetGyroValues(AxisValues &results) = 0;
    virtual bool    GetMagValues(AxisValues &results) = 0;
    virtual bool    GetTempValue(float &result) = 0;

protected:
    bool                    mVerbose;
    bool                    mReady;
    Interface::IfacePtr     mIface;

};
