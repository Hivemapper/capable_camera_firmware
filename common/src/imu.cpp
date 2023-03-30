/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu.hpp - Implementation for IMU base class
 */

#include "imu.hpp"

#include "interface.hpp"

Imu::Imu(Interface::IfacePtr &iface, bool verbose) :
    mIface(iface),
    mVerbose(verbose)
{}

Imu::~Imu() = default;

bool Imu::IsReady()
{
    return mReady;
}