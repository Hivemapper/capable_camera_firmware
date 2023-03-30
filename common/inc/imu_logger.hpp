/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu_logger.hpp - Definition for IMU logging class
 */

#pragma once

#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "logger.hpp"
#include "imu_data.hpp"

class ImuLogger : public Logger
{
public:
    ImuLogger(
        std::string &path,
        std::string &tempPath,
        std::string &ext,
        int maxSize,
        int fileDuration,
        bool verbose = false,
        int debugLevel = 0,
        bool live = false
    );
    virtual ~ImuLogger();
    
    void AddData(ImuData::Data &data);

protected:
    json OrganizeData(ImuData::Data &data);

private:
};
