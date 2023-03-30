/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * gnss_logger.hpp - Session logger for gnss communications
 */

#pragma once

#include <string>

#include "gps.h"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "logger.hpp"

class GnssLogger : public Logger
{
public:

    GnssLogger(
        std::string &path,
        std::string &tempPath,
        std::string &readyLoc,
        std::string &ext,
        int maxSize,
        int fileDuration,
        bool logSnr,
        bool verbose = false,
        int debugLevel = 0
    );
    virtual ~GnssLogger();
    
    void AddData(gps_data_t &data);

protected:
    json OrganizeData(gps_data_t &data);
    void ShareData(json organizedData);

private:
    const bool  mLogSnr;
    std::string mReadyLoc;
};
