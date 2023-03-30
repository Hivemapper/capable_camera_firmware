/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * app_options.cpp - Options parser for imu userspace driver
 */

#pragma once

#include <string>

#include "boost/program_options.hpp"

struct AppOptions
{
    AppOptions();
    virtual ~AppOptions();

    // App information
    bool help;
    bool verbose;
    bool version;
    int debugLevel;
    bool live;

    // Meta configruation
    std::string configFile;

    // Bus configuration values

    // Device configuration values
    unsigned char gyroFs;
    unsigned char gyroOdr;
    unsigned char accelFs;
    unsigned char accelOdr;

    // Output configuration
    unsigned int maxSize;
    unsigned int logInterval;
    unsigned int logDuration;
    std::string path;
    std::string tempPath;
    std::string ext;

    // Configuration Functions
    virtual bool JSONParse();
    virtual bool Parse(int argc, char *argv[]);
    virtual void Print() const;

protected:
    boost::program_options::options_description mOptions;
};
