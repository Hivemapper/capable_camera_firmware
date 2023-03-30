/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Chris Niessl for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * app_options.cpp - Options parser for imu userspace driver
 */

#include <fstream>
#include <iostream>
#include <string>

#include "app_options.hpp"

AppOptions::AppOptions() : mOptions("Valid options are", 120, 80)
{
    using namespace boost::program_options;
    mOptions.add_options()
        ("help,h", value<bool>(&help)->default_value(false)->implicit_value(true),
            "Print this help message")
        ("version", value<bool>(&version)->default_value(false)->implicit_value(true),
            "Displays the build version number")
        ("verbose,v", value<bool>(&verbose)->default_value(false)->implicit_value(true),
            "Output extra debug and diagnostics")
        ("debugLevel,d", value<int>(&debugLevel)->default_value(0)->implicit_value(true),
            "Debug output level")
        ("gyroScale,gs", value<unsigned char>(&gyroFs)->default_value(0)->implicit_value(true),
            "Gyroscope out scale enum. Enum values:\n"
            "     0 - +/- 2000dps (default)\n"
            "     1 - +/- 1000dps\n"
            "     2 - +/- 500dps\n"
            "     3 - +/- 250dps\n"
            "     4 - +/- 125dps\n"
            "     5 - +/- 62.5dps\n"
            "     6 - +/- 31.25dps\n"
            "     7 - +/- 15.62dps\n")
        ("gyroRate,gr", value<unsigned char>(&gyroOdr)->default_value(6)->implicit_value(true),
            "Gyroscope out rate enum. Enum values:\n"
            "     1 - 32kHz\n"
            "     2 - 16kHz\n"
            "     3 - 8kHz\n"
            "     4 - 4kHz\n"
            "     5 - 2kHz\n"
            "     6 - 1kHz (default)\n"
            "     7 - 200Hz\n"
            "     8 - 100Hz\n"
            "     9 - 50Hz\n"
            "    10 - 25Hz\n"
            "    11 - 12.5Hz\n"
            "    15 - 500Hz\n")
        ("accelScale,as", value<unsigned char>(&accelFs)->default_value(0)->implicit_value(true),
            "Gyroscope out scale enum. Enum values:\n"
            "     0 - +/- 16g (default)\n"
            "     1 - +/- 8g\n"
            "     2 - +/- 4g\n"
            "     3 - +/- 2g\n")
        ("accelRate,ar", value<unsigned char>(&accelOdr)->default_value(6)->implicit_value(true),
            "Gyroscope out rate enum. Enum values:\n"
            "     1 - 32kHz\n"
            "     2 - 16kHz\n"
            "     3 - 8kHz\n"
            "     4 - 4kHz\n"
            "     5 - 2kHz\n"
            "     6 - 1kHz (default)\n"
            "     7 - 200Hz\n"
            "     8 - 100Hz\n"
            "     9 - 50Hz\n"
            "    10 - 25Hz\n"
            "    11 - 12.5Hz\n"
            "    15 - 500Hz\n")
        ("maxSize,s", value<unsigned int>(&maxSize)->default_value(30000)->implicit_value(true),
            "Max size of all logs in kilobytes")
        ("logInterval,i", value<unsigned int>(&logInterval)->default_value(100)->implicit_value(true),
            "Interval duration, in milliseconds, data collection")
        ("logDuration,l", value<unsigned int>(&logDuration)->default_value(60)->implicit_value(true),
            "Duration of each log file in seconds")
        ("live,lo", value<bool>(&live)->default_value(false)->implicit_value(true),
            "Output samples to stdout")
        ("path,p", value<std::string>(&path)->default_value("/tmp/"),
            "Path to for data log")
        ("tempPath,t", value<std::string>(&tempPath)->default_value(""),
             "Path to for data log")
        ("extension,e", value<std::string>(&ext)->default_value("ext"),
            "Extension to use for data log")
        ("config,c", value<std::string>(&configFile)->implicit_value("config.txt"),
            "Read the options from a file. If no filename is specified, default to config.txt. "
            "In case of duplicate options, the ones provided on the command line will be used. "
            "Note that the config file must only contain the long form options.")
        ;
}

AppOptions::~AppOptions() = default;

bool AppOptions::JSONParse()
{
    return true;
}

bool AppOptions::Parse(int argc, char *argv[])
{
    using namespace boost::program_options;
    variables_map vm;

    // Read options from the command line
    store(parse_command_line(argc, argv, mOptions), vm);
    notify(vm);

    // Read options from json file if specified
    std::ifstream ifs(configFile.c_str());
    if (ifs)
    {
        store(parse_config_file(ifs, mOptions), vm);
        notify(vm);
    }

    if (help)
    {
        std::cout << mOptions;
        return false;
    }

    if (version)
    {
        std::cout << "IMU Controller v0.1" << std::endl;
        return false;
    }

    return true;
}

void AppOptions::Print() const
{
    std::cout << "Options:" << std::endl;
    std::cout << "    verbose: " << verbose << std::endl;
    if (!configFile.empty())
        std::cout << "    config file: " << configFile << std::endl;
}
