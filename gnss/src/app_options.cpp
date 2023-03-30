/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * app_options.cpp - Options parser for gnss logger communication
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
        ("snr", value<bool>(&logSnr)->default_value(false)->implicit_value(true),
            "Output SNR values to log")
        ("debugLevel,d", value<int>(&debugLevel)->default_value(0)->implicit_value(true),
            "Debug output level")
        ("maxSize,s", value<int>(&maxSize)->default_value(30000)->implicit_value(true),
            "Max size of all logs in kilobytes")
        ("noFilter,n" , value<bool>(&noFilter)->default_value(false)->implicit_value(true),
             "Do not filter out null entries/no lock")
        ("logDuration,l", value<int>(&logDuration)->default_value(60)->implicit_value(true),
            "Duration of each log file in seconds")
        ("path,p", value<std::string>(&path)->default_value("/mnt/data/gps"),
            "Path to for data log")
        ("readyPath,r", value<std::string>(&readyPath)->default_value("/tmp/GPS_READY"),
            "Path for ready file when lock is obtained")
        ("tempPath,t", value<std::string>(&tempPath)->default_value("/tmp"),
             "Path for ready file when lock is obtained")
        ("extension,e", value<std::string>(&ext)->default_value("ext"),
            "Extension to use for data log")
        ("config,c", value<std::string>(&config_file)->implicit_value("config.txt"),
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

    // Read options from a file if specified
    std::ifstream ifs(config_file.c_str());
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
        std::cout << "GNSS Logger v0.1" << std::endl;
        return false;
    }

    return true;
}

void AppOptions::Print() const
{
    std::cout << "Options:" << std::endl;
    std::cout << "    verbose: " << verbose << std::endl;
    if (!config_file.empty())
        std::cout << "    config file: " << config_file << std::endl;
}
