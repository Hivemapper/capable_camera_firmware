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
        ("debugLevel,d", value<int>(&debugLevel)->default_value(0)->implicit_value(true),
            "Debug output level")
        ("refreshRate,r", value<int>(&refreshRate)->default_value(100)->implicit_value(true),
            "Rate, in milliseconds, at which updates are processed")
        ("path,p", value<std::string>(&path)->default_value("/tmp/"),
            "Path to look for LED configuration file")
        ("fileName,f", value<std::string>(&fileName)->default_value("led.json"),
            "LED configuration file")
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

    // Read options from a file if specified
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
        std::cout << "LED Controller v0.1" << std::endl;
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
