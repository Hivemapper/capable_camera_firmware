/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * app.cpp - Main entry point for GNSS over Serial UART communicator
 */

#include <exception>
#include <iostream>

#include <signal.h>

#include "gnss_data.hpp"
#include "gnss_logger.hpp"
#include "app_options.hpp"

bool doExit = false;
AppOptions *gOptions = nullptr;

void SigHandle(int sigNum)
{
    if ((gOptions != nullptr) && (gOptions->verbose == true))
    {
        std::cerr << "Received signal: " << sigNum << std::endl;
    }

    if (sigNum == SIGINT)
    {
        doExit = true;
    }
}

int main(int argc, char *argv[])
{
    bool optionsValid = false;
    AppOptions options; 
            
    // Handle parsing input arguments
    try
    {
        if (options.Parse(argc, argv))
        {
            if (options.verbose)
            {
                options.Print();
            }
            optionsValid = true;
        }
    }
    catch (std::exception const &e)
    {
        std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
        return -1;
    }
    
    // If the options are valid, continue with the application
    if(optionsValid)
    {
        gOptions = &options;
        signal(SIGINT, &SigHandle);

        // Setup objects
        GnssData data(
            options.verbose,
            options.debugLevel,
            options.noFilter
        );
        GnssLogger logger(
            options.path,
            options.tempPath,
            options.readyPath,
            options.ext,
            options.maxSize,
            options.logDuration,
            options.logSnr,
            options.verbose,
            options.debugLevel
        );

        // Setup connection/stream and prepare for data handling
        data.SetupGpsdConnect();
        data.StartStream();
        data.SetLogFunc([&logger](gps_data_t &data) { logger.AddData(data); });

        // Start logging
        logger.Start();

        // Continue until exit signal received
        while (doExit == false)
        {
            usleep(1000);
        }

        logger.Stop();
        data.TeardownGpsdConnect();
    }
    
    return 0;
}
