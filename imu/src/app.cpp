/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Chris Niessl for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * app.cpp - Main entry point for IMU Userspace driver
 */

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace std::string_literals;

#include <linux/spi/spidev.h>
#include <signal.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "app_options.hpp"
#include "spi.hpp"
#include "iim42652.hpp"
#include "imu_data.hpp"
#include "imu_logger.hpp"

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
        // Setup the SPI interface
        std::shared_ptr<Spi> spi = std::make_shared<Spi>("/dev/spidev0.0"s, options.verbose);
        Spi::SpiOptions spiOptions[] = {
            {SPI_IOC_WR_MODE,           SPI_MODE_0},
            {SPI_IOC_WR_BITS_PER_WORD,  8},
            {SPI_IOC_WR_MAX_SPEED_HZ,   1000000}
        };
        spi->UpdateOptions(spiOptions, sizeof(spiOptions)/sizeof(Spi::SpiOptions));
        Interface::IfacePtr iface = spi;

        // Setup the IMU driver
        std::shared_ptr<Iim42652> iim42652 = std::make_shared<Iim42652>(iface, true);
        iim42652->UpdateAccelConfig(
            (Iim42652::Rates)options.accelOdr,
            (Iim42652::AccelScale)options.accelFs
        );
        iim42652->UpdateGyroConfig(
            (Iim42652::Rates)options.gyroOdr,
            (Iim42652::GyroScale)options.gyroFs
        );
        ImuData::ImuPtr imu = iim42652;

        // Setup the logger
        ImuLogger logger(
            options.path,
            options.tempPath,
            options.ext,
            options.maxSize,
            options.logDuration,
            options.verbose,
            options.debugLevel,
            options.live
        );

        // Setup the data aquisition
        ImuData dataHandler(
            imu,
            options.logInterval,
            options.verbose,
            options.debugLevel
        );
        dataHandler.SetLogFunc(
            [&logger](ImuData::Data &data) {
                logger.AddData(data);
            }
        );

        // Start the threads
        logger.Start();
        dataHandler.Start();
        
        // Wait for the call to exit
        while (doExit == false)
        {
            usleep(10000);
        }

        // Stop the threads
        dataHandler.Stop();
        logger.Stop();
    }
    
    return 0;
}
