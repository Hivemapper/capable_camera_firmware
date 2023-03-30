/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * i2c.cpp - Implementation for i2c communication class
 */

#include "i2c.hpp"

#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "interface.hpp"

I2c::I2c(std::string busPath, int address, bool verbose) :
    Interface(Type::I2C, verbose),
    mBusPath(busPath),
    mAddr(address),
    mFd(0) { Open(); }

I2c::~I2c() { Close(); }

bool I2c::IsOpen()
{
    return mFd > 0;
}

int I2c::Read(DataArray &data)
{
    int numRead = -1;

    if (IsOpen() == true)
    {
        // Read the desired size
        numRead = read(mFd, data.data(), data.size());
        
        if (mVerbose == true)
        {
            if (numRead != -1)
            {
                std::cout << "Read " << numRead << " bytes to "
                    << mBusPath << std::endl;
                PrintBuf(std::cout, data, numRead);
                std::cout << std::endl << std::flush;
            }
            else
            {
                std::cout << "Error while reading from " << mBusPath
                    << std::endl;
            }
        }
    }

    return numRead;
}

int I2c::Write(DataArray &data)
{
    int numWrote = -1;
    
    if (IsOpen() == true)
    {
        // Write all the data
        DataArray sendData(data);
        numWrote = write(mFd, sendData.data(), sendData.size());

        if (mVerbose == true)
        {
            if (numWrote != -1)
            {
                std::cout << "Wrote " << sendData.size() << " bytes to "
                    << mBusPath << std::endl;
                PrintBuf(std::cout, sendData, numWrote);
                std::cout << std::endl << std::flush;
            }
            else
            {
                std::cout << "Error while writing to " << mBusPath
                    << std::endl;
            }
        }
    }

    return numWrote;
}

void I2c::DoOpen()
{
    // Open the interface
    mFd = open(mBusPath.c_str(), O_RDWR);
    if (mFd > 0)
    {
        // Set the slave address
        int status = ioctl(mFd, I2C_SLAVE, mAddr);
        if (status != 0)
        {
            DoClose();
        }
        else if (mVerbose)
        {
            std::cout << "Device (" << mBusPath << ") opened" << std::endl;
        }
    }
}

void I2c::DoClose()
{
    close(mFd);
    mFd = 0;
    if (mVerbose)
    {
        std::cout << "Device (" << mBusPath << ") closed" << std::endl;
    }
}