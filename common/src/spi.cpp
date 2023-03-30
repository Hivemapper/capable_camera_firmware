/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * spi.h - Implementation for SPI interface class
 */

#include "spi.hpp"

#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>

Spi::Spi(std::string busPath, bool verbose) : 
    Interface(Interface::Type::SPI, verbose),
    mBusPath(busPath),
    mFd(0)
{
    Open();
}

Spi::~Spi()
{
    Close();
}

bool Spi::IsOpen()
{
    return (mFd > 0);
}

int Spi::Read(DataArray &data, Value other)
{
    DataArray writeData{other};
    return Transfer(writeData, data);
}

int Spi::Write (DataArray &data, Value other)
{
    DataArray writeData{data};
    writeData.insert(writeData.begin(), other);
    return Write(writeData);
}

int Spi::Transfer(DataArray &write, DataArray &read, Value other)
{
    DataArray writeData{write};
    writeData.insert(writeData.begin(), other);
    return Transfer(writeData, read);
}


int Spi::Read(DataArray &data)
{
    int readCount = -1;

    memset(&mTransfers[0], 0, sizeof(spi_ioc_transfer));
    mTransfers[0].rx_buf = (unsigned long long)data.data();
    mTransfers[0].len = data.size();

    int result = ioctl(mFd, SPI_IOC_MESSAGE(1), mTransfers);
    if (result == 0)
    {
        readCount = data.size();
    }

    return readCount;
}

int Spi::Write (DataArray &data)
{
    int wroteCount = -1;

    if (mVerbose == true)
    {
        std::cout << "Writing " << data.size() << " bytes to " << mBusPath
            << std::endl << "Write Data:" << std::endl;
        PrintBuf(std::cout, data);
        std::cout << std::endl;
    } 

    memset(&mTransfers[0], 0, sizeof(spi_ioc_transfer));
    mTransfers[0].tx_buf = (unsigned long long)data.data();
    mTransfers[0].len = data.size();

    int result = ioctl(mFd, SPI_IOC_MESSAGE(1), mTransfers);
    if (result == 0)
    {
        wroteCount = data.size();
    }

    if (mVerbose == true)
    {
        std::cout << "Result: " << result << std::endl;
    }  

    return wroteCount;
}

int Spi::Transfer(DataArray &write, DataArray &read)
{
    int readCount = -1;
    
    if (mVerbose == true)
    {
        std::cout << "Writing " << write.size() << " bytes to " << mBusPath
            << std::endl << "Space to read " << read.size() << " bytes."
            << std::endl << "Write Data:" << std::endl;
        PrintBuf(std::cout, write);
        std::cout << std::endl;
    }   

    memset(mTransfers, 0, sizeof(mTransfers));
    mTransfers[0].tx_buf = (unsigned long long)write.data();
    mTransfers[0].len = write.size();
    mTransfers[1].rx_buf = (unsigned long long)read.data();
    mTransfers[1].len = read.size();

    int result = ioctl(mFd, SPI_IOC_MESSAGE(2), mTransfers);

    if (mVerbose == true)
    {
        std::cout << "Result: " << result << std::endl;
        PrintBuf(std::cout, read);
        std::cout << std::endl;
    }   

    if (result >= 0)
    {
        readCount = read.size();
    }

    return readCount;
}

int Spi::UpdateOptions(SpiOptions *options, unsigned int count)
{
    int status = 0;
    if (IsOpen() == true)
    {
        for (unsigned int index = 0; (index < count) && (status != -1); index++)
        {
            status = ioctl(mFd, options[index].option, &options[index].value);

            if (mVerbose == true)
            {
                std::cout << "SPI UpdateOptions - Index: " << index
                    << ", Option: " << options[index].option << ", Status: "
                    << status << std::endl;
            }
        }
    }

    return status;
}

void Spi::DoOpen()
{
    mFd = open(mBusPath.c_str(), O_RDWR);
    if (mVerbose == true)
    {
        std::cout << "Opening " << mBusPath << ". Status: "
            << ((IsOpen() == true) ? "success" : "fail") << std::endl;
    }   
}

void Spi::DoClose()
{
    close(mFd);
    mFd = 0;
    
    if (mVerbose == true)
    {
        std::cout << "Closed " << mBusPath << "." << std::endl;
    }   
}