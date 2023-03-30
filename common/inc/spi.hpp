/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * spi.h - Definition for SPI interface class
 */

#pragma once

#include <string>

#include <linux/spi/spidev.h>

#include "interface.hpp"
#define NUM_TRANSFERS   2

class Spi : public Interface
{
public:  
    Spi(std::string busPath, bool verbose = false);
    ~Spi();

    virtual bool    IsOpen() override;

    virtual int     Read(DataArray &data, Value other) override;
    virtual int     Write (DataArray &data, Value other) override;
    virtual int     Transfer(DataArray &write, DataArray &read, Value other) override;
    
    virtual int     Read(DataArray &data) override;
    virtual int     Write (DataArray &data) override;
    virtual int     Transfer(DataArray &write, DataArray &read) override;

    struct SpiOptions
    {
        unsigned long   option;
        unsigned int    value;
    };

    int             UpdateOptions(SpiOptions *options, unsigned int count);

protected:
    virtual void    DoOpen() override;
    virtual void    DoClose() override;

    const std::string   mBusPath;

    int                 mFd;
    spi_ioc_transfer    mTransfers[NUM_TRANSFERS];
};

