/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * i2c.hpp - Definition for i2c communication class
 */

#pragma once

#include <string>
#include <vector>

#include "interface.hpp"

class I2c : public Interface
{
public:
    I2c(std::string busPath, int address, bool verbose = false);
    ~I2c();

    virtual bool    IsOpen() override;
    virtual int     Read(DataArray &data) override;
    virtual int     Write(DataArray &data) override;

protected:
    virtual void    DoOpen() override;
    virtual void    DoClose() override;

    const int           mAddr;
    const std::string   mBusPath;

    int                 mFd;
};
