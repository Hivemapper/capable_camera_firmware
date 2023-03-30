/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * interface.hpp - Definition for basic interface class
 */

#pragma once

#include <limits>
#include <memory>
#include <vector>

class Interface
{
public:
    using Value = unsigned char;
    using DataArray = std::vector<Value>;
    using IfacePtr = std::shared_ptr<Interface>;

    enum class Type
    {
        I2C,
        UART,
        SPI
    };

    Interface(Type type, bool verbose = false);
    ~Interface();

    virtual bool    IsOpen() = 0;

    void            Open();
    void            Close();
    void            Reconnect();

    virtual int     Read(DataArray &data, Value other);
    virtual int     Write(DataArray &data, Value other);
    virtual int     Transfer(DataArray &write, DataArray &read, Value other);
    
    virtual int     Read(DataArray &data);
    virtual int     Write (DataArray &data);
    virtual int     Transfer(DataArray &write, DataArray &read);
    
    void            PrintBuf(
                        std::ostream &stream,
                        DataArray data,
                        unsigned int maxIndex = std::numeric_limits<unsigned int>::max()
                    );

    const Type      mType;

protected:
    virtual void    DoOpen() = 0;
    virtual void    DoClose() = 0;

    bool            mVerbose;

private:

};
