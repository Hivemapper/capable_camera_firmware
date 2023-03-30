/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * led_ctrlr.cpp - Definition for basic LED controller class
 */

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "interface.hpp"

class LedCtrlr
{
public:
    using Value = unsigned char;
    using LedData = std::vector<Value>;
    LedCtrlr(Interface::IfacePtr &iface, Value numLeds, Value numColors);
    ~LedCtrlr();

    virtual void    Init() = 0;
    virtual void    Reset() = 0;

    void            SetColor(Value index, LedData &newColor);
    void            SetState(Value index, bool enable);

protected:
    virtual void    DoSetColor(Value index, LedData &newColor) = 0;
    virtual void    DoSetState(Value index, bool enable) = 0;

    const Value         mNumLeds;
    const Value         mNumColors;

    Interface::IfacePtr mIface;

private:
    inline bool IsValidIndex(Value index);
    inline bool IsValidColor(LedData &color);
};
