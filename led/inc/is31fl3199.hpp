/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * is31fl3199.hpp - Definition for is31fl3199 LED controller class
 */

#pragma once

#include "i2c.hpp"
#include "led_ctrlr.hpp"

#define NUM_LEDS    3

class Is31fl3199 : public LedCtrlr
{
public:
    Is31fl3199(Interface::IfacePtr &iface);
    ~Is31fl3199();

    virtual void    Init() override;
    virtual void    Reset() override;

protected:
    virtual void    DoSetColor(Value index, LedData &newColor) override;
    virtual void    DoSetState(Value index, bool enable) override;

private:
    void            UpdateData();
    
    struct EnableData
    {
        Value   reg;
        Value   mask;
    };

    static const LedData    mLedRegs[NUM_LEDS];
    static const EnableData mEnableData[NUM_LEDS];
    bool                    mLedEnable[NUM_LEDS];
};
