/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * is31fl3199.cpp - Implementation for is31fl3199 LED controller class
 */

#include "is31fl3199.hpp"

#include <unistd.h>

#define NUM_COLORS  3

#define SHUTDOWN_REG    0x00u
#define LED_CTRL1_REG   0x01u
#define LED_CTRL2_REG   0x02u
#define LED_CFG2_REG    0x04u
#define LED_PWM1_REG    0x07u
#define LED_PWM2_REG    0x08u
#define LED_PWM3_REG    0x09u
#define LED_PWM4_REG    0x0Au
#define LED_PWM5_REG    0x0Bu
#define LED_PWM6_REG    0x0Cu
#define LED_PWM7_REG    0x0Du
#define LED_PWM8_REG    0x0Eu
#define LED_PWM9_REG    0x0Fu
#define UPDATE_DATA_REG 0x10u
#define RESET_REG       0xFFu

const LedCtrlr::LedData Is31fl3199::mLedRegs[NUM_LEDS] =
{
    {   LED_PWM1_REG,   LED_PWM3_REG,   LED_PWM2_REG},
    {   LED_PWM4_REG,   LED_PWM6_REG,   LED_PWM5_REG},
    {   LED_PWM7_REG,   LED_PWM9_REG,   LED_PWM8_REG}
};

const Is31fl3199::EnableData Is31fl3199::mEnableData[NUM_LEDS] =
{
    {.reg = LED_CTRL1_REG, .mask = 0x07u},
    {.reg = LED_CTRL1_REG, .mask = 0x70u},
    {.reg = LED_CTRL2_REG, .mask = 0x07u}
};

Is31fl3199::Is31fl3199(Interface::IfacePtr &iface) :
    LedCtrlr(iface, NUM_LEDS, NUM_COLORS)
{
    Reset();
    Init();
}

Is31fl3199::~Is31fl3199()
{
    Reset();
}

void Is31fl3199::Init()
{
    Interface::DataArray enable{0x01u};
    mIface->Write(enable, SHUTDOWN_REG);

    for (unsigned char index = 0; index < NUM_LEDS; index++)
    {
        mLedEnable[index] = false;
    }

    // Update the max current for the led's to the lowest setting
    Interface::DataArray current{0x30u};
    mIface->Write(current, LED_CFG2_REG);
}

void Is31fl3199::Reset()
{
    Interface::DataArray disable{0x00u};
    mIface->Write(disable, RESET_REG);
}

void Is31fl3199::DoSetColor(Value index, LedData &newColor)
{
    Interface::DataArray value(1);

    // Loop through and update all led value registers
    for (Value color = 0; color < NUM_COLORS; color++)
    {
        value[0] = newColor[color];
        mIface->Write(value, mLedRegs[index][color]);
    }

    UpdateData();
}

void Is31fl3199::DoSetState(Value index, bool enable)
{
    Interface::DataArray value(1);

    // Update LED state register
    mLedEnable[index] = enable;
    value[0] = (mLedEnable[index] == true) ? mEnableData[index].mask : 0x00u;
    if ((index == 0) || (index == 1))
    {
        Value otherIndex = index ^ 1;
        value[0] |= (mLedEnable[otherIndex] == true) ? mEnableData[otherIndex].mask : 0x00u;
    }
    mIface->Write(value, mEnableData[index].reg);

    UpdateData();
}

void Is31fl3199::UpdateData()
{
    Interface::DataArray init{0x00u};

    // Tickle the update register
    mIface->Write(init, UPDATE_DATA_REG);
}