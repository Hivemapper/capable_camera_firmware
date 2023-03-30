/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * led_ctrlr.cpp - Implementation for basic LED controller class
 */

#include "led_ctrlr.hpp"

LedCtrlr::LedCtrlr(Interface::IfacePtr &iface, Value numLeds, Value numColors) :
    mIface(iface),
    mNumLeds(numLeds),
    mNumColors(numColors) {};

LedCtrlr::~LedCtrlr() = default;

void LedCtrlr::SetColor(Value index, LedData &newColor)
{
    // If the index and the new color are valid, call the derived setting function
    if ((IsValidIndex(index == true)) && (IsValidColor(newColor) == true))
    {
        DoSetColor(index, newColor);
    }
}

void LedCtrlr::SetState(Value index, bool enable)
{
    // If the index is valid, call the derived setting function
    if (IsValidIndex(index == true))
    {
        DoSetState(index, enable);
    }
}

inline bool LedCtrlr::IsValidIndex(Value index)
{
    return (index < mNumLeds);
}

inline bool LedCtrlr::IsValidColor(LedData &color)
{
    return (color.size() == mNumColors);
}