#!/usr/bin/python

#
# gps_assist_now.py
#
# Sample key generation
#
# Copyright 2022 Hellbender Inc.
# All Rights Reserved
#
# Changelog:
# Author Email, Date,     , Comment
# caleb       , 2022-12-16, Found components
# niessl      , 2022-12-20, Integration

# Based on code found here (MIT License):
# https://gist.github.com/veproza/55ec6eaa612781ac29e7
# https://github.com/gokuhs/ublox-agps


import sys

import gps
import gps.ubx

PREAMBLE = b'\xb5\x62'

def process_assistnow_data(path):
    msgs = []
    with open(path, 'rb') as f:
        assistnow_data = f.read()
        # split on binary preamble
        msgs = assistnow_data.split(PREAMBLE)
        # add preamble back into msg
        for i in range(0, len(msgs)):
            msgs[i] = PREAMBLE + msgs[i]
        # hand craft the first message to enable mga acks, else script will later hang
        msgs = msgs[1:]
    return msgs

def send_assist_now_msg(msgs):

    gps_model = gps.ubx.ubx()
    gps_model.protver = 32.01
    gps_model.timestamp = 0

    io_handle = gps.gps_io()
    gps_model.io_handle = io_handle

    # enable mga acks
    gps_model.send_cfg_valset(["CFG-NAVSPG-ACKAIDING,1"])

    for msg in msgs:
        gps_model.gps_send_raw(msg)

if __name__=="__main__":
    msgs = process_assistnow_data(sys.argv[1])
    send_assist_now_msg(msgs)
      
