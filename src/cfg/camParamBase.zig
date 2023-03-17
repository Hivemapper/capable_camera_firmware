// Copyright 2022 Chris Niessl for Capable Robot Components, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

const std = @import("std");
const mem = std.mem;

pub const filePath: []const u8 = "./bridge.sh";

pub const scriptLines = 
    \\#!/bin/bash
    \\
    \\trap "i2cset -y 1 0x14 0xc 0x00 b" EXIT
    \\
    \\while :  
    \\do  
    \\sleep 1
    \\value=$(i2cget -y 1 0x14 0xf b)
    \\if [ "$value" == "0xff" ]  
    \\then
    \\    echo $value
    \\    echo "GPS Locked"
    \\    break
    \\fi
    \\done  
    \\
    \\i2cset -y 1 0x14 0xc 0xFF b
    \\
;

pub const execLine1 = 
    \\setarch linux32 ./libcamera-bridge --codec mjpeg --segment 0 -o sck:///tmp/bridge.sock --width {} --height {} --framerate {} \
    \\
; 

pub const execLine2 =
    \\--awb {s} --awbgains {},{} --brightness {} --contrast {} --exposure {s} --ev {} --gain {} --metering {s} --saturation {} --sharpness {} --tuning-file imx477.json --timeout 0;
;
