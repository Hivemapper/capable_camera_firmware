// Copyright 2021 Chris Osterwood for Capable Robot Components, Inc.
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
const fs = std.fs;

const i2c = @import("bus/i2c.zig");

// TODO : generate colorwheel pattern via comptime calls
const colors = [_][3]u8{
    [_]u8{ 255, 0, 0 },
    [_]u8{ 183, 72, 0 },
    [_]u8{ 111, 144, 0 },
    [_]u8{ 39, 216, 0 },
    [_]u8{ 0, 222, 33 },
    [_]u8{ 0, 150, 105 },
    [_]u8{ 0, 78, 177 },
    [_]u8{ 0, 6, 249 },
    [_]u8{ 66, 0, 189 },
    [_]u8{ 138, 0, 117 },
    [_]u8{ 210, 0, 45 },
    [_]u8{ 231, 24, 0 },
    [_]u8{ 159, 96, 0 },
    [_]u8{ 87, 168, 0 },
    [_]u8{ 15, 240, 0 },
    [_]u8{ 0, 198, 57 },
    [_]u8{ 0, 126, 129 },
    [_]u8{ 0, 54, 201 },
    [_]u8{ 18, 0, 237 },
    [_]u8{ 90, 0, 165 },
    [_]u8{ 162, 0, 93 },
    [_]u8{ 234, 0, 21 },
};

pub const LP50xx = struct {
    addr: u8 = 0x14,
    fd: fs.File,

    pub fn enable(self: LP50xx) void {
        var buffer = [_]u8{ 0x00, 0x40 };
        _ = i2c.write_block(self.fd, self.addr, &buffer);
    }

    pub fn set_brightness_index(self: LP50xx, index: u8, value: u8) void {
        var buffer = [_]u8{ 0x07 + index, value };
        _ = i2c.write_block(self.fd, self.addr, &buffer);
    }

    pub fn set_brightness(self: LP50xx, value: u8) void {
        self.set_brightness_index(0, value);
        self.set_brightness_index(1, value);
        self.set_brightness_index(2, value);
        self.set_brightness_index(3, value);
    }

    pub fn set(self: LP50xx, index: u8, color: [3]u8) void {
        var buffer = [_]u8{ 0x0B + index * 3, color[0], color[1], color[2] };
        _ = i2c.write_block(self.fd, self.addr, &buffer);
    }

    pub fn off(self: LP50xx) void {
        self.set(0, [_]u8{ 0, 0, 0 });
        self.set(1, [_]u8{ 0, 0, 0 });
        self.set(2, [_]u8{ 0, 0, 0 });
        self.set(3, [_]u8{ 0, 0, 0 });
    }

    pub fn read_register(self: LP50xx, register: u8, length: u8) ?[]u8 {
        return i2c.read_block(self.fd, self.addr, register, length);
    }

    pub fn spin(self: LP50xx) void {
        var step: u8 = 0;
        while (true) {
            for ([_]u8{ 0, 1, 2 }) |index| {
                self.set(index, colors[(step -% index) % colors.len]);
            }

            step +%= 1;
            std.time.sleep(100 * std.time.ns_per_ms);
        }
    }
};
