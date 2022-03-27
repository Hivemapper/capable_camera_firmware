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
const mem = std.mem;
const print = std.debug.print;

const web = @import("zhp");

pub const Handler = struct {
    pub fn get(self: *Handler, request: *web.Request, response: *web.Response) !void {
        if (cached == false) {
            try fill_cpu_data();
            cached = true;
        }

        try response.headers.append("Content-Type", "application/json");
        try std.json.stringify(data, std.json.StringifyOptions{
            .whitespace = .{ .indent = .{ .Space = 2 } },
        }, response.stream);
    }
};

const info_data = struct {
    cpu_serial: [8]u8 = [_]u8{'0'} ** 8,
    cpu_revision: [6]u8 = [_]u8{'0'} ** 6,
};

var cached: bool = false;
var data: info_data = info_data{};

fn fill_cpu_data() !void {
    const path = "/proc/cpuinfo";
    var buf: [1024]u8 = undefined;

    var fd = fs.openFileAbsolute(path, fs.File.OpenFlags{ .read = true, .write = false }) catch unreachable;
    defer fd.close();

    const reader = std.io.bufferedReader(fd.reader()).reader();

    while (try reader.readUntilDelimiterOrEof(&buf, '\n')) |line| {
        if (mem.indexOf(u8, line, ":") != null) {
            var it = std.mem.split(line, ": ");
            const ident = it.next() orelse @panic("malformed /proc/cpuinfo");
            const value = it.next() orelse @panic("malformed /proc/cpuinfo");

            if (ident.len >= 8 and mem.eql(u8, ident[0..8], "Revision")) {
                mem.copy(u8, data.cpu_revision[0..], value[0..]);
            } else if (ident.len >= 6 and mem.eql(u8, ident[0..6], "Serial")) {
                mem.copy(u8, data.cpu_serial[0..], value[8..]);
            }
        }
    }
}
