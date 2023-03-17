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
const system = @import("../system.zig");

pub const Handler = struct {
    pub fn get(self: *Handler, request: *web.Request, response: *web.Response) !void {
        try response.headers.append("Content-Type", "application/json");

        if (try stat()) |data| {
            try std.json.stringify(data, std.json.StringifyOptions{
                .whitespace = .{ .indent = .{ .Space = 2 } },
            }, response.stream);
        }
    }
};

const c = @cImport({
    @cInclude("unistd.h");
    @cInclude("sys/types.h");
    @cInclude("sys/param.h");
});

const proce_stat_core_times = packed struct {
    user: u64 = 0,
    nice: u64 = 0,
    system: u64 = 0,
    idle: u64 = 0,
    iowait: u64 = 0,
    irq: u64 = 0,
    softirq: u64 = 0,
    // steal: u64 = 0,
    // guest: u64 = 0,
    // guestnice: u64 = 0,
};

// TODO : don't hard code this
const num_cores = 4;

// TODO : don't hard code this
const jiffies = 100;

const proc_stat = struct {
    tick_per_sec: u8 = jiffies,
    uptime: u64 = 0,
    idletime: u64 = 0,
    cpu: proce_stat_core_times,
    cores: [num_cores]proce_stat_core_times,
};

fn parse_cpu_line(line: []const u8) proce_stat_core_times {
    var it = std.mem.split(line, " ");

    const ident = it.next() orelse @panic("stat: no core name");

    // 'cpu' has two spaces after it instead of one, so advance over the null between them
    if (ident.len == 3) {
        _ = it.next();
    }

    return proce_stat_core_times{
        .user = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core user time"), 10) catch unreachable,
        .nice = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core nice time"), 10) catch unreachable,
        .system = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core system time"), 10) catch unreachable,
        .idle = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core idle time"), 10) catch unreachable,
        .iowait = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core iowait time"), 10) catch unreachable,
        .irq = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core irq time"), 10) catch unreachable,
        .softirq = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core softirq time"), 10) catch unreachable,
        // .steal = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core steal time"), 10) catch unreachable,
        // .guest = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core guest time"), 10) catch unreachable,
        // .guestnice = std.fmt.parseInt(u64, it.next() orelse @panic("stat: no core guestnice time"), 10) catch unreachable,
    };
}

pub fn stat() !?proc_stat {
    const path = "/proc/stat";
    var buf: [1024]u8 = undefined;

    var fd = fs.openFileAbsolute(path, fs.File.OpenFlags{ .read = true, .write = false }) catch unreachable;
    defer fd.close();

    const reader = std.io.bufferedReader(fd.reader()).reader();

    // Fill struct with default (e.g. 0) core_time structs
    var data: proc_stat = undefined;
    data.cpu = proce_stat_core_times{};

    var core_idx: usize = 0;
    while (core_idx < num_cores) {
        data.cores[core_idx] = proce_stat_core_times{};
        core_idx += 1;
    }

    core_idx = 0;

    while (try reader.readUntilDelimiterOrEof(&buf, '\n')) |line| {
        var it = std.mem.split(line, " ");
        const ident = it.next() orelse @panic("malformed /proc/stat");
        // print("{s}\n", .{line});

        if (ident.len >= 3 and mem.eql(u8, ident[0..3], "cpu")) {
            const times = parse_cpu_line(line);

            if (ident.len == 3) {
                data.cpu = times;
            } else {
                data.cores[core_idx] = times;
                core_idx += 1;
            }
        }
    }

    if (system.uptime_idletime()) |value| {
        data.uptime = @floatToInt(u64, value[0] * jiffies);
        data.idletime = @floatToInt(u64, value[1] * jiffies / num_cores);
    }

    data.tick_per_sec = jiffies;

    return data;
}
