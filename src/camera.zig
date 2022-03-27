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
const print = std.debug.print;
const assert = std.debug.assert;

const threads = @import("threads.zig");
const config = @import("config.zig");

pub const Camera = struct {
    ctx: threads.CameraContext,
    child: std.ChildProcess,

    pub fn start(self: *Camera) bool {}

    pub fn stop(self: *Camera) bool {}

    pub fn restart(self: *Camera) bool {}
};

pub fn init(ctx: threads.CameraContext) Camera {
    return Camera{ .ctx = ctx };
}

fn build_argv(ctx: threads.CameraContext) !std.ArrayList([]const u8) {
    var argv = std.ArrayList([]const u8).init(ctx.allocator);

    try argv.appendSlice(&[_][]const u8{ "setarch", "linux32" });

    try argv.append("$HOME/capable_camera_firmware/camera/build/libcamera-bridge");
    try argv.appendSlice(&[_][]const u8{ "--codec", "mjpeg" });
    try argv.appendSlice(&[_][]const u8{ "--segment", "0" });
    try argv.appendSlice(&[_][]const u8{ "--timeout", "0" });

    std.log.info("socket {s}", .{ctx.socket});

    const sock_path = try std.fmt.allocPrint(ctx.allocator, "sck://{s}", .{ctx.socket});
    try argv.appendSlice(&[_][]const u8{ "-o", sock_path });

    return argv;
}

pub fn bridge_thread(ctx: threads.CameraContext) void {
    var argv = build_argv(ctx) catch |err| {
        std.log.err("Could not build argv for camera bridge: {s}", .{err});
        return;
    };

    defer argv.deinit();

    std.log.info("starting camera bridge: {s}", .{argv.toOwnedSlice()});

    while (true) {
        var proc = std.ChildProcess.init(argv.items, ctx.allocator) catch |err| {
            std.log.err("Could not init camera bridge child process: {s}", .{err});
            return;
        };

        defer proc.deinit();

        proc.spawn() catch |err| {
            std.log.err("Could not spawn camera bridge child process: {s}", .{err});
            continue;
        };

        wait_for_child(proc) catch |err| {
            std.log.err("Could not wait for camera bridge child process: {s}", .{err});
            continue;
        };

        std.time.sleep(1 * std.time.ns_per_s);
    }
}

fn wait_for_child(proc: *std.ChildProcess) !void {
    while (true) {
        switch (try proc.spawnAndWait()) {
            .Exited => |code| {
                std.log.err("[restarter] child process exited with {}", .{code});
                return;
            },
            .Stopped => |sig| std.log.err("[restarter] child process has stopped ({})", .{sig}),
            .Signal => |sig| std.log.info("[restarter] child process signal ({})", .{sig}),
            .Unknown => |sig| std.log.info("[restarter] child process unknown ({})", .{sig}),
        }
    }
}
