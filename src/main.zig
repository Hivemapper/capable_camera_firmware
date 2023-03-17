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
const print = @import("std").debug.print;
const fs = std.fs;
const os = std.os;
const mem = @import("std").mem;

var gpa = std.heap.GeneralPurposeAllocator(.{}){};
pub const io_mode = .evented;

const web = @import("zhp");

const spi = @import("bus/spi.zig");

const config = @import("config.zig");
const threads = @import("threads.zig");
const camera = @import("camera.zig");

const led_driver = @import("led_driver.zig");
const imu = @import("imu.zig");
const gnss = @import("gnss.zig");
const info = @import("info.zig");
const system = @import("system.zig");

const handlers = @import("handlers.zig");

pub const routes = handlers.routes;

var led: led_driver.LP50xx = undefined;

pub const log_level: std.log.Level = .debug;

// Define root.log to override the std implementation
pub fn log(comptime level: std.log.Level, comptime scope: @TypeOf(.EnumLiteral), comptime format: []const u8, args: anytype) void {

    // Ignore all non-critical logging from sources other than specified
    const scope_prefix = "" ++ switch (scope) {
        .main, .gnss, .config, .trace, .system, .imu, .default => @tagName(scope),
        else => if (@enumToInt(level) <= @enumToInt(std.log.Level.crit))
            @tagName(scope)
        else
            return,
    } ++ " : ";

    const prefix = " [" ++ @tagName(level) ++ "] " ++ scope_prefix;

    const held = std.debug.getStderrMutex().acquire();
    defer held.release();
    const stderr = std.io.getStdErr().writer();

    // Log timestamp cannot be included in second call to print due to comptime unknowns
    nosuspend stderr.print("{d: >12.3}", .{system.logstamp()}) catch return;
    nosuspend stderr.print(prefix ++ format ++ "\n", args) catch return;
}

fn write_info_json() !void {
    if (try info.stat()) |stat| {
        print("stat {any}\n", .{stat});

        const file = try std.fs.cwd().createFile("test.json", .{});
        defer file.close();
        try std.json.stringify(stat, std.json.StringifyOptions{
            .whitespace = .{ .indent = .{ .Space = 2 } },
        }, file.writer());
    }
}

pub fn main() anyerror!void {
    system.init();

    attachSegfaultHandler();

    const slog = std.log.scoped(.main);

    defer std.debug.assert(!gpa.deinit());
    const allocator = &gpa.allocator;

    var cfg = config.load(allocator);
    threads.configuration = cfg;

    var loop: std.event.Loop = undefined;
    try loop.initMultiThreaded();
    defer loop.deinit();

    var i2c_fd = try fs.openFileAbsolute("/dev/i2c-1", fs.File.OpenFlags{ .read = true, .write = true });
    defer i2c_fd.close();

    var spi00_fd = try fs.openFileAbsolute("/dev/spidev0.0", fs.File.OpenFlags{ .read = true, .write = true });
    defer spi00_fd.close();

    var spi01_fd = try fs.openFileAbsolute("/dev/spidev0.1", fs.File.OpenFlags{ .read = true, .write = true });
    defer spi01_fd.close();

    var imu_handle = spi.SPI{ .fd = spi00_fd };
    slog.debug("SPI00 configure {any}", .{imu_handle.configure(0, 10000)});

    var iim = imu.init(imu_handle);
    slog.info("INIT", .{});
    iim.config(imu.ACCEL_FS.G8, imu.GYRO_FS.DPS_1000);
    slog.info("CONFIG", .{});

    threads.imu_ctx = threads.ImuContext{
        .imu = &iim,
        .interval = 1000,
        .trace_dir = cfg.recording.dir,
        .allocator = allocator,
    };

    try loop.runDetached(allocator, threads.imu_thread, .{threads.imu_ctx});

    led = led_driver.LP50xx{ .fd = i2c_fd };

    if (led.read_register(0x00, 1)) |value| {
        slog.debug("CONFIG0 = 0x{s}", .{std.fmt.fmtSliceHexUpper(value)});
    }

    if (led.read_register(0x01, 1)) |value| {
        slog.debug("CONFIG1 = 0x{s}", .{std.fmt.fmtSliceHexUpper(value)});
    }

    led.off();
    led.enable();
    led.set_brightness(0x30);

    // Set PWR led to green, others off
    led.set(0, [_]u8{ 0, 0, 0 });
    led.set(1, [_]u8{ 0, 0, 0 });
    led.set(2, [_]u8{ 0, 255, 0 });

    var led_ctx = threads.HeartBeatContext{ .led = led, .idx = 2 };
    try loop.runDetached(allocator, threads.heartbeat_thread, .{led_ctx});

    var gnss_handle = spi.SPI{ .fd = spi01_fd };
    slog.debug("SPI01 configure {any}", .{gnss_handle.configure(0, 5500)});

    var pos = gnss.init(gnss_handle);
    var gnss_interval = @divFloor(1000, @intCast(u16, cfg.camera.encoding.fps));

    if (cfg.gnss.reset_on_start) {
        pos.reset(null);
    }

    pos.configure();
    pos.set_interval(gnss_interval);

    threads.gnss_ctx = threads.GnssContext{
        .led = led,
        .gnss = &pos,
        .interval = gnss_interval,
        .config = cfg.gnss,
        .trace_dir = cfg.recording.dir,
        .allocator = allocator,
    };

    try loop.runDetached(allocator, threads.gnss_thread, .{threads.gnss_ctx});

    // This will error if either socket doesn't exists.  We ignore that error
    std.fs.cwd().deleteFile(cfg.recording.connection.socket) catch {};
    std.fs.cwd().deleteFile(cfg.cfg_socket) catch {};    

    const address = std.net.Address.initUnix(cfg.recording.connection.socket) catch |err| {
        slog.err("Error creating unix socket: {}", .{err});
        std.debug.panic("Error creating unix socket: {}", .{err});
    };
    
    const cfg_address = std.net.Address.initUnix(cfg.cfg_socket) catch |err| {
        slog.err("Error creating unix socket: {}", .{err});
        std.debug.panic("Error creating unix socket: {}", .{err});
    };

    var server = std.net.StreamServer.init(.{});
    defer server.deinit();

    server.listen(address) catch |err| {
        slog.err("Error listening to unix socket: {}", .{err});
        std.debug.panic("Error listening to unix socket: {}", .{err});
    };

    // Set up cfg communication for camera thread
    var cfg_server = std.net.StreamServer.init(.{});
    defer cfg_server.deinit();

    cfg_server.listen(cfg_address) catch |err| {
        slog.err("Error listening to unix socket: {}", .{err});
        std.debug.panic("Error listening to unix socket: {}", .{err});
    };

    threads.rec_ctx = threads.RecordingContext{
        .config = cfg.recording,
        .allocator = allocator,
        .server = &server,
        .stop = std.atomic.Atomic(bool).init(false),
        .gnss = threads.gnss_ctx,
    };

    threads.brdg_cfg_ctx = threads.BridgeCfgContext{
        .cfg_server = &cfg_server,
        .cfg_lock = .{},
        .cfg_ready = false,
        .cfg_data  = std.ArrayList(u8).init(allocator),
    };
    defer threads.brdg_cfg_ctx.cfg_data.deinit();

    try loop.runDetached(allocator, threads.bridge_cfg_thread, .{&threads.brdg_cfg_ctx});
    try loop.runDetached(allocator, threads.recording_cleanup_thread, .{threads.rec_ctx});
    try loop.runDetached(allocator, threads.recording_server_thread, .{&threads.rec_ctx});

    threads.camera_ctx = threads.CameraContext{
        .config = cfg.camera,
        .socket = threads.rec_ctx.config.connection.socket,
    };

    //try loop.runDetached(allocator, camera.bridge_thread, .{threads.camera_ctx});

    var app = web.Application.init(allocator, .{ .debug = true });
    var app_ctx = threads.AppContext{ .app = &app, .config = cfg.api };
    try loop.runDetached(allocator, threads.app_thread, .{app_ctx});

    loop.run();
}

fn resetSegfaultHandler() void {
    var act = os.Sigaction{
        .handler = .{ .sigaction = os.SIG_DFL },
        .mask = os.empty_sigset,
        .flags = 0,
    };

    os.sigaction(os.SIGSEGV, &act, null);
    os.sigaction(os.SIGILL, &act, null);
    os.sigaction(os.SIGBUS, &act, null);
}

fn handleSignal(sig: i32, sig_info: *const os.siginfo_t, ctx_ptr: ?*const c_void) callconv(.C) noreturn {
    // Reset to the default handler so that if a segfault happens in this handler it will crash
    // the process. Also when this handler returns, the original instruction will be repeated
    // and the resulting segfault will crash the process rather than continually dump stack traces.
    resetSegfaultHandler();

    led.off();

    const addr = @ptrToInt(sig_info.fields.sigfault.addr);

    // Don't use std.debug.print() as stderr_mutex may still be locked.
    nosuspend {
        const stderr = std.io.getStdErr().writer();
        _ = switch (sig) {
            os.SIGSEGV => stderr.print("Segmentation fault at address 0x{x}\n", .{addr}),
            os.SIGILL => stderr.print("Illegal instruction at address 0x{x}\n", .{addr}),
            os.SIGBUS => stderr.print("Bus error at address 0x{x}\n", .{addr}),
            os.SIGINT => stderr.print("Exit due to CTRL-C\n", .{}),
            else => stderr.print("Exit due to signal {}\n", .{sig}),
        } catch os.abort();
    }

    os.abort();
}

/// Attaches a global SIGSEGV handler
pub fn attachSegfaultHandler() void {
    var act = os.Sigaction{
        .handler = .{ .sigaction = handleSignal },
        .mask = os.empty_sigset,
        .flags = (os.SA_SIGINFO | os.SA_RESTART | os.SA_RESETHAND),
    };

    os.sigaction(os.SIGINT, &act, null);
    os.sigaction(os.SIGTERM, &act, null);
    os.sigaction(os.SIGQUIT, &act, null);
    os.sigaction(os.SIGILL, &act, null);
    os.sigaction(os.SIGTRAP, &act, null);
    os.sigaction(os.SIGABRT, &act, null);
    os.sigaction(os.SIGBUS, &act, null);
}
