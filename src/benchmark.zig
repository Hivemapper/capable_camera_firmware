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

pub const log_level = .debug;
var gpa = std.heap.GeneralPurposeAllocator(.{}){};

const usage =
    \\Usage: zig build run-bench -- [command] [options]
    \\
    \\Commands:
    \\
    \\  disc [DIR]       Run disc IO benchmark in the DIR path. 
    \\					 A randomly named folder will be created in that directory, populated with files,
    \\					 and both files and the created directory will be deleted after the test is complete.
    \\
    \\General Options:
    \\
    \\  -h, --help       Print command-specific usage
    \\
;

pub fn main() anyerror!void {
    const allocator = &gpa.allocator;
    const args = try std.process.argsAlloc(allocator);

    if (args.len <= 1) {
        std.log.info("{s}", .{usage});
        std.log.err("expected command argument", .{});
        return;
    }

    const cmd = args[1];
    const cmd_args = args[2..];

    if (std.mem.eql(u8, cmd, "disc")) {
        try cmd_disc(allocator, cmd_args);
    } else {
        std.log.info("{s}", .{usage});
        std.log.err("unknown command: {s}", .{args[1]});
    }
}

pub const DiscIO = struct {
    dir: std.fs.Dir,
    file_size: usize = 1024 * 1024,
    file_count: usize = 100,
    test_interations: usize = 10,
};

const random_bytes_count = 12;
const sub_path_len = std.fs.base64_encoder.calcSize(random_bytes_count);

pub const TmpDir = struct {
    dir: std.fs.Dir,
    parent_dir: std.fs.Dir,
    sub_path: [sub_path_len]u8,

    pub fn cleanup(self: *TmpDir) void {
        self.dir.close();
        self.parent_dir.deleteTree(&self.sub_path) catch {};
    }
};

pub fn absolute_temp_dir(path: []const u8) TmpDir {
    var random_bytes: [random_bytes_count]u8 = undefined;
    std.crypto.random.bytes(&random_bytes);

    var sub_path: [sub_path_len]u8 = undefined;
    _ = std.fs.base64_encoder.encode(&sub_path, &random_bytes);

    std.fs.makeDirAbsolute(path) catch {};
    var parent_dir = std.fs.openDirAbsolute(path, .{}) catch @panic("unable to open dir for testing");
    var dir = parent_dir.makeOpenPath(&sub_path, .{}) catch @panic("unable to make tmp dir for testing");

    return .{
        .dir = dir,
        .parent_dir = parent_dir,
        .sub_path = sub_path,
    };
}

pub fn local_temp_dir() TmpDir {
    var random_bytes: [random_bytes_count]u8 = undefined;
    std.crypto.random.bytes(&random_bytes);

    var sub_path: [sub_path_len]u8 = undefined;
    _ = std.fs.base64_encoder.encode(&sub_path, &random_bytes);

    var parent_dir = std.fs.cwd();
    var dir = parent_dir.makeOpenPath(&sub_path, .{}) catch @panic("unable to make tmp dir for testing");

    return .{
        .dir = dir,
        .parent_dir = parent_dir,
        .sub_path = sub_path,
    };
}

fn cmd_disc(allocator: *std.mem.Allocator, args: []const []const u8) !void {
    var dir: std.fs.Dir = undefined;
    var do_cleanup: bool = false;
    var test_dir: TmpDir = undefined;

    if (args.len < 1) {
        std.log.info("No test target directory specified, using the current working directory.", .{});
        test_dir = local_temp_dir();
    } else {
        test_dir = absolute_temp_dir(args[0]);
    }

    const config = DiscIO{ .dir = test_dir.dir };
    try disc_io_init(allocator, config);

    test_dir.cleanup();
}

fn disc_io_init(allocator: *std.mem.Allocator, config: DiscIO) anyerror!void {
    var dir_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const dir_path = try std.os.getFdPath(config.dir.fd, &dir_buffer);
    std.log.info("Disc IO occurring in {s}", .{dir_path});

    var test_index: u8 = 0;

    while (test_index < config.test_interations) {
        try disc_io_run(allocator, config, test_index);
        test_index += 1;
    }
}

fn disc_io_run(allocator: *std.mem.Allocator, config: DiscIO, test_index: usize) anyerror!void {
    var file_index = test_index * config.file_count;
    const end_file_index = file_index + config.file_count;

    var file_name_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const buffer_slice = file_name_buffer[0..];

    var file_data_buffer = try allocator.alloc(u8, config.file_size);
    defer allocator.free(file_data_buffer);

    var start_timestamp: i64 = undefined;
    var milliseconds: i64 = 0;

    while (file_index < end_file_index) {
        const file_name = try std.fmt.bufPrint(buffer_slice, "file_{d}.ext", .{file_index});

        // Write random data into the file
        try std.os.getrandom(file_data_buffer[0..]);

        start_timestamp = std.time.milliTimestamp();
        try config.dir.writeFile(file_name, file_data_buffer);
        milliseconds += std.time.milliTimestamp() - start_timestamp;

        file_index += 1;
    }

    start_timestamp = std.time.milliTimestamp();
    try run_command(allocator, &[_][]const u8{"sync"});
    const sync_milliseconds = std.time.milliTimestamp() - start_timestamp;
    milliseconds += sync_milliseconds;

    const total_MB = @intToFloat(f64, config.file_size * config.file_count) / 1024.0 / 1024.0;

    const sync_seconds = @intToFloat(f64, sync_milliseconds) / std.time.ms_per_s;
    const seconds = @intToFloat(f64, milliseconds) / std.time.ms_per_s;

    std.log.info("{d} MB in {d: >5.2} sec (sync was {d: >5.2} sec) : {d: >5.2} MB/sec", .{ total_MB, seconds, sync_seconds, total_MB / seconds });
}

fn run_command(allocator: *std.mem.Allocator, argv: []const []const u8) anyerror!void {
    var child = try std.ChildProcess.init(argv, allocator);

    const term = child.spawnAndWait() catch |err| {
        std.log.warn("Unable to spawn {s}: {s}\n", .{ argv[0], @errorName(err) });
        return err;
    };

    switch (term) {
        .Exited => |code| {
            if (code != 0) {
                std.log.warn("Command {s} exited with error code {}", .{ argv, code });
                return error.UncleanExit;
            }
        },
        else => {
            std.log.warn("Command {s} terminated unexpectedly", .{argv});
            return error.UncleanExit;
        },
    }
}
