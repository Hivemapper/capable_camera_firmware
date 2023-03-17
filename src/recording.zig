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
const datetime = @import("datetime.zig");
const bounded_array = @import("bounded_array.zig");

const MAX_FILENAME_LENGTH = 64;

const FileData = struct {
    name: [MAX_FILENAME_LENGTH:0]u8,
    name_length: usize,
    mtime: i128,
    ctime: i128,
    size: u64,
    file: bool,
};

const FolderListing = struct {
    count: usize,
    bytes: u64,
    items: []FileData,
};

pub const TraceLogType = enum { GNSS, IMU };
const MAX_SIZE: usize = 60 * 10 * 2;

const slog = std.log.scoped(.trace);

pub fn TraceLog(comptime T: type) type {
    return struct {
        const Self = @This();

        // These are used to allow for remapping between local milliTimestamp() and global time
        init_at: i64 = 0,
        init_datetime: datetime.Datetime = undefined,
        timestamp_needed: bool = true,

        // Reset whenever a new log is started
        start_at: i64 = 0,

        // Target timestamp when log will end and new one started
        end_at: i64 = 0,

        // Maximum duration of the log, could be shorter if capacity is reached
        seconds: i64 = 60,

        // Used to create the log file path : {dir}/{timestamp}.{ext}
        dir: []const u8,
        timestamp: [24]u8 = undefined,
        ext: TraceLogType,

        allocator: *std.mem.Allocator,

        items: bounded_array.BoundedArray(T, MAX_SIZE),

        pub fn init(allocator: *std.mem.Allocator, dir: []const u8, ext: TraceLogType) Self {
            return Self{
                .allocator = allocator,
                .dir = dir,
                .ext = ext,
                .items = bounded_array.BoundedArray(T, MAX_SIZE).init(0) catch unreachable,
            };
        }

        pub fn setDuration(self: *Self, seconds: i64) void {
            self.seconds = seconds;
        }

        pub fn setTimestamp(self: *Self, value: [24]u8) void {
            self.timestamp_needed = false;
            self.init_at = std.time.milliTimestamp();
            self.init_datetime = datetime.Datetime.parseIso(value[0..]) catch datetime.Datetime.now();
            _ = self.init_datetime.formatIsoBuf(self.timestamp[0..]) catch unreachable;
        }

        pub fn updateTimestamp(self: *Self) void {
            self.start_at = std.time.milliTimestamp();
            self.end_at = self.start_at + self.seconds * std.time.ms_per_s - 1;

            const start_at_dt = self.init_datetime.shiftMilliseconds(self.start_at - self.init_at);
            _ = start_at_dt.formatIsoBuf(self.timestamp[0..]) catch unreachable;
        }

        pub fn save(self: *Self) void {
            var ext = "TBDT";

            switch (self.ext) {
                .GNSS => {
                    ext = "gpsT";
                },
                .IMU => {
                    ext = "imuT";
                },
            }

            const path = std.fmt.allocPrint(self.allocator, "{s}/{s}.{s}", .{
                self.dir,
                self.timestamp,
                ext,
            }) catch |err| {
                slog.err("[{any}] when allocPrint path for {s}", .{ err, self.timestamp });
                return;
            };

            defer self.allocator.free(path);

            slog.info("saving to {s}", .{path});

            var file = std.fs.cwd().createFile(path[0..], .{}) catch |err| {
                slog.err("failed to create file", .{});
                return;
            };
            defer file.close();

            var output_str = std.ArrayList(u8).init(self.allocator);
            defer output_str.deinit();

            std.json.stringify(self.items.constSlice(), .{}, output_str.writer()) catch |err| {
                slog.err("[{any}] when creating JSON", .{err});
            };

            file.writeAll(output_str.items) catch |err| {
                slog.err("[{any}] when writing to file {s}", .{ err, path });
            };

            // Rename file to remove last T (which prevents it from being listed by the web API)
            std.fs.renameAbsolute(path[0..], path[0 .. path.len - 1]) catch |err| {
                slog.err("[{any}] cannot rename {s}", .{ err, path });
            };
        }

        pub fn append(self: *Self, item: T) void {
            var do_save: bool = false;

            // This is the first time we're adding to the log
            // Therefore, need to setup start and end timestamps
            if (self.start_at == 0) {
                self.updateTimestamp();
            }

            if (item.received_at > self.end_at) {
                do_save = true;
            }

            self.items.ensureUnusedCapacity(1) catch |err| {
                do_save = true;
                slog.warn("Saving trace due to capacity limit", .{});
            };

            if (do_save) {
                self.save();
                self.updateTimestamp();

                // Clear items
                self.items.resize(0) catch |err| {
                    slog.err("[{any}] when clearing items", .{err});
                };
            }

            self.items.append(item) catch |err| {
                slog.err("[{any}] when appending item", .{err});
            };
        }
    };
}

fn max(comptime T: type, a: T, b: T) T {
    return if (a > b) a else b;
}

fn min(comptime T: type, a: T, b: T) T {
    return if (a > b) b else a;
}

fn directory_entry(dir: std.fs.Dir, entry: std.fs.Dir.Entry) ?FileData {
    if (dir.openFile(entry.name, .{ .read = true, .write = false })) |handle| {
        defer handle.close();

        if (handle.stat()) |stat| {
            var bytes = min(usize, MAX_FILENAME_LENGTH, entry.name.len);

            var data = FileData{
                .name = [_:0]u8{0} ** MAX_FILENAME_LENGTH,
                .size = stat.size,
                .mtime = @divTrunc(stat.mtime, std.time.ns_per_ms),
                .ctime = @divTrunc(stat.ctime, std.time.ns_per_ms),
                .file = true,
                .name_length = bytes,
            };

            std.mem.copy(u8, data.name[0..], entry.name[0..bytes]);

            if (stat.kind == std.fs.File.Kind.Directory) {
                data.file = false;
                data.size = 0;
            }

            return data;
        } else |err| {
            std.log.warn("could not stat file {s}\n", .{entry.name});
        }
    } else |err| {
        std.log.warn("could not open file {s}\n", .{entry.name});
    }

    return null;
}

fn cmp_file_listing_name(context: void, a: FileData, b: FileData) bool {
    _ = context;
    return std.mem.lessThan(u8, a.name, b.name);
}

fn cmp_file_listing_ctime(context: void, a: FileData, b: FileData) bool {
    _ = context;
    return a.ctime < b.ctime;
}

fn cmp_file_listing_mtime(context: void, a: FileData, b: FileData) bool {
    _ = context;
    return a.mtime < b.mtime;
}

pub fn directory_listing(allocator: *std.mem.Allocator, path: []const u8, suffix: []const u8) ?FolderListing {
    var total_size: u64 = 0;
    var count: usize = 0;

    const stdout = std.io.getStdOut().writer();

    var list = std.ArrayList(FileData).init(allocator);
    // defer list.deinit();

    if (std.fs.openDirAbsolute(path, .{ .iterate = true, .no_follow = false })) |dir| {
        var it = dir.iterate();

        while (it.next()) |opt| {
            if (opt) |entry| {

                // Skip files starting with '.'
                if (entry.name[0] == 0x2E) {
                    continue;
                }

                // Skip files which do not end with suffix
                if (!std.mem.endsWith(u8, entry.name, suffix)) {
                    continue;
                }

                if (directory_entry(dir, entry)) |data| {
                    // std.json.stringify(data, std.json.StringifyOptions{
                    //     .whitespace = .{ .indent = .{ .Space = 2 } },
                    // }, stdout) catch unreachable;
                    // print("\n", .{});

                    list.append(data) catch {
                        std.log.err("failed to append entry: {s}", .{data.name});
                        return null;
                    };

                    total_size += data.size;
                    count += 1;
                }
            } else {
                break;
            }
        } else |err| {
            std.log.warn("directory iter error\n", .{});
            return null;
        }
    } else |err| {
        std.log.warn("[{any}] when opening recording directory {s}\n", .{ err, path });
        return null;
    }

    std.sort.sort(FileData, list.items, {}, cmp_file_listing_mtime);

    return FolderListing{ .count = count, .bytes = total_size, .items = list.toOwnedSlice() };
}

pub fn directory_cleanup(ctx: threads.RecordingContext) void {
    if (directory_listing(ctx.allocator, ctx.config.dir, ".jpg")) |listing| {
        defer ctx.allocator.free(listing.items);

        // std.log.info("count: {d}", .{listing.count});
        // std.log.info("size: {d}", .{listing.bytes});

        // for (listing.items) |elem| {
        //     std.log.info("node: {s}", .{elem.name});
        // }

        if (ctx.config.max_size * 1024 * 1024 < listing.bytes) {
            const to_delete = listing.bytes - ctx.config.max_size * 1024 * 1024;
            var deleted: u64 = 0;

            std.log.info("recordings will be trimmed by at least {d} kB", .{@divTrunc(to_delete, 1024)});

            for (listing.items) |elem| {
                deleted += elem.size;
                // std.log.info("node: {s} is {d} kB", .{ elem.name, @divTrunc(elem.size, 1024) });

                // acutally do the delete here
                if (std.fs.openDirAbsolute(ctx.config.dir, .{ .iterate = true, .no_follow = false })) |dir| {
                    dir.deleteFileZ(elem.name[0..]) catch unreachable;
                } else |err| {
                    std.log.warn("[{any}] when opening recording directory to delete\n", .{err});
                }

                if (deleted > to_delete) {
                    break;
                }
            }
        }
    }
}

test "recording directory cleanup" {
    const alloc = std.testing.allocator;

    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();

    var dir_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const dir_path = try std.os.getFdPath(tmp.dir.fd, &dir_buffer);
    const ext = ".ext";

    const max_size = 1; // MB
    var cfg = config.Recording{ .max_size = max_size, .dir = dir_path };
    var ctx = threads.RecordingContext{ .config = cfg, .allocator = alloc };

    if (directory_listing(ctx.allocator, ctx.config.dir, ext)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expectEqual(listing.count, 0);
    }

    var file_name_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const file_suffixes: []const u8 = "012345";
    const buffer_slice = file_name_buffer[0..];

    const file_size = 1024 * 256;
    var file_data_buffer: [file_size]u8 = undefined;

    for (file_suffixes) |suffix| {
        const file_name = try std.fmt.bufPrint(buffer_slice, "frame_{c}.{s}", .{ suffix, ext });

        // Write randome data into the file
        std.crypto.random.bytes(file_data_buffer[0..]);
        try tmp.dir.writeFile(file_name, &file_data_buffer);

        // Sleep a bit to ensure that mtime listing works correctly
        std.time.sleep(std.time.ns_per_ms);
    }

    // Check that the directory listing finds the files and counts the file sizes correctly
    if (directory_listing(ctx.allocator, ctx.config.dir, ext)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expectEqual(listing.count, file_suffixes.len);
        try std.testing.expectEqual(listing.bytes, file_size * file_suffixes.len);
    }

    directory_cleanup(ctx);

    // Check that cleanup removed the correct number of files
    // TODO : check that cleanup removed the correct (oldest) files
    if (directory_listing(ctx.allocator, ctx.config.dir, ext)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expect(listing.count < file_suffixes.len);
        try std.testing.expectEqual(listing.count, @divTrunc(max_size * 1024 * 1024, file_size));
        try std.testing.expect(listing.bytes < 1024 * 1024 * max_size);
    }
}
