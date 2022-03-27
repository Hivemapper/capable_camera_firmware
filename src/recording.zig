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

pub fn directory_listing(allocator: *std.mem.Allocator, path: []const u8) ?FolderListing {
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
    if (directory_listing(ctx.allocator, ctx.config.dir)) |listing| {
        defer ctx.allocator.free(listing.items);

        std.log.info("count: {d}", .{listing.count});
        std.log.info("size: {d}", .{listing.bytes});

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

    const max_size = 1; // MB
    var cfg = config.Recording{ .max_size = max_size, .dir = dir_path };
    var ctx = threads.RecordingContext{ .config = cfg, .allocator = alloc };

    if (directory_listing(ctx.allocator, ctx.config.dir)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expectEqual(listing.count, 0);
    }

    var file_name_buffer: [std.fs.MAX_PATH_BYTES]u8 = undefined;
    const file_suffixes: []const u8 = "012345";
    const buffer_slice = file_name_buffer[0..];

    const file_size = 1024 * 256;
    var file_data_buffer: [file_size]u8 = undefined;

    for (file_suffixes) |suffix| {
        const file_name = try std.fmt.bufPrint(buffer_slice, "frame_{c}.ext", .{suffix});

        // Write randome data into the file
        std.crypto.random.bytes(file_data_buffer[0..]);
        try tmp.dir.writeFile(file_name, &file_data_buffer);

        // Sleep a bit to ensure that mtime listing works correctly
        std.time.sleep(std.time.ns_per_ms);
    }

    // Check that the directory listing finds the files and counts the file sizes correctly
    if (directory_listing(ctx.allocator, ctx.config.dir)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expectEqual(listing.count, file_suffixes.len);
        try std.testing.expectEqual(listing.bytes, file_size * file_suffixes.len);
    }

    directory_cleanup(ctx);

    // Check that cleanup removed the correct number of files
    // TODO : check that cleanup removed the correct (oldest) files
    if (directory_listing(ctx.allocator, ctx.config.dir)) |listing| {
        defer alloc.free(listing.items);
        try std.testing.expect(listing.count < file_suffixes.len);
        try std.testing.expectEqual(listing.count, @divTrunc(max_size * 1024 * 1024, file_size));
        try std.testing.expect(listing.bytes < 1024 * 1024 * max_size);
    }
}
