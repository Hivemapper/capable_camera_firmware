// Copyright 2022 Chris Osterwood for Capable Robot Components, Inc.
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

const web = @import("zhp");
const Datetime = web.datetime.Datetime;

const threads = @import("../threads.zig");
const recording = @import("../recording.zig");

const FolderListing = struct {
    count: usize,
    bytes: u64,
    files: []FileData,
};

const FileData = struct {
    name: []u8,
    mtime: i128,
    ctime: i128,
    size: u64,
};

fn indexHandler(request: *web.Request, response: *web.Response, suffix: []const u8) !void {
    try response.headers.append("Content-Type", "application/json");

    const ctx = threads.rec_ctx;

    if (recording.directory_listing(ctx.allocator, ctx.config.dir, suffix)) |listing| {
        defer ctx.allocator.free(listing.items);

        var list = std.ArrayList(FileData).init(ctx.allocator);

        for (listing.items) |elem| {
            var buffer = ctx.allocator.alloc(u8, elem.name_length) catch {
                std.log.err("failed to allocate memory for entry: {s}", .{elem.name});

                response.status = web.responses.INTERNAL_SERVER_ERROR;
                try response.stream.print("ERROR: failed to allocate memory for entry: {s}", .{elem.name});
                return;
            };

            var obj = FileData{
                .name = buffer,
                .size = elem.size,
                .mtime = elem.mtime,
                .ctime = elem.ctime,
            };

            std.mem.copy(u8, obj.name, elem.name[0..elem.name_length]);

            list.append(obj) catch {
                std.log.err("failed to append entry: {s}", .{elem.name});

                response.status = web.responses.INTERNAL_SERVER_ERROR;
                try response.stream.print("ERROR: failed to append entry: {s}", .{elem.name});
                return;
            };
        }

        const out = FolderListing{
            .count = listing.count,
            .bytes = listing.bytes,
            .files = list.items,
        };

        try std.json.stringify(out, std.json.StringifyOptions{
            .whitespace = .{ .indent = .{ .Space = 2 } },
        }, response.stream);

        defer {
            for (list.items) |node| {
                ctx.allocator.free(node.name);
            }
            list.deinit();
        }
    }
}

pub const GnssIndexHandler = struct {
    pub fn get(self: *GnssIndexHandler, request: *web.Request, response: *web.Response) !void {
        try indexHandler(request, response, ".gps");
    }
};

pub const ImuIndexHandler = struct {
    pub fn get(self: *ImuIndexHandler, request: *web.Request, response: *web.Response) !void {
        try indexHandler(request, response, ".imu");
    }
};

pub const AuxFileHandler = struct {
    handler: FileHandler = undefined,

    pub fn get(self: *AuxFileHandler, request: *web.Request, response: *web.Response) !void {
        const allocator = response.allocator;
        const ctx = threads.rec_ctx;
        const args = request.args.?;

        const full_path = try std.fs.path.join(allocator, &[_][]const u8{ ctx.config.dir, args[0].? });

        self.handler = FileHandler{ .path = full_path, .content_type = "application/json" };
        return self.handler.dispatch(request, response);
    }

    pub fn stream(self: *AuxFileHandler, io: *web.IOStream) !u64 {
        return self.handler.stream(io);
    }
};

pub const RecordingIndexHandler = struct {
    pub fn get(self: *RecordingIndexHandler, request: *web.Request, response: *web.Response) !void {
        try indexHandler(request, response, ".jpg");
    }
};

pub const RecordingLastHandler = struct {
    handler: FileHandler = undefined,

    pub fn get(self: *RecordingLastHandler, request: *web.Request, response: *web.Response) !void {
        const allocator = response.allocator;
        const ctx = threads.rec_ctx;

        const path = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ ctx.config.dir, ctx.last_file });

        self.handler = FileHandler{ .path = path };
        return self.handler.dispatch(request, response);
    }

    pub fn stream(self: *RecordingLastHandler, io: *web.IOStream) !u64 {
        return self.handler.stream(io);
    }
};

pub const RecordingFileHandler = struct {
    handler: FileHandler = undefined,

    pub fn get(self: *RecordingFileHandler, request: *web.Request, response: *web.Response) !void {
        const allocator = response.allocator;
        const ctx = threads.rec_ctx;
        const args = request.args.?;

        // Replace instances of '%20' in filename with ' ' chars, to handle URI-encoding.
        // The new filename might be shorter (in bytes) than the original, so we use that
        // new length when creating the full filesystem path later on.
        const filename = try allocator.alloc(u8, args[0].?.len);
        const spaces = std.mem.replace(u8, args[0].?, "%20", " ", filename[0..]);
        const filename_length = filename.len - spaces * 2;
        defer allocator.free(filename);

        const full_path = try std.fs.path.join(allocator, &[_][]const u8{ ctx.config.dir, filename[0..filename_length] });

        self.handler = FileHandler{ .path = full_path };
        return self.handler.dispatch(request, response);
    }

    pub fn stream(self: *RecordingFileHandler, io: *web.IOStream) !u64 {
        return self.handler.stream(io);
    }
};

pub const FileHandler = struct {
    file: ?std.fs.File = null,
    start: u64 = 0,
    end: u64 = 0,
    path: []const u8,
    content_type: ?[]const u8 = null,

    pub fn dispatch(self: *FileHandler, request: *web.Request, response: *web.Response) !void {
        const allocator = response.allocator;
        const mimetypes = &web.mimetypes.instance.?;

        const file = std.fs.cwd().openFile(self.path, .{ .read = true }) catch |err| {
            // TODO: Handle debug page
            std.log.warn("recording file {s} error {}", .{ self.path, err });
            return self.renderNotFound(request, response);
        };
        errdefer file.close();

        // Get file info
        var stat = try file.stat();

        // File handle is on disk, but it does not have contents yet.
        // We need to wait, otherwise assertion at start of stream function will fail.
        while (stat.size == 0) {
            std.time.sleep(std.time.ns_per_ms * 10);
            stat = try file.stat();
        }

        var modified = Datetime.fromModifiedTime(stat.mtime);

        // If the file was not modified, return 304
        if (self.checkNotModified(request, modified)) {
            response.status = web.responses.NOT_MODIFIED;
            file.close();
            return;
        }

        // Set last modified time for caching purposes
        // NOTE: The modified result doesn't need freed since the response handles that
        var buf = try response.allocator.alloc(u8, 32);
        try response.headers.append("Last-Modified", try modified.formatHttpBuf(buf));

        self.end = stat.size;
        var size: u64 = stat.size;

        if (self.content_type) |value| {
            try response.headers.append("Content-Type", value);
        } else {
            // Try to get the content type
            const content_type = mimetypes.getTypeFromFilename(self.path) orelse "application/octet-stream";
            try response.headers.append("Content-Type", content_type);
        }

        try response.headers.append("Content-Length", try std.fmt.allocPrint(allocator, "{}", .{size}));
        self.file = file;
        response.send_stream = true;
    }

    // Return true if not modified and a 304 can be returned
    pub fn checkNotModified(self: *FileHandler, request: *web.Request, mtime: Datetime) bool {
        // Check if the file was modified since the header
        // See https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/If-Modified-Since
        const v = request.headers.getDefault("If-Modified-Since", "");
        const since = Datetime.parseModifiedSince(v) catch return false;
        return since.gte(mtime);
    }

    // Stream the file
    pub fn stream(self: *FileHandler, io: *web.IOStream) !u64 {
        std.debug.assert(self.end > self.start);
        const total_wrote = self.end - self.start;
        var bytes_left: u64 = total_wrote;
        if (self.file) |file| {
            defer file.close();

            // Jump to requested range
            if (self.start > 0) {
                try file.seekTo(self.start);
            }

            // Send it
            var reader = file.reader();
            try io.flush();
            while (bytes_left > 0) {
                // Read into buffer
                const end = std.math.min(bytes_left, io.out_buffer.len);
                const n = try reader.read(io.out_buffer[0..end]);
                if (n == 0) break; // Unexpected EOF
                bytes_left -= n;
                try io.flushBuffered(n);
            }
        }
        return total_wrote - bytes_left;
    }

    pub fn renderNotFound(self: *FileHandler, request: *web.Request, response: *web.Response) !void {
        var handler = web.handlers.NotFoundHandler{};
        try handler.dispatch(request, response);
    }
};
