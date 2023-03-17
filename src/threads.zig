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

const led_driver = @import("led_driver.zig");
const gnss = @import("gnss.zig");
const imu = @import("imu.zig");
const config = @import("config.zig");
const recording = @import("recording.zig");
const exif = @import("exif.zig");
const datetime = @import("datetime.zig");
const system = @import("system.zig");

const web = @import("zhp");
const mutex = @import("std").Thread.Mutex;

pub const GnssContext = struct {
    gnss: *gnss.GNSS,
    led: led_driver.LP50xx,
    interval: u16 = 1000,
    config: config.Gnss,

    trace_dir: []const u8,
    allocator: *std.mem.Allocator,
};

pub const ImuContext = struct {
    imu: *imu.IMU,
    interval: u16 = 100,

    trace_dir: []const u8,
    allocator: *std.mem.Allocator,

    pub fn latest(self: *ImuContext) imu.Sample {
        var data = self.imu.latest();
        data.age = system.timestamp() - data.received_at;
        return data;
    }

    pub fn history(self: *ImuContext) []const imu.Sample {
        return self.imu.fifo.readableSlice(0);
    }
};

pub const AppContext = struct {
    config: config.Api,
    app: *web.Application,
};

pub var gnss_ctx: GnssContext = undefined;
pub var imu_ctx: ImuContext = undefined;

pub const HeartBeatContext = struct {
    idx: u8 = 0,
    on: u32 = 100,
    off: u32 = 900,
    color: [3]u8 = [_]u8{ 255, 255, 255 },
    led: led_driver.LP50xx,
};

pub var rec_ctx: RecordingContext = undefined;
pub var brdg_cfg_ctx: BridgeCfgContext = undefined;

pub const BridgeCfgContext = struct {
    cfg_server: *std.net.StreamServer,
    cfg_lock: mutex,
    cfg_ready: bool,
    cfg_data: std.ArrayList(u8),
};

pub const RecordingContext = struct {
    config: config.Recording,
    allocator: *std.mem.Allocator,
    server: *std.net.StreamServer,
    stop: std.atomic.Atomic(bool),
    last_file: [28]u8 = [_]u8{'0'} ** 28,
    gnss: GnssContext,
};

pub const CameraContext = struct {
    config: config.Camera,
    socket: []const u8,
};

pub var camera_ctx: CameraContext = undefined;
pub var configuration: config.Config = undefined;

const JPEG_SOI = [_]u8{ 0xFF, 0xD8 };
const JPEG_EOI = [_]u8{ 0xFF, 0xD9 };

const PUB = "PUB ";
const EOL = "\r\n";

const HELLO = "{\"Heartbeat\":\"Hello!\"}";

const SLEEP = std.time.ns_per_ms * 1000;

pub var use_fake_pvt = false;

fn find_som(buffer: []const u8, start: usize, end: usize) ?usize {
    return std.mem.indexOf(u8, buffer[start..end], PUB[0..]);
}

pub fn bridge_cfg_thread(ctx: *BridgeCfgContext) void {
    while (true) {
        const conn = ctx.cfg_server.accept() catch |err| {
            std.log.err("WRITE | server accept | ERR {}", .{err});
            continue;
        };
        defer conn.stream.close();
        
        //Perform initial configuration send-over
        if(ctx.cfg_lock.tryAcquire()) |held| {
            defer held.release();
            ctx.cfg_ready = true;
            jsonify_cfg_data(ctx) catch |err| {
                std.log.err("config: send failed: {s}", .{err});
                ctx.cfg_ready = false;
            };
        }
        
        var sendover = async handle_cfg_bridge(ctx, conn);
        await sendover;
    }
}

pub fn jsonify_cfg_data(ctx: *BridgeCfgContext) !void {
    try std.json.stringify(configuration.data(), .{}, ctx.cfg_data.writer());
}

fn handle_cfg_bridge(ctx: *BridgeCfgContext, conn: std.net.StreamServer.Connection) void {
    var doDelay = false;
    var sendHeartbeat: u8 = 3;
    while(true) {
        //Check our bridge context for more data if it exists 
        if(ctx.cfg_lock.tryAcquire()) |held| {
            defer held.release();
            if(ctx.cfg_ready){
                const data_len = conn.stream.writer().write(ctx.cfg_data.items) catch |err| {
                    std.log.err("CFG_WRITE | ERR {}", .{err});
                    break;
                };
                std.log.info("Writing {} bytes over config.", .{data_len});
                std.log.info("{s}", .{ctx.cfg_data.items});
                ctx.cfg_ready = false;
                ctx.cfg_data.clearRetainingCapacity();
            }
            else{
                doDelay = true;
            }
        }
        else{
            doDelay = true;
        }
        if(doDelay){
            std.time.sleep(SLEEP);
            doDelay = false;
            //sendHeartbeat -= 1;
        }
        
        //if(sendHeartbeat == 0){
        //    sendHeartbeat = 3;
        //    const data_len = conn.stream.writer().write(HELLO) catch |err| {
        //        std.log.err("CFG_WRITE | ERR {}", .{err});
        //        break;
        //    };
        //    ctx.cfg_data.clearRetainingCapacity();
        //}
        
    }
}

pub fn recording_server_thread(ctx: *RecordingContext) void {
    while (true) {
        const conn = ctx.server.accept() catch |err| {
            std.log.err("REC | server accept | ERR {}", .{err});
            continue;
        };
        std.log.info("REC | client connected", .{});
        defer conn.stream.close();

        var frame = async handle_connection(ctx, conn);
        await frame;
    }
}

fn handle_connection(ctx: *RecordingContext, conn: std.net.StreamServer.Connection) void {
    // Create large buffer for collecting image data into
    // Should be larger than the largest image size we expect
    var buffer: [1024 * 1024 * 4]u8 = undefined;
    std.mem.set(u8, buffer[0..], 0);

    // Buffer for incoming UDP data (should be larger than max MTU size)
    var incoming: [100000]u8 = undefined;

    // Current position within the buffer to write into
    var head: usize = 0;

    // Current position within the buffer to read from
    var read: usize = 0;

    // Positions within the image buffer where JPEG start / end bytes are
    var idx_start: usize = 0;
    var idx_end: usize = 0;

    // Flags for JPEG SOF / EOF
    var found_start: bool = false;
    var found_end: bool = false;

    // TODO : detect last frame count on disk and start above that number
    var frame_count: usize = 0;

    // Error recovery flag -- when non-null the buffers and flags will be reset
    var reset_to: ?usize = null;

    var topic_name: ?[]const u8 = null;
    var message_size: usize = 0;

    while (true) {
        if (reset_to != null) {
            head = reset_to.?;
            read = 0;

            frame_count += 1;
            found_start = false;
            found_end = false;

            reset_to = null;
        }

        // std.log.info("SOCK READ SIZE {}", .{ctx.sock.getReadBufferSize()});

        const data_len = conn.stream.reader().read(incoming[0..]) catch |err| {
            std.log.err("REC RECV | ERR {}", .{err});
            continue;
        };

        if (data_len == 0) {
            std.log.info("REC RECV | client disconnected", .{});
            break;
        }

        // std.log.info("head {} data_len {}", .{ head, data_len });

        if (head + data_len > buffer.len) {
            std.log.err("REC RECV | buffer will overflow. reset.", .{});
            std.mem.set(u8, incoming[0..], 0);
            reset_to = 0;
            continue;
        }

        std.mem.copy(u8, buffer[head .. head + data_len], incoming[0..data_len]);

        if (!found_start) {

            // Using read as start of search space as PUB might be split across UDP packets
            // and head.. might skip over start of sequence.

            // if (std.mem.indexOf(u8, buffer[read..head+data_len], PUB[0..])) |idx_msg_start| 
            if (find_som(&buffer, read, head + data_len)) |idx_msg_start| {
                const idx_topic_start = idx_msg_start + PUB.len;

                if (std.mem.indexOf(u8, buffer[idx_topic_start .. idx_topic_start + 20], " ")) |idx_space| {
                    topic_name = buffer[idx_topic_start .. idx_topic_start + idx_space];

                    // std.log.info(". idx_msg_start {}", .{idx_msg_start});
                    // std.log.info(". idx_space {}", .{idx_space});
                    // std.log.info(". topic {s}", .{topic_name});

                    if (std.mem.indexOf(u8, buffer[idx_topic_start .. idx_topic_start + 20], "\r\n")) |idx_eol| {
                        if (std.fmt.parseInt(usize, buffer[idx_topic_start + idx_space + 1 .. idx_topic_start + idx_eol], 10)) |value| {
                            message_size = value;
                            // std.log.info(". size {}", .{message_size});

                            // Advance read to end of PUB line (including line break)
                            read += idx_topic_start + idx_eol + 2;

                            if (buffer[read] == JPEG_SOI[0] and buffer[read + 1] == JPEG_SOI[1]) {
                                idx_start = read;
                                // std.log.info(". idx_start {}", .{idx_start});
                                found_start = true;
                            } else {
                                std.log.err("REC RECV | did not find start of JPEG data at {}", .{read});
                                reset_to = 0;
                                continue;
                            }
                        } else |err| {
                            std.log.err("REC RECV | cannot parse message size : {s}", .{buffer[idx_topic_start + idx_space + 1 .. idx_topic_start + idx_eol]});
                            reset_to = 0;
                            continue;
                        }
                    }
                }
            }
        }

        // std.log.info("acculum {} to {}", .{ read, head + data_len });

        if (found_start and !found_end and (head + data_len - read) >= message_size + 2) {
            // std.log.info("buffer start {s}", .{std.fmt.fmtSliceHexUpper(buffer[0 .. read + 20])});
            // std.log.info("buffer end {s}", .{std.fmt.fmtSliceHexUpper(buffer[read + message_size - 4 .. read + message_size + 4])});

            // Check for the EOL bytes and for the valid end of a JPEG frame
            if (buffer[read + message_size - 2] == JPEG_EOI[0] and
                buffer[read + message_size - 1] == JPEG_EOI[1] and
                buffer[read + message_size] == EOL[0] and
                buffer[read + message_size + 1] == EOL[1])
            {
                idx_end = read + message_size;
                // std.log.info("idx_end {}", .{idx_end});
                found_end = true;
            } else {
                if (std.mem.indexOf(u8, buffer[idx_start..], PUB[0..])) |value| {
                    std.log.info("REC RECV | NO EOL. FOUND PUB at {}", .{idx_start + value});

                    const length: usize = head + data_len - (value + idx_start);
                    std.mem.copy(u8, buffer[0..length], buffer[idx_start + value .. head + data_len]);
                    std.mem.set(u8, buffer[length..], 0);

                    reset_to = length;
                    continue;
                } else {
                    std.log.info("REC RECV | NO EOL | HARD RESET", .{});
                    std.mem.set(u8, buffer[0..], 0);

                    reset_to = 0;
                    continue;
                }
            }
        }

        head += data_len;

        if (found_start and found_end) {
            if(use_fake_pvt){
                var pvt = gnss.fake_pvt;
                ctx.gnss.led.set(0, [_]u8{ 0, 255, 0 }); // TODO : better access method for recording LED
                write_image(ctx, buffer[idx_start..idx_end], pvt, 0, true) catch |err| {
                    std.log.err("REC RECV | could not write image : {}", .{err});
                    reset_to = 0;
                    continue;
                };
                if(ctx.config.write_aux) {
                    write_image(ctx, buffer[idx_start..idx_end], pvt, 1, true) catch |err| {
                    std.log.err("REC RECV | could not write image : {}", .{err});
                    reset_to = 0;
                    continue;
                    };                    
                }
            } else {
                var pvt = ctx.gnss.gnss.last_nav_pvt();
                if (pvt) |value| {
                    if (value.fix_type > 0) {
                        // std.log.info("REC RECV | Frame {} is {}", .{ frame_count, idx_end - idx_start });
                        ctx.gnss.led.set(0, [_]u8{ 0, 255, 0 }); // TODO : better access method for recording LED

                        write_image(ctx, buffer[idx_start..idx_end], value, 0, false) catch |err| {
                            std.log.err("REC RECV | could not write image : {}", .{err});
                            reset_to = 0;
                            continue;
                        };
                        if(ctx.config.write_aux) {
                            write_image(ctx, buffer[idx_start..idx_end], value, 1, false) catch |err| {
                            std.log.err("REC RECV | could not write image : {}", .{err});
                            reset_to = 0;
                            continue;
                            };                    
                        }
                    } else {
                        std.log.info("REC SKIP | Frame {} is {}", .{ frame_count, idx_end - idx_start });
                        ctx.gnss.led.set(0, [_]u8{ 255, 127, 0 }); // TODO : better access method for recording LED
                    }
                }
            }
            
            // Copy any partial data we have to the start of the acculumation buffer
            if (idx_end + 2 < head) {
                std.log.info("REC RECV | copy tail bytes : {} {}", .{ idx_end, head });
                std.log.info("CUR CPU TIME: {}", .{std.time.nanoTimestamp()});
                std.mem.copy(u8, buffer[0 .. head - idx_end - 2], buffer[idx_end + 2 .. head]);
                reset_to = head - idx_end - 2;
            } else {
                reset_to = 0;
            }
        }
    }
}

fn alloc_filename(ctx: *RecordingContext, timestamp: ?[24]u8, dirType: u8) ![]u8 {
    var dirStr: []const u8 = undefined;
    if (dirType == 0){
        dirStr = ctx.config.dir;
    } else {
        dirStr = ctx.config.dirS;
    }

    const temp = try std.fmt.allocPrint(ctx.allocator, "{s}/{s}.jpg", .{ dirStr, timestamp });
    defer ctx.allocator.free(temp);
    const filename = try ctx.allocator.alloc(u8, temp.len);
    _ = std.mem.replace(u8, temp, ":", "-", filename[0..]);
    return filename;
}

fn get_fake_frametime(pvt: gnss.PVT) [24]u8 {
    var timestamp: [24]u8 = undefined;
    const default = "1970-01-01T00-00-00.000Z";
    std.mem.copy(u8, timestamp[0..], default[0..]);  

    const age_sec = std.time.timestamp();
    const age_nsec = std.time.nanoTimestamp() - (age_sec * std.time.ns_per_s);
    
    var stamp = datetime.Datetime.now();
    
    stamp = stamp.shift(datetime.Datetime.Delta{ .seconds = age_sec, .nanoseconds = @intCast(i32, age_nsec) });

    _ = std.fmt.bufPrint(&timestamp, "{d:0>4}-{d:0>2}-{d:0>2}T{d:0>2}:{d:0>2}:{d:0>6.3}Z", .{
        stamp.date.year,
        stamp.date.month,
        stamp.date.day,
        stamp.time.hour,
        stamp.time.minute,
        @intToFloat(f64, stamp.time.second) + @intToFloat(f64, stamp.time.nanosecond) * 1e-9,
    }) catch |err| {};

    return timestamp;
}


fn determine_frametime(pvt: gnss.PVT) [24]u8 {
    var timestamp: [24]u8 = undefined;
    const default = "0000-00-00T00-00-00.000Z";
    std.mem.copy(u8, timestamp[0..], default[0..]);

    const t = pvt.time;
    const age_sec: i64 = @divFloor(pvt.age, 1000);
    const age_nsec: i64 = (pvt.age - age_sec * 1000) * std.time.ns_per_ms;

    // U-Blox module can report negative nanoseconds (e.g. times just before a second boundary)
    //
    // The datetime library can't handle this, so we just set nanoseconds to zero when it occurs.
    // The alternative would be to add 1 sec to 'nanosecond' and subtract 1 sec from 'seconds', but
    // there is the slight change that this would occur on the 0th second of a minute, which would
    // cause a cascading rollback on the minute field, etc.
    //
    // Rounding up to zero is acceptable due to the small time scales here.  An instance of this occuring:
    // t.second = 13, t.nanosecond = -116172 => 12.999883828 seconds
    // When this value is rounded to 3 decimal places (for the filename), it would be 13.000
    var t_nanosecond = t.nanosecond;
    if (t_nanosecond < 0) {
        t_nanosecond = 0;
    }

    var stamp = datetime.Datetime.create(t.year, t.month, t.day, t.hour, t.minute, @intCast(u32, t.second), @intCast(u32, t_nanosecond)) catch |err| {
        std.log.info("REC RECV | Error creating datetime", .{});
        return timestamp;
    };

    stamp = stamp.shift(datetime.Datetime.Delta{ .seconds = age_sec, .nanoseconds = @intCast(i32, age_nsec) });

    _ = std.fmt.bufPrint(&timestamp, "{d:0>4}-{d:0>2}-{d:0>2}T{d:0>2}:{d:0>2}:{d:0>6.3}Z", .{
        stamp.date.year,
        stamp.date.month,
        stamp.date.day,
        stamp.time.hour,
        stamp.time.minute,
        @intToFloat(f64, stamp.time.second) + @intToFloat(f64, stamp.time.nanosecond) * 1e-9,
    }) catch |err| {};

    return timestamp;
}

fn write_image(ctx: *RecordingContext, buffer: []const u8, pvt: gnss.PVT, dirType: u8, useFakeTS: bool) !void {
    var timestamp : [24]u8 = undefined;
    if(useFakeTS){
        timestamp = get_fake_frametime(pvt);
    } else {
        timestamp = determine_frametime(pvt);
    }
     
    // std.log.info("FNAME | GNSS {s} + {d:.3} -> {s}", .{ pvt.timestamp, @intToFloat(f64, pvt.age) / 1000.0, timestamp });
    //std.log.info("REC RECV | Frame {s} is {} bytes", .{ timestamp, buffer.len });

    var filename = try alloc_filename(ctx, timestamp, dirType);
    defer ctx.allocator.free(filename);

    var file = try std.fs.cwd().createFile(filename, .{});
    defer file.close();

    var buf_stream = std.io.bufferedWriter(file.writer());
    const st = buf_stream.writer();

    var exif_tags = exif.init();
    exif_tags.set_gnss(pvt);
    exif_tags.set_frametime(timestamp);

    if (exif_tags.bytes()) |exif_array| {
        const exif_len = exif_array.len + 2;
        const exif_buffer = exif_array.constSlice();

        // std.log.info("EXIF | [{d}]     {s}", .{ exif_buffer.len, std.fmt.fmtSliceHexUpper(exif_buffer) });

        try st.writeAll(JPEG_SOI[0..]);
        try st.writeAll(exif.APP0_HEADER[0..]);
        try st.writeAll(exif.MARK_APP1[0..]);

        try st.writeByte(@truncate(u8, exif_len >> 8));
        try st.writeByte(@truncate(u8, exif_len));

        try st.writeAll(exif_buffer);

        try st.writeAll(buffer[exif.image_offset..]);
    } else {
        // No EXIF data, so just write out the image part of the buffer -- it is valid JPEG data.
        std.log.info("EXIF embedding failed", .{});
        try st.writeAll(buffer[0..]);
    }

    try buf_stream.flush();

    // Save the last filename so the web API can serve that data
    // TODO : don't use fs here and instead keep frame in memory for use by the web API -- this will remove disk flush issues
    std.mem.copy(u8, ctx.last_file[0..], filename[filename.len - 28 ..]);
}

pub fn recording_cleanup_thread(ctx: RecordingContext) void {
    const sleep_ns = @intCast(u64, ctx.config.cleanup_period) * std.time.ns_per_s;

    const path = ctx.config.dir;

    if (std.fs.openDirAbsolute(path, .{ .iterate = true, .no_follow = false })) |dir| {} else |err| switch (err) {
        error.FileNotFound => {
            std.log.info("recording directory {s} does not exists, creating folder\n", .{path});
            if (std.fs.makeDirAbsolute(path)) {} else |mkerr| {
                std.log.warn("[{any}] when creating recording directory {s}\n", .{ mkerr, path });
            }
        },
        else => {
            std.log.warn("[{any}] when testin recording directory {s}\n", .{ err, path });
        },
    }

    while (true) {
        recording.directory_cleanup(ctx);

        // Over time we'll drift behind desired cleanup_period, due to time it takes
        // to do the cleanup -- but that is fine.  And, it's possible that the recording
        // directory has so many files that the it takes longer than cleanup_period
        // to scan and cleanup.  In that case, we still want to wait cleanup_period
        // before the next scan -- not start a new scan immediately.
        std.time.sleep(sleep_ns);
    }
}

pub fn heartbeat_thread(ctx: HeartBeatContext) void {
    while (true) {
        // ctx.led.set(ctx.idx, ctx.color);
        std.time.sleep(ctx.on * std.time.ns_per_ms);

        // ctx.led.set(ctx.idx, [_]u8{ 0, 0, 0 });
        std.time.sleep(ctx.off * std.time.ns_per_ms);
    }
}

pub fn gnss_thread(ctx: GnssContext) void {
    ctx.gnss.set_timeout(ctx.interval + 50);

    var last_debug_ms = std.time.milliTimestamp();
    var last_debug_pvt_ms = std.time.milliTimestamp();
    var last_debug: i8 = -1;

    const debug_interval: usize = ctx.config.debug_period * 1000;
    const debug_interval_pvt: usize = ctx.config.debug_period_pvt * 1000;
    const slog = std.log.scoped(.gnss);

    var trace = recording.TraceLog(gnss.PVT).init(ctx.allocator, ctx.trace_dir, recording.TraceLogType.GNSS);
    var record: bool = false;

    while (true) {
        const this_ms = std.time.milliTimestamp();

        if (debug_interval > 0 and this_ms - last_debug_ms > debug_interval) {
            last_debug += 1;
            ctx.gnss.set_next_timeout(2000);

            if (last_debug == 0) {
                ctx.gnss.get_mon_rf();
            } else if (last_debug == 1) {
                ctx.gnss.get_mon_span();
            } else if (last_debug == 2) {
                ctx.gnss.get_nav_sat();
                last_debug = -1;
            }

            last_debug_ms = this_ms;
        }

        if (ctx.gnss.poll_pvt()) {
            if (ctx.gnss.last_nav_pvt()) |pvt| {
                if (pvt.fix_type == 0) {
                    // If no position fix, color LED orange
                    ctx.led.set(1, [_]u8{ 255, 127, 0 });
                } else {
                    // If there is a fix, color LED green
                    ctx.led.set(1, [_]u8{ 0, 255, 0 });

                    if (record == false) {
                        slog.info("starting GNSS log {s}", .{pvt.timestamp});
                        trace.setTimestamp(pvt.timestamp);
                        record = true;
                    }

                    if (system.state.gnss_has_locked == false) {
                        system.state.gnssInitLockAt(pvt.received_at, pvt.timestamp);
                    }
                }

                if (debug_interval_pvt > 0 and this_ms - last_debug_pvt_ms > debug_interval_pvt) {
                    last_debug_pvt_ms = this_ms;

                    slog.info("{s} {any}", .{ pvt.timestamp, pvt });
                }

                //print("PVT {s} at ({d:.6},{d:.6}) height {d:.2} dop {d:.2}", .{ pvt.timestamp, pvt.latitude, pvt.longitude, pvt.height, pvt.dop });
                //print(" heading {d:.2} velocity ({d:.2},{d:.2},{d:.2}) speed {d:.2}", .{ pvt.heading, pvt.velocity[0], pvt.velocity[1], pvt.velocity[2], pvt.speed });
                //print(" fix {d} sat {} flags {} {} {}\n", .{ pvt.fix_type, pvt.satellite_count, pvt.flags[0], pvt.flags[1], pvt.flags[2] });

                if (record) {
                    trace.append(pvt);
                }
            }
        } else {
            // If no communications, color LED red
            ctx.led.set(1, [_]u8{ 255, 0, 0 });
        }

        // std.time.sleep(std.time.ns_per_ms * @intCast(u64, ctx.interval / 4));
    }
}

pub fn imu_thread(ctx: ImuContext) void {
    var trace = recording.TraceLog(imu.Sample).init(ctx.allocator, ctx.trace_dir, recording.TraceLogType.IMU);
    const slog = std.log.scoped(.imu);

    while (true) {
        const this_ms = std.time.milliTimestamp();
        const data = ctx.imu.poll();

        if (system.state.gnss_has_locked) {
            if (trace.timestamp_needed) {
                var buf: [24]u8 = undefined;
                _ = system.state.isoBuf(buf[0..]) catch |err| {
                    slog.warn("state.isoBuf failed", .{});
                };

                slog.info("starting IMU log {s}", .{buf});
                trace.setTimestamp(buf);
            }

            trace.append(data);
        }

        const duration = std.time.milliTimestamp() - this_ms;
        std.time.sleep(std.time.ns_per_ms * @intCast(u64, ctx.interval - duration));
    }
}

pub fn app_thread(ctx: AppContext) void {
    defer ctx.app.deinit();

    ctx.app.listen("0.0.0.0", ctx.config.port) catch |err| {
        print("app : could not open server port {d}\n", .{ctx.config.port});

        ctx.app.listen("0.0.0.0", 5000) catch |err_fallback| {
            print("app : could not open fallback server port\n", .{});
            return;
        };
    };

    ctx.app.start() catch |err| {
        print("app : could not start\n", .{});
        return;
    };
}
