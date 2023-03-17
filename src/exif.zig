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

const gnss = @import("gnss.zig");

var gpa = std.heap.GeneralPurposeAllocator(.{}){};

const c = @cImport({
    @cInclude("libexif/exif-data.h");
});

pub extern "c" fn free(?*c_void) void;

const bounded_array = @import("bounded_array.zig");

const MAX_SIZE: usize = 1024;

pub const APP0_HEADER = [_]u8{ 0xFF, 0xE0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00 };
pub const MARK_APP1 = [_]u8{ 0xFF, 0xE1 };

pub const image_offset: usize = 20; // offset of image in JPEG buffer

fn exif_create_tag(arg_exif: [*c]c.ExifData, arg_ifd: c.ExifIfd, arg_tag: c.ExifTag) callconv(.C) [*c]c.ExifEntry {
    var exif = arg_exif;
    var ifd = @intCast(usize, @enumToInt(arg_ifd));
    var tag = arg_tag;

    var entry = c.exif_content_get_entry(exif.*.ifd[ifd], tag);

    if (entry == null) {
        // Tag does not exist, so we have to create one

        entry = c.exif_entry_new();

        if (entry == null) {
            std.log.err("EXIF | failed to allocate exif memory", .{});
            return null;
        }

        // tag must be set before calling exif_content_add_entry
        entry.*.tag = tag;

        // Attach the ExifEntry to an IFD
        c.exif_content_add_entry(exif.*.ifd[ifd], entry);

        // Allocate memory for the entry and fill with default data
        c.exif_entry_initialize(entry, tag);

        // Ownership of the ExifEntry has now been passed to the IFD.
        c.exif_entry_unref(entry);
    }

    return entry;
}

fn exif_rational(numerator: c.ExifLong, denominator: c.ExifLong) c.ExifRational {
    return c.ExifRational{ .numerator = numerator, .denominator = denominator };
}

fn exif_set_latitude_or_longitude(entry: [*c]c.ExifEntry, byte_order: c.ExifByteOrder, value: f64) void {
    const arcfrac = 1000000;

    const degrees = @floatToInt(c.ExifLong, @fabs(value));
    const remainder = 60.0 * (@fabs(value) - @intToFloat(f64, degrees));
    const minutes = @floatToInt(c.ExifLong, remainder);
    const seconds = @floatToInt(c.ExifLong, 60.0 * (remainder - @intToFloat(f64, minutes)) * arcfrac);

    c.exif_set_rational(entry.*.data, byte_order, exif_rational(degrees, 1));
    c.exif_set_rational(entry.*.data + 1 * @sizeOf(c.ExifRational), byte_order, exif_rational(minutes, 1));
    c.exif_set_rational(entry.*.data + 2 * @sizeOf(c.ExifRational), byte_order, exif_rational(seconds, arcfrac));
}

fn exif_set_string(entry: [*c]c.ExifEntry, s: []const u8) void {
    if (entry.*.data != null) {
        free(@ptrCast(*c_void, entry.*.data));
    }

    entry.*.size = s.len;
    entry.*.components = s.len;

    var cstr = std.cstr.addNullByte(&gpa.allocator, s) catch |err| {
        std.log.err("EXIF | failed to terminate exif string : {any}", .{err});
        return;
    };

    entry.*.data = @ptrCast([*c]u8, cstr);

    if (entry.*.data == null) {
        std.log.err("EXIF | failed to copy exif string", .{});
    }

    entry.*.format = c.ExifFormat.EXIF_FORMAT_ASCII;
}

pub const Exif = struct {
    image_x: usize = 0,
    image_y: usize = 0,
    gnss_nav_pvt: ?gnss.PVT = null,
    capture_timestamp: ?[24]u8 = null,
    byte_order: c.ExifByteOrder = c.ExifByteOrder.EXIF_BYTE_ORDER_MOTOROLA,

    pub fn set_gnss(self: *Exif, nav_pvt: gnss.PVT) void {
        self.gnss_nav_pvt = nav_pvt;
    }

    pub fn set_frametime(self: *Exif, timestamp: [24]u8) void {
        self.capture_timestamp = timestamp;
    }

    pub fn bytes(self: *Exif) ?bounded_array.BoundedArray(u8, MAX_SIZE) {
        var exif: [*c]c.ExifData = c.exif_data_new();

        var entry: [*c]c.ExifEntry = undefined;
        var exif_data: []u8 = undefined;
        var exif_len: usize = 0;

        const ifd_exif = c.ExifIfd.EXIF_IFD_EXIF;
        const ifd_gps = c.ExifIfd.EXIF_IFD_GPS;

        // Create the mandatory EXIF fields with default data
        c.exif_data_fix(exif);
        c.exif_data_set_byte_order(exif, self.byte_order);

        if (self.image_x != 0) {
            entry = exif_create_tag(exif, c.ExifIfd.EXIF_IFD_EXIF, c.ExifTag.EXIF_TAG_PIXEL_X_DIMENSION);
            c.exif_set_long(entry.*.data, self.byte_order, self.image_x);
        }

        if (self.image_y != 0) {
            entry = exif_create_tag(exif, c.ExifIfd.EXIF_IFD_EXIF, c.ExifTag.EXIF_TAG_PIXEL_Y_DIMENSION);
            c.exif_set_long(entry.*.data, self.byte_order, self.image_y);
        }

        // entry = exif_create_tag(exif, ifd, c.ExifTag.EXIF_TAG_MAKE);
        // exif_set_string(entry, "");

        entry = exif_create_tag(exif, ifd_exif, c.ExifTag.EXIF_TAG_MODEL);
        exif_set_string(entry, "Open Dashcam");

        // entry = exif_create_tag(exif, ifd, c.ExifTag.EXIF_TAG_SOFTWARE);
        // exif_set_string(entry, "");

        entry = exif_create_tag(exif, ifd_exif, c.ExifTag.EXIF_TAG_COLOR_SPACE);
        c.exif_set_short(entry.*.data, self.byte_order, 1);

        entry = exif_create_tag(exif, ifd_exif, c.ExifTag.EXIF_TAG_COMPRESSION);
        c.exif_set_short(entry.*.data, self.byte_order, 6);

        // Embed GNSS data if it has been set
        if (self.gnss_nav_pvt) |nav_pvt| {

            // EXIF_TAG_GPS_LATITUDE_REF and EXIF_TAG_INTEROPERABILITY_INDEX both have the value : 0x0001
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_INTEROPERABILITY_INDEX));

            if (nav_pvt.latitude > 0) {
                exif_set_string(entry, "N");
            } else {
                exif_set_string(entry, "S");
            }

            // EXIF_TAG_GPS_LATITUDE and EXIF_TAG_INTEROPERABILITY_VERSION both have the value : 0x0002
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_INTEROPERABILITY_VERSION));
            exif_set_latitude_or_longitude(entry, self.byte_order, nav_pvt.latitude);

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_LONGITUDE_REF));

            if (nav_pvt.longitude > 0) {
                exif_set_string(entry, "E");
            } else {
                exif_set_string(entry, "W");
            }

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_LONGITUDE));
            exif_set_latitude_or_longitude(entry, self.byte_order, nav_pvt.longitude);

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_ALTITUDE_REF));
            if (nav_pvt.height > 0) {
                c.exif_set_short(entry.*.data, self.byte_order, 0);
            } else {
                c.exif_set_short(entry.*.data, self.byte_order, 1);
            }

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_ALTITUDE));
            c.exif_set_rational(entry.*.data, self.byte_order, exif_rational(@floatToInt(c.ExifLong, @fabs(nav_pvt.height) * 1000), 1000));

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_SATELLITES));
            var satellite_count: [2]u8 = undefined;
            _ = std.fmt.bufPrint(&satellite_count, "{d}", .{nav_pvt.satellite_count}) catch unreachable;
            exif_set_string(entry, satellite_count[0..]);

            var datestamp: [10]u8 = undefined;
            _ = std.fmt.bufPrint(&datestamp, "{d:0>4}:{d:0>2}:{d:0>2}", .{ nav_pvt.time.year, nav_pvt.time.month, nav_pvt.time.day }) catch unreachable;
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_DATE_STAMP));
            exif_set_string(entry, datestamp[0..]);

            // Care taken here to round nanoseconds correctly, so that 27.499656311 rounds to 27.50 (scaled with rational denominator of course)
            const second_scale: u8 = 100;
            const second_fraction = @floatToInt(i32, @round(@intToFloat(f64, nav_pvt.time.nanosecond) * @intToFloat(f64, second_scale) * 1e-9));
            const second_value = @as(i32, nav_pvt.time.second) * second_scale + second_fraction;

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_TIME_STAMP));
            c.exif_set_rational(entry.*.data, self.byte_order, exif_rational(nav_pvt.time.hour, 1));
            c.exif_set_rational(entry.*.data + 1 * @sizeOf(c.ExifRational), self.byte_order, exif_rational(nav_pvt.time.minute, 1));
            c.exif_set_rational(entry.*.data + 2 * @sizeOf(c.ExifRational), self.byte_order, exif_rational(@intCast(u32, second_value), second_scale));

            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_SPEED_REF));
            exif_set_string(entry, "K");

            // GNSS module provide speed in m/s, this converts it to km / hour (the reference noted above)
            const speed_scale: u8 = 100;
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_SPEED));
            c.exif_set_rational(entry.*.data, self.byte_order, exif_rational(@floatToInt(u32, @round(nav_pvt.speed * 3.6 * @intToFloat(f32, speed_scale))), speed_scale));

            const track_scale: u8 = 100;
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_TRACK));
            c.exif_set_rational(entry.*.data, self.byte_order, exif_rational(@floatToInt(u32, @round(nav_pvt.heading * @intToFloat(f32, track_scale))), track_scale));

            const dop_scale: u8 = 100;
            entry = exif_create_tag(exif, ifd_gps, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_GPS_DOP));
            c.exif_set_rational(entry.*.data, self.byte_order, exif_rational(@floatToInt(u32, @round(nav_pvt.dop * @intToFloat(f32, dop_scale))), dop_scale));
        }

        if (self.capture_timestamp) |timestamp| {
            var datetime: [19]u8 = undefined;
            var subseconds: [3]u8 = undefined;

            _ = std.fmt.bufPrint(&datetime, "{s} {s}", .{ timestamp[0..10], timestamp[11..19] }) catch unreachable;
            _ = std.fmt.bufPrint(&subseconds, "{s}", .{timestamp[20..23]}) catch unreachable;
            //std.log.info("EXIF | datetime : {s} {s} {s}", .{ timestamp, datetime, subseconds });

            entry = exif_create_tag(exif, ifd_exif, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_DATE_TIME_ORIGINAL));
            exif_set_string(entry, datetime[0..]);

            entry = exif_create_tag(exif, ifd_exif, @ptrCast(c.ExifTag, c.ExifTag.EXIF_TAG_SUB_SEC_TIME_ORIGINAL));
            exif_set_string(entry, subseconds[0..]);
        }

        // Get a pointer to the EXIF data block we just created
        c.exif_data_save_data(exif, @ptrCast([*][*c]u8, &exif_data), &exif_len);

        // std.log.info("EXIF | exif_data {any}", .{exif_data[0..exif_len]});

        var output = bounded_array.BoundedArray(u8, MAX_SIZE).fromSlice(exif_data[0..exif_len]) catch |err| {
            std.log.err("EXIF | could not created BoundedArray : {}", .{err});
            return null;
        };

        std.c.free(@ptrCast(*c_void, exif_data));
        std.c.free(@ptrCast(*c_void, exif));

        return output;
    }
};

pub fn init() Exif {
    return Exif{};
}
