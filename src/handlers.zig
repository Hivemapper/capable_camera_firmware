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

const web = @import("zhp");

const files = @import("handlers/files.zig");
const status = @import("handlers/status.zig");
const info = @import("handlers/info.zig");
const imu = @import("handlers/imu.zig");
const configure = @import("handlers/configure.zig");

const fileAPI = @import("handlers/fileAPI.zig");
const threads = @import("threads.zig");

const ISO_DATETIME_REGEX = "\\d{4}-\\d{2}-\\d{2}T\\d{2}:\\d{2}:\\d{2}.\\d{3}Z";

pub const routes = [_]web.Route{
    web.Route.create("", "/", MainHandler),
    web.Route.create("api", "/api", MainHandler),
    web.Route.create("api", "/api/", MainHandler),

    web.Route.create("api", "/api/1/start_stream", configure.PreviewHandler),
    web.Route.create("api", "/api/1/stop_stream",  configure.StopPreviewHandler),
    
    web.Route.create("api", "/api/1", MainHandler),
    web.Route.create("api", "/api/1/", MainHandler),

    web.Route.create("api", "/api/config/img", configure.ImgCfgHandler),
    web.Route.create("get-device-status", "/api/status", status.Handler),
    web.Route.create("get-device-information", "/api/1/info", info.Handler),

    web.Route.create("list-imu", "/api/1/imu", files.ImuIndexHandler),
    web.Route.create("list-imu", "/api/1/imu/", files.ImuIndexHandler),
    web.Route.create("get-last-imu", "/api/1/imu/sample", imu.Sample),
    web.Route.create("get-recent-imu", "/api/1/imu/recent", imu.Recent),
    web.Route.create("get-imu-by-name", "/api/1/imu/(" ++ ISO_DATETIME_REGEX ++ ".imu)", files.AuxFileHandler),

    web.Route.create("list-recordings", "/api/1/recordings", files.RecordingIndexHandler),
    web.Route.create("list-recordings", "/api/1/recordings/", files.RecordingIndexHandler),
    web.Route.create("get-last-recording", "/api/1/recordings/last.jpg", files.RecordingLastHandler),
    web.Route.create("get-recordings-by-name", "/api/1/recordings/(.+)", files.RecordingFileHandler),

    web.Route.create("list-gnss", "/api/1/gnss", files.GnssIndexHandler),
    web.Route.create("list-gnss", "/api/1/gnss/", files.GnssIndexHandler),
    web.Route.create("get-last-gnss", "/api/1/gnss/sample", GnssPvtHandler),
    web.Route.create("get-gnss-by-name", "/api/1/gnss/(" ++ ISO_DATETIME_REGEX ++ ".gps)", files.AuxFileHandler),

    web.Route.static("static", "/static/", "static/"),
};

pub const MainHandler = struct {
    pub fn get(self: *MainHandler, request: *web.Request, response: *web.Response) !void {
        try response.headers.append("Content-Type", "text/plain");
        try response.stream.writeAll("");
    }
};

pub const GnssPvtHandler = struct {
    pub fn get(self: *GnssPvtHandler, request: *web.Request, response: *web.Response) !void {
        try response.headers.append("Content-Type", "application/json");

        if (threads.gnss_ctx.gnss.last_nav_pvt()) |pvt| {
            try std.json.stringify(pvt, std.json.StringifyOptions{
                .whitespace = .{ .indent = .{ .Space = 2 } },
            }, response.stream);
        }
    }
};
