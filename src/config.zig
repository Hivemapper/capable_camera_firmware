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
const mem = @import("std").mem;
const fmt = std.fmt;

const imgCfg = @import("cfg/camParamBase.zig");

const defaultAWBGains = [2]f32{ 1.0, 1.0 };

pub const Api = struct {
    port: u16 = 5000,
};

pub const Gnss = struct {
    debug_period: u16 = 0,
    debug_period_pvt: u16 = 10,
    reset_on_start: bool = true,
};

pub const Connection = struct {
    socket: []const u8 = "/tmp/bridge.sock",
    socketType: []const u8 = "sck://",
    listen: bool = false,
};

pub const Recording = struct {
    dir: []const u8 = "/tmp/recording",
    dirS: []const u8 = "/media/pi/265D-57F2",
    write_aux: bool = false,
    max_size: u64 = 100, // MB
    cleanup_period: u16 = 10, // seconds
    connection: Connection = Connection{},
};

//Note that the JSON parser currently doesn't support
//processing enums by their string value, although this
//is an open issue that may be integrated in the future:
//see https://github.com/ziglang/zig/issues/9451
pub const Codec_enum = enum { mjpeg, h264 };

pub const ColorBalance = struct {
    awb: []const u8 = "normal",
    awbGains: [2]f32 = defaultAWBGains,
    brightness: u32 = 0.0,
    contrast: u32 = 0.0,
    saturation: u64 = 0.0,
};

pub const Exposure = struct {
    exposure: []const u8 = "normal",
    ev: u32 = 0.0,
    fixedGain: u32 = 0.0,
    metering: []const u8 = "centre",
    sharpness: u32 = 0.0,
};

pub const Encoding = struct {
    fps: u8 = 10,
    width: u16 = 4056,
    height: u16 = 2016,
    codec: []const u8 = "mjpeg",
    quality: u8 = 50,    
};

pub const Camera = struct {
    encoding: Encoding = Encoding{},
    colorBalance: ColorBalance = ColorBalance{},
    exposure: Exposure = Exposure{},
};

pub const ConfigData = struct {
    api: Api = Api{},
    recording: Recording = Recording{},
    camera: Camera = Camera{},
    gnss: Gnss = Gnss{},
};

pub const ConfigError = error{Update};
pub const CameraConfigError = error{ FPS, Width, Height, Codec, Quality };

pub const ConfigValidation = struct {
    valid: bool = false,
    message: ?[]const u8 = null,
    err: ?anyerror = null,
};

pub const Config = struct {
    api: Api = Api{},
    recording: Recording = Recording{},
    camera: Camera = Camera{},
    gnss: Gnss = Gnss{},

    cfg_socket: []const u8 = "/tmp/config.sock",

    allocator: *std.mem.Allocator,

    pub fn data(self: *Config) ConfigData {
        return ConfigData{
            .api = self.api,
            .recording = self.recording,
            .camera = self.camera,
            .gnss = self.gnss,
        };
    }

    pub fn load(self: *Config, cfg: ConfigData) void {
        self.api = cfg.api;
        self.recording = cfg.recording;
        self.camera = cfg.camera;
        self.gnss = cfg.gnss;
    }

    pub fn save(self: *Config) !void {
        const output_file = std.fs.cwd().createFile("config.json", .{ .read = true }) catch |err| {
            std.log.err("config: failed to open config file", .{});
            return err;
        };
        defer output_file.close();

        var output_str = std.ArrayList(u8).init(self.allocator);
        defer output_str.deinit();

        try std.json.stringify(self.data(), .{}, output_str.writer());
        try output_file.writeAll(output_str.items);
    }

    pub fn writeBridgeScript(self: *Config, cfg_filename: []const u8) anyerror!void {
        const output_file = try std.fs.cwd().createFile(cfg_filename, .{ .read = true });
        defer output_file.close();

        var execLineBuff: [512]u8 = undefined;

        const execLineSlice1 = execLineBuff[0..256];
        const execLineSlice2 = execLineBuff[256..512];

        const firstExecStr = try fmt.bufPrint(execLineSlice1, imgCfg.execLine1, .{ self.camera.encoding.width, self.camera.encoding.height, self.camera.encoding.fps });

        const secndExecStr = try fmt.bufPrint(execLineSlice2, imgCfg.execLine2, .{ self.camera.colorBalance.awb, self.camera.colorBalance.awbGains[0], self.camera.colorBalance.awbGains[1], self.camera.colorBalance.brightness, self.camera.colorBalance.contrast, self.camera.exposure.exposure, self.camera.exposure.ev, self.camera.exposure.fixedGain, self.camera.exposure.metering, self.camera.colorBalance.saturation, self.camera.exposure.sharpness });

        try output_file.writeAll(imgCfg.scriptLines);
        try output_file.writeAll(firstExecStr);
        try output_file.writeAll(secndExecStr);

        return;
    }

    pub fn validateCamera(self: *Config, camera: Camera) ConfigValidation {
        if (camera.encoding.width <= 0 or camera.encoding.width > 4096) {
            return ConfigValidation{ .err = error.Width };
        }

        if (camera.encoding.height <= 0 or camera.encoding.height > 2048) {
            return ConfigValidation{ .err = error.Height };
        }

        if (camera.encoding.fps <= 0 or camera.encoding.fps > 10) {
            return ConfigValidation{ .err = error.FPS };
        }

        if (camera.encoding.quality <= 0 or camera.encoding.quality > 100) {
            return ConfigValidation{ .err = error.Quality };
        }

        return ConfigValidation{ .valid = true };
    }

    pub fn updateCamera(self: *Config, params: Camera) ConfigValidation {
        var check = self.validateCamera(params);

        if (check.valid) {
            self.camera = params;

            self.writeBridgeScript(imgCfg.filePath) catch |err| {
                std.log.err("config: update_bridge_script failed : {s}", .{err});
                check.err = ConfigError.Update;
                check.message = "Error : update_bridge_script failed";
            };

            self.save() catch |err| {
                std.log.err("config: save failed : {s}", .{err});
                check.err = ConfigError.Update;
                check.message = "Error : config save failed";
            };
        }

        return check;
    }
};

pub fn load(allocator: *mem.Allocator) Config {
    const max_size = 1024 * 1024;

    const input_file = std.fs.cwd().openFile("config.json", .{}) catch |err| {
        std.log.err("config: failed to open config file", .{});
        return Config{ .allocator = allocator };
    };

    const input = input_file.readToEndAlloc(
        allocator,
        max_size,
    ) catch |err| switch (err) {
        error.FileTooBig => {
            std.log.err("config: file too large", .{});
            return Config{ .allocator = allocator };
        },
        else => {
            std.log.err("config: file read error", .{});
            return Config{ .allocator = allocator };
        },
    };
    defer allocator.free(input);

    var tokens = std.json.TokenStream.init(input);

    var data = std.json.parse(ConfigData, &tokens, std.json.ParseOptions{
        .allocator = allocator,
        .ignore_unknown_fields = true,
        .allow_trailing_data = true,
    }) catch |err| {
        std.log.err("config: failed to parse config file : {any}", .{err});
        return Config{ .allocator = allocator };
    };

    std.log.info("config: {any}", .{data});

    var config = Config{ .allocator = allocator };
    config.load(data);

    // Code below allows a second config.json file inside the recording directory to be detected
    // and used to update the configuration object before software starts.
    //
    // Configuration hierarchy is therefore:
    //
    // - Defaults values in structs (above) can be update by:
    // - Contents of config.json delievered alongside firmware, which can be update by:
    // - Contents of config.json inside the target recording folder

    const patch_path = std.fs.path.join(allocator, &[_][]const u8{ config.recording.dir, "config.json" }) catch |err| {
        std.log.info("config: could not create overrider file path", .{});
        return config;
    };

    const patch_file = fs.openFileAbsolute(patch_path, .{ .read = true }) catch |err| {
        std.log.info("config: no override file found", .{});
        return config;
    };

    var parser = std.json.Parser.init(allocator, false);
    defer parser.deinit();

    const patch_input = patch_file.readToEndAlloc(
        allocator,
        max_size,
    ) catch |err| switch (err) {
        error.FileTooBig => {
            std.log.err("config: file too large", .{});
            return config;
        },
        else => {
            std.log.err("config: file read error", .{});
            return config;
        },
    };
    defer allocator.free(patch_input);

    var tree = parser.parse(patch_input) catch |err| {
        std.log.info("config: error parsing override file, returning config", .{});
        return config;
    };
    defer tree.deinit();

    // NOTE : implementation below supports the current hierarchy of configuration "group: { key: value }"
    // If the configuration tree goes deeper than the current parent.child nesting, this code will have to
    // be refactored to support additional nesting depth.

    // inline required for this is unrolled at compliation time
    inline for (@typeInfo(ConfigData).Struct.fields) |root_field| {

        // Check if override config file has root keys which match valid config fields
        if (tree.root.Object.get(root_field.name)) |root_node| {

            // Get the config field which we are updating
            var object = @field(config, root_field.name);

            // inline required for this is unrolled at compliation time
            inline for (@typeInfo(@TypeOf(object)).Struct.fields) |child_field| {

                // Get the child field which we are updating
                if (root_node.Object.get(child_field.name)) |child_node| {

                    // Based on the type of the child field, we need to change how the
                    // JSON value union is acesses, and we may need to cast from the
                    // JSON numeric types (i64, f64) to smaller numeric types used in the
                    // configuration structs.  These nested switch cases handle checking
                    // that the JSON value type and the struct type match.
                    //
                    // Once the child field has been updated, that update object has to
                    // be reassigned back to appropiate field of the parent/root config variable.
                    //
                    // TODO : support string / const u8 values
                    switch (child_field.field_type) {
                        bool => {
                            switch (child_node) {
                                .Bool => {
                                    @field(object, child_field.name) = child_node.Bool;
                                    @field(config, root_field.name) = object;
                                },
                                else => std.log.warn("config: expected bool for {s}", .{child_field.name}),
                            }
                        },
                        u8, u16, u32, u64 => {
                            switch (child_node) {
                                .Integer => {
                                    @field(object, child_field.name) = @intCast(child_field.field_type, child_node.Integer);
                                    @field(config, root_field.name) = object;
                                },
                                else => std.log.warn("config: expected integer for {s}", .{child_field.name}),
                            }
                        },
                        f32, f64 => {
                            switch (child_node) {
                                .Integer => {
                                    @field(object, child_field.name) = @floatCast(child_field.field_type, child_node.Float);
                                    @field(config, root_field.name) = object;
                                },
                                else => std.log.warn("config: expected float for {s}", .{child_field.name}),
                            }
                        },
                        else => std.log.warn("config: override failed for {s} type {any}", .{ child_field.name, child_field.field_type }),
                    }
                }
            }
        }
    }

    std.log.info("config: {any}", .{config});

    return config;
}
