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

const imgCfg = @import("cfg/camParamBase.zig");

pub const Api = struct {
    port: u16 = 5000,
};

pub const Gnss = struct {
    debug_period: u16 = 0,
    reset_on_start: bool = true,
};

pub const Recording = struct {
    dir: []const u8 = "/tmp/recording",
    max_size: u64 = 100, // MB
    cleanup_period: u16 = 10, // seconds
    socket: []const u8 = "/tmp/bridge.sock",
};

//Note that the JSON parser currently doesn't support
//processing enums by their string value, although this
//is an open issue that may be integrated in the future:
//see https://github.com/ziglang/zig/issues/9451
pub const Codec_enum = enum { mjpeg, h264 };

pub const Camera = struct {
    fps: u8 = 10,
    width: u16 = 4056,
    height: u16 = 2016,
    quality: u8 = 50,
    //codec: Codec = Codec.mjpeg,
    codec: []const u8 = "mjpeg",
};

pub const Context = struct {
    api: Api = Api{},
    recording: Recording = Recording{},
    camera: Camera = Camera{},
    gnss: Gnss = Gnss{},
};

pub const Config = struct {

    ctx: Context = Context{},
    allocator: *std.mem.Allocator,


    pub fn load(allocator: *mem.Allocator) Config {
        const max_size = 1024 * 1024;

        const input_file = std.fs.cwd().openFile("config.json", .{}) catch |err| {
          std.log.err("config: failed to open config file\n", .{});
          return Config{.ctx = Context{}, .allocator = allocator};
        };

        const input = input_file.readToEndAlloc(
            allocator,
            max_size,
        ) catch |err| switch (err) {
            error.FileTooBig => {
                std.log.err("config: file too large\n", .{});
                return Config{.ctx = Context{}, .allocator = allocator};
            },
            else => {
                std.log.err("config: file read error\n", .{});
                return Config{.ctx = Context{}, .allocator = allocator};
            },
        };
        defer allocator.free(input);

        var tokens = std.json.TokenStream.init(input);

        var new_ctx = std.json.parse(Context, &tokens, std.json.ParseOptions{
            .allocator = allocator,
            .ignore_unknown_fields = true,
            .allow_trailing_data = true,
        }) catch |err| {
            std.log.err("config: failed to parse config file : {any}\n", .{err});
            return Config{.ctx = Context{}, .allocator = allocator};
        };

        std.log.info("config: {any}", .{new_ctx});

        // Code below allows a second config.json file inside the recording directory to be detected
        // and used to update the configuration object before software starts.
        //
        // Configuration hierarchy is therefore:
        //
        // - Defaults values in structs (above) can be update by:
        // - Contents of config.json delievered alongside firmware, which can be update by:
        // - Contents of config.json inside the target recording folder

        const patch_path = std.fs.path.join(allocator, &[_][]const u8{ new_ctx.recording.dir, "config.json" }) catch |err| {
            std.log.info("config: could not create overrider file path\n", .{});
            return Config{.ctx = new_ctx, .allocator = allocator};
        };

        const patch_file = fs.openFileAbsolute(patch_path, .{ .read = true }) catch |err| {
            std.log.info("config: no override file found\n", .{});
            return Config{.ctx = new_ctx, .allocator = allocator};
        };

        var parser = std.json.Parser.init(allocator, false);
        defer parser.deinit();
    
        const patch_input = patch_file.readToEndAlloc(
            allocator,
            max_size,
        ) catch |err| switch (err) {
            error.FileTooBig => {
                std.log.err("config: file too large\n", .{});
                return Config{.ctx = Context{}, .allocator = allocator};
            },
            else => {
                std.log.err("config: file read error\n", .{});
                return Config{.ctx = Context{}, .allocator = allocator};
            },
        };
        defer allocator.free(patch_input);
    

        var tree = parser.parse(patch_input) catch |err| {
            std.log.info("config: error parsing override file, returning config\n", .{});
            return Config{.ctx = new_ctx, .allocator = allocator};
        };
        defer tree.deinit();

        // NOTE : implementation below supports the current hierarchy of configuration "group: { key: value }"
        // If the configuration tree goes deeper than the current parent.child nesting, this code will have to
        // be refactored to support additional nesting depth.

        // inline required for this is unrolled at compliation time
        inline for (@typeInfo(Context).Struct.fields) |root_field| {

            // Check if override config file has root keys which match valid config fields
            if (tree.root.Object.get(root_field.name)) |root_node| {
    
                // Get the config field which we are updating
                var object = @field(new_ctx, root_field.name);

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
                                        @field(new_ctx, root_field.name) = object;
                                    },
                                    else => std.log.warn("config: expected bool for {s}", .{child_field.name}),
                                }
                            },
                            u8, u16, u32, u64 => {
                                switch (child_node) {
                                    .Integer => {
                                        @field(object, child_field.name) = @intCast(child_field.field_type, child_node.Integer);
                                        @field(new_ctx, root_field.name) = object;
                                    },
                                    else => std.log.warn("config: expected integer for {s}", .{child_field.name}),
                                }
                            },
                            f32, f64 => {
                                switch (child_node) {
                                    .Integer => {
                                        @field(object, child_field.name) = @floatCast(child_field.field_type, child_node.Float);
                                        @field(config.ctx, root_field.name) = object;
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

        std.log.info("config: {any}", .{new_ctx});

        return Config{.ctx = new_ctx, .allocator = allocator};
    }

    pub fn writeJsonCfg(self: *Config) !void {        
        const output_file = std.fs.cwd().createFile("config.json", .{.read = true}) catch |err| {
            std.log.err("config: failed to open config file\n", .{});
            return err;
        };
    
        defer output_file.close();
        var output_str = std.ArrayList(u8).init(self.allocator);
        defer output_str.deinit();
        try std.json.stringify(self.ctx, .{}, output_str.writer());
        try output_file.writeAll(output_str.items);
    }

    pub fn validateCamCfg(self: *Config, camera: Camera) bool{
        var isGood: bool = true;
        if (camera.width  == 0){ isGood = false; }
        if (camera.height == 0){ isGood = false; }
        if (camera.fps    == 0){ isGood = false; }
        if (camera.width  > 4096){ isGood = false; }
        if (camera.height > 2048){ isGood = false; }
        if (camera.fps    >   30){ isGood = false; }    
    
        if(isGood){
            self.ctx.camera = camera;
        }
        return isGood;
    }

    pub fn updateCameraCfg(self: *Config, reqContent: []const u8) !bool {
        var goodInput = false;
        var contentStream = std.json.TokenStream.init(reqContent);
        var cam_param = try std.json.parse(Camera, &contentStream, .{});
        goodInput = self.validateCamCfg(cam_param);
        if(goodInput){
            try imgCfg.update_bridge_script(imgCfg.fullFilePath,
                                            self.ctx.camera.width,
                                            self.ctx.camera.height,
                                            self.ctx.camera.fps);
            try self.writeJsonCfg();
        }
        return goodInput;
    }
};







