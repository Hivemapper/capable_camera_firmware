// Copyright 2022 Chris Niessl for Capable Robot Components, Inc.
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
const mem = std.mem;
const print = std.debug.print;

const web = @import("zhp");

const threads = @import("../threads.zig");
const cfg = @import("../config.zig");

const updateStr: []const u8 = 
\\Updated Imager Parameters:
\\Width  = {},
\\Height = {},
\\FPS    = {}
;

const failStr: []const u8 =
\\Failed to update!
;

pub const ImgCfgHandler = struct {
    
    pub fn post(self: *ImgCfgHandler, 
                request: *web.Request, 
                response: *web.Response) !void {
       try response.headers.append("Content-Type", "text/plain");
        
       var respBuff: [256]u8 = undefined;
       const outputSlice = respBuff[0..];
       var goodInput: bool = true;
     
       var content_type = request.headers.getDefault("Content-Type", "");
       if (std.mem.startsWith(u8, content_type, "application/json")) {
         if (!request.read_finished) {
           if (request.stream) |stream| {
             try request.readBody(stream);
           }
         }
         if (request.content) |content| {
           switch (content.type) {
             .TempFile => {
               return error.NotImplemented; 
               // TODO: Parsing should use a stream
             },
             .Buffer => {
                goodInput = try threads.camera_ctx.ctx.updateCameraCfg(content.data.buffer);
             }
           }
         }     
       }
       if (goodInput){
           const cfg_params = threads.camera_ctx.ctx.ctx.camera;
           var outputStr = try std.fmt.bufPrint(outputSlice, updateStr, 
            .{cfg_params.width, cfg_params.height, cfg_params.fps});
           try response.stream.writeAll(outputStr);
       } else {
           var outputStr = try std.fmt.bufPrint(outputSlice, failStr, .{});
           try response.stream.writeAll(outputStr);       
       }
    }
    
};
