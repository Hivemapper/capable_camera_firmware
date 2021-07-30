const std = @import("std");
const print = @import("std").debug.print;

const led_driver = @import("led_driver.zig");
const gnss = @import("gnss.zig");
const web = @import("zhp");

pub const GnssContext = struct {
    gnss: *gnss.GNSS,
    led: led_driver.LP50xx,
    timeout: u16 = 1000,
};

pub const AppContext = struct {
    app: *web.Application,
};

pub var gnss_ctx: GnssContext = undefined;

pub const HeartBeatContext = struct {
    idx: u8 = 0,
    on: u32 = 100,
    off: u32 = 900,
    color: [3]u8 = [_]u8{ 255, 255, 255 },
    led: led_driver.LP50xx,
};

pub fn heartbeat_thread(ctx: HeartBeatContext) void {
    while (true) {
        ctx.led.set(ctx.idx, ctx.color);
        std.time.sleep(ctx.on * std.time.ns_per_ms);

        ctx.led.set(ctx.idx, [_]u8{ 0, 0, 0 });
        std.time.sleep(ctx.off * std.time.ns_per_ms);
    }
}

pub fn gnss_thread(ctx: GnssContext) void {
    while (true) {
        ctx.gnss.set_next_timeout(ctx.timeout);

        if (ctx.gnss.get_pvt()) {
            ctx.led.set(0, [_]u8{ 0, 255, 0 });

            if (ctx.gnss.last_nav_pvt()) |pvt| {
                print("PVT {s} at ({d:.6},{d:.6}) height {d:.2}", .{ pvt.timestamp, pvt.latitude, pvt.longitude, pvt.height });
                print(" heading {d:.2} velocity ({d:.2},{d:.2},{d:.2}) speed {d:.2}", .{ pvt.heading, pvt.velocity[0], pvt.velocity[1], pvt.velocity[2], pvt.speed });
                print(" fix {d} sat {} flags {} {} {}\n", .{ pvt.fix_type, pvt.satellite_count, pvt.flags[0], pvt.flags[1], pvt.flags[2] });
            }
        } else {
            ctx.led.set(0, [_]u8{ 255, 0, 0 });
        }

        std.time.sleep(std.time.ns_per_ms * @intCast(u64, ctx.timeout / 2));
    }
}

pub fn app_thread(ctx: AppContext) void {
    defer ctx.app.deinit();

    ctx.app.listen("0.0.0.0", 5000) catch |err| {
        print("app : could not open server port\n", .{});
        return;
    };

    ctx.app.start() catch |err| {
        print("app : could not start\n", .{});
        return;
    };
}
