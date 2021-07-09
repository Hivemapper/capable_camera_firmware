const std = @import("std");
const print = @import("std").debug.print;
const fs = std.fs;
const mem = @import("std").mem;

const led_driver = @import("led_driver.zig");
const info = @import("info.zig");

const i2c_device = "/dev/i2c-1";

pub fn main() anyerror!void {
    if (info.uptime()) |uptime| {
        print("uptime {d} {d}\n", .{ uptime[0], uptime[1] });
    }

    if (try info.stat()) |stat| {
        print("stat {any}\n", .{stat});
    }

    var fd = try fs.openFileAbsolute(i2c_device, fs.File.OpenFlags{ .read = true, .write = true });
    defer fd.close();

    const led = led_driver.LP50xx{ .fd = fd };

    if (led.read_register(0x00, 1)) |value| {
        print("CONFIG0 = 0x{s}\n", .{std.fmt.fmtSliceHexUpper(value)});
    }

    if (led.read_register(0x01, 1)) |value| {
        print("CONFIG1 = 0x{s}\n", .{std.fmt.fmtSliceHexUpper(value)});
    }

    led.off();
    led.enable();
    led.set_brightness(0x30);
    led.spin();
}
