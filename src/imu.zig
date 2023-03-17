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
const slog = std.log.scoped(.imu);
const print = @import("std").debug.print;

const spi = @import("bus/spi.zig");
const system = @import("system.zig");

const SPI_BUFFER_SIZE = 64;
const FIFO_SIZE = 64;

pub fn init(handle: spi.SPI) IMU {
    var obj = IMU{
        .handle = handle,
        .write_buffer = [_]u8{0x0} ** SPI_BUFFER_SIZE,
        .read_buffer = [_]u8{0x0} ** SPI_BUFFER_SIZE,
    };

    obj.reset();

    return obj;
}

pub const RawSample = struct {
    received_at: i64 = 0,
    temperature: i16 = 0,
    accelerometer: [3]i16 = [_]i16{ 0, 0, 0 },
    gyroscope: [3]i16 = [_]i16{ 0, 0, 0 },
};

pub const Sample = struct {
    age: i64 = 0,
    received_at: i64,
    temperature: f32,
    accelerometer: [3]f64,
    gyroscope: [3]f64,
};

const SPI_READ_MASK: u8 = 0x80;
const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_TEMP_DATA1_UI: u8 = 0x1D;
const REG_TEMP_DATA0_UI: u8 = 0x1E;
const REG_ACCEL_DATA_X1_UI: u8 = 0x1F;
const REG_ACCEL_DATA_X0_UI: u8 = 0x20;
const REG_ACCEL_DATA_Y1_UI: u8 = 0x21;
const REG_ACCEL_DATA_Y0_UI: u8 = 0x22;
const REG_ACCEL_DATA_Z1_UI: u8 = 0x23;
const REG_ACCEL_DATA_Z0_UI: u8 = 0x24;
const REG_GYRO_DATA_X1_UI: u8 = 0x25;
const REG_GYRO_DATA_X0_UI: u8 = 0x26;
const REG_GYRO_DATA_Y1_UI: u8 = 0x27;
const REG_GYRO_DATA_Y0_UI: u8 = 0x28;
const REG_GYRO_DATA_Z1_UI: u8 = 0x29;
const REG_GYRO_DATA_Z0_UI: u8 = 0x2A;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;

const SET_TEMPERATURE_DISABLED: u8 = 0x20;

const SET_ACCEL_OFF_MODE: u8 = 0x00;
const SET_ACCEL_LOW_POWER_MODE: u8 = 0x02;
const SET_ACCEL_LOW_NOISE_MODE: u8 = 0x03;

const SET_GYRO_OFF_MODE: u8 = 0x00;
const SET_GYRO_STANDBY_MODE: u8 = 0x04;
const SET_GYRO_LOW_NOISE_MODE: u8 = 0x0C;

const FS_MAX: f64 = 32768.0;

pub const GYRO_FS = enum(u8) {
    DPS_2000,
    DPS_1000,
    DPS_500,
    DPS_250,
    DPS_125,
    DPS_62_5,
    DPS_31_25,
    DPS_16_625,
};

const GYRO_FS_VALUES = [8]f64{ 2000.0, 1000.0, 500.0, 250.0, 125.0, 62.5, 31.25, 16.615 };

pub const ACCEL_FS = enum(u8) { G16, G8, G4, G2 };

const ACCEL_FS_VALUES = [4]f64{ 16.0, 8.0, 4.0, 2.0 };

// Common to both accelerometer and gyroscope sensors
pub const SENSOR_UI_FILTER = enum(u8) { ORD_1, ORD_2, ORD_3 };

// Common to both accelerometer and gyroscope sensors
pub const SENSOR_ODR = enum(u8) {
    KHZ_32,
    KHZ_16,
    KHZ_8,
    KHZ_4,
    KHZ_2,
    KHZ_1,
    HZ_200,
    HZ_100,
    HZ_50,
    HZ_25,
    HZ_12_5,
};

// Common to both accelerometer and gyroscope sensors
pub const SENSOR_UI_FILTER_BW = enum(u8) {
    ORD_2,
    ORD_4,
    ORD_5,
    ORD_8,
    ORD_10,
    ORD_16,
    ODR_20,
    ORD_40,
    LOW_LATENCY_0,
    LOW_LATENCY_1,
};

fn extract(buffer: [SPI_BUFFER_SIZE]u8, comptime T: type, idx: u16) T {
    return std.mem.readIntSliceBig(T, buffer[idx .. idx + @divExact(@typeInfo(T).Int.bits, 8)]);
}

fn convert_temperature(reading: i16) f32 {
    return @intToFloat(f32, reading) / 132.48 + 25.0;
}

pub const IMU = struct {
    handle: spi.SPI,
    write_buffer: [SPI_BUFFER_SIZE]u8,
    read_buffer: [SPI_BUFFER_SIZE]u8,

    accel_fs: f64 = ACCEL_FS_VALUES[0],
    accel_odr: u8 = 0x06,
    gyro_fs: f64 = GYRO_FS_VALUES[0],
    gyro_odr: u8 = 0x06,

    fifo: FifoType = FifoType.init(),
    _last_raw_sample: RawSample = RawSample{},

    const FifoType = std.fifo.LinearFifo(Sample, std.fifo.LinearFifoBufferType{ .Static = FIFO_SIZE });

    fn write_register(self: *IMU, addr: u8, value: u8) void {
        const buf = [_]u8{ addr, value };
        self.write_buffer[0] = addr;
        self.write_buffer[1] = value;
        _ = self.handle.transfer(&self.write_buffer, &self.read_buffer, 2);
        self.write_buffer[1] = 0x0;
    }

    fn read_register(self: *IMU, addr: u8) u8 {
        self.write_buffer[0] = addr | SPI_READ_MASK;
        _ = self.handle.transfer(&self.write_buffer, &self.read_buffer, 2);
        return self.read_buffer[1];
    }

    fn read(self: *IMU, addr: u8, length: u8) void {
        self.write_buffer[0] = addr | SPI_READ_MASK;
        _ = self.handle.transfer(&self.write_buffer, &self.read_buffer, length + 1);
    }

    fn config_delay(self: *IMU) void {
        std.time.sleep(std.time.ns_per_ms * 50);
    }

    fn convert_accelerometer(self: *IMU, xyz: [3]i16) [3]f64 {
        const x = @intToFloat(f64, xyz[0]) * self.accel_fs / FS_MAX;
        const y = @intToFloat(f64, xyz[1]) * self.accel_fs / FS_MAX;
        const z = @intToFloat(f64, xyz[2]) * self.accel_fs / FS_MAX;

        return [_]f64{ x, y, z };
    }

    fn convert_gyroscope(self: *IMU, xyz: [3]i16) [3]f64 {
        const x = @intToFloat(f64, xyz[0]) * self.gyro_fs / FS_MAX;
        const y = @intToFloat(f64, xyz[1]) * self.gyro_fs / FS_MAX;
        const z = @intToFloat(f64, xyz[2]) * self.gyro_fs / FS_MAX;

        return [_]f64{ x, y, z };
    }

    pub fn reset(self: *IMU) void {
        slog.info("RESET", .{});

        var rst = self.read_register(REG_DEVICE_CONFIG);
        self.write_register(REG_DEVICE_CONFIG, rst | 0x01);
        self.config_delay();
    }

    pub fn config(self: *IMU, accel: ACCEL_FS, gyro: GYRO_FS) void {
        self.init(false, false, false);

        self.config_accel(accel);
        self.config_gyro(gyro);

        self.init(true, true, true);
    }

    fn config_accel(self: *IMU, fs: ACCEL_FS) void {
        const orig = self.read_register(REG_ACCEL_CONFIG0);

        const fs_idx = @enumToInt(fs);
        var tmp = (fs_idx << 5) | self.accel_odr;

        self.accel_fs = ACCEL_FS_VALUES[fs_idx];

        slog.info("ACCEL_CONFIG0 {} -> {}", .{ orig, tmp });

        self.write_register(REG_ACCEL_CONFIG0, tmp);
        self.config_delay();
    }

    fn config_gyro(self: *IMU, fs: GYRO_FS) void {
        const orig = self.read_register(REG_GYRO_CONFIG0);

        const fs_idx = @enumToInt(fs);
        var tmp = (fs_idx << 5) | self.gyro_odr;

        self.gyro_fs = GYRO_FS_VALUES[fs_idx];

        slog.info("GYRO_CONFIG0 {} -> {}", .{ orig, tmp });

        self.write_register(REG_GYRO_CONFIG0, tmp);
        self.config_delay();
    }

    pub fn init(self: *IMU, temp: bool, accel: bool, gyro: bool) void {
        var tmp = self.read_register(REG_PWR_MGMT0);
        const orig = tmp;

        if (temp) {
            tmp &= ~SET_TEMPERATURE_DISABLED;
        } else {
            tmp |= SET_TEMPERATURE_DISABLED;
        }

        // Set bit 4 to keep RC oscillator on
        tmp |= 0x10;

        if (accel) {
            tmp |= SET_ACCEL_LOW_NOISE_MODE;
        } else {
            tmp &= ~SET_ACCEL_LOW_NOISE_MODE;
        }

        if (gyro) {
            tmp |= SET_GYRO_LOW_NOISE_MODE;
        } else {
            tmp &= ~SET_GYRO_LOW_NOISE_MODE;
        }

        slog.info("REG_PWR_MGMT0 {} -> {}", .{ orig, tmp });

        self.write_register(REG_PWR_MGMT0, tmp);
        self.config_delay();
    }

    pub fn temperature(self: *IMU) f32 {
        self.read(REG_TEMP_DATA1_UI, 2);
        const tmp = extract(self.read_buffer, i16, 1);
        return convert_temperature(tmp);
    }

    pub fn accelerometer(self: *IMU) [3]i16 {
        self.read(REG_ACCEL_DATA_X1_UI, 6);

        const x = extract(self.read_buffer, i16, 1);
        const y = extract(self.read_buffer, i16, 3);
        const z = extract(self.read_buffer, i16, 5);

        return [3]i16{ x, y, z };
    }

    pub fn gyroscope(self: *IMU) [3]i16 {
        self.read(REG_GYRO_DATA_X1_UI, 6);

        const x = extract(self.read_buffer, i16, 1);
        const y = extract(self.read_buffer, i16, 3);
        const z = extract(self.read_buffer, i16, 5);

        return [3]i16{ x, y, z };
    }

    pub fn poll(self: *IMU) Sample {
        self.read(REG_TEMP_DATA1_UI, 2 + 6 + 6);

        const temp = extract(self.read_buffer, i16, 1);

        const ax = extract(self.read_buffer, i16, 3);
        const ay = extract(self.read_buffer, i16, 5);
        const az = extract(self.read_buffer, i16, 7);

        const gx = extract(self.read_buffer, i16, 9);
        const gy = extract(self.read_buffer, i16, 11);
        const gz = extract(self.read_buffer, i16, 13);

        const data = self.convert_last_raw_sample(RawSample{
            .received_at = std.time.milliTimestamp(),
            .temperature = temp,
            .accelerometer = [3]i16{ ax, ay, az },
            .gyroscope = [3]i16{ gx, gy, gz },
        });

        if (self.fifo.writableLength() < 1) {
            self.fifo.discard(1);
        }

        self.fifo.writeItem(data) catch |err| {
            slog.err("cannot write item: {s}", .{err});
        };

        // var idx: u8 = 0;
        // while (idx < self.fifo.count) {
        //     slog.info("FIFO {} {} {any}", .{ self.fifo.head, idx, self.fifo.peekItem(idx) });
        //     idx += 1;
        // }

        return data;
    }

    fn convert_last_raw_sample(self: *IMU, last: RawSample) Sample {
        self._last_raw_sample = last;

        return Sample{
            .received_at = last.received_at,
            .temperature = convert_temperature(last.temperature),
            .accelerometer = self.convert_accelerometer(last.accelerometer),
            .gyroscope = self.convert_gyroscope(last.gyroscope),
        };
    }

    pub fn latest(self: *IMU) Sample {
        return self.fifo.peekItem(self.fifo.count - 1);
    }

    pub fn latest_raw_sample(self: *IMU) RawSample {
        return self._last_raw_sample;
    }
};
