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
const mem = std.mem;

const spi = @import("bus/spi.zig");
const bounded_array = @import("bounded_array.zig");

const MAX_PAYLOAD_SIZE = 512 * 2;
const SPI_BUFFER_SIZE = 128;

const MAX_WAIT = 250;
const SLEEP = std.time.ns_per_ms * 10;

//Registers
const UBX_SYNCH_1: u8 = 0xB5;
const UBX_SYNCH_2: u8 = 0x62;

//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const UBX_CLASS_NAV: u8 = 0x01; //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const UBX_CLASS_RXM: u8 = 0x02; //Receiver Manager Messages: Satellite Status, RTC Status
const UBX_CLASS_INF: u8 = 0x04; //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const UBX_CLASS_ACK: u8 = 0x05; //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const UBX_CLASS_CFG: u8 = 0x06; //Configuration Input Messages: Configure the receiver.
const UBX_CLASS_UPD: u8 = 0x09; //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const UBX_CLASS_MON: u8 = 0x0A; //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const UBX_CLASS_AID: u8 = 0x0B; //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const UBX_CLASS_TIM: u8 = 0x0D; //Timing Messages: Time Pulse Output, Time Mark Results
const UBX_CLASS_ESF: u8 = 0x10; //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const UBX_CLASS_MGA: u8 = 0x13; //Multiple GNSS Assistance Messages: Assistance data for various GNSS
const UBX_CLASS_LOG: u8 = 0x21; //Logging Messages: Log creation, deletion, info and retrieval
const UBX_CLASS_SEC: u8 = 0x27; //Security Feature Messages
const UBX_CLASS_HNR: u8 = 0x28; //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
const UBX_CLASS_NMEA: u8 = 0xF0; //NMEA Strings: standard NMEA strings

//Class: CFG
//The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const UBX_CFG_ANT: u8 = 0x13; //Antenna Control Settings. Used to configure the antenna control settings
const UBX_CFG_BATCH: u8 = 0x93; //Get/set data batching configuration.
const UBX_CFG_CFG: u8 = 0x09; //Clear, Save, and Load Configurations. Used to save current configuration
const UBX_CFG_DAT: u8 = 0x06; //Set User-defined Datum or The currently defined Datum
const UBX_CFG_DGNSS: u8 = 0x70; //DGNSS configuration
const UBX_CFG_ESFALG: u8 = 0x56; //ESF alignment
const UBX_CFG_ESFA: u8 = 0x4C; //ESF accelerometer
const UBX_CFG_ESFG: u8 = 0x4D; //ESF gyro
const UBX_CFG_GEOFENCE: u8 = 0x69; //Geofencing configuration. Used to configure a geofence
const UBX_CFG_GNSS: u8 = 0x3E; //GNSS system configuration
const UBX_CFG_HNR: u8 = 0x5C; //High Navigation Rate
const UBX_CFG_INF: u8 = 0x02; //Depending on packet length, either: poll configuration for one protocol, or information message configuration
const UBX_CFG_ITFM: u8 = 0x39; //Jamming/Interference Monitor configuration
const UBX_CFG_LOGFILTER: u8 = 0x47; //Data Logger Configuration
const UBX_CFG_MS: u8G = 0x01; //Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const UBX_CFG_NAV5: u8 = 0x24; //Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const UBX_CFG_NAVX5: u8 = 0x23; //Navigation Engine Expert Settings
const UBX_CFG_NMEA: u8 = 0x17; //Extended NMEA protocol configuration V1
const UBX_CFG_ODO: u8 = 0x1E; //Odometer, Low-speed COG Engine Settings
const UBX_CFG_PM2: u8 = 0x3B; //Extended power management configuration
const UBX_CFG_PMS: u8 = 0x86; //Power mode setup
const UBX_CFG_PRT: u8 = 0x00; //Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const UBX_CFG_PWR: u8 = 0x57; //Put receiver in a defined power state
const UBX_CFG_RATE: u8 = 0x08; //Navigation/Measurement Rate Settings. Used to set port baud rates.
const UBX_CFG_RINV: u8 = 0x34; //Contents of Remote Inventory
const UBX_CFG_RST: u8 = 0x04; //Reset Receiver / Clear Backup Data Structures. Used to reset device.
const UBX_CFG_RXM: u8 = 0x11; //RXM configuration
const UBX_CFG_SBAS: u8 = 0x16; //SBAS configuration
const UBX_CFG_TMODE3: u8 = 0x71; //Time Mode Settings 3. Used to enable Survey In Mode
const UBX_CFG_TP5: u8 = 0x31; //Time Pulse Parameters
const UBX_CFG_USB: u8 = 0x1B; //USB Configuration
const UBX_CFG_VALDEL: u8 = 0x8C; //Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const UBX_CFG_VALGET: u8 = 0x8B; //Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const UBX_CFG_VALSET: u8 = 0x8A; //Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

// Note that key values here are in little endian order, as that is how they should be sent over the wire
const CFG_NAVSPG_SIGATTCOMP: [4]u8 = [_]u8{ 0xd6, 0x00, 0x11, 0x20 }; // Permanently attenuated signal compensation mode
const CFG_NAVSPG_DYNMODEL: [4]u8 = [_]u8{ 0x21, 0x00, 0x11, 0x20 };

const CFG_NAVSPG_DYNMODEL_MODE = enum { PORT, STAT, RED, AUTOMOT, SEA, AIR1, AIR2 };

const CFG_RST_MODE = enum(u8) {
    HARDWARE = 0x00,
    SOFTWARE = 0x01,
    GNSS_RESET = 0x02,
    SHUTDOWN_RESET = 0x04,
    GNSS_STOP = 0x08,
    GNSS_START = 0x09,
};

//Class: NAV
//The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const UBX_NAV_ATT: u8 = 0x05; //Vehicle "Attitude" Solution
const UBX_NAV_CLOCK: u8 = 0x22; //Clock Solution
const UBX_NAV_DOP: u8 = 0x04; //Dilution of precision
const UBX_NAV_EOE: u8 = 0x61; //End of Epoch
const UBX_NAV_GEOFENCE: u8 = 0x39; //Geofencing status. Used to poll the geofence status
const UBX_NAV_HPPOSECEF: u8 = 0x13; //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const UBX_NAV_HPPOSLLH: u8 = 0x14; //High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const UBX_NAV_ODO: u8 = 0x09; //Odometer Solution
const UBX_NAV_ORB: u8 = 0x34; //GNSS Orbit Database Info
const UBX_NAV_POSECEF: u8 = 0x01; //Position Solution in ECEF
const UBX_NAV_POSLLH: u8 = 0x02; //Geodetic Position Solution
const UBX_NAV_PVT: u8 = 0x07; //All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const UBX_NAV_RELPOSNED: u8 = 0x3C; //Relative Positioning Information in NED frame
const UBX_NAV_RESETODO: u8 = 0x10; //Reset odometer
const UBX_NAV_SAT: u8 = 0x35; //Satellite Information
const UBX_NAV_SIG: u8 = 0x43; //Signal Information
const UBX_NAV_STATUS: u8 = 0x03; //Receiver Navigation Status
const UBX_NAV_SVIN: u8 = 0x3B; //Survey-in data. Used for checking Survey In status
const UBX_NAV_TIMEBDS: u8 = 0x24; //BDS Time Solution
const UBX_NAV_TIMEGAL: u8 = 0x25; //Galileo Time Solution
const UBX_NAV_TIMEGLO: u8 = 0x23; //GLO Time Solution
const UBX_NAV_TIMEGPS: u8 = 0x20; //GPS Time Solution
const UBX_NAV_TIMELS: u8 = 0x26; //Leap second event information
const UBX_NAV_TIMEUTC: u8 = 0x21; //UTC Time Solution
const UBX_NAV_VELECEF: u8 = 0x11; //Velocity Solution in ECEF
const UBX_NAV_VELNED: u8 = 0x12; //Velocity Solution in NED

//Class: HNR
//The following are used to configure the HNR message rates
const UBX_HNR_ATT: u8 = 0x01; //HNR Attitude
const UBX_HNR_INS: u8 = 0x02; //HNR Vehicle Dynamics
const UBX_HNR_PVT: u8 = 0x00; //HNR PVT

// Class: MON
// Used to report the receiver status, such as hardware status or I/O subsystem statistics
const UBX_MON_GNSS: u8 = 0x28;
const UBX_MON_RF: u8 = 0x38;
const UBX_MON_SPAN: u8 = 0x31;
const UBX_MON_TEMP: u8 = 0x0E;
const UBX_MON_VER: u8 = 0x04;

//Class: ESF
// The following constants are used to get External Sensor Measurements and Status
// Information.
const UBX_ESF_MEAS: u8 = 0x02;
const UBX_ESF_RAW: u8 = 0x03;
const UBX_ESF_STATUS: u8 = 0x10;
const UBX_ESF_RESETALG: u8 = 0x13;
const UBX_ESF_ALG: u8 = 0x14;
const UBX_ESF_INS: u8 = 0x15; //36 bytes

//Class: RXM
//The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const UBX_RXM_MEASX: u8 = 0x14; //Satellite Measurements for RRLP
const UBX_RXM_PMREQ: u8 = 0x41; //Requests a Power Management task (two differenent packet sizes)
const UBX_RXM_RAWX: u8 = 0x15; //Multi-GNSS Raw Measurement Data
const UBX_RXM_RLM: u8 = 0x59; //Galileo SAR Short-RLM report (two different packet sizes)
const UBX_RXM_RTCM: u8 = 0x32; //RTCM input status
const UBX_RXM_SFRBX: u8 = 0x13; //Boradcast Navigation Data Subframe

//Class: TIM
//The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const UBX_TIM_TM2: u8 = 0x03; //Time mark data
const UBX_TIM_TP: u8 = 0x01; //Time Pulse Timedata
const UBX_TIM_VRFY: u8 = 0x06; //Sourced Time Verification

// Class: ACK
const UBX_ACK_NACK: u8 = 0x00;
const UBX_ACK_ACK: u8 = 0x01;
const UBX_ACK_NONE: u8 = 0x02; //Not a real value

const UBX_Packet_Validity = extern enum {
    NOT_VALID,
    VALID,
    NOT_DEFINED,
    NOT_ACKNOWLEDGED, // This indicates that we received a NACK
};

const UBX_Status = extern enum {
    SUCCESS,
    FAIL,
    CRC_FAIL,
    TIMEOUT,
    COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
    OUT_OF_RANGE,
    INVALID_ARG,
    INVALID_OPERATION,
    MEM_ERR,
    HW_ERR,
    DATA_SENT, // This indicates that a 'set' was successful
    DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
    I2C_COMM_FAILURE,
    DATA_OVERWRITTEN, // This is an error - the data was valid but has been or _is being_ overwritten by another packet
};

const SentenceTypes = extern enum { NONE, NMEA, UBX, RTCM };

const PacketBuffer = extern enum { NONE, CFG, ACK, BUF, AUTO };

const ubxPacket = extern struct {
    cls: u8 = 0,
    id: u8 = 0,
    len: u16 = 0, // Length of the payload. Does not include cls, id, or checksum bytes
    counter: u16 = 0, // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
    starting_spot: u16 = 0, // The counter value needed to go past before we begin recording into payload array
    payload: [MAX_PAYLOAD_SIZE]u8 = [_]u8{0} ** MAX_PAYLOAD_SIZE,
    checksum_a: u8 = 0, // Given to us from module. Checked against the rolling calculated A/B checksums.
    checksum_b: u8 = 0,
    valid: UBX_Packet_Validity = UBX_Packet_Validity.NOT_DEFINED, //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
    class_id_match: UBX_Packet_Validity = UBX_Packet_Validity.NOT_DEFINED, // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

const AntennaState = extern enum { INIT, UNK, OK, SHORT, OPEN };
const AntennaPower = extern enum { OFF, ON, UNK };
const RFInfo = struct {
    block_id: u8,
    flags: u8,
    antenna_state: AntennaState,
    antenna_power: AntennaPower,
    post_status: u32,
    noise: u16,
    agc: u16,
    jam: u8,
    i_imbalance: i8,
    i_magnitude: u8,
    q_imbalance: i8,
    q_magnitude: u8,
};

const RFSpan = struct {
    spectrum: [256]u8 = [_]u8{0} ** 256,
    span: u32,
    resolution: u32,
    center: u32,
    pga: u8,
};

const TimeData = struct {
    epoch: u32,
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
    nanosecond: i32,
    valid: u8,
    accuracy: u32,
};

const PositionData = struct {
    longitude: i32,
    latitude: i32,
    height_ellipsoid: i32,
    height_sea_level: i32,
    geometric_dilution: u16,
    heading: i32,
    heading_accuracy: u32,
    // declination: i16,
    // declination_accuracy: u16,
};

const VelocityData = struct {
    north: i32,
    east: i32,
    down: i32,
    speed: i32,
    speed_accuracy: u32,
};

const PVTData = struct {
    received_at: i64,
    time: TimeData,
    position: PositionData,
    velocity: VelocityData,
    satellite_count: u8,
    fix_type: u8,
    flags1: u8,
    flags2: u8,
    flags3: u8,
};

pub const PVT = struct {
    age: i64,
    timestamp: [24]u8,
    time: TimeData,
    longitude: f64,
    latitude: f64,
    height: f32,
    heading: f32,
    speed: f32,
    velocity: [3]f32,
    satellite_count: u8,
    fix_type: u8,
    flags: [3]u8,
    dop: f32,
};

const GNSSID = extern enum { GPS, SBAS, GAL, BDS, IMES, QZSS, GLO, NONE };
const SATQUAL = extern enum { NONE, SEARCH, ACQ, DETECT, LOCK, SYNCA, SYNCB, SYNCC };
const SATHEALTH = extern enum { UNK, GOOD, BAD };

pub const SatDetail = struct {
    gnss: GNSSID = GNSSID.NONE,
    id: u8 = 0,
    cno: u8 = 0,
    elev: i8 = 0,
    azim: i16 = 0,
    pr_res: i16 = 0,
    quality: SATQUAL = SATQUAL.NONE,
    used: bool = false,
    health: SATHEALTH = SATHEALTH.UNK,
};

const MAX_SAT_INFO_COUNT: u8 = 64;
pub const SatInfo = struct {
    itow: u32,
    count: u8,
    cno_max: u8 = 0,
    satellites: bounded_array.BoundedArray(SatDetail, MAX_SAT_INFO_COUNT),
};

pub fn init(handle: spi.SPI) GNSS {
    // var payload_ack = [_]u8{0} ** 2;
    // var payload_buf = [_]u8{0} ** 2;
    // var payload_cfg = [_]u8{0} ** MAX_PAYLOAD_SIZE;
    // var payload_auto = [_]u8{0} ** MAX_PAYLOAD_SIZE;

    return GNSS{
        .handle = handle,

        .write_buffer = [_]u8{0} ** SPI_BUFFER_SIZE,
        .read_buffer = [_]u8{0xFF} ** SPI_BUFFER_SIZE,

        // .payload_ack = &payload_ack,
        // .payload_buf = &payload_buf,
        // .payload_auto = &payload_auto,
        // .payload_cfg = &payload_cfg,

        // .packet_ack = ubxPacket{ .payload = &payload_ack },
        // .packet_buf = ubxPacket{ .payload = &payload_buf },
        // .packet_cfg = ubxPacket{ .payload = &payload_cfg },
        // .packet_auto = ubxPacket{ .payload = &payload_auto },

        .packet_ack = ubxPacket{},
        .packet_buf = ubxPacket{},
        .packet_cfg = ubxPacket{},
        .packet_auto = ubxPacket{},
    };
}

fn calc_checksum(msg: *ubxPacket) void {
    msg.checksum_a = msg.cls;
    msg.checksum_b = msg.checksum_a;

    msg.checksum_a +%= msg.id;
    msg.checksum_b +%= msg.checksum_a;

    msg.checksum_a +%= @truncate(u8, msg.len);
    msg.checksum_b +%= msg.checksum_a;

    msg.checksum_a +%= @truncate(u8, msg.len >> 8);
    msg.checksum_b +%= msg.checksum_a;

    var idx: u8 = 0;
    while (idx < msg.len) {
        msg.checksum_a +%= msg.payload[idx];
        msg.checksum_b +%= msg.checksum_a;

        idx += 1;
    }
}

fn get_payload_size_nav(id: u8) u16 {
    return switch (id) {
        UBX_NAV_POSECEF => 20,
        UBX_NAV_STATUS => 16,
        UBX_NAV_DOP => 18,
        UBX_NAV_ATT => 32,
        UBX_NAV_PVT => 92,
        UBX_NAV_ODO => 20,
        UBX_NAV_VELECEF => 20,
        UBX_NAV_VELNED => 20,
        UBX_NAV_HPPOSECEF => 28,
        UBX_NAV_HPPOSLLH => 36,
        UBX_NAV_CLOCK => 20,
        UBX_NAV_TIMELS => 24,
        UBX_NAV_SVIN => 40,
        UBX_NAV_RELPOSNED => 64,
        else => 0,
    };
}

fn get_payload_size_rxm(id: u8) u16 {
    return switch (id) {
        UBX_RXM_SFRBX => 8 + (4 * 16),
        UBX_RXM_RAWX => 16 + (32 * 64),
        else => 0,
    };
}

fn get_payload_size_cfg(id: u8) u16 {
    return switch (id) {
        UBX_CFG_RATE => 6,
        else => 0,
    };
}

fn get_payload_size_tim(id: u8) u16 {
    return switch (id) {
        UBX_TIM_TM2 => 28,
        else => 0,
    };
}

const DEF_NUM_SENS = 7;

fn get_payload_size_esf(id: u8) u16 {
    return switch (id) {
        UBX_ESF_ALG => 16,
        UBX_ESF_INS => 36,
        UBX_ESF_MEAS => 8 + (4 * DEF_NUM_SENS) + 4,
        UBX_ESF_RAW => 4 + (8 * DEF_NUM_SENS),
        UBX_ESF_STATUS => 16 + (4 + DEF_NUM_SENS),
        else => 0,
    };
}

fn get_payload_size_hnr(id: u8) u16 {
    return switch (id) {
        UBX_HNR_PVT => 72,
        UBX_HNR_ATT => 32,
        UBX_HNR_INS => 36,
        else => 0,
    };
}

fn get_payload_size(class: u8, id: u8) u16 {
    const size: u16 = switch (class) {
        UBX_CLASS_NAV => get_payload_size_nav(id),
        UBX_CLASS_RXM => get_payload_size_rxm(id),
        UBX_CLASS_CFG => get_payload_size_cfg(id),
        UBX_CLASS_TIM => get_payload_size_tim(id),
        UBX_CLASS_ESF => get_payload_size_esf(id),
        UBX_CLASS_HNR => get_payload_size_hnr(id),
        else => 0,
    };

    return size;
}

fn extract(msg: *ubxPacket, comptime T: type, idx: u16) T {
    return mem.readIntSliceLittle(T, msg.payload[idx .. idx + @divExact(@typeInfo(T).Int.bits, 8)]);
}

fn print_packet(msg: *ubxPacket) void {
    print("Packet(cls: {X} id: {X} counter: {} len: {} checksum: ({X} {X}) ", .{ msg.cls, msg.id, msg.counter, msg.len, msg.checksum_a, msg.checksum_b });
    print("payload:(", .{});

    for (msg.payload[0..msg.len]) |value| {
        print("{X} ", .{value});
    }

    print(") )\n", .{});
}

pub const GNSS = struct {
    handle: spi.SPI,

    write_buffer: [SPI_BUFFER_SIZE]u8,
    read_buffer: [SPI_BUFFER_SIZE]u8,
    read_buffer_index: u16 = 0,

    message_type: SentenceTypes = SentenceTypes.NONE,
    active_buffer: PacketBuffer = PacketBuffer.NONE,
    frame_counter: u16 = 0,
    cur_checksum_a: u8 = 0,
    cur_checksum_b: u8 = 0,
    ignore_payload: bool = false,

    // payload_ack: [*]u8,
    // payload_buf: [*]u8,
    // payload_cfg: [*]u8,
    // payload_auto: [*]u8,

    packet_ack: ubxPacket,
    packet_buf: ubxPacket,
    packet_cfg: ubxPacket,
    packet_auto: ubxPacket,

    max_wait: u16 = MAX_WAIT,
    cur_wait: u16 = MAX_WAIT,

    _last_nav_pvt: ?PVTData = null,
    _last_nav_sat: ?SatInfo = null,
    _last_mon_rf: ?RFInfo = null,
    _last_mon_span: ?RFSpan = null,

    fn send_command(self: *GNSS, packet: *ubxPacket) UBX_Status {
        calc_checksum(packet);
        self.send_spi_command(packet);

        // Only CFG commands have ACKS
        if (packet.cls == UBX_CLASS_CFG) {
            return self.wait_for_ack(packet, packet.cls, packet.id);
        } else {
            return self.wait_for_no_ack(packet, packet.cls, packet.id);
        }
    }

    fn send_command_nowait(self: *GNSS, packet: *ubxPacket) UBX_Status {
        calc_checksum(packet);
        self.send_spi_command(packet);

        return UBX_Status.SUCCESS;
    }

    fn wait_for_no_ack(self: *GNSS, packet: *ubxPacket, requested_class: u8, requested_id: u8) UBX_Status {
        // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
        packet.valid = UBX_Packet_Validity.NOT_DEFINED;

        self.packet_ack.valid = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_buf.valid = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_auto.valid = UBX_Packet_Validity.NOT_DEFINED;

        // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
        packet.class_id_match = UBX_Packet_Validity.NOT_DEFINED;

        self.packet_ack.class_id_match = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_buf.class_id_match = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_auto.class_id_match = UBX_Packet_Validity.NOT_DEFINED;

        const start_time = std.time.milliTimestamp();
        while (std.time.milliTimestamp() - start_time < self.cur_wait) {
            // See if new data is available. Process bytes as they come in.
            if (self.check_for_data(packet, requested_class, requested_id)) {
                // If outgoingUBX->classAndIDmatch is VALID
                // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
                // then we can be confident that the data in outgoingUBX is valid
                if ((packet.class_id_match == UBX_Packet_Validity.VALID) and
                    (packet.valid == UBX_Packet_Validity.VALID) and
                    (packet.cls == requested_class) and
                    (packet.id == requested_id))
                {
                    self.cur_wait = self.max_wait;
                    // We received valid data!
                    return UBX_Status.DATA_RECEIVED;
                }
            }

            // If the outgoingUBX->classAndIDmatch is VALID
            // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
            // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
            // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
            // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
            // So we cannot use outgoingUBX->valid as part of this check.
            // Note: the addition of packetBuf should make this check redundant!
            else if ((packet.class_id_match == UBX_Packet_Validity.VALID) and
                ((packet.cls != requested_class) or (packet.id != requested_id)))
            {
                self.cur_wait = self.max_wait;
                // Data was valid but has been or is being overwritten
                return UBX_Status.DATA_OVERWRITTEN;
            }

            // If outgoingUBX->classAndIDmatch is NOT_DEFINED
            // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
            else if ((packet.class_id_match == UBX_Packet_Validity.NOT_DEFINED) and
                (packet.valid == UBX_Packet_Validity.VALID))
            {
                self.cur_wait = self.max_wait;
                print("wait_for_no_ack : valid but unwanted data\n", .{});
            }

            // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
            else if (packet.class_id_match == UBX_Packet_Validity.NOT_VALID) {
                self.cur_wait = self.max_wait;
                return UBX_Status.CRC_FAIL;
            }

            std.time.sleep(SLEEP);
        }

        self.cur_wait = self.max_wait;
        print("TIMEOUT\n", .{});

        // Wait has timed out
        return UBX_Status.TIMEOUT;
    }

    fn wait_for_ack(self: *GNSS, packet: *ubxPacket, requested_class: u8, requested_id: u8) UBX_Status {
        // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
        packet.valid = UBX_Packet_Validity.NOT_DEFINED;

        self.packet_ack.valid = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_buf.valid = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_auto.valid = UBX_Packet_Validity.NOT_DEFINED;

        // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
        packet.class_id_match = UBX_Packet_Validity.NOT_DEFINED;

        self.packet_ack.class_id_match = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_buf.class_id_match = UBX_Packet_Validity.NOT_DEFINED;
        self.packet_auto.class_id_match = UBX_Packet_Validity.NOT_DEFINED;

        const start_time = std.time.milliTimestamp();
        while (std.time.milliTimestamp() - start_time < self.cur_wait) {
            // print("WAIT {} from {}\n", .{ std.time.milliTimestamp() - start_time, start_time });

            // See if new data is available. Process bytes as they come in.
            if (self.check_for_data(packet, requested_class, requested_id)) {
                // If both the packet.class_id_match and packet_ack.class_id_match are VALID
                // and packet.valid is _still_ VALID and the class and ID _still_ match
                // then we can be confident that the data in outgoing packet is valid
                if ((packet.class_id_match == UBX_Packet_Validity.VALID) and
                    (self.packet_ack.class_id_match == UBX_Packet_Validity.VALID) and
                    (packet.valid == UBX_Packet_Validity.VALID) and
                    (packet.cls == requested_class) and
                    (packet.id == requested_id))
                {
                    self.cur_wait = self.max_wait;

                    // We received valid data and a correct ACK!
                    return UBX_Status.DATA_RECEIVED;
                }

                // We can be confident that the data packet (if we are going to get one) will always arrive
                // before the matching ACK. So if we sent a config packet which only produces an ACK
                // then packet.class_id_match will be NOT_DEFINED and the packet_ack.class_id_match will VALID.
                // We should not check packet.valid, packet.cls or packet.id
                // as these may have been changed by an automatic packet.
                else if ((packet.class_id_match == UBX_Packet_Validity.NOT_DEFINED) and
                    (self.packet_ack.class_id_match == UBX_Packet_Validity.VALID))
                {
                    self.cur_wait = self.max_wait;

                    // We got an ACK but no data...
                    return UBX_Status.DATA_SENT;
                }

                // If both the packet.class_id_match and self.packet_ack.class_id_match are VALID
                // but the packet.cls or ID no longer match then we can be confident that we had
                // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
                // If (e.g.) a PVT packet is _being_ received: packet.valid will be NOT_DEFINED
                // If (e.g.) a PVT packet _has been_ received: packet.valid will be VALID (or just possibly NOT_VALID)
                // So we cannot use packet.valid as part of this check.
                // Note: the addition of packetBuf should make this check redundant!
                else if ((packet.class_id_match == UBX_Packet_Validity.VALID) and
                    (self.packet_ack.class_id_match == UBX_Packet_Validity.VALID) and
                    ((packet.cls != requested_class) or (packet.id != requested_id)))
                {
                    self.cur_wait = self.max_wait;

                    // Data was valid but has been or is being overwritten
                    return UBX_Status.DATA_OVERWRITTEN;
                }

                // If self.packet_ack.class_id_match is VALID but both packet.valid and packet.class_id_match
                // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
                else if ((self.packet_ack.class_id_match == UBX_Packet_Validity.VALID) and
                    (packet.class_id_match == UBX_Packet_Validity.NOT_VALID) and
                    (packet.valid == UBX_Packet_Validity.NOT_VALID))
                {
                    self.cur_wait = self.max_wait;

                    // Checksum fail
                    return UBX_Status.CRC_FAIL;
                }

                // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
                // So you would expect packet.valid and packet.class_id_match to still be NOT_DEFINED
                // But if a full PVT packet arrives afterwards packet.valid could be VALID (or just possibly NOT_VALID)
                // but packet.cls and packet.id would not match...
                // So I think this is telling us we need a special state for self.packet_ack.class_id_match to tell us
                // the packet was definitely NACK'd otherwise we are possibly just guessing...
                // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
                else if (self.packet_ack.class_id_match == UBX_Packet_Validity.NOT_ACKNOWLEDGED) {
                    self.cur_wait = self.max_wait;

                    // We received a NACK!
                    return UBX_Status.COMMAND_NACK;
                }

                // If the packet.class_id_match is VALID but the packetAck.class_id_match is NOT_VALID
                // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
                // If we were playing safe, we should return FAIL instead
                else if ((packet.class_id_match == UBX_Packet_Validity.VALID) and
                    (self.packet_ack.class_id_match == UBX_Packet_Validity.NOT_VALID) and
                    (packet.valid == UBX_Packet_Validity.VALID) and
                    (packet.cls == requested_class) and
                    (packet.id == requested_id))
                {
                    self.cur_wait = self.max_wait;
                    // We received valid data and an invalid ACK!
                    return UBX_Status.DATA_RECEIVED;
                }

                // If the packet.class_id_match is NOT_VALID and the self.packet_ack.class_id_match is NOT_VALID
                // then we return a FAIL. This must be a double checksum failure?
                else if ((packet.class_id_match == UBX_Packet_Validity.NOT_VALID) and
                    (self.packet_ack.class_id_match == UBX_Packet_Validity.NOT_VALID))
                {
                    self.cur_wait = self.max_wait;

                    // We received invalid data and an invalid ACK!
                    return UBX_Status.FAIL;
                }

                // If the packet.class_id_match is VALID and the self.packet_ack.class_id_match is NOT_DEFINED
                // then the ACK has not yet been received and we should keep waiting for it
            }

            std.time.sleep(SLEEP);
        }

        self.cur_wait = self.max_wait;
        print("TIMEOUT\n", .{});

        // Wait has timed out
        return UBX_Status.TIMEOUT;
    }

    fn check_for_data(self: *GNSS, packet: *ubxPacket, requested_class: u8, requested_id: u8) bool {
        // Process the contents of the SPI buffer if not empty
        var idx: u8 = 0;
        while (idx < self.read_buffer_index) {
            // print("check_for_data : read_buffer {} {} {any}\n", .{ idx, self.read_buffer_index, self.read_buffer });
            self.process_byte(self.read_buffer[idx], packet, requested_class, requested_id);
            idx += 1;
        }

        self.read_buffer_index = 0;

        while (true) {
            if (self.handle.read_byte()) |value| {
                if (value == 0xFF and self.message_type == SentenceTypes.NONE) {
                    // print("check_for_data : read_byte got EOM\n", .{});
                    break;
                }
                // print("check_for_data : read_byte got 0x{X}\n", .{value});
                self.process_byte(value, packet, requested_class, requested_id);
            } else {
                // print("check_for_data : read_byte failed\n", .{});
                break;
            }
        }

        return true;
    }

    fn process_byte(self: *GNSS, incoming: u8, packet: *ubxPacket, requested_class: u8, requested_id: u8) void {
        if (self.message_type == SentenceTypes.NONE or self.message_type == SentenceTypes.NMEA) {
            if (incoming == UBX_SYNCH_1) {
                self.message_type = SentenceTypes.UBX;
                self.frame_counter = 0;

                self.packet_buf.counter = 0;
                self.ignore_payload = false;
                self.active_buffer = PacketBuffer.BUF;
            } else if (incoming == '$') {
                print("Start of NMEA packet\n", .{});
                // self.message_type = SentenceTypes.NMEA;
                self.message_type = SentenceTypes.NONE;
                self.frame_counter = 0;
            } else if (incoming == 0xD3) {
                print("Start of RTCM packet\n", .{});
                // self.message_type = SentenceTypes.RTCM;
                self.message_type = SentenceTypes.NONE;
                self.frame_counter = 0;
            } else {
                // This character is unknown or we missed the previous start of a sentence
            }
        }

        if (self.message_type == SentenceTypes.UBX) {
            if (self.frame_counter == 0 and incoming != UBX_SYNCH_1) {
                // Something went wrong, reset
                self.message_type = SentenceTypes.NONE;
            } else if (self.frame_counter == 1 and incoming != UBX_SYNCH_2) {
                // Something went wrong, reset
                self.message_type = SentenceTypes.NONE;
            } else if (self.frame_counter == 2) {
                // Class
                self.packet_buf.cls = incoming;
                self.packet_buf.counter = 0;
                self.packet_buf.valid = UBX_Packet_Validity.NOT_DEFINED;
                self.packet_buf.starting_spot = packet.starting_spot;

                self.cur_checksum_a = 0;
                self.cur_checksum_b = 0;
            } else if (self.frame_counter == 3) {
                // ID
                self.packet_buf.id = incoming;

                // We can now identify the type of response
                // If the packet we are receiving is not an ACK then check for a class and ID match
                if (self.packet_buf.cls != UBX_CLASS_ACK) {
                    // This is not an ACK so check for a class and ID match
                    if ((self.packet_buf.cls == requested_class) and (self.packet_buf.id == requested_id)) {
                        // This is not an ACK and we have a class and ID match
                        // So start diverting data into incomingUBX (usually packetCfg)
                        self.active_buffer = PacketBuffer.CFG;

                        self.packet_cfg.cls = self.packet_buf.cls;
                        self.packet_cfg.id = self.packet_buf.id;
                        self.packet_cfg.counter = self.packet_buf.counter;
                        self.packet_cfg.checksum_a = 0;
                        self.packet_cfg.checksum_b = 0;

                        var idx: u16 = 0;
                        while (idx < MAX_PAYLOAD_SIZE) {
                            self.packet_cfg.payload[idx] = 0;
                            idx += 1;
                        }
                    }

                    // This is not an ACK and we do not have a complete class and ID match
                    // So let's check if this is an "automatic" message which has its own storage defined
                    else if (self.check_automatic(self.packet_buf.cls, self.packet_buf.id)) {
                        // This is not the message we were expecting but it has its own storage and so we should process it anyway.
                        // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
                        // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
                        // reception is complete.
                    } else {
                        // This is not an ACK and we do not have a class and ID match
                        // so we should keep diverting data into packetBuf and ignore the payload
                        self.ignore_payload = true;
                    }
                } else {
                    // This is an ACK so it is to early to do anything with it
                    // We need to wait until we have received the length and data bytes
                    // So we should keep diverting data into packetBuf
                }
            } else if (self.frame_counter == 4) {
                // Length LSB
                self.packet_buf.len = incoming;
            } else if (self.frame_counter == 5) {
                self.packet_buf.len += (@intCast(u16, incoming) << 8);
            } else if (self.frame_counter == 6) {
                // This should be the first byte of the payload unless .len is zero
                if (self.packet_buf.len == 0) {
                    // If length is zero (!) this will be the first byte of the checksum so record it
                    self.packet_buf.checksum_a = incoming;
                } else {
                    // The length is not zero so record this byte in the payload
                    self.packet_buf.payload[0] = incoming;
                }
            } else if (self.frame_counter == 7) {
                // This should be the second byte of the payload unless .len is zero or one
                if (self.packet_buf.len == 0) {
                    // If length is zero (!) this will be the second byte of the checksum so record it
                    self.packet_buf.checksum_b = incoming;
                } else if (self.packet_buf.len == 1) {
                    // The length is one so this is the first byte of the checksum
                    self.packet_buf.checksum_a = incoming;
                } else {
                    // Length is >= 2 so this must be a payload byte
                    self.packet_buf.payload[1] = incoming;
                }

                // Now that we have received two payload bytes, we can check for a matching ACK/NACK
                if ((self.active_buffer == PacketBuffer.BUF) // If we are not already processing a data packet
                and (self.packet_buf.cls == UBX_CLASS_ACK) // and if this is an ACK/NACK
                and (self.packet_buf.payload[0] == requested_class) // and if the class matches
                and (self.packet_buf.payload[1] == requested_id)) // and if the ID matches
                {
                    if (self.packet_buf.len == 2) {
                        // Then this is a matching ACK so copy it into packetAck
                        self.active_buffer = PacketBuffer.ACK;
                        self.packet_ack.cls = self.packet_buf.cls;
                        self.packet_ack.id = self.packet_buf.id;
                        self.packet_ack.len = self.packet_buf.len;
                        self.packet_ack.counter = self.packet_buf.counter;
                        self.packet_ack.payload[0] = self.packet_buf.payload[0];
                        self.packet_ack.payload[1] = self.packet_buf.payload[1];
                    } else {
                        print("process: ACK received with .len != 2 | Class {} ID {} len {}\n", .{ self.packet_buf.payload[0], self.packet_buf.payload[1], self.packet_buf.len });
                    }
                }
            }

            if (self.active_buffer == PacketBuffer.ACK) {
                self.process_ubx_byte(incoming, &self.packet_ack, requested_class, requested_id, "ACK");
            } else if (self.active_buffer == PacketBuffer.CFG) {
                self.process_ubx_byte(incoming, packet, requested_class, requested_id, "INC");
            } else if (self.active_buffer == PacketBuffer.BUF) {
                self.process_ubx_byte(incoming, &self.packet_buf, requested_class, requested_id, "BUF");
            } else if (self.active_buffer == PacketBuffer.AUTO) {
                self.process_ubx_byte(incoming, &self.packet_auto, requested_class, requested_id, "AUTO");
            } else {
                print("process: Active buffer is NONE, cannot continue\n", .{});
            }

            self.frame_counter += 1;
        }

        // else if (self.message_type == SentenceTypes.NMEA) {
        //     print("process: Got NMEA message\n", .{});

        // } else if (self.message_type == SentenceTypes.RTCM) {
        //     print("process: Got RTCM message\n", .{});
        //     self.message_type = SentenceTypes.NONE;
        // }
    }

    // Given a character, file it away into the uxb packet structure
    // Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
    fn process_ubx_byte(self: *GNSS, incoming: u8, packet: *ubxPacket, requested_class: u8, requested_id: u8, label: []const u8) void {
        var max_payload_size: u16 = 0;

        if (self.active_buffer == PacketBuffer.CFG) {
            max_payload_size = MAX_PAYLOAD_SIZE;
        } else if (self.active_buffer == PacketBuffer.AUTO) {
            max_payload_size = get_payload_size(packet.cls, packet.id);
        } else {
            max_payload_size = 2;
        }

        var overrun: bool = false;

        if (packet.counter < packet.len + 4) {
            self.add_to_checksum(incoming);
        }

        if (packet.counter == 0) {
            packet.cls = incoming;
        } else if (packet.counter == 1) {
            packet.id = incoming;
        } else if (packet.counter == 2) {
            packet.len = incoming;
        } else if (packet.counter == 3) {
            packet.len += (@intCast(u16, incoming) << 8);
        } else if (packet.counter == packet.len + 4) {
            packet.checksum_a = incoming;
        } else if (packet.counter == packet.len + 5) {
            packet.checksum_b = incoming;

            self.message_type = SentenceTypes.NONE;

            if ((packet.checksum_a == self.cur_checksum_a) and (packet.checksum_b == self.cur_checksum_b)) {
                // Flag the packet as valid
                packet.valid = UBX_Packet_Validity.VALID;

                // Let's check if the class and ID match the requestedClass and requestedID
                // Remember - this could be a data packet or an ACK packet
                if ((packet.cls == requested_class) and (packet.id == requested_id)) {
                    packet.class_id_match = UBX_Packet_Validity.VALID;
                }

                // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
                else if ((packet.cls == UBX_CLASS_ACK) and (packet.id == UBX_ACK_ACK) and (packet.payload[0] == requested_class) and (packet.payload[1] == requested_id)) {
                    packet.class_id_match = UBX_Packet_Validity.VALID;
                    // print("gnss process_ubx :  ACK | Class {} ID {}\n", .{ packet.payload[0], packet.payload[1] });
                }

                // If this is an NACK then let's check if the class and ID match the requestedClass and requestedID
                else if ((packet.cls == UBX_CLASS_ACK) and (packet.id == UBX_ACK_NACK) and (packet.payload[0] == requested_class) and (packet.payload[1] == requested_id)) {
                    packet.class_id_match = UBX_Packet_Validity.NOT_ACKNOWLEDGED;
                    print("gnss process_ubx : NACK | Class {} ID {}\n", .{ packet.payload[0], packet.payload[1] });
                }

                // This is not an ACK and we do not have a complete class and ID match
                // So let's check for an "automatic" message arriving
                else if (self.check_automatic(packet.cls, packet.id)) {
                    // This isn't the message we are looking for...
                    print("gnss process_ubx : automatic | Class {} ID {}\n", .{ packet.cls, packet.id });
                }

                if (self.ignore_payload == false) {
                    // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
                    self.process_packet(packet);
                }
            } else {
                // Checksum failure
                packet.valid = UBX_Packet_Validity.NOT_VALID;
                packet.class_id_match = UBX_Packet_Validity.NOT_VALID;

                print("gnss process_ubx : checksum failed | {} {} vs {} {}\n", .{ packet.checksum_a, packet.checksum_b, self.cur_checksum_a, self.cur_checksum_b });
            }
        } else {
            // Load this byte into the payload array
            var starting_spot: u16 = packet.starting_spot;

            // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
            if (self.check_automatic(packet.cls, packet.id)) {
                starting_spot = 0;
            }

            // Check if this is payload data which should be ignored
            if (self.ignore_payload == false) {
                if ((packet.counter - 4) >= starting_spot) {
                    if ((packet.counter - 4 - starting_spot) < max_payload_size) {
                        packet.payload[packet.counter - 4 - starting_spot] = incoming;
                        // print("payload[{}] = {X} {X}\n", .{ packet.counter - 4 - starting_spot, incoming, packet.payload[packet.counter - 4 - starting_spot] });
                    } else {
                        overrun = true;
                    }
                }
            }
        }

        if (overrun or (packet.counter == max_payload_size + 6) and self.ignore_payload == false) {
            self.message_type = SentenceTypes.NONE;
            print("gnss process_ubx : overrun | buffer {} size {}\n", .{ self.active_buffer, max_payload_size });
        }

        // if (incoming < 16) {
        //     print("0{X} -> {} {s} {}\n", .{ incoming, self.active_buffer, label, packet.valid });
        // } else {
        //     print("{X} -> {} {s} {}\n", .{ incoming, self.active_buffer, label, packet.valid });
        // }
        // print_packet(packet);
        // print("\n", .{});

        packet.counter += 1;
    }

    fn process_packet(self: *GNSS, packet: *ubxPacket) void {
        switch (packet.cls) {
            UBX_CLASS_ACK => return,
            UBX_CLASS_NAV => self.process_nav_packet(packet),
            UBX_CLASS_MON => self.process_mon_packet(packet),
            UBX_CLASS_CFG => self.process_cfg_packet(packet),
            // UBX_CLASS_TIM => self.process_tim_packet(packet),
            // UBX_CLASS_ESF => self.process_esf_packet(packet),
            // UBX_CLASS_HNR => self.process_hnr_packet(packet),
            else => print("gnss process_packet : unknown class {}\n", .{packet.cls}),
        }
    }

    fn process_mon_packet(self: *GNSS, packet: *ubxPacket) void {
        switch (packet.id) {
            UBX_MON_RF => {
                const data = RFInfo{
                    .block_id = extract(packet, u8, 4),
                    .flags = extract(packet, u8, 5),
                    .antenna_state = @intToEnum(AntennaState, extract(packet, u8, 6)),
                    .antenna_power = @intToEnum(AntennaPower, extract(packet, u8, 7)),
                    .post_status = extract(packet, u32, 8),
                    .noise = extract(packet, u16, 16),
                    .agc = extract(packet, u16, 18),
                    .jam = extract(packet, u8, 20),
                    .i_imbalance = extract(packet, i8, 21),
                    .i_magnitude = extract(packet, u8, 22),
                    .q_imbalance = extract(packet, i8, 23),
                    .q_magnitude = extract(packet, u8, 24),
                };

                self._last_mon_rf = data;
            },
            UBX_MON_SPAN => {
                var data = RFSpan{
                    .span = extract(packet, u32, 260),
                    .resolution = extract(packet, u32, 264),
                    .center = extract(packet, u32, 268),
                    .pga = extract(packet, u8, 272),
                };
                std.mem.copy(u8, data.spectrum[0..], packet.payload[4..260]);

                self._last_mon_span = data;
            },
            else => print("gnss process_mon_packet : unknown id {}\n", .{packet.id}),
        }
    }

    fn process_nav_packet(self: *GNSS, packet: *ubxPacket) void {
        switch (packet.id) {
            UBX_NAV_PVT => {
                if (packet.len == get_payload_size(packet.cls, packet.id)) {
                    const pvt = PVTData{
                        .received_at = std.time.milliTimestamp(),
                        .time = TimeData{
                            .epoch = extract(packet, u32, 0),
                            .year = extract(packet, u16, 4),
                            .month = extract(packet, u8, 6),
                            .day = extract(packet, u8, 7),
                            .hour = extract(packet, u8, 8),
                            .minute = extract(packet, u8, 9),
                            .second = extract(packet, u8, 10),
                            .valid = extract(packet, u8, 11),
                            .accuracy = extract(packet, u32, 12),
                            .nanosecond = extract(packet, i32, 16),
                        },
                        .position = PositionData{
                            .longitude = extract(packet, i32, 24),
                            .latitude = extract(packet, i32, 28),
                            .height_ellipsoid = extract(packet, i32, 32),
                            .height_sea_level = extract(packet, i32, 36),
                            .geometric_dilution = extract(packet, u16, 76),
                            .heading = extract(packet, i32, 64),
                            .heading_accuracy = extract(packet, u32, 72),
                            // .declination = extract(packet, i16, 88),
                            // .declination_accuracy = extract(packet, u16, 90),
                        },
                        .velocity = VelocityData{
                            .north = extract(packet, i32, 48),
                            .east = extract(packet, i32, 52),
                            .down = extract(packet, i32, 56),
                            .speed = extract(packet, i32, 60),
                            .speed_accuracy = extract(packet, u32, 68),
                        },
                        .satellite_count = extract(packet, u8, 23),
                        .fix_type = extract(packet, u8, 20),
                        .flags1 = extract(packet, u8, 21),
                        .flags2 = extract(packet, u8, 22),
                        .flags3 = extract(packet, u8, 78),
                    };

                    self._last_nav_pvt = pvt;
                } else {
                    print("gnss nav_packet : incorrect length for PVT : {}\n", .{packet.len});
                }
            },
            UBX_NAV_SAT => {
                const count: u8 = extract(packet, u8, 5);

                var satellites = bounded_array.BoundedArray(SatDetail, MAX_SAT_INFO_COUNT).init(count) catch |err| {
                    std.log.err("GNSS | could not created satellite defailt BoundedArray : {}", .{err});
                    return;
                };

                var data = SatInfo{ .itow = extract(packet, u32, 0), .count = count, .satellites = satellites };
                var idx: u16 = 0;

                while (idx < data.count and idx < MAX_SAT_INFO_COUNT) {
                    const loc = idx * 12;
                    const flag: u32 = extract(packet, u32, loc + 16);
                    const detail = SatDetail{
                        .gnss = @intToEnum(GNSSID, extract(packet, u8, 8 + idx * 12)),
                        .id = extract(packet, u8, loc + 9),
                        .cno = extract(packet, u8, loc + 10),
                        .elev = extract(packet, i8, loc + 11),
                        .azim = extract(packet, i16, loc + 12),
                        .pr_res = extract(packet, i16, loc + 14),
                        .quality = @intToEnum(SATQUAL, @intCast(c_int, flag & 0b111)),
                        .used = (flag >> 3) & 0b1 == 0b1,
                        .health = @intToEnum(SATHEALTH, @intCast(c_int, (flag >> 4) & 0b11)),
                    };

                    data.satellites.set(idx, detail);

                    if (data.cno_max < detail.cno) {
                        data.cno_max = detail.cno;
                    }

                    idx += 1;
                }
                self._last_nav_sat = data;
            },
            else => print("gnss process_nav_packet : unknown id {}\n", .{packet.id}),
        }
    }

    fn process_cfg_packet(self: *GNSS, packet: *ubxPacket) void {
        if (packet.id == UBX_CFG_RATE and packet.len == 6) {
            const measure_rate = extract(packet, u16, 0);
            const nav_rate = extract(packet, u16, 2);
            const time_ref = extract(packet, u16, 4);
            print("gnss CFG_RATE : measure_rate {} nav_rate {} time_ref {}\n", .{ measure_rate, nav_rate, time_ref });
        }
    }

    fn send_spi_command(self: *GNSS, packet: *ubxPacket) void {
        self.write_buffer[0] = UBX_SYNCH_1;
        self.write_buffer[1] = UBX_SYNCH_2;
        self.write_buffer[2] = packet.cls;
        self.write_buffer[3] = packet.id;
        self.write_buffer[4] = @truncate(u8, packet.len);
        self.write_buffer[5] = @truncate(u8, packet.len >> 8);

        mem.copy(u8, self.write_buffer[6 .. 6 + packet.len], packet.payload[0..packet.len]);

        self.write_buffer[6 + packet.len] = packet.checksum_a;
        self.write_buffer[7 + packet.len] = packet.checksum_b;

        var rv = self.handle.transfer(&self.write_buffer, &self.read_buffer, packet.len + 8);

        self.read_buffer_index += packet.len + 8;

        // print("send_spi_command pkt : {any}\n", .{self.write_buffer[0 .. 8 + packet.len]});
        // print("                 rv  : {}\n", .{rv});
    }

    fn add_to_checksum(self: *GNSS, incoming: u8) void {
        self.cur_checksum_a +%= incoming;
        self.cur_checksum_b +%= self.cur_checksum_a;
    }

    fn check_automatic(self: *GNSS, requested_class: u8, requested_id: u8) bool {
        // TODO : implement this
        return false;
    }

    fn do_is_connected(self: *GNSS) bool {
        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_RATE;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        const value = self.send_command(&self.packet_cfg);

        if (value == UBX_Status.DATA_RECEIVED) {
            return true;
        }

        if (value == UBX_Status.DATA_RECEIVED) {
            return true;
        }

        return false;
    }

    pub fn is_connected(self: *GNSS) bool {
        var connected = self.do_is_connected();

        if (!connected) {
            connected = self.do_is_connected();
        }

        if (!connected) {
            connected = self.do_is_connected();
        }

        return connected;
    }

    pub fn set_auto_pvt_rate(self: *GNSS, rate: u8) void {
        if (rate > 127) {
            rate = 127;
        }

        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_MSG;
        self.packet_cfg.len = 3;
        self.packet_cfg.startingSpot = 0;

        self.packet_cfg.payload[0] = UBX_CLASS_NAV;
        self.packet_cfg.payload[1] = UBX_NAV_PVT;
        self.packet_cfg.payload[2] = rate; // rate relative to navigation freq.

        print("gnss set_auto_pvt_rate({})\n", .{rate});
        const value = self.send_command(&self.packet_cfg);

        if (value == UBX_Status.DATA_SENT) {
            print("gnss ack\n", .{});
        }
    }

    pub fn set_auto_pvt(self: *GNSS, value: bool) void {
        if (value) {
            self.set_auto_pvt_rate(1);
        } else {
            self.set_auto_pvt_rate(0);
        }
    }

    pub fn set_next_timeout(self: *GNSS, wait: u16) void {
        self.cur_wait = wait;
    }

    pub fn set_timeout(self: *GNSS, wait: u16) void {
        self.max_wait = wait;
    }

    pub fn poll_pvt(self: *GNSS) bool {
        self.packet_cfg.cls = UBX_CLASS_NAV;
        self.packet_cfg.id = UBX_NAV_PVT;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        // print("poll_pvt()\n", .{});
        const value = self.send_command(&self.packet_cfg);
        // print("poll_pvt() -> {}\n", .{value});
        return (value == UBX_Status.DATA_RECEIVED);
    }

    pub fn last_nav_pvt_data(self: *GNSS) ?PVTData {
        return self._last_nav_pvt;
    }

    // TODO : cache creation of the PVT struct, except for the age field --
    // that should always be updated when the method is called
    pub fn last_nav_pvt(self: *GNSS) ?PVT {
        if (self.last_nav_pvt_data()) |pvt| {
            var timestamp: [24]u8 = undefined;
            _ = std.fmt.bufPrint(&timestamp, "{d:0>4}-{d:0>2}-{d:0>2}T{d:0>2}:{d:0>2}:{d:0>6.3}Z", .{
                pvt.time.year,
                pvt.time.month,
                pvt.time.day,
                pvt.time.hour,
                pvt.time.minute,
                @intToFloat(f64, pvt.time.second) + @intToFloat(f64, pvt.time.nanosecond) * 1e-9,
            }) catch unreachable;

            return PVT{
                .age = std.time.milliTimestamp() - pvt.received_at,
                .timestamp = timestamp,
                .time = pvt.time,

                .longitude = @intToFloat(f64, pvt.position.longitude) * 1e-7,
                .latitude = @intToFloat(f64, pvt.position.latitude) * 1e-7,
                .height = @intToFloat(f32, pvt.position.height_sea_level) * 1e-3,
                .heading = @intToFloat(f32, pvt.position.heading) * 1e-5,
                .speed = @intToFloat(f32, pvt.velocity.speed) * 1e-3,
                .velocity = [3]f32{
                    @intToFloat(f32, pvt.velocity.north) * 1e-3,
                    @intToFloat(f32, pvt.velocity.east) * 1e-3,
                    @intToFloat(f32, pvt.velocity.down) * 1e-3,
                },

                .satellite_count = pvt.satellite_count,
                .flags = [_]u8{ pvt.flags1, pvt.flags2, pvt.flags3 },
                .fix_type = pvt.fix_type,
                .dop = @intToFloat(f32, pvt.position.geometric_dilution) * 1e-2,
            };
        }
        return null;
    }

    pub fn get_mon_rf(self: *GNSS) void {
        self.packet_cfg.cls = UBX_CLASS_MON;
        self.packet_cfg.id = UBX_MON_RF;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        const value = self.send_command(&self.packet_cfg);

        if (value == UBX_Status.DATA_RECEIVED) {
            while (self._last_mon_rf == null) {
                std.time.sleep(SLEEP);
            }

            print("{any}\n", .{self._last_mon_rf});
            self._last_mon_rf = null;
        } else {
            print("get_mon_rf() -> {}\n", .{value});
        }
    }

    pub fn get_mon_span(self: *GNSS) void {
        self.packet_cfg.cls = UBX_CLASS_MON;
        self.packet_cfg.id = UBX_MON_SPAN;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        const value = self.send_command(&self.packet_cfg);

        if (value == UBX_Status.DATA_RECEIVED) {
            while (self._last_mon_span == null) {
                std.time.sleep(SLEEP);
            }

            print("{any}\n", .{self._last_mon_span});
            self._last_mon_span = null;
        } else {
            print("get_mon_span() -> {}\n", .{value});
        }
    }

    pub fn get_nav_sat(self: *GNSS) void {
        self.packet_cfg.cls = UBX_CLASS_NAV;
        self.packet_cfg.id = UBX_NAV_SAT;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        const value = self.send_command(&self.packet_cfg);

        if (value == UBX_Status.DATA_RECEIVED) {
            while (self._last_nav_sat == null) {
                std.time.sleep(SLEEP);
            }

            if (self._last_nav_sat) |info| {
                print("SatInfo{{ .itow = {}, .count = {}, .cno_max = {}}}\n", .{ info.itow, info.count, info.cno_max });

                var idx: u8 = 0;
                while (idx < info.count) {
                    print("  {any}\n", .{info.satellites.get(idx)});
                    idx += 1;
                }
            }

            self._last_nav_sat = null;
        } else {
            print("get_mon_span() -> {}\n", .{value});
        }
    }

    pub fn reset(self: *GNSS, mode: ?CFG_RST_MODE) void {
        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_RST;
        self.packet_cfg.len = 4;
        self.packet_cfg.starting_spot = 0;

        const reset_mode = mode orelse CFG_RST_MODE.GNSS_RESET;

        const payload = [4]u8{ 0xFF, 0xFF, @enumToInt(reset_mode), 0x00 };
        std.mem.copy(u8, self.packet_cfg.payload[0..], payload[0..]);

        // Build the packet ourselves, as it is a CFG packet, but no ACK is expected
        calc_checksum(&self.packet_cfg);
        self.send_spi_command(&self.packet_cfg);

        var value = self.wait_for_no_ack(&self.packet_cfg, self.packet_cfg.cls, self.packet_cfg.id);
        print("gnss reset({any}) -> {}\n", .{ reset_mode, value });

        // self.read_buffer = [_]u8{0xFF} ** SPI_BUFFER_SIZE;
        std.time.sleep(std.time.ns_per_ms * 100);
    }

    pub fn configure(self: *GNSS) void {
        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_PRT;
        self.packet_cfg.len = 1;
        self.packet_cfg.starting_spot = 0;

        // Get setting for port 4 (e.g. SPI)
        self.packet_cfg.payload[0] = 4;

        var value = self.send_command(&self.packet_cfg);
        print("gnss CFG_PRT(SPI) GET -> {}\n", .{value});

        self.packet_cfg.len = 20;

        // Enable only UBX messages (e.g. bit 1 is set)
        self.packet_cfg.payload[14] = 1;
        value = self.send_command(&self.packet_cfg);
        print("gnss CFG_PRT(SPI) SET {any} -> {}\n", .{ self.packet_cfg.payload[0..self.packet_cfg.len], value });

        // Configure signal attenuation compensation
        // 0   -> disables signal attenuation compensation
        // 255 -> automatic signal attenuation compensation
        // 1..63 -> maximum expected C/NO level is this dB value
        const sig_att_comp: u8 = 0;
        const dynmodel: u8 = @enumToInt(CFG_NAVSPG_DYNMODEL_MODE.AUTOMOT);

        const payload: [10]u8 = CFG_NAVSPG_SIGATTCOMP ++ [_]u8{sig_att_comp} ++ CFG_NAVSPG_DYNMODEL ++ [_]u8{dynmodel};
        print("gnss CFG_NAVSPG_SIGATTCOMP : {}\n", .{sig_att_comp});
        print("gnss CFG_NAVSPG_DYNMODEL : {}\n", .{dynmodel});

        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_VALSET;
        self.packet_cfg.len = 4 + payload.len;
        self.packet_cfg.starting_spot = 0;

        // Copy in the message header, bit 1 of byte 1 tells modules to apply setting to RAM layer only
        const header: [4]u8 = [_]u8{ 0x00, 0x01, 0x00, 0x00 };
        std.mem.copy(u8, self.packet_cfg.payload[0..], header[0..]);
        std.mem.copy(u8, self.packet_cfg.payload[4..], payload[0..]);

        value = self.send_command(&self.packet_cfg);
        print("gnss CFG_VALSET {any} -> {}\n", .{ payload, value });
    }

    pub fn set_interval(self: *GNSS, rate: u16) void {
        self.packet_cfg.cls = UBX_CLASS_CFG;
        self.packet_cfg.id = UBX_CFG_RATE;
        self.packet_cfg.len = 0;
        self.packet_cfg.starting_spot = 0;

        var value = self.send_command(&self.packet_cfg);

        self.packet_cfg.len = 6;
        self.packet_cfg.payload[0] = @truncate(u8, rate);
        self.packet_cfg.payload[1] = @truncate(u8, rate >> 8);

        value = self.send_command(&self.packet_cfg);
        print("gnss set_interval({}) -> {}\n", .{ rate, value });

        self.packet_cfg.payload[0] = 0;
        self.packet_cfg.payload[1] = 0;

        // Do read back
        self.packet_cfg.len = 0;
        value = self.send_command(&self.packet_cfg);
    }
};
