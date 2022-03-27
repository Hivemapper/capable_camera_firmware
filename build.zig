const std = @import("std");
const Target = std.Target;
const CrossTarget = std.zig.CrossTarget;

pub fn build(b: *std.build.Builder) void {

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();
    const target = CrossTarget{
        .cpu_arch = Target.Cpu.Arch.arm,
        .os_tag = Target.Os.Tag.linux,
        .abi = Target.Abi.musleabihf,
    };

    const exif = b.addStaticLibrary("exif", null);
    exif.setTarget(target);
    exif.setBuildMode(mode);
    exif.linkLibC();
    exif.addIncludeDir("packages/c");
    exif.addCSourceFiles(&.{
        "packages/c/libexif/exif-data.c",
        "packages/c/libexif/exif-utils.c",
        "packages/c/libexif/exif-format.c",
        "packages/c/libexif/exif-content.c",
        "packages/c/libexif/exif-loader.c",
        "packages/c/libexif/exif-mnote-data.c",
        "packages/c/libexif/exif-entry.c",
        "packages/c/libexif/exif-gps-ifd.c",
        "packages/c/libexif/exif-tag.c",
        "packages/c/libexif/exif-ifd.c",
        "packages/c/libexif/exif-log.c",
        "packages/c/libexif/exif-mem.c",
    }, &.{
        "-Wall",
        "-W",
        "-Wstrict-prototypes",
        "-Wwrite-strings",
        "-Wno-missing-field-initializers",
    });

    const exe = b.addExecutable("capable_camera_firmware", "src/main.zig");
    exe.linkSystemLibrary("c");
    exe.linkLibrary(exif);

    exe.addIncludeDir("packages/c");
    exe.addPackagePath("zhp", "packages/zhp/src/zhp.zig");

    exe.setTarget(target);
    exe.setBuildMode(mode);
    exe.install();

    const run_cmd = exe.run();
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);
}
