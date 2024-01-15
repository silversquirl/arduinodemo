const std = @import("std");

pub fn build(b: *std.Build) !void {
    var arduino = @import("arduino").buildKit(b, "arduino");
    const target_device = arduino.standardTargetDeviceOptions(.{});
    
    // Target { .ofmt = .raw } is WIP, so manually extract the binary for now.
    // FIXME: We probably need a linker script from `arduino`.
    const sketch_elf = arduino.addExecutable(.{
        .name = "sketch",
        .root_source_file = .{ .path = "src/main.zig" },
        .optimize = .ReleaseSmall,
        // Single threaded mode lets us use `stderr_mutex` for now. TODO: Add our own mutex implementation that handles interrupts right.
        .single_threaded = true,
    }, .{ .target = target_device.target, });
    sketch_elf.setLinkerScript(.{ .path = "src/linker.ld" });
    const firmware = b.addObjCopy(sketch_elf.getEmittedBin(), .{ .format = .bin });
    
    arduino.addUpload(target_device, firmware.getOutput());
    
    // Install the firmware in zig-out by default
    b.getInstallStep().dependOn(&b.addInstallBinFile(sketch_elf.getEmittedBin(), "sketch.elf").step);
    b.getInstallStep().dependOn(&b.addInstallBinFile(firmware.getOutput(), "sketch.bin").step);
}