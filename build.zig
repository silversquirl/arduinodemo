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
    const imports = b.allocator.alloc(std.Build.Module.Import, 2) catch @panic("OOM");
    const uno = b.addModule("arduino_uno_rev3", .{ .root_source_file = .{ .path = "src/arduino_uno_rev3.zig" }});
    const init = b.addModule("compiler_rt", .{ .root_source_file = .{.path ="src/init.zig" }, .imports = &.{.{
        .name = "arduino_uno_rev3",
        .module = uno,
    }}});
    imports[0] = .{
        .name = "compiler_rt",
        .module = init
    };
    imports[1] = .{
        .name = "arduino_uno_rev3",
        .module = uno,
    };
    sketch_elf.options.root_module.imports = imports;
    // std.Build.addExecutable
    const firmware = b.addObjCopy(sketch_elf.getEmittedBin(), .{ .format = .bin });
    
    arduino.addUpload(target_device, firmware.getOutput());
    
    // Install the firmware in zig-out by default
    b.getInstallStep().dependOn(&b.addInstallBinFile(sketch_elf.getEmittedBin(), "sketch.elf").step);
    b.getInstallStep().dependOn(&b.addInstallBinFile(firmware.getOutput(), "sketch.bin").step);
}