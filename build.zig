const std = @import("std");
pub fn build(b: *std.Build) !void {
    const target = std.Build.resolveTargetQuery(b, .{
        .cpu_arch = .avr,
        .os_tag = .freestanding,
        .abi = .eabi,

        .cpu_model = .{ .explicit = &std.Target.avr.cpu.atmega328p },
    });
    const bootsector = b.addObject(.{
        .name = "firmware",
        .root_source_file = .{ .path = "src/main.zig" },
        .target = target,
        .optimize = .ReleaseSmall,
    });
    const out = std.Build.Step.Compile.create(b, .{
        .name = "final",
        .kind = .exe,
        .root_module = .{
            .target = target,
        },
    });
    out.addObjectFile(bootsector.getEmittedBin());
    const bootsector_bin = b.addObjCopy(out.getEmittedBin(), .{
        .format = .bin,
    });
    b.getInstallStep().dependOn(&b.addInstallBinFile(bootsector_bin.getOutput(), "main.bin").step);
}