const std = @import("std");
const arduino = @import("arduino");

// pub fn build(b: *std.Build) !void {
//     const target_device = arduino.standardTargetDeviceOptions(b, .{});

//     const sketch = b.addExecutable(.{
//         .name = "sketch",
//         .root_source_file = .{ .path = "src/main.zig" },
//         .target = target_device.resolvedTarget(),
//         .optimize = .ReleaseSmall,
//     }).getEmittedBin();
//     const firmware = b.addObjCopy(sketch.getEmittedBin(), .{ .format = .bin });

//     arduino.addUpload(b, target_device, firmware.getOutput());

//     // Install the firmware in zig-out by default
//     b.getInstallStep().dependOn(&b.addInstallBinFile(sketch.getEmittedBin(), "sketch.elf").step);
//     b.getInstallStep().dependOn(&b.addInstallBinFile(firmware.getOutput(), "sketch.bin").step);
// }

const DeferredCompile = struct {
    step: std.Build.Step,
    options: std.Build.Step.Compile.Options,
    override_target: ?arduino.Lazy(std.Build.ResolvedTarget) = null,

    out_bin: std.Build.GeneratedFile,
    fn init(b: *std.Build) *@This() {
        const self = b.allocator.create(DeferredCompile) catch @panic("OOM");
        self.* = .{
            .step = std.Build.Step.init(.{
                .id = .compile,
                .name = "zig-build sketch ReleaseSmall",
                .makeFn = DeferredCompile.makeFn,
                .owner = b,
            }),
            .out_bin = .{
                .step = &self.step,
            },
            .options = .{
                .name = "sketch",
                .root_module = .{
                    .root_source_file = .{ .path = "src/main.zig" },
                    
                    .optimize = .ReleaseSmall,
                },
                .kind = .exe,
            },
        };
        return self;
    }
    fn setTarget(self: *DeferredCompile, target: arduino.Lazy(std.Build.ResolvedTarget)) void {
        target.addStepDependencies(&self.step);
        self.override_target = target;
    }

    fn makeFn(step: *std.Build.Step, node: *std.Progress.Node) !void {
        const self = @fieldParentPtr(@This(), "step", step);

        if (self.override_target) |target| self.options.root_module.target = target.get().*;
        const sketch_elf_ =  std.Build.Step.Compile.create(step.owner, self.options);
        const bin = sketch_elf_.getEmittedBin();

        std.debug.assert(sketch_elf_.step.dependencies.items.len == 0);
        sketch_elf_.step.make(node) catch |e| {
            step.result_error_msgs.appendSlice(step.owner.allocator , sketch_elf_.step.result_error_msgs.items) catch @panic("OOM");
            step.result_error_bundle = sketch_elf_.step.result_error_bundle;
            return e;
        };

        self.out_bin.path = bin.getPath(step.owner);
    }
    fn getEmittedBin(self: *const DeferredCompile) std.Build.LazyPath {
        return .{ .generated = &self.out_bin };
    }
};
pub fn build(b: *std.Build) !void {
    const target_device = arduino.standardTargetDeviceOptions(b, .{});
    // b.addExecutable(options: ExecutableOptions)
    // Target { .ofmt = .raw } is WIP, so manually extract the binary for now.
    // FIXME: We probably need a linker script from `arduino`.
    // const sketch_elf = b.addExecutable(.{
    //     .name = "sketch",
    //     .root_source_file = .{ .path = "src/main.zig" },
    //     .target = target_device.resolvedTarget(),
    //     .optimize = .ReleaseSmall,
    // }).getEmittedBin();
    
                    // .target = target_device.resolvedTarget(),
    
    const sketch = DeferredCompile.init(b);
    sketch.setTarget(target_device.target);
    const firmware = b.addObjCopy(sketch.getEmittedBin(), .{ .format = .bin });

    arduino.addUpload(b, target_device, firmware.getOutput());
    
    // Install the firmware in zig-out by default
    b.getInstallStep().dependOn(&b.addInstallBinFile(sketch.getEmittedBin(), "sketch.elf").step);
    b.getInstallStep().dependOn(&b.addInstallBinFile(firmware.getOutput(), "sketch.bin").step);
}
    // const target = std.Build.resolveTargetQuery(b, .{
    //     .cpu_arch = .avr,
    //     .os_tag = .freestanding,
    //     .abi = .eabi,

    //     // TODO: Directly output to .bin when it's implemented
    //     // .ofmt = .raw,

    //     // .cpu_features_add = featureSet(&.{std.Target.avr.Feature.jmpcall}),
    //     // .cpu_model = std.Target..Qyavr.cpu.atmega328p,
    //     .cpu_model = .{ .explicit = &std.Target.avr.cpu.atmega328p },
    // });
