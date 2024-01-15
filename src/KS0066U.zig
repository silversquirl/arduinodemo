const mmio_ = @import("mmio.zig");
const std = @import("std");
const small_sleep = @import("sleep.zig").small_sleep;


fn ComptimePin(comptime impl: anytype) type {
    return struct {
        pub inline fn set(_: @This(), value: bool) void {
            return impl.set(value);
        }
        pub inline fn setMode(_: @This(), mode: mmio_.PinMode) void {
            return impl.setMode(mode);
        }
        pub inline fn data_register(_: @This()) *volatile u8 {
            return impl.data_register();
        }
        pub inline fn pin_idx(_: @This()) u8 {
            return impl.pin_idx();
        }
        pub inline fn singleton() @This() {
            return .{};
        }
    };
}
pub fn init(comptime enable: anytype, comptime rs: anytype, comptime d4: anytype, comptime d5: anytype, comptime d6: anytype, comptime d7: anytype)
KS0066U(ComptimePin(enable), ComptimePin(rs), ComptimePin(d4), ComptimePin(d5), ComptimePin(d6), ComptimePin(d7))
{
    return KS0066U(ComptimePin(enable), ComptimePin(rs), ComptimePin(d4), ComptimePin(d5), ComptimePin(d6), ComptimePin(d7))
    .init(.{}, .{}, .{}, .{}, .{}, .{});
}
pub fn KS0066U(
    comptime ENABLE: anytype,
    comptime RS: anytype,
    comptime D4: anytype,
    comptime D5: anytype,
    comptime D6: anytype,
    comptime D7: anytype
) type {
    // const PINS = .{ D4, D5, D6, D7 };
    const Data = struct { d4: D4, d5: D5, d6: D6, d7: D7 };
    const set_bits = brk: while (true) {
        if (@hasDecl(D4, "singleton")
        and @hasDecl(D5, "singleton")
        and @hasDecl(D6, "singleton")
        and @hasDecl(D7, "singleton")
        ) {
            const d4 = D4.singleton();
            const d5 = D5.singleton();
            const d6 = D6.singleton();
            const d7 = D7.singleton();
            if (d4.data_register() == d5.data_register() 
            and d4.data_register() == d6.data_register()
            and d4.data_register() == d7.data_register()) {
                const mask: u8 = (1 << d4.pin_idx()) | (1 << d5.pin_idx()) | (1 << d6.pin_idx()) | (1 << d7.pin_idx());

                break :brk struct {
                    fn f(port: *Data, bits: u8) void {
                        
                        d4.data_register().* &= ~mask;

                        // These ifs should be compiling as sbrs, but use (in->and->or->out) instead.
                        if (bits & 0x01 != 0) port.d4.set( true);
                        if (bits & 0x02 != 0) port.d5.set( true);
                        if (bits & 0x04 != 0) port.d6.set( true);
                        if (bits & 0x08 != 0) port.d7.set( true);
                    }
                }.f;
            }

        }
        break :brk struct {
            fn f(port: *Data, value: u8) void {
                port.d4.set(value & 0x01 != 0);
                port.d5.set(value & 0x02 != 0);
                port.d6.set(value & 0x04 != 0);
                port.d7.set(value & 0x08 != 0);
            }
        }.f;
    };
    return struct {
        pub const Error = error {};
        // comptime {
        // }
        enable: ENABLE,
        rs: RS,
        port: Data,

        pub fn init(rs: RS, enable: ENABLE, d4: D4, d5: D5, d6: D6, d7: D7) @This() {
            
            rs.setMode(.output);
            enable.setMode(.output);

            // Be careful with optimizations! Removing `inline` here generates 60 bytes of iteration and translation logic
            // FIXME: Improve codegen for setPinMode
            inline for (.{ d4, d5, d6, d7 }) |pin| pin.setMode( .output);

            rs.set( false);
            enable.set( false);

            const self = @This() {
                .enable = enable,
                .rs = rs,
                .port = .{ .d4 = d4, .d5 = d5, .d6 = d6, .d7 = d7 },
            };

            small_sleep(0);
            self.write_four_bits(0x3); // long call. 6 bytes
            small_sleep(1);       // inlined : 16 bytes. codegen is not great here, it shuffles a lot of registers around
                                // A call is 2 bytes, and 2 more for the arg somewhere. + 1 nop prelude in a sleep block makes this 3x smaller, and even better when a sleep is reused.

            self.write_four_bits(0x3); // 6
            small_sleep(1);        // 10! It's smart abt reusing the registers from above. It's probably optimal at runtime, cool!

            self.write_four_bits(0x3); // 6
            small_sleep(2);         // 12

            self.write_four_bits(0x2); // 6

            self.command(function_set | two_line | five_by_eight_dots | four_bit_mode); // 6
            self.command(display_control | display_on | cursor_off | blink_off); // 6
            self.command(clear_display); // 6
            small_sleep(1); // 12
            self.command(entry_mode_set | entry_left | entry_disable_shift); // 6 (commands r all great. RCALLs could be better)


            return self;
        }

        fn write_four_bits(self_: @This(), bits: u8) void {
            var self = self_;
            set_bits(&self.port, bits);
            
            self_.pulse_enable(); // inlined.
        }
        pub fn set_cursor(self: @This(), col: u8, row: u8) void {
            self.set_ddram_addr(row * 0x40 + col);
        }
        pub fn clear(self: @This()) void {
            self.command(clear_display); 
            small_sleep(1); 
        }
        
        pub fn print(self: @This(), comptime format: []const u8, args: anytype) void {
            return std.fmt.format(self, format, args) catch |e| {
                switch(e) {}
            };
        }
        pub fn display(self: @This(), on: bool, cursor: bool, blink: bool) void {
            var cmd: u8 = display_control;
            if (on) cmd |= display_on;
            if (cursor) cmd |= cursor_on;
            if (blink) cmd |= blink_on;
            self.command(cmd);
        }
        
        pub fn set_ddram_addr(self: @This(), addr: u8) void {
            self.command(SET_DRAM_ADDR | addr); 
        }
        pub fn set_entry_mode(self: @This(), direction: Direction, follow_cursor: ViewMode) void {
            self.command(entry_mode_set | @intFromEnum(direction) | @intFromEnum(follow_cursor)); // 6 bytes
        }
        pub fn command(self: @This(), value: u8) void {
            var self_ = self;
            self_.rs.set( false); // 2 bytes
            self.write_eight_bits(value); // 4 bytes
            // 2 bytes for return (aw. should've tailed)
        }
        pub fn write(self: @This(), value: u8) void {
            self.rs.set( true);
            self.write_eight_bits(value);
        }
        pub fn writeByteNTimes(self: @This(), byte: u8, n: usize) Error!void {
            for (0..n) |_| {
                self.write(byte);
            }
        }
        pub fn writeAll(self: @This(), value: []const u8) Error!void {
            for (value) |byte| {
                self.write(byte);
            }
        }
        fn swap_nibbles(value: u8) u8 {
            return (value << 4) | (value >> 4);
        }
        fn write_eight_bits(self: @This(), value: u8) void {
            self.write_four_bits(swap_nibbles(value)); // 16 bytes (need to move registers around. callconv doesn't give any scratch)
            self.write_four_bits(value); // 4 bytes
            // return 2
        }
        
        fn pulse_enable(self: @This()) void {
            self.enable.set( false); // perfect: 2 bytes
            small_sleep(4);                 // 14 bytes
            self.enable.set( true);
            small_sleep(4); // enable pulse must be >450ns  // 8 bytes! good job :)
            self.enable.set( false);
            small_sleep(2); // commands need > 37us to settle // 12 bytes
        }
    };
}

pub const Direction = enum(u8) { increment = 2, decrement = 0 };
pub const ViewMode = enum(u8) { follow = 1, fixed = 0 };
// // LCD driven using the KS0066U controller
// // https://pdf1.alldatasheet.com/datasheet-pdf/download/37318/SAMSUNG/KS0066.html
// // Definitions:

// Commands
pub const clear_display = 0x01;
const return_home = 0x02;
pub const entry_mode_set = 0x04;
pub const display_control = 0x08;
const cursor_shift = 0x10;
const function_set = 0x20;
const set_cgram_addr = 0x40;
pub const SET_DRAM_ADDR = 0x80;

// Entry mode set flags
pub const entry_right = 0x00;
pub const entry_left = 0x02;
pub const entry_enable_shift = 0x01;
pub const entry_disable_shift = 0x00;


// Display control flags
pub const display_on = 0x04;
const display_off = 0x00;
const cursor_on = 0x02;
const cursor_off = 0x00;
const blink_on = 0x01;
const blink_off = 0x00;

// Cursor shift flags
const display_move = 0x08;
const cursor_move = 0x00;
const move_right = 0x04;
const move_left = 0x00;

// Function set flags
const eight_bit_mode = 0x10;
const four_bit_mode = 0x00;
const two_line = 0x08;
const one_line = 0x00;
const five_by_ten_dots = 0x04;
const five_by_eight_dots = 0x00;

// Busy flag
const busy_flag = 0x80;

pub const LINE2_ADDRESS = 0x40;