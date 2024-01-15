const mmio_ = @import("mmio.zig");
const std = @import("std");
const small_sleep = @import("sleep.zig").small_sleep;
pub fn KS0066U(
    comptime mmio: mmio_.Mmio,
    comptime ENABLE: mmio_.Pin,
    comptime RS: mmio_.Pin,
    comptime D4: mmio_.Pin,
    comptime D5: mmio_.Pin,
    comptime D6: mmio_.Pin,
    comptime D7: mmio_.Pin
) type {
    const PINS: [4]mmio_.Pin = .{ D4, D5, D6, D7 };
    return struct {
        const mask: u8 = (1 << D4.pin_idx()) | (1 << D5.pin_idx()) | (1 << D6.pin_idx()) | (1 << D7.pin_idx());
        pub const Error = error {};
        comptime {
            std.debug.assert(D4.data_register() == D5.data_register());
            std.debug.assert(D4.data_register() == D6.data_register());
            std.debug.assert(D4.data_register() == D7.data_register());
        }

        pub fn init() @This() {
            
            mmio.setPinMode(RS,.output);
            mmio.setPinMode(ENABLE,.output);

            // Be careful with optimizations! Removing `inline` here generates 60 bytes of iteration and translation logic
            // FIXME: Improve codegen for setPinMode
            inline for (PINS) |pin| mmio.setPinMode(pin, .output);

            mmio.setPin(RS, false);
            mmio.setPin(ENABLE, false);

            small_sleep(0);
            write_four_bits(0x3); // long call. 6 bytes
            small_sleep(1);       // inlined : 16 bytes. codegen is not great here, it shuffles a lot of registers around
                                // A call is 2 bytes, and 2 more for the arg somewhere. + 1 nop prelude in a sleep block makes this 3x smaller, and even better when a sleep is reused.

            write_four_bits(0x3); // 6
            small_sleep(1);        // 10! It's smart abt reusing the registers from above. It's probably optimal at runtime, cool!

            write_four_bits(0x3); // 6
            small_sleep(2);         // 12

            write_four_bits(0x2); // 6

            command(function_set | two_line | five_by_eight_dots | four_bit_mode); // 6
            command(display_control | display_on | cursor_off | blink_off); // 6
            command(clear_display); // 6
            small_sleep(1); // 12
            command(entry_mode_set | entry_left | entry_disable_shift); // 6 (commands r all great. RCALLs could be better)


            return @This() {};
        }

        fn write_four_bits(bits: u8) void {
            mmio.io[D4.data_register()] &= ~mask;

            // These ifs should be compiling as sbrs, but use (in->and->or->out) instead.
            if (bits & 0x01 != 0) mmio.setPin(D4, true);
            if (bits & 0x02 != 0) mmio.setPin(D5, true);
            if (bits & 0x04 != 0) mmio.setPin(D6, true);
            if (bits & 0x08 != 0) mmio.setPin(D7, true);
            
            pulse_enable(); // inlined.
        }
        pub fn set_cursor(self: @This(), col: u8, row: u8) void {
            self.set_ddram_addr(row * 0x40 + col);
        }
        pub fn clear(_: @This()) void {
            command(clear_display); 
            small_sleep(1); 
        }
        
        pub fn print(self: @This(), comptime format: []const u8, args: anytype) void {
            return std.fmt.format(self, format, args) catch |e| {
                switch(e) {}
            };
        }
        pub fn display(_: @This(), on: bool, cursor: bool, blink: bool) void {
            var cmd: u8 = display_control;
            if (on) cmd |= display_on;
            if (cursor) cmd |= cursor_on;
            if (blink) cmd |= blink_on;
            command(cmd);
        }
        
        pub fn set_ddram_addr(_: @This(), addr: u8) void {
            command(SET_DRAM_ADDR | addr); 
        }
        pub fn set_entry_mode(_: @This(), direction: Direction, follow_cursor: ViewMode) void {
            command(entry_mode_set | @intFromEnum(direction) | @intFromEnum(follow_cursor)); // 6 bytes
        }
        pub fn command(value: u8) void {
            mmio.setPin(RS, false); // 2 bytes
            write_eight_bits(value); // 4 bytes
            // 2 bytes for return (aw. should've tailed)
        }
        pub fn write(_: @This(), value: u8) void {
            mmio.setPin(RS, true);
            write_eight_bits(value);
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
            // return value.len;
        }
        fn swap_nibbles(value: u8) u8 {
            return (value << 4) | (value >> 4);
        }
        fn write_eight_bits(value: u8) void {
            write_four_bits(swap_nibbles(value)); // 16 bytes (need to move registers around. callconv doesn't give any scratch)
            write_four_bits(value); // 4 bytes
            // return 2
        }
        
        fn pulse_enable() void {
            mmio.setPin(ENABLE, false); // perfect: 2 bytes
            small_sleep(4);                 // 14 bytes
            mmio.setPin(ENABLE, true);
            small_sleep(4); // enable pulse must be >450ns  // 8 bytes! good job :)
            mmio.setPin(ENABLE, false);
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