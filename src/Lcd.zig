const std = @import("std");
const small_sleep = @import("sleep.zig").small_sleep;

// // LCD driven using the KS0066U controller
// // https://pdf1.alldatasheet.com/datasheet-pdf/download/37318/SAMSUNG/KS0066.html
// // Definitions:

// Commands
const clear_display = 0x01;
const return_home = 0x02;
const entry_mode_set = 0x04;
pub const display_control = 0x08;
const cursor_shift = 0x10;
const function_set = 0x20;
const set_cgram_addr = 0x40;
const set_ddram_addr = 0x80;

// Entry mode set flags
const entry_right = 0x00;
const entry_left = 0x02;
const entry_shift_increment = 0x01;
const entry_shift_decrement = 0x00;

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



pub fn Lcd(comptime Pin: type, comptime enable: Pin, comptime rs: Pin, comptime pins: [4]Pin) type {
    std.debug.assert(pins[0].data_register() == pins[1].data_register());
    std.debug.assert(pins[0].data_register() == pins[2].data_register());
    std.debug.assert(pins[0].data_register() == pins[3].data_register());

    const mask = pins[0].mask() | pins[1].mask() | pins[2].mask() | pins[3].mask();
    return struct {
        pub fn init() @This() {
            var me = @This() {};
            rs.setMode(.output);
            // mmio.setPinMode(ENABLE,.output);
            enable.setMode(.output);

            // Be careful with optimizations! Removing `inline` here generates 60 bytes of iteration and translation logic
            // FIXME: Improve codegen for setPinMode
            // inline for (PINS) |pin| mmio.setPinMode(pin, .output);
            inline for (pins) |pin| pin.setMode(.output);

            // mmio.setPin(RS, false);
            rs.set(false);
            // mmio.setPin(ENABLE, false);
            enable.set(false);

            small_sleep(0);
            write_four_bits(0x3);
            small_sleep(1);

            write_four_bits(0x3);
            small_sleep(1);

            write_four_bits(0x3); 
            small_sleep(3);

            write_four_bits(0x2); 

            me.command(function_set | two_line | five_by_eight_dots | four_bit_mode); 
            me.command(display_control | display_on | cursor_on | blink_on); 
            me.command(clear_display); 
            small_sleep(1); 
            me.command(entry_mode_set | entry_left | entry_shift_decrement); 

            return me;
        }

        fn write_four_bits(bits: u8) void {
            pins[0].data_register().* &= ~mask;

            // These ifs should be compiling as sbrs, but use (in->and->or->out) instead.
            if (bits & 0x01 != 0) pins[0].set(true);
            if (bits & 0x02 != 0) pins[1].set(true);
            if (bits & 0x04 != 0) pins[2].set(true);
            if (bits & 0x08 != 0) pins[3].set(true);
            
            pulse_enable(); // inlined.
        }
        pub fn command(_: *@This(), value: u8) void {
            rs.set(false); // 2 bytes
            write_eight_bits(value); // 4 bytes
            // 2 bytes for return (aw. should've tailed)
        }
        pub fn write(_: *@This(), value: u8) void {
            rs.set(true);
            write_eight_bits(value);
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
            enable.set(true);
            small_sleep(4);                 // 14 bytes
            enable.set(false);
            small_sleep(4); 
            enable.set(true);
            small_sleep(2); // commands need > 37us to settle // 12 bytes
        }
    };
}