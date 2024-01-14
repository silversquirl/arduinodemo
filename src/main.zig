const std = @import("std");
const init = @import("init.zig");
const small_sleep = @import("sleep.zig").small_sleep;

const uno = @import("arduino_uno_rev3.zig");
const rt = uno.use();
const io = rt.mcu.get_memory_space();

const RAM_END = 0x8FF;
const F_CPU = 16000000;
const LED_PIN = 5;
const DDRB: *volatile u8 = @ptrFromInt(0x24);
const PORTB: *volatile u8 = @ptrFromInt(0x25);
const PINB: *volatile u8 = @ptrFromInt(0x23);
const SP: *volatile u16 = @ptrFromInt(0x5D);
    
fn sbi(reg: *volatile u8, bit: u3) void {
    reg.* |= @as(u8, 1) << bit;
}
const BAUD_RATE = 9600; // can go up to 500_000 stably. Datasheet claims 1M but messages start getting garbled in arduino studio
const BAUD_PRESCALE = ((F_CPU / 8 / BAUD_RATE) - 1);

const UDR0 = 0xC6;
const UDRE0 = 0x05;
const UCSR0A = 0xC0;
fn spin_until_bit_set(comptime reg: *volatile u8, comptime bit: u8) void {
    var tmp: u8 = undefined;
    asm volatile(
        \\ 0:
        \\   lds r16, %[reg]
        \\   sbrs r16, %[bit]
        \\   rjmp 0b
        : [tmp] "=&r" (tmp)
        : [bit] "I" (bit),
          [reg] "i" (reg)
    );
}
fn out(c: u8) void {
    spin_until_bit_set(&io.UCSR0A.byte, 5);
    io.UDR0 = c;
}

fn cli() void {
    asm volatile(
        \\ cli
        ::: "cc"
    );
}
const PINS: [4]Pin = .{ D4, D5, D6, D7 };

// Ideally we'd use a calling convention with full caller-preserves, but that's not supported yet
fn main() noreturn {
    cli();
    init.copy_data_to_ram();
    init.clear_bss();

    io.UCSR0A.byte = 0b00000010;
    io.UBRR0L = @truncate(BAUD_PRESCALE);
    io.UBRR0H = @truncate(BAUD_PRESCALE >> 8);
    io.UCSR0C.byte = 0b00000110;
    io.UCSR0B.byte = 0b00011000;

    for ("Hello, World!\n") |c| out(c);
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
    small_sleep(3);         // 12

    write_four_bits(0x2); // 6

    command(function_set | two_line | five_by_eight_dots | four_bit_mode); // 6
    command(display_control | display_on | cursor_on | blink_on); // 6
    command(clear_display); // 6
    small_sleep(1); // 12
    command(entry_mode_set | entry_left | entry_shift_decrement); // 6 (commands r all great. RCALLs could be better)

    for ("It's alive!") |b| write(b); // loop is 22 byte prelude, 6 byte prologue (register allocation is still awful), string is 11 bytes. `write` in inlined: 14 bytes, nice!
    
    var cmd: u8 = display_control; // 2 bytes

    sbi(DDRB, LED_PIN);
    while (true) {
        sbi(PINB, LED_PIN);
        cmd ^= display_on; // 2 bytes + 2 bytes allocating reg
        command(cmd); // 6 bytes - failed to allocate registers again. This should be 4 bytes
        small_sleep(3);
    }
}
fn pulse_enable() void {
    mmio.setPin(ENABLE, false); // perfect: 2 bytes
    small_sleep(4);                 // 14 bytes
    mmio.setPin(ENABLE, true);
    small_sleep(4); // enable pulse must be >450ns  // 8 bytes! good job :)
    mmio.setPin(ENABLE, false);
    small_sleep(2); // commands need > 37us to settle // 12 bytes
}
const mask: u8 = (1 << D4.pin_idx()) | (1 << D5.pin_idx()) | (1 << D6.pin_idx()) | (1 << D7.pin_idx());
comptime {
    std.debug.assert(D4.data_register() == D5.data_register());
    std.debug.assert(D4.data_register() == D6.data_register());
    std.debug.assert(D4.data_register() == D7.data_register());
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
fn command(value: u8) void {
    mmio.setPin(RS, false); // 2 bytes
    write_eight_bits(value); // 4 bytes
    // 2 bytes for return (aw. should've tailed)
}
fn write(value: u8) void {
    mmio.setPin(RS, true);
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
export fn trampoline() callconv(.C) noreturn {
    @call(.always_inline, main, .{});
}

export fn _start() callconv(.Naked) noreturn {
    asm volatile(
        \\ ldi r16, lo8(%[RAM_END])
        \\ out (%[SP]-0x20), r16
        \\ ldi r16, hi8(%[RAM_END])
        \\ out (%[SP]-0x20+1), r16
        \\
        \\ jmp trampoline
        :
        :   [RAM_END] "i" (RAM_END),
            [SP] "i" (SP),
        : "r16", "m"
    );

}
const mmio = Mmio { .io = rt.mcu.get_io_space() };

const RS = uno.digital_pin(12).?;
const ENABLE = uno.digital_pin(11).?;
const D4 = uno.digital_pin(5).?;
const D5 = uno.digital_pin(4).?;
const D6 = uno.digital_pin(3).?;
const D7 = uno.digital_pin(2).?;

const Pin = uno.MegaAVR.PortPin;
const PinMode = enum {
    input,
    output
};

const Mmio = struct {
    io: *volatile [256]u8,
    pub inline fn high(self: *const @This(), addr: u8, bit: u3) void {
        self.io[addr] |= @as(u8, 1) << bit;
    }
    pub inline fn low(self: *const @This(), addr: u8, bit: u3) void {
        self.io[addr] &= ~(@as(u8, 1) << bit);
    }
    pub inline fn set(self: *const @This(), addr: u8, bit: u3, value: bool) void {
        if (value) {
            self.high(addr, bit);
        } else {
            self.low(addr, bit);
        }
    }
    pub fn setPin(self: *const @This(), pin: Pin, value: bool) void {
        self.set(pin.data_register(), pin.pin_idx(), value);
    }
    pub fn togglePin(self: *const @This(), pin: Pin) void {
        self.set(pin.input_register(), pin.pin_idx(), true);
    }
    pub fn setPinMode(self: *const @This(), pin: Pin, mode: PinMode) void {
        self.set(pin.data_direction_register(), pin.pin_idx(), mode == .output);
    }
};


// // LCD driven using the KS0066U controller
// // https://pdf1.alldatasheet.com/datasheet-pdf/download/37318/SAMSUNG/KS0066.html
// // Definitions:

// Commands
const clear_display = 0x01;
const return_home = 0x02;
const entry_mode_set = 0x04;
const display_control = 0x08;
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
const display_on = 0x04;
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

