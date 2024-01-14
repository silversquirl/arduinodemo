const std = @import("std");
const init = @import("init.zig");
const small_sleep = @import("sleep.zig").small_sleep;
const Mmio = @import("mmio.zig").Mmio;
const LCD = @import("KS0066U.zig");
const lcd = @import("KS0066U.zig").KS0066U(mmio, ENABLE, RS, D4, D5, D6, D7);

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
    lcd.init();

    lcd.set_cursor(0,0);
    var idx: u8 = '0';
    while (idx <= '9') : (idx += 1) {
        lcd.write(idx);
        small_sleep(3);
    }
    // lcd.set_cursor(16, 1);
    lcd.command(LCD.set_ddram_addr | 0x40 + 16);
    // enable autoscrol
    lcd.command(LCD.entry_mode_set | LCD.entry_left | LCD.entry_enable_shift);
    idx = '0';
    while (idx <= '9') : (idx += 1) {
        lcd.write(idx);
        small_sleep(3);
    }
    lcd.command(LCD.entry_mode_set | LCD.entry_left);
    lcd.command(LCD.clear_display);
    small_sleep(1);



    for ("It's alive!") |b| lcd.write(b); // loop is 22 byte prelude, 6 byte prologue (register allocation is still awful), string is 11 bytes. `write` in inlined: 14 bytes, nice!
    
    var cmd: u8 = LCD.display_control; // 2 bytes

    sbi(DDRB, LED_PIN);
    while (true) {
        sbi(PINB, LED_PIN);
        cmd ^= LCD.display_on; // 2 bytes + 2 bytes allocating reg
        lcd.command(cmd); // 6 bytes - failed to allocate registers again. This should be 4 bytes
        small_sleep(3);
    }
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


