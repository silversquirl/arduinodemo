const std = @import("std");
const init = @import("init.zig");
const small_sleep = @import("sleep.zig").small_sleep;
const Mmio = @import("mmio.zig").Mmio;
const LCD = @import("KS0066U.zig");
const lcd = @import("KS0066U.zig").KS0066U(mmio, ENABLE, RS, D4, D5, D6, D7);

const uno = @import("arduino_uno_rev3.zig");
const rt = uno.use();
const io = rt.mcu.get_memory_space();

const F_CPU = 16000000;
const LED_PIN = 5;
const DDRB: *volatile u8 = @ptrFromInt(0x24);
const PORTB: *volatile u8 = @ptrFromInt(0x25);
const PINB: *volatile u8 = @ptrFromInt(0x23);
    
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
pub export const os = init.install(.{});

// Ideally we'd use a calling convention with full caller-preserves, but that's not supported yet
pub fn main() noreturn {
    cli();
    init.copy_data_to_ram();
    init.clear_bss();

    io.UCSR0A.byte = 0b00000010;
    io.UBRR0L = @truncate(BAUD_PRESCALE);
    io.UBRR0H = @truncate(BAUD_PRESCALE >> 8);
    io.UCSR0C.byte = 0b00000110;
    io.UCSR0B.byte = 0b00011000;

    for ("Hello, World!\n") |c| out(c);
    const instance = lcd.init();

    lcd.set_cursor(0,0);
    var idx: u8 = '0';
    while (idx <= '9') : (idx += 1) {
        instance.write(idx);
        small_sleep(3);
    }
    // lcd.set_cursor(16, 1);
    lcd.command(LCD.set_ddram_addr | 0x40 + 16);
    // enable autoscrol
    lcd.command(LCD.entry_mode_set | LCD.entry_left | LCD.entry_enable_shift);
    idx = '0';
    while (idx <= '9') : (idx += 1) {
        instance.write(idx);
        small_sleep(3);
    }
    lcd.command(LCD.entry_mode_set | LCD.entry_left);
    lcd.command(LCD.clear_display);
    small_sleep(1);



    for ("It's alive!") |b| instance.write(b); // loop is 22 byte prelude, 6 byte prologue (register allocation is still awful), string is 11 bytes. `write` in inlined: 14 bytes, nice!
    
    var cmd: u8 = LCD.display_control; // 2 bytes

    sbi(DDRB, LED_PIN);
    var i: u8 = 0;
    while (true) {
        sbi(PINB, LED_PIN);
        cmd ^= LCD.display_on; // 2 bytes + 2 bytes allocating reg
        if ((cmd & LCD.display_on) != 0) {
            i += 1;
            lcd.set_cursor(0, 12);
            std.fmt.format(instance, "{: >3}", .{i}) catch {};
        }
        lcd.command(cmd); // 6 bytes - failed to allocate registers again. This should be 4 bytes
        small_sleep(3);
    }
}

// https://github.com/gcc-mirror/gcc/blob/8414f10ad5bad6d522b72f9ae35e0bb86bb290ea/libgcc/config/avr/lib1funcs.S#L1342-L1356
comptime {

    @export(__udivmodqi4, .{ .name = "__udivmodqi4", .linkage = .Weak });
}
fn __udivmodqi4() linksection("rt") callconv(.Naked) void {
    var r_rem: u8 = undefined;
    var r_cnt: u8 = undefined;
    var r_arg1: u8 = undefined;
    var r_arg2: u8 = undefined;
    asm volatile(
        \\ 	   sub	%[r_rem],%[r_rem]	; clear remainder and carry
        \\     ldi	%[r_cnt],9		; init loop counter
        \\     rjmp	1f	; jump to entry point
        \\ 0:
        \\     rol	%[r_rem]		; shift dividend into remainder
        \\     cp	%[r_rem],%[r_arg2]	; compare remainder & divisor
        \\     brcs	1f	; remainder <= divisor
        \\     sub	%[r_rem],%[r_arg2]	; restore remainder
        \\ 1:
        \\     rol	%[r_arg1]		; shift dividend (with CARRY)
        \\     dec	%[r_cnt]		; decrement loop counter
        \\     brne	0b
        \\     com	%[r_arg1]		; complement result
        \\                 ; because C flag was complemented in loop
        \\     ret
        : [r_rem] "={r25}" (r_rem),
          [r_cnt] "={r23}" (r_cnt),
          [r_arg1] "+{r24}" (r_arg1),
          [r_arg2] "+{r22}" (r_arg2),
        :
        :
    );
}
const mmio = Mmio { .io = rt.mcu.get_io_space() };

const RS = uno.digital_pin(12).?;
const ENABLE = uno.digital_pin(11).?;
const D4 = uno.digital_pin(5).?;
const D5 = uno.digital_pin(4).?;
const D6 = uno.digital_pin(3).?;
const D7 = uno.digital_pin(2).?;


