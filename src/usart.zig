
const uno = @import("arduino_uno_rev3.zig");
const rt = uno.use();
const io = rt.mcu.get_memory_space();

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
pub fn out(c: u8) void {
    spin_until_bit_set(&io.UCSR0A.byte, 5);
    io.UDR0 = c;
}

const F_CPU = 16000000;
const BAUD_RATE = 9600; // can go up to 500_000 stably. Datasheet claims 1M but messages start getting garbled in arduino studio
const BAUD_PRESCALE = ((F_CPU / 8 / BAUD_RATE) - 1);
pub fn init() void {
    
    io.UCSR0A.byte = 0b00000010;
    io.UBRR0L = @truncate(BAUD_PRESCALE);
    io.UBRR0H = @truncate(BAUD_PRESCALE >> 8);
    io.UCSR0C.byte = 0b00000110;
    io.UCSR0B.byte = 0b00011000;
}