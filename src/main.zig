// const std = @import("std");
const uno = @import("arduino_uno_rev3.zig");
// const atmega328p = @import("atmega328p.zig");
const std = @import("std");
// const builtin = std.builtin;

const LED = uno.led_pin();
var rt = uno.use();
var mmio = Mmio{ .io = rt.mcu.get_io_space() };
var io = rt.mcu.get_memory_space();

const RS = uno.digital_pin(12).?;
const ENABLE = uno.digital_pin(11).?;
const D4 = uno.digital_pin(5).?;
const D5 = uno.digital_pin(4).?;
const D6 = uno.digital_pin(3).?;
const D7 = uno.digital_pin(2).?;

fn main() noreturn {
    // asm volatile (
    //     \\ sei
    // );
    mmio.setPinMode(LED, .output);
    // for (0..6) |_| {

    //     mmio.togglePin(LED);
    //     for (0..20) |_| delayMicroseconds(10 * 1000);
    // }
    const SERIAL_8N1 = 0x06;
    const freq = 16 * 1000 * 1000;

    const baud = 9600;
    const config = SERIAL_8N1;

    // const setting = ((freq / @as(u32, @intCast(baud_rate))) / 16) - 1;
    const setting: u12 = ((freq / 8 / baud) - 1) / 2;
    // io.UCSR0A.byte |= (1 << 1);

    io.UCSR0B.byte = 0;
    io.UBRR0H = @truncate(setting >> 8);
    io.UBRR0L = @truncate(setting);
    
    io.UCSR0C.byte = config;
    // io.UCSR0B.bits.RXEN0 = true;
    io.UCSR0B.byte |= (1 << 4);
    // io.UCSR0B.bits.TXEN0 = true;
    io.UCSR0B.byte |= (1 << 3);
    // io.UCSR0B.bits.RXCIE0 = true;
    io.UCSR0B.byte |= (1 << 7);
    // io.UCSR0B.bits.UDRIE0 = false;
    io.UCSR0B.byte &= ~@as(u8, 1 << 5);
    // // io.UDR0;
    
    for (0..6) |_| {

        mmio.togglePin(LED);
        for (0..20) |_| delayMicroseconds(10 * 1000);
    }
    // // // for ("hi there\nhows things id like to say some more and\nkeep you busy") |b| 
    send_char('h');
    // send_char('i');
    // send_char(' ');
    // send_char('b');
    // send_char('u');
    // send_char('d');
    // var lcd = Lcd.create_mini(ENABLE, RS, .{ D4, D5, D6, D7 });
    // lcd.begin(16, 2, 0) catch {};
    // lcd.command(return_home);
    // for ("Allonzi!") |b| lcd.write(b);
    
    while(true){
        mmio.togglePin(LED);
        for (0..100) |_| delayMicroseconds(10 * 1000);
    }
    
    // mmio.setPinMode(LED, .output);
    

    // _ = lcd;
    // while(true) {
    //     mmio.setHigh(LED.input());
    //     for (0..10) |_|
    //         delayMicroseconds(16384);
    // }
}
comptime {

    // @compileLog(@intFromPtr(&@as(*const uno.Mcu.Abi, @ptrFromInt(0x20)).UDR0));
}
fn send_char(c: u8) void {
    while ((io.UCSR0A.byte & (1 << 5)) == 0) {}
    io.UDR0 = c;
    io.UCSR0A.byte &= (1 << 6);
}
// export fn trampoline() callconv(.C) noreturn {
//     main();
// }
const RAM_END = 0x8FF;
const F_CPU = 16000000;
const LED_PIN = 5;
const DDRB: *volatile u8 = @ptrFromInt(0x24);
const PORTB = 0x25;
const PINB: *volatile u8 = @ptrFromInt(0x23);
// When inlined, easily optimizes to sbi %[reg - 20], %[bit]
fn sbi(reg: *volatile u8, bit: u3) void {
    reg.* |= @as(u8, 1) << bit;
}
const LOOP_CYCLES = 2 + 1 + 2;
const ITERS_PER_MS = F_CPU / 1000 / LOOP_CYCLES;
fn burn_24bit(low: u16, high: u8)  void {
    asm volatile(
        \\ 0:
        \\  sbiw %[low], 1
        \\  sbci %[high], 0
        \\  brne 0b
        : [low] "+w" (low),
          [high] "+r" (high),
    );
}
const SP: *volatile u16 = @ptrFromInt(0x5D);
fn main2() noreturn {
    sbi(DDRB, LED_PIN);
    // asm volatile(
    //     \\ sbi (%[DDRB]-0x20), %[LED_PIN]
    //     :
    //     : [DDRB] "I" (DDRB),
    //       [LED_PIN] "I" (LED_PIN),
    //     : "memory"
    // );
    while (true) { 
        // asm volatile(
        //     \\   sbi (%[PINB]-0x20), %[LED_PIN]
        //     :
        //     : 
        //         [PINB] "i" (PINB),
        //         [LED_PIN] "i" (LED_PIN),
        //         [DELAY] "i" (@as(u16, @truncate(500*ITERS_PER_MS))),
        //         [DELAY_HIGH] "i" ((500*ITERS_PER_MS) >> 16),
        //     : "memory", "r28", "r29", "r30", "cc"
        // );
        sbi(PINB, LED_PIN);
        const DELAY = 500 * ITERS_PER_MS;
        burn_24bit(@as(u16, @truncate(DELAY)), @as(u8, @truncate(DELAY >> 16)));
    }
}
export fn trampoline() callconv(.C) noreturn {
    @call(.always_inline, main2, .{});
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
    // std.builtin.CallModifier.always_tail
    // @call(.al, main2, .{});
    // sbi(DDRB, LED_PIN);
    // while (true) {
    //     sbi(PINB, LED_PIN);
    //     for (0..10) |_| {
    //         delayMicroseconds(16384);
    //     }
    // }

}
 fn _start_old () callconv(.Naked) noreturn {
    copy_data_to_ram();
    clear_bss();
    const stack_top: u16 = 0x8FF;
    asm volatile (
        \\ ldi r28, hi8(%[stack_top])
        \\ out %[SPH], r28
        \\ ldi r28, lo8(%[stack_top])
        \\ out %[SPL], r28
        \\ jmp trampoline
        :
        : [stack_top] "i" (stack_top),
        // These should be the io vectors for the stack pointer
        //   [SPH] "i" (SPH),
        //     [SPL] "i" (SPL)
    );
}

// Less accurate than arduino's implementation
fn delayMicroseconds(us: u16) void {
    const freq = 16 * 1000 * 1000;
    const micro = 1000 * 1000;
    const loop_cycles = 4;
    const iters = freq / micro / loop_cycles * us;
    var out: u16 = undefined;
    asm volatile (
        \\ 1:
        \\   sbiw %[iters], 1
        \\   brne 1b
        : [_] "=w" (out)
        : [iters] "0" (iters)
        :
    );
}
const Pin = uno.MegaAVR.PortPin;
const PinMode = enum {
    input,
    output
};

const Mmio = struct {
    io: *volatile [256]u8,
    pub inline fn high(self: *@This(), addr: u8, bit: u3) void {
        self.io[addr] |= @as(u8, 1) << bit;
    }
    pub inline fn low(self: *@This(), addr: u8, bit: u3) void {
        self.io[addr] &= ~(@as(u8, 1) << bit);
    }
    pub inline fn set(self: *@This(), addr: u8, bit: u3, value: bool) void {
        if (value) {
            self.high(addr, bit);
        } else {
            self.low(addr, bit);
        }
    }
    pub fn setPin(self: *@This(), pin: Pin, value: bool) void {
        self.set(pin.data_register(), pin.pin_idx(), value);
    }
    pub fn togglePin(self: *@This(), pin: Pin) void {
        self.set(pin.input_register(), pin.pin_idx(), true);
    }
    pub fn setPinMode(self: *@This(), pin: Pin, mode: PinMode) void {
        self.set(pin.data_direction_register(), pin.pin_idx(), mode == .output);
    }
};


comptime {
    const root = @This();
    const VectorTable: [26][]const u8 = .{
        "RESET",
        "INT0",
        "INT1",
        "PCINT0",
        "PCINT1",
        "PCINT2",
        "WDT",
        "TIMER2_COMPA",
        "TIMER2_COMPB",
        "TIMER2_OVF",
        "TIMER1_CAPT",
        "TIMER1_COMPA",
        "TIMER1_COMPB",
        "TIMER1_OVF",
        "TIMER0_COMPA",
        "TIMER0_COMPB",
        "TIMER0_OVF",
        "SPI_STC",
        "USART_RX",
        "USART_UDRE",
        "USART_TX",
        "ADC",
        "EE_READY",
        "ANALOG_COMP",
        "TWI",
        "SPM_Ready",
    };
    std.debug.assert(std.mem.eql(u8, "RESET", VectorTable[0]));
    var asm_str: []const u8 = ".section .vectors\njmp _start\n";

    if (@hasDecl(root, "RESET"))
        @compileError("Not allowed to overload the reset vector");

    for (VectorTable[1..]) |vector| {
        const new_insn =  overload: {
            if (@hasDecl(root, vector)) {
                const handler = @field(root.interrupts, vector);
                const calling_convention = switch (@typeInfo(@TypeOf(@field(root.interrupts, vector)))) {
                    .Fn => |info| info.calling_convention,
                    else => @compileError("Interrupts must all be functions. '" ++ vector ++ "' is not a function"),
                };

                const exported_fn = switch (calling_convention) {
                    .Unspecified => struct {
                        fn wrapper() callconv(.C) void {
                            //if (calling_convention == .Unspecified) // TODO: workaround for some weird stage1 bug
                            @call(.{ .modifier = .always_inline }, handler, .{});
                        }
                    }.wrapper,
                    else => @compileError("Just leave interrupt handlers with an unspecified calling convention"),
                };

                const options = .{ .name = vector, .linkage = .Strong };
                @export(exported_fn, options);
                break :overload "jmp " ++ vector;
            } else {
                break :overload "jmp _unhandled_vector";
            }
        };

        asm_str = asm_str ++ new_insn ++ "\n";
    }
    asm (asm_str);
}

export fn _unhandled_vector() void {
    while (true) {}
}

inline fn copy_data_to_ram() void {
    asm volatile (
        \\  ; load Z register with the address of the data in flash
        \\  ldi r30, lo8(__data_load_start)
        \\  ldi r31, hi8(__data_load_start)
        \\  ; load X register with address of the data in ram
        \\  ldi r26, lo8(__data_start)
        \\  ldi r27, hi8(__data_start)
        \\  ; load address of end of the data in ram
        \\  ldi r24, lo8(__data_end)
        \\  ldi r25, hi8(__data_end)
        \\  rjmp .L2
        \\
        \\.L1:
        \\  lpm r18, Z+ ; copy from Z into r18 and increment Z
        \\  st X+, r18  ; store r18 at location X and increment X
        \\
        \\.L2:
        \\  cp r26, r24
        \\  cpc r27, r25 ; check and branch if we are at the end of data
        \\  brne .L1
    );
    // Probably a good idea to add clobbers here, but compiler doesn't seem to care
}

inline fn clear_bss() void {
    asm volatile (
        \\  ; load X register with the beginning of bss section
        \\  ldi r26, lo8(__bss_start)
        \\  ldi r27, hi8(__bss_start)
        \\  ; load end of the bss in registers
        \\  ldi r24, lo8(__bss_end)
        \\  ldi r25, hi8(__bss_end)
        \\  ldi r18, 0x00
        \\  rjmp .L4
        \\
        \\.L3:
        \\  st X+, r18
        \\
        \\.L4:
        \\  cp r26, r24
        \\  cpc r27, r25 ; check and branch if we are at the end of bss
        \\  brne .L3
    );
    // Probably a good idea to add clobbers here, but compiler doesn't seem to care
}
fn abort() callconv(.Naked) noreturn {
    while (true) {}
}
// LCD driven using the KS0066U controller
// https://pdf1.alldatasheet.com/datasheet-pdf/download/37318/SAMSUNG/KS0066.html
// Definitions:

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


const Lcd = struct {
    pins_: [8]Pin,
    enable_: Pin,
    rs_: Pin,
    rw_: ?Pin = null,
    flags: packed struct {
        mini: bool = false,
    } = .{},

    // _display_mode: u8 = entry_left | entry_shift_decrement,
    // _display_function: u8 = four_bit_mode | two_line | five_by_eight_dots,
    // _display_control: u8 = display_on | cursor_off | blink_off,

    fn create_mini(enable: Pin, rs: Pin, lines: [4]Pin) @This() {
        var lcd: @This() = .{
            .pins_ = std.mem.zeroes([8]Pin),
            .enable_ = enable,
            .rs_ = rs
        };
        lcd.pins_[0] = lines[0];
        lcd.pins_[1] = lines[1];
        lcd.pins_[2] = lines[2];
        lcd.pins_[3] = lines[3];
        lcd.flags.mini = true;
        return lcd;
    }
    fn pins(self: *@This()) []Pin {
        const pin_count: usize = if (self.flags.mini) 4 else 8;
        return self.pins_[0..pin_count];
    }
    fn begin(self: *@This(), _: u8, _: u8, _: u8) !void {
        mmio.setPinMode(self.rs_, .output);
        if (self.rw_) |rw| mmio.setPinMode(rw, .output);
        mmio.setPinMode(self.enable_, .output);
        for (self.pins()) |pin| mmio.setPinMode(pin, .output);

        delayMicroseconds(50000); 
        mmio.setPin(self.rs_, false);
        mmio.setPin(self.enable_, false);
        if (self.rw_) |rw| mmio.setPin(rw, false);

        self.write_four_bits(0x03);
        delayMicroseconds(4500); // wait min 4.1ms

        // second try
        self.write_four_bits(0x03);
        delayMicroseconds(4500); // wait min 4.1ms

        // third go!
        self.write_four_bits(0x03);
        delayMicroseconds(150);

        // finally, set to 4-bit interface
        self.write_four_bits(0x02);
            
        // finally, set # lines, font size, etc.
        self.command(function_set | one_line | two_line | five_by_eight_dots | four_bit_mode);

        // turn the display on with no cursor or blinking default
        self.command(display_control | display_on | cursor_off | blink_off);

        // clear it off
        self.clear();

        // Initialize to default text direction (for romance languages)
        self.command(entry_mode_set | entry_left | entry_shift_increment);
    }
    fn write_four_bits(self: *@This(), value: u8) void {
        for (self.pins(), 0..) |pin, i| {
            mmio.setPin(pin, (value >> @truncate(i)) & 0x01 != 0);
        }
        self.pulse_enable();
    }
    fn pulse_enable(self: *@This()) void {
        mmio.setPin(self.enable_, false);
        delayMicroseconds(1);
        mmio.setPin(self.enable_, true);
        delayMicroseconds(1); // enable pulse must be >450ns
        mmio.setPin(self.enable_, false);
        delayMicroseconds(100 * 5); // commands need > 37us to settle
    }
    fn clear(self: *@This()) void {
        self.command(clear_display);
        delayMicroseconds(2000); // this command takes a long time!
    }
    fn command(self: *@This(), command_: u8) void {
        self.send(command_, false);
    }
    fn write(self: *@This(), command_: u8) void {
        self.send(command_, true);
    }
    fn send(self: *@This(), value: u8, mode: bool) void {
        mmio.setPin(self.rs_, mode);
        if (self.rw_) |rw| mmio.setPin(rw, false);

        self.write_four_bits(value >> 4);
        self.write_four_bits(value);
    }
};