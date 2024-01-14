const std = @import("std");

const uno = @import("arduino_uno_rev3.zig");
const rt = uno.use();
const io = rt.mcu.get_memory_space();
const sleep = @import("sleep.zig");
const small_sleep = sleep.small_sleep;
fn Pin_(comptime mmio_: Mmio) type {
    return struct {
        v: Pin,
        pub fn setMode(self: *const @This(), mode: PinMode) void {
            mmio_.setPinMode(self.v, mode);
        }
        pub fn set(self: *const @This(), value: bool) void {
            mmio_.setPin(self.v, value);
        }
        pub fn data_register(self: *const @This()) *volatile u8 {
            return &mmio_.io[self.v.data_register()];
        }
        pub fn mask(self: *const @This()) u8 {
            return 1 << self.v.pin_idx();
        }
    };
}
const Lcd = @import("Lcd.zig");

// fn abort() callconv(.Naked) noreturn {
//     while (true) {}
// }

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

const UnoPin = Pin_(mmio);
const MyLcd = Lcd.Lcd(UnoPin, .{ .v = ENABLE },
    .{ .v = RS },
    .{
        .{ .v = D4 },
        .{ .v = D5 },
        .{ .v = D6 },
        .{ .v = D7 },
    }
);
// Ideally we'd use a calling convention with full caller-preserves, but that's not supported yet
fn main() noreturn {
    cli();
    copy_data_to_ram();
    clear_bss();

    io.UCSR0A.byte = 0b00000010;
    io.UBRR0L = @truncate(BAUD_PRESCALE);
    io.UBRR0H = @truncate(BAUD_PRESCALE >> 8);
    io.UCSR0C.byte = 0b00000110;
    io.UCSR0B.byte = 0b00011000;

    for ("Hello, World!\n") |c| out(c);
    var lcd = MyLcd.init();

    for ("It's alive!") |b| lcd.write(b);
    
    var cmd: u8 = Lcd.display_control; // 2 bytes

    sbi(DDRB, LED_PIN);
    while (true) {
        sbi(PINB, LED_PIN);
        cmd ^= Lcd.display_on; // 2 bytes + 2 bytes allocating reg
        lcd.command(cmd); // 6 bytes - failed to allocate registers again. This should be 4 bytes
        small_sleep(3);
    }
}
const mask: u8 = (1 << D4.pin_idx()) | (1 << D5.pin_idx()) | (1 << D6.pin_idx()) | (1 << D7.pin_idx());
comptime {
    std.debug.assert(D4.data_register() == D5.data_register());
    std.debug.assert(D4.data_register() == D6.data_register());
    std.debug.assert(D4.data_register() == D7.data_register());
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

fn copy_data_to_ram() void {
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
        ::: "memory", "cc"
    );
}

fn clear_bss() void {
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
        ::: "memory", "cc"
    );
}