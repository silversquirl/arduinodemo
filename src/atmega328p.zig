
pub const VectorTable = extern struct {
    /// External Pin, Power-on Reset, Brown-out Reset and Watchdog Reset
    RESET: InterruptVector = unhandled,
    /// External Interrupt Request 0
    INT0: InterruptVector = unhandled,
    /// External Interrupt Request 1
    INT1: InterruptVector = unhandled,
    /// Pin Change Interrupt Request 0
    PCINT0: InterruptVector = unhandled,
    /// Pin Change Interrupt Request 1
    PCINT1: InterruptVector = unhandled,
    /// Pin Change Interrupt Request 2
    PCINT2: InterruptVector = unhandled,
    /// Watchdog Time-out Interrupt
    WDT: InterruptVector = unhandled,
    /// Timer/Counter2 Compare Match A
    TIMER2_COMPA: InterruptVector = unhandled,
    /// Timer/Counter2 Compare Match B
    TIMER2_COMPB: InterruptVector = unhandled,
    /// Timer/Counter2 Overflow
    TIMER2_OVF: InterruptVector = unhandled,
    /// Timer/Counter1 Capture Event
    TIMER1_CAPT: InterruptVector = unhandled,
    /// Timer/Counter1 Compare Match A
    TIMER1_COMPA: InterruptVector = unhandled,
    /// Timer/Counter1 Compare Match B
    TIMER1_COMPB: InterruptVector = unhandled,
    /// Timer/Counter1 Overflow
    TIMER1_OVF: InterruptVector = unhandled,
    /// TimerCounter0 Compare Match A
    TIMER0_COMPA: InterruptVector = unhandled,
    /// TimerCounter0 Compare Match B
    TIMER0_COMPB: InterruptVector = unhandled,
    /// Timer/Couner0 Overflow
    TIMER0_OVF: InterruptVector = unhandled,
    /// SPI Serial Transfer Complete
    SPI_STC: InterruptVector = unhandled,
    /// USART Rx Complete
    USART_RX: InterruptVector = unhandled,
    /// USART, Data Register Empty
    USART_UDRE: InterruptVector = unhandled,
    /// USART Tx Complete
    USART_TX: InterruptVector = unhandled,
    /// ADC Conversion Complete
    ADC: InterruptVector = unhandled,
    /// EEPROM Ready
    EE_READY: InterruptVector = unhandled,
    /// Analog Comparator
    ANALOG_COMP: InterruptVector = unhandled,
    /// Two-wire Serial Interface
    TWI: InterruptVector = unhandled,
    /// Store Program Memory Read
    SPM_Ready: InterruptVector = unhandled,
};

const std = @import("std");

pub fn mmio(addr: usize, comptime size: u8, comptime PackedT: type) *volatile Mmio(size, PackedT) {
    return @ptrFromInt(addr);
}

pub fn Mmio(comptime size: u8, comptime PackedT: type) type {
    if ((size % 8) != 0)
        @compileError("size must be divisible by 8!");

    if (!std.math.isPowerOfTwo(size / 8))
        @compileError("size must encode a power of two number of bytes!");

    const IntT = std.meta.Int(.unsigned, size);

    if (@sizeOf(PackedT) != (size / 8))
        @compileError(std.fmt.comptimePrint("IntT and PackedT must have the same size!, they are {} and {} bytes respectively", .{ size / 8, @sizeOf(PackedT) }));

    return extern struct {
        const Self = @This();

        raw: IntT,

        pub const underlying_type = PackedT;

        pub inline fn read(addr: *volatile Self) PackedT {
            return @bitCast(addr.raw);
        }

        pub inline fn write(addr: *volatile Self, val: PackedT) void {
            // This is a workaround for a compiler bug related to miscompilation
            // If the tmp var is not used, result location will fuck things up
            const tmp: IntT = @bitCast(val);
            addr.raw = tmp;
        }

        pub inline fn modify(addr: *volatile Self, fields: anytype) void {
            var val = read(addr);
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(val, field.name) = @field(fields, field.name);
            }
            write(addr, val);
        }

        pub inline fn toggle(addr: *volatile Self, fields: anytype) void {
            var val = read(addr);
            inline for (@typeInfo(@TypeOf(fields)).Struct.fields) |field| {
                @field(val, @tagName(field.default_value.?)) = !@field(val, @tagName(field.default_value.?));
            }
            write(addr, val);
        }
    };
}

pub fn MmioInt(comptime size: u8, comptime T: type) type {
    return extern struct {
        const Self = @This();

        raw: std.meta.Int(.unsigned, size),

        pub inline fn read(addr: *volatile Self) T {
            return @truncate(addr.raw);
        }

        pub inline fn modify(addr: *volatile Self, val: T) void {
            const Int = std.meta.Int(.unsigned, size);
            const mask = ~@as(Int, (1 << @bitSizeOf(T)) - 1);

            const tmp = addr.raw;
            addr.raw = (tmp & mask) | val;
        }
    };
}

pub fn mmioInt(addr: usize, comptime size: usize, comptime T: type) *volatile MmioInt(size, T) {
    return @ptrFromInt(addr);
}

const InterruptVector = extern union {
    C: *const fn () callconv(.C) void,
    Naked: *const fn () callconv(.Naked) void,
    // Interrupt is not supported on arm
};

const unhandled = InterruptVector{
    .C = &(struct {
        fn tmp() callconv(.C) noreturn {
            @panic("unhandled interrupt");
        }
    }.tmp),
};