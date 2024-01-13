const std = @import("std");

pub fn Cpu(comptime variant_: Variant, comptime package_: Package) type {
    return struct {
        pub const package = package_;
        pub const variant = variant_;

        const REGISTER_SPACE_SIZE = 256;
        pub fn use() @This() {
            comptime {
                const cpu_name = @import("builtin").target.cpu.model.name;
                if (!(std.mem.eql(u8, variant.to_std().name, @import("builtin").target.cpu.model.name)
                    or (cpu_name[cpu_name.len - 1] == 'p' and std.mem.eql(u8, variant.to_std().name, cpu_name[0..cpu_name.len - 1])))
                ) {
                    const missing_cpu_features = "Cannot use the API of the " ++ @tagName(variant) ++ " CPU because this program is compiled for a " ++ @import("builtin").target.cpu.model.name ++ " CPU"
                    ++ "\nnote: you can use the -Dcpu=" ++ variant.to_std().name ++ " option to change the target CPU";
                    @compileError(missing_cpu_features);
                }
            }
            return @This() { .is_current_target = .{} };
        }
        is_current_target: struct {},

        pub fn get_io_space(_: *@This()) *volatile [REGISTER_SPACE_SIZE]u8 {
            return @ptrFromInt(0x20);
        }
        pub fn get_pin(pin: u8) Pin {
            return package_.pins()[pin - 1];
        }
    };
}

pub const Pin = enum {
    PC0,
    PC1,
    PC2,
    PC3,
    PC4,
    PC5,
    PC6,
    PD0,
    PD1,
    PD2,
    PD3,
    PD4,
    PD5,
    PD6,
    PD7,
    PB0,
    PB1,
    PB2,
    PB3,
    PB4,
    PB5,
    PB6,
    PB7,

    // ADC pins exclusive to the 32 pin packages. The other ADC channels (0-5) are shared with Port C
    ADC6,
    ADC7,

    AVCC, AREF, 
    VCC, GND,

    pub fn to_port_pin(self: Pin) ?PortPin {
        switch (self) {
            inline else => |p| {
                const name = @tagName(p);
                if (@hasField(PortPin, name[1..])) return @field(PortPin, name[1..]);
                return null;
            }
        }
    }

};

pub const Package = enum {
    TQFP32,
    VQFN32,
    TQFP28,
    PDIP28,
    pub fn pins(self: Package) []const Pin {
        switch (self) {
            .PDIP28 => return &.{
                .PC6, .PD0, .PD1, .PD2, .PD3, .PD4,
                .VCC, .GND, 
                .PB6, .PB7,
                .PD5, .PD6, .PD7,
                .PB0, .PB1, .PB2, .PB3, .PB4, .PB5,
                .AVCC, .AREF, .GND,
                .PC0, .PC1, .PC2, .PC3, .PC4, .PC5,
            },
            .TQFP32 => @panic("unimplemented"),
            .VQFN32 => @panic("unimplemented"),
            .TQFP28 => @panic("unimplemented"),
        }
    }
};
// The variations of the megaAVR with different memory sizes. "P" variants are not included as they are identical to the non-"P" variants
pub const Variant = enum {
    ATmega48A,
    ATmega88A,
    ATmega168A,
    ATmega328,

    fn to_std(self: Variant) std.Target.Cpu.Model {
        switch (self) {
            .ATmega48A => return std.Target.avr.cpu.atmega48a,
            .ATmega88A => return std.Target.avr.cpu.atmega88a,
            .ATmega168A => return std.Target.avr.cpu.atmega168a,
            .ATmega328 => return std.Target.avr.cpu.atmega328,
        }
    }
};

pub const PortPin = enum {
    B0, B1, B2, B3, B4, B5, B6, B7,
    C0, C1, C2, C3, C4, C5, C6,
    D0, D1, D2, D3, D4, D5, D6, D7,

    pub fn input(self: PortPin) u12 {
        switch (self) {
            inline else => |variant| return @field(io, "PIN" ++ @tagName(variant))
        }
    }
    pub fn data(self: PortPin) u12 {
        switch (self) {
            inline else => |variant| return @field(io, "PORT" ++ @tagName(variant))
        }
    }
    pub fn data_direction(self: PortPin) u12 {
        switch (self) {
            inline else => |variant| {
                return @field(io, "DD" ++ @tagName(variant));
            }
        }
    }
};


const io = make_registers(&.{.{0x03, &.{
    "PINB", "PINB0", "PINB1", "PINB2", "PINB3", "PINB4", "PINB5", "PINB6", "PINB7",
    "DDRB", "DDB0", "DDB1", "DDB2", "DDB3", "DDB4", "DDB5", "DDB6", "DDB7",
    "PORTB", "PORTB0", "PORTB1", "PORTB2", "PORTB3", "PORTB4", "PORTB5", "PORTB6", "PORTB7",

    "PINC", "PINC0", "PINC1", "PINC2", "PINC3", "PINC4", "PINC5", "PINC6", "",
    "DDRC", "DDC0", "DDC1", "DDC2", "DDC3", "DDC4", "DDC5", "DDC6", "",
    "PORTC", "PORTC0", "PORTC1", "PORTC2", "PORTC3", "PORTC4", "PORTC5", "PORTC6", "",

    "PIND", "PIND0", "PIND1", "PIND2", "PIND3", "PIND4", "PIND5", "PIND6", "PIND7",
    "DDRD", "DDD0", "DDD1", "DDD2", "DDD3", "DDD4", "DDD5", "DDD6", "DDD7",
    "PORTD", "PORTD0", "PORTD1", "PORTD2", "PORTD3", "PORTD4", "PORTD5", "PORTD6", "PORTD7",
}}, .{0x15, &.{
    "TIFR0", "TOV0", "OCF0A", "OCF0B", "", "", "",     "", "",
    "TIFR1", "TOV1", "OCF1A", "OCF1B", "", "", "ICF1", "", "",
    "TIFR2", "TOV2", "OCF2A", "OCF2B", "", "", "",     "", "",
}}, .{0x1B, &.{
    "PCIFR", "PCIF0", "PCIF1", "PCIF2", "", "", "", "", "",
    "EIFR", "INTF0", "INTF1", "", "", "", "", "", "",
    "EIMSK", "INT0", "INT1", "", "", "", "", "", "",
    "GPIOR0", "", "", "", "", "", "", "", 
    "EECR", "EERE", "EEPE", "EEMPE", "EERIE", "EEPM0", "EEPM1", "", "",
    "EEDR", "", "", "", "", "", "", "",
    "EEARL", "", "", "", "", "", "", "",
    "EEARH", "", "", "", "", "", "", "",
    "GTCCR", "PSRSYNC", "PSRASY", "", "", "", "", "", "TSM",
    "TCCR0A", "WGM00", "WGM01", "", "", "COM0B0", "COM0B1", "COM0A0", "COM0A1",
    "TCCR0B", "CS00", "CS01", "CS02", "WGM02", "", "", "FOC0B", "FOC0A",
    "TCNT0", "", "", "", "", "", "", "", "",
    "OCR0A", "", "", "", "", "", "", "", "",
    "OCR0B", "", "", "", "", "", "", "", "",

}}, .{0x2A, &.{"GPIOR1"}}, .{0x2B, &.{"GPIOR2"}}, .{0x2C, &.{
    "SPCR", "SPR0", "SPR1", "CPHA", "CPOL", "MSTR", "DORD", "SPE", "SPIE",
    "SPSR", "SPI2X", "", "", "", "", "", "SPIF", "WCOL",
    "SPDR",
}}, .{0x30, &.{
    "ACSR", "ACIS0", "ACIS1", "ACIC", "ACIE", "ACI", "ACO", "ACBG", "ACD",
}}, .{0x33, &.{
    "SMCR", "SE", "SM0", "SM1", "SM2", "", "", "", "",
    "MCUSR", "PORF", "EXTRF", "BORF", "WDRF", "", "", "", "",
    "MCUCR", "IVCE", "IVSEL", "", "", "PUD", "BODSE", "BODS", "",
}}, .{0x37, &.{
    "SPMCSR", "SELFPRGN", "PGERS", "PGWRT", "BLBSET", "RWWSRE", "", "RWWSB", "SPMIE",
}}, .{0x3D, &.{
    "SPL", "SP0", "SP1", "SP2", "SP3", "SP4", "SP5", "SP6", "SP7",
    "SPH", "SP8", "SP9", "SP10", "", "", "", "", "",
    "SREG", "C", "Z", "N", "V", "S", "H", "T", "I",
}}});
fn Registers(comptime register_blocks: []const struct { comptime_int, []const [:0]const u8 }) type {
    var n_registers = 0;
    for (register_blocks) |block| n_registers += block[1].len;
    var registers: [n_registers]std.builtin.Type.StructField = undefined;
    var i = 0;
    for (register_blocks) |block| {
        for (block[1], 0..) |reg, j| {
            if (reg.len == 0) continue;
            const byte = j / 9;
            const offset = (j % 9) -| 1;
            registers[i] = .{
                .name = reg,
                .type = u12,
                .default_value = &((block[0] + byte) * 8 + offset),
                .is_comptime = true,
                .alignment = 0,
            };
            i += 1;
        }
    }
    return @Type(.{ .Struct = .{ .fields = registers[0..i], .layout = .Auto, .decls = &.{}, .is_tuple = false, } });
}
fn make_registers(comptime register_blocks: anytype) Registers(register_blocks) { return .{}; }