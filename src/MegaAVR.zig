const std = @import("std");

const AbiCapture = Abi;
pub fn Cpu(comptime variant_: Variant, comptime package_: Package) type {
    return struct {
        pub const package = package_;
        pub const variant = variant_;
        pub const mmio = io;
        pub const Abi = AbiCapture;

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

        pub fn get_io_space(_: *const @This()) *volatile [REGISTER_SPACE_SIZE]u8 {
            return @ptrFromInt(0x20);
        }
        pub fn get_memory_space(_: *const @This()) *volatile @This().Abi {
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

const PORTB_IDX = (0 << 3);
const PORTC_IDX = (1 << 3);
const PORTD_IDX = (2 << 3);
pub const PortPin = enum(u8) {
    B0 = PORTB_IDX | 0,
    B1 = PORTB_IDX | 1,
    B2 = PORTB_IDX | 2,
    B3 = PORTB_IDX | 3,
    B4 = PORTB_IDX | 4,
    B5 = PORTB_IDX | 5,
    B6 = PORTB_IDX | 6,
    B7 = PORTB_IDX | 7,

    C0 = PORTC_IDX | 0,
    C1 = PORTC_IDX | 1,
    C2 = PORTC_IDX | 2,
    C3 = PORTC_IDX | 3,
    C4 = PORTC_IDX | 4,
    C5 = PORTC_IDX | 5,
    C6 = PORTC_IDX | 6,

    D0 = PORTD_IDX | 0,
    D1 = PORTD_IDX | 1,
    D2 = PORTD_IDX | 2,
    D3 = PORTD_IDX | 3,
    D4 = PORTD_IDX | 4,
    D5 = PORTD_IDX | 5,
    D6 = PORTD_IDX | 6,
    D7 = PORTD_IDX | 7,

    const PORT_GPIO_SIZE = io.PINC - io.PINB;
    fn port_idx(self: PortPin) u8 {
        return @intFromEnum(self) >> 3;
    }
    pub fn pin_idx(self: PortPin) u3 {
        return @truncate(@intFromEnum(self));
    }
    pub fn input_register(self: PortPin) u8 {
        return io.PINB + self.port_idx() * PORT_GPIO_SIZE;
    }
    pub fn data_register(self: PortPin) u8 {
        return io.PORTB + self.port_idx() * PORT_GPIO_SIZE;
    }
    pub inline fn data_direction_register(self: PortPin) u8 {
        return io.DDRB + self.port_idx() * PORT_GPIO_SIZE;
    }
};
const Block = struct { comptime_int, []const [:0]const u8 };
const io_blocks: []const Block = &.{.{0x03, &.{
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
    "GPIOR0", "", "", "", "", "", "", "", "", 
    "EECR", "EERE", "EEPE", "EEMPE", "EERIE", "EEPM0", "EEPM1", "", "",
    "EEDR", "", "", "", "", "", "", "","",
    "EEARL", "", "", "", "", "", "", "", "",
    "EEARH", "", "", "", "", "", "", "", "",
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
}}};
const mem_blocks: []const Block = blocks: while (true) {
    var blocks: []const Block = &.{};
    for (io_blocks) |block| blocks = blocks ++ @as([]const Block, &.{Block{block[0] + 0x20, block[1]}});
    break :blocks blocks ++ @as([]const Block, &.{
        .{0x60, &.{
            "WDTCSR", "WDP0", "WDP1", "WDP2", "WDE", "WDCE", "WDP3", "WDIE", "WDIF",
            "CLKPR", "CLKPS0", "CLKPS1", "CLKPS2", "CLKPS3", "", "", "", "CLKPCE", 
        }},
        .{0xC0, &.{
            "UCSR0A", "MPCM0", "U2X0", "UPE0", "DOR0", "FE0", "UDRE0", "TXC0", "RXC0",
            "UCSR0B", "TXB80", "RXB80", "UCSZ02", "TXEN0", "RXEN0", "UDRIE0", "TXCIE0", "RXCIE0",
            "UCSR0C", "UCPOL0", "UCSZ00", "UCSZ01", "USBS0", "UPM00", "UPM01", "UMSEL00", "UMSEL01",
        }},
        .{0xC4, &.{
            "UBRR0L", "", "", "", "", "", "", "", "",
            "UBRR0H", "", "", "", "", "", "", "", "",
            "UDR0", "", "", "", "", "", "", "", "",
        }}
    });
};
pub const Abi = make_mem_abi(mem_blocks);
fn make_mem_abi(comptime blocks: []const Block) type {
    var fields: []const std.builtin.Type.StructField = &.{};
    var padding_fields = 0;
    var current_offset = 0x20;
    for (blocks) |block| {
        const addr = block[0];
        if (addr > current_offset) {
            padding_fields += 1;
            const name = "padding" ++ @as([]const u8, &.{'0' + current_offset});
            // @compileLog(name);
            fields = fields ++ .{
                .{ .name = name,
                    .type = @Type(std.builtin.Type {  .Array = .{ .child = u8, .len = addr - current_offset, .sentinel = null } }),
                    .default_value = null,
                    .is_comptime = false,
                    .alignment = 0
                }
            };
        }
        const data = block[1];
        const registers = (data.len + 8) / 9;
        current_offset = addr + registers;
        for (0..registers) |n| {
            const name = data[n * 9];
            fields = fields ++ .{
                .{
                    .name = name,
                    .type = register_type(data[n * 9 + 1..@min((n+1) * 9, block[1].len)]),
                    .default_value = null,
                    .is_comptime = false,
                    .alignment = 0
                },
            };
        }
    }
    const typeInfo: std.builtin.Type = .{
        .Struct = .{
            .layout = .Extern,
            .fields = fields,
            .decls = &.{},
            .is_tuple = false,
        }
    };
    return @Type(typeInfo);
}
fn register_type(comptime bits: []const [:0]const u8) type {
    var fields: [8]std.builtin.Type.StructField = undefined;
    var padding_fields = 0;
    for (0..8) |i| {
        const padding = i >= bits.len or bits[i].len == 0;
        if (padding) padding_fields += 1;
        fields[i] = .{
            .name = if (padding) "padding" ++ .{'0' + padding_fields} else bits[i],
            .type = bool,
            .default_value = null,
            .is_comptime = false,
            .alignment = 0,
        };
    }
    if (padding_fields == 8) return u8;
    const packed_bits = @Type(.{
        .Struct = .{
            .layout = .Packed,
            .fields = &fields,
            .decls = &.{},
            .is_tuple = false,
        }
    });

    return @Type(std.builtin.Type {
        .Union = .{
            .layout = .Extern,
            .tag_type = null,
            .fields = &.{ .{
                .name = "bits",
                .type = packed_bits,
                .alignment = 0,
            }, .{
                .name = "byte",
                .type = u8,
                .alignment = 0
            } },
            .decls = &.{},
        }
    });
}
pub const io = make_registers(io_blocks);
fn Registers(comptime register_blocks: []const Block) type {
    var n_registers = 0;
    for (register_blocks) |block| n_registers += block[1].len;
    var registers: [n_registers]std.builtin.Type.StructField = undefined;
    var i = 0;
    for (register_blocks) |block| {
        for (block[1], 0..) |reg, j| {
            if (reg.len == 0) continue;
            const byte = j / 9;
            const offset = (j % 9);
            registers[i] = .{
                .name = reg,
                .type = u12,
                .default_value = null,
                .is_comptime = true,
                .alignment = 0,
            };
            if (offset == 0) {
                registers[i].default_value = &(block[0] + byte);
                registers[i].type = u8;
            } else {
                registers[i].default_value = &((block[0] + byte) * 8 + (offset -| 1));
            }
            i += 1;
        }
    }
    return @Type(.{ .Struct = .{ .fields = registers[0..i], .layout = .Auto, .decls = &.{}, .is_tuple = false, } });
}
fn make_registers(comptime register_blocks: anytype) Registers(register_blocks) { return .{}; }