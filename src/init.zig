const std = @import("std");

// https://github.com/FireFox317/avr-arduino-zig/tree/master
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

pub fn copy_data_to_ram() void {
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

pub fn clear_bss() void {
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