const std = @import("std");
const init = @import("compiler_rt");
const small_sleep = @import("sleep.zig").small_sleep;
const Mmio = @import("mmio.zig").Mmio;
const LCD = @import("KS0066U.zig");
// const usart = @import("usart.zig");

const uno = @import("arduino_uno_rev3");
const rt = uno.use();
const io = rt.mcu.get_memory_space();
const mmio = Mmio { .io = rt.mcu.get_io_space() };

const LED_PIN = 5;
const DDRB: *volatile u8 = @ptrFromInt(0x24);
const PORTB: *volatile u8 = @ptrFromInt(0x25);
const PINB: *volatile u8 = @ptrFromInt(0x23);

const RS = uno.digital_pin(12).?;
const ENABLE = uno.digital_pin(11).?;
const D4 = uno.digital_pin(5).?;
const D5 = uno.digital_pin(4).?;
const D6 = uno.digital_pin(3).?;
const D7 = uno.digital_pin(2).?;
const lcd = @import("KS0066U.zig").KS0066U(mmio, ENABLE, RS, D4, D5, D6, D7);

fn sbi(reg: *volatile u8, bit: u3) void {
    reg.* |= @as(u8, 1) << bit;
}
pub const os = init.install(.{});


pub fn main() noreturn {
    std.debug.print("Hello, World!\n", .{});
    const screen = lcd.init();

    screen.set_cursor(0,0);
    var idx: u8 = '0';
    while (idx <= '9') : (idx += 1) {
        screen.write(idx);
        small_sleep(3);
    }
    screen.set_cursor(16, 1);
    screen.set_entry_mode(.increment, .follow);
    idx = '0';
    while (idx <= '9') : (idx += 1) {
        screen.write(idx);
        small_sleep(3);
    }
    screen.set_entry_mode(.increment, .fixed);
    screen.clear();

    for ("It's alive!") |b| screen.write(b);
    
    sbi(DDRB, LED_PIN);

    var on = false; // 2 bytes
    var i: u8 = 0;
    while (true) {
        sbi(PINB, LED_PIN);
        on = !on;
        if (!on) {
            i += 1;
            screen.set_cursor(12, 0);
            screen.print("{: >3}", .{i});
        }
        screen.display(on, false, false); 
        small_sleep(3);
    }
}


