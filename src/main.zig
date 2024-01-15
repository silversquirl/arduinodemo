const std = @import("std");
const init = @import("init.zig");
const small_sleep = @import("sleep.zig").small_sleep;
const Mmio = @import("mmio.zig").Mmio;
const LCD = @import("KS0066U.zig");
const lcd = @import("KS0066U.zig").KS0066U(mmio, ENABLE, RS, D4, D5, D6, D7);
const usart = @import("usart.zig");

const uno = @import("arduino_uno_rev3.zig");
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

fn sbi(reg: *volatile u8, bit: u3) void {
    reg.* |= @as(u8, 1) << bit;
}
pub export const os = init.install(.{});

// Ideally we'd use a calling convention with full caller-preserves, but that's not supported yet
pub fn main() noreturn {

    usart.init();
    for ("Hello, World!\n") |c| usart.out(c);
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

    for ("It's alive!") |b| instance.write(b);
    
    sbi(DDRB, LED_PIN);

    var cmd: u8 = LCD.display_control; // 2 bytes
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


