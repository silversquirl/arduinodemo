const std = @import("std");
const init = @import("compiler_rt");
const small_sleep = @import("sleep.zig").small_sleep;
const m = @import("mmio.zig");

const uno = @import("arduino_uno_rev3");
const rt = uno.use();
const io = rt.mcu.get_memory_space();
const mmio = m.Mmio { .io = rt.mcu.get_io_space() };

const Pin = struct {
    pin: uno.MegaAVR.PortPin,
    pub inline fn set(self: Pin, value: bool) void {
        mmio.setPin(self.pin, value);
    }
    pub inline fn setMode(self: Pin, mode: m.PinMode) void {
        mmio.setPinMode(self.pin, mode);
    }
    pub inline fn data_register(self: Pin) *volatile u8 {
        return &mmio.io[self.pin.data_register()];
    }
    pub inline fn pin_idx(self: Pin) u8 {
        return self.pin.pin_idx();
    }
};

fn sbi(reg: *volatile u8, bit: u3) void {
    reg.* |= @as(u8, 1) << bit;
}
pub const os = init.install(.{});

const LED_PIN = uno.led_pin().pin_idx();

pub fn main() noreturn {
    std.debug.print("Hello, World!\n", .{});
    const screen = @import("KS0066U.zig").init(
        Pin { .pin = uno.digital_pin(11).? },
        Pin { .pin = uno.digital_pin(12).? },
        Pin { .pin = uno.digital_pin(5).? },
        Pin { .pin = uno.digital_pin(4).? },
        Pin { .pin = uno.digital_pin(3).? },
        Pin { .pin = uno.digital_pin(2).? },
    );

    screen.set_cursor(0,0);
    for ('0'..'9') |idx| {
        screen.write(@intCast(idx));
        small_sleep(3);
    }
    screen.set_cursor(16, 1);
    screen.set_entry_mode(.increment, .follow);
    for ('0'..'9') |idx| {
        screen.write(@intCast(idx));
        small_sleep(3);
    }
    screen.set_entry_mode(.increment, .fixed);
    screen.clear();

    for ("It's alive!") |b| screen.write(b);
    
    sbi(&io.DDRB.byte, LED_PIN);

    var on = false; 
    var i: u8 = 0;
    while (true) {
        sbi(&io.PINB.byte, LED_PIN);
        on = !on;
        if (!on) {
            screen.set_cursor(12, 0);
            screen.print("{: >3}", .{i});
            i += 1;
        }
        screen.display(on, false, false); 
        small_sleep(3);
    }
}


