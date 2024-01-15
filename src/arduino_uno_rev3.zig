pub const MegaAVR = @import("MegaAVR.zig");

pub const LED_PIN_ID = 13;
pub const LED_PIN = digital_pin(LED_PIN_ID).?;

// https://docs.arduino.cc/hacking/hardware/PinMapping168
pub const analog_pins: [6]u8 = .{23, 24, 25, 26, 27, 28};
pub const digital_pins: [14]u8 = .{2, 3, 4, 5, 6, 11, 12, 13, 14, 15, 16, 17, 18, 19};

pub const Mcu = MegaAVR.Cpu(.ATmega328, .PDIP28);
mcu: Mcu,
pub fn use() @This() {
    // FIXME: assert that the env is arduino_uno_rev3
    return .{ .mcu = Mcu.use() };
}
pub fn digital_pin(id: u8) ?MegaAVR.PortPin {
    if (id >= digital_pins.len) return null;
    return Mcu.get_pin(digital_pins[id]).to_port_pin() orelse unreachable;
}