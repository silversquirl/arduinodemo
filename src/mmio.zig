const uno = @import("arduino_uno_rev3");
pub const Pin = uno.MegaAVR.PortPin;

pub const PinMode = enum {
    input,
    output
};
pub const Mmio = struct {
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

