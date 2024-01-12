const std = @import("std");
const uno = @import("arduino_uno_rev3.zig");

const led = uno.led_pin();
var rt = uno.use();
var mmio = Mmio { .io = rt.mcu.get_io_space() };

export fn _start () void {
    mmio.setIoHigh(led.data_direction());
    while(true){
        mmio.setIoHigh(led.input());
        delay(40000);
    }
}

const Mmio = struct {
    io: *volatile [256]u8,
    pub fn setIoHigh(self: *@This(), addr: u12) void {
        self.io[addr >> 3] |= @as(u8, 1) << @truncate(addr);
    }
};
fn delay(count: u32) void {
    var i: u32 = 0;
    while (i < count) : (i += 1) asm volatile("nop");
}
