
const LED = bit(5);
export fn _start () void{
    PORTBDIR.* = 0xff;
    while(true){
        PORTBDATA.* ^= LED;
        delay(100000);
    }
}

const PORTBDATA: *volatile u8 = @ptrFromInt(0x25);
const PORTBDIR: *volatile u8 = @ptrFromInt(0x24);
fn delay(count: u32) void {
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        _ = PORTBDATA.*;
    }
}
fn bit(n: u8) u8 {
    return 1 << n;
}