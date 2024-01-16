// returns after (low + high * 2^16) * (2 + 1 + 2) + 1 + (4) cycles
fn burn_24bit(_low: u16, _high: u8) void {
    var low = _low;
    var high = _high;
    asm volatile (
        \\ 0:
        \\  sbiw %[low], 1  // 2 cycles
        \\  sbci %[high], 0 // 1 cycle
        \\  brne 0b         // 2 cycles when taken, 1 cycle when not taken
        : [low] "+w" (low),
          [high] "+r" (high),
    );
    // return in 4 cycles for 16 bit PC - atmega328p has 32k of flash
    return;
}
const sleeps: [5]u32 = .{
    50 * 1000,
    4500,
    150,
    500 * 1000,
    1,
};
const sleeps_low: [5]u16 = .{
    @truncate(calc_iters(sleeps[0])),
    @truncate(calc_iters(sleeps[1])),
    @truncate(calc_iters(sleeps[2])),
    @truncate(calc_iters(sleeps[3])),
    @truncate(calc_iters(sleeps[4])),
};
const sleeps_high: [5]u8 = .{
    @truncate(calc_iters(sleeps[0]) >> 16),
    @truncate(calc_iters(sleeps[1]) >> 16),
    @truncate(calc_iters(sleeps[2]) >> 16),
    @truncate(calc_iters(sleeps[3]) >> 16),
    @truncate(calc_iters(sleeps[4]) >> 16),
};

fn small_sleep_inner(id: u8) void {
    // Ideally we'd use a single array here, but codegen is best like this. LLVM can't consolodate the loads of a [3]u8.
    return burn_24bit(sleeps_low[id], sleeps_high[id]);
}
pub inline fn small_sleep(id: u8) void {
    @call(.never_inline, small_sleep_inner, .{id});
}
fn calc_iters(us: u32) u32 {
    const cycles_to_burn = (us * cycles_per_micro);
    // cycles_to_burn = (low + high * 2^16) * (2 + 1 + 2) + 1 + (4)
    // (cycles_to_burn - 5) / 5 = (low + high * 2^16) = iters
    return (cycles_to_burn - 5) / 5;
}
const freq = 16 * 1000 * 1000;
const micro = 1000 * 1000;
const cycles_per_micro = freq / micro;
fn burn_u24(iters: u24) void {
    burn_24bit(@truncate(iters), @truncate(iters >> 16));
}
fn burn_us(us: u32) void {
    const cycles_to_burn = (us * cycles_per_micro);
    // cycles_to_burn = (low + high * 2^16) * (2 + 1 + 2) + 1 + (4)
    // (cycles_to_burn - 5) / 5 = (low + high * 2^16) = iters
    burn_u24(@truncate((cycles_to_burn - 5) / 5));
}
