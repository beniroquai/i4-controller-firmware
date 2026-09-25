//! Electrical dither (E5): a sine modulation superimposed on the drive currents
//! to break static friction. Applied only while an axis is energized.
//!
//! mode 0: off (TIM6 ISR returns immediately, drive is exactly as without dither)
//! mode 1: phase dither, field angle ± amp (1/1024 cycle)
//! mode 2: amplitude dither, current magnitude ± amp (%)

use portable_atomic::{AtomicI16, AtomicI32, AtomicU16, AtomicU32, AtomicU8, Ordering};

use crate::pac;
use crate::sine::sin1024;

pub const TICK_HZ: u32 = 20_000;
pub const MAX_FREQ: u16 = 5_000; // 4 samples per period at the top end

static MODE: AtomicU8 = AtomicU8::new(0);
static AMP: AtomicU16 = AtomicU16::new(0);
static FREQ: AtomicU16 = AtomicU16::new(0);
static ACC: AtomicU32 = AtomicU32::new(0);

/// Read by Stepper::currents(): phase offset (1/1024 cycle) and gain (/32767).
pub static PHASE_OFF: AtomicI16 = AtomicI16::new(0);
pub static GAIN: AtomicI32 = AtomicI32::new(32767);

/// TIM6 as a TICK_HZ periodic interrupt.
pub fn init(clk_freq: u32) {
    pac::RCC.apb1enr1().modify(|w| w.set_tim6en(true));
    let tim = pac::TIM6;
    tim.psc().write(|w| w.set_psc(0));
    tim.arr().write(|w| w.set_arr((clk_freq / TICK_HZ - 1) as u16));
    tim.egr().write(|w| w.set_ug(true));
    tim.sr().write(|w| w.0 = 0);
    tim.dier().modify(|w| w.set_uie(true));
    tim.cr1().modify(|w| w.set_cen(true));
}

pub fn set(mode: u8, amp: u16, freq: u16) {
    AMP.store(amp, Ordering::Relaxed);
    FREQ.store(freq.min(MAX_FREQ), Ordering::Relaxed);
    if mode == 0 {
        PHASE_OFF.store(0, Ordering::Relaxed);
        GAIN.store(32767, Ordering::Relaxed);
    }
    MODE.store(mode, Ordering::Relaxed);
}

pub fn get() -> (u8, u16, u16) {
    (MODE.load(Ordering::Relaxed), AMP.load(Ordering::Relaxed), FREQ.load(Ordering::Relaxed))
}

/// Advance the dither waveform one tick. Returns false when dither is off.
pub fn tick() -> bool {
    let mode = MODE.load(Ordering::Relaxed);
    if mode == 0 {
        return false;
    }
    // 32-bit phase accumulator; top 10 bits index the 1/1024-cycle sine table
    let inc = FREQ.load(Ordering::Relaxed) as u32 * (u32::MAX / TICK_HZ);
    let acc = ACC.fetch_add(inc, Ordering::Relaxed).wrapping_add(inc);
    let w = sin1024((acc >> 22) as i32); // ±32767
    let amp = AMP.load(Ordering::Relaxed) as i32;
    match mode {
        1 => PHASE_OFF.store((amp * w / 32767) as i16, Ordering::Relaxed),
        _ => GAIN.store(32767 + amp * w / 100, Ordering::Relaxed),
    }
    true
}
