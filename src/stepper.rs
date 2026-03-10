//! Bipolar stepper motor driver with configurable microstepping and holding torque.
//!
//! This module provides a microstepping driver for bipolar stepper motors using
//! sine/cosine commutation. Features include:
//!
//! - Configurable microstepping resolution (16, 32, 64 microsteps per electrical cycle)
//! - Holding torque: maintains current through coils when stationary to prevent position loss
//! - Smooth acceleration via the motion control system
//!
//! # Microstepping
//!
//! Higher microstepping values provide smoother motion but require faster step interrupts.
//! The default is 32 microsteps per electrical cycle.
//!
//! # Holding Torque
//!
//! When enabled, the motor maintains a reduced current (configurable percentage of peak)
//! through the coils when stationary. This prevents the rotor from drifting under load
//! but increases power consumption and heat generation.

#![allow(dead_code)]

use core::cell::Cell;

use critical_section::Mutex;
use crossbeam::atomic::AtomicCell;
use portable_atomic::{AtomicI32, AtomicU16, Ordering};

use crate::current_control::IChannel;

/// Microstepping resolution options.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MicrostepMode {
    /// 16 microsteps per electrical cycle (coarse, lower CPU load)
    Steps16 = 16,
    /// 32 microsteps per electrical cycle (default, good balance)
    Steps32 = 32,
    /// 64 microsteps per electrical cycle (smooth, higher CPU load)
    Steps64 = 64,
}

impl MicrostepMode {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            16 => Some(Self::Steps16),
            32 => Some(Self::Steps32),
            64 => Some(Self::Steps64),
            _ => None,
        }
    }
}

impl Default for MicrostepMode {
    fn default() -> Self {
        Self::Steps32
    }
}

pub struct BipolarMicrostepper {
    // Lookup table for current microstep mode (quarter cycle, max 16 entries for 64 microsteps)
    scaled_lut: [u16; 16],
    nsteps: u8,
}

impl BipolarMicrostepper {
    // Pre-computed sine lookup tables (quarter cycle, 0 to 90 degrees, scaled to 32767)
    // sin(i * 90 / N) for i = 1..N (excluding 0 which is always 0)
    
    // 16 microsteps: 4 entries for quarter cycle
    const LUT_16: [u16; 4] = [12539, 23169, 30272, 32767];
    
    // 32 microsteps: 8 entries for quarter cycle
    const LUT_32: [u16; 8] = [
        6392, 12539, 18204, 23169, 27244, 30272, 32137, 32767,
    ];
    
    // 64 microsteps: 16 entries for quarter cycle
    const LUT_64: [u16; 16] = [
        3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169,
        25329, 27244, 28897, 30272, 31356, 32137, 32609, 32767,
    ];

    pub fn new(mode: MicrostepMode) -> Self {
        let nsteps = mode as u8;
        let mut scaled_lut = [0u16; 16];
        
        let quarter = (nsteps / 4) as usize;
        let src = match mode {
            MicrostepMode::Steps16 => &Self::LUT_16[..],
            MicrostepMode::Steps32 => &Self::LUT_32[..],
            MicrostepMode::Steps64 => &Self::LUT_64[..],
        };
        scaled_lut[..quarter].copy_from_slice(&src[..quarter]);

        Self { scaled_lut, nsteps }
    }

    pub fn set_scale(&mut self, full_scale: u16) {
        let quarter = (self.nsteps / 4) as usize;
        let base_lut = match self.nsteps {
            16 => &Self::LUT_16[..],
            32 => &Self::LUT_32[..],
            64 => &Self::LUT_64[..],
            _ => &Self::LUT_32[..],
        };
        for i in 0..quarter {
            self.scaled_lut[i] = ((base_lut[i] as i32 * full_scale as i32) / 32768) as u16;
        }
    }

    pub fn scale(&self) -> u16 {
        let quarter = (self.nsteps / 4) as usize;
        self.scaled_lut[quarter - 1]
    }

    fn lookup(&self, idx: usize) -> i32 {
        let nsteps = self.nsteps as usize;
        let negate = idx >= nsteps / 2;
        let mod_index = idx % (nsteps / 2);
        let quarter = nsteps / 4;

        // In order to shrink the table, we take advantage of some symmetry.
        // This works as long as NSTEPS % 4 == 0.
        // First, the second half of the sine function is the negative of the first.
        // Second, the first sample is always 0, so we don't store it.
        // Finally, after the peak, there's another mirroring.

        let mut lut_value = if mod_index == 0 {
            0
        } else if mod_index <= quarter {
            self.scaled_lut[mod_index - 1] as i32
        } else {
            self.scaled_lut[nsteps / 2 - mod_index - 1] as i32
        };
        if negate {
            lut_value = -lut_value;
        }

        lut_value
    }

    pub fn get(&self, step: usize) -> (i32, i32) {
        let nsteps = self.nsteps as usize;
        let step = step % nsteps;
        let a = self.lookup(step);
        let b = self.lookup((step + nsteps / 4) % nsteps);
        (a, b)
    }

    pub fn nsteps(&self) -> usize {
        self.nsteps as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Motor coils de-energized (no holding torque)
    Off,
    /// Motor stepping forward
    Forward,
    /// Motor stepping backward
    Reverse,
    /// Motor stationary but coils energized for holding torque
    Hold,
}

pub struct Stepper<'a> {
    stepper: BipolarMicrostepper,
    phase_a: Mutex<IChannel<'a>>,
    phase_b: Mutex<IChannel<'a>>,
    mode: AtomicCell<Mode>,
    stepper_pos: Mutex<Cell<u16>>,
    power: AtomicCell<u16>,
    /// Holding power as percentage (0-100) of peak power. 0 = holding disabled.
    hold_power_pct: AtomicU16,
    step_count: AtomicI32,
}

impl<'a> Stepper<'a> {
    pub fn new(phase_a: IChannel<'a>, phase_b: IChannel<'a>) -> Self {
        Self::with_microstep_mode(phase_a, phase_b, MicrostepMode::default())
    }
    
    pub fn with_microstep_mode(
        phase_a: IChannel<'a>, 
        phase_b: IChannel<'a>,
        mode: MicrostepMode,
    ) -> Self {
        let stepper = BipolarMicrostepper::new(mode);

        Self {
            stepper,
            phase_a: Mutex::new(phase_a),
            phase_b: Mutex::new(phase_b),
            mode: AtomicCell::new(Mode::Off),
            stepper_pos: Mutex::new(Cell::new(0)),
            power: AtomicCell::new(0),
            hold_power_pct: AtomicU16::new(0),
            step_count: AtomicI32::new(0),
        }
    }

    pub fn set_duty_cycles(&self, a: i16, b: i16) {
        critical_section::with(|cs| {
            let phase_a = self.phase_a.borrow(cs);
            let phase_b = self.phase_b.borrow(cs);
            phase_a.set_duty_cycle(a);
            phase_b.set_duty_cycle(b);
            self.mode.store(Mode::Off);
        })
    }

    pub fn disable(&self) {
        let hold_pct = self.hold_power_pct.load(Ordering::Relaxed);
        
        if hold_pct > 0 {
            // Enable holding torque: maintain current position with reduced power
            self.apply_hold_current();
            self.mode.store(Mode::Hold);
        } else {
            // No holding torque: de-energize coils completely
            critical_section::with(|cs| {
                let a = self.phase_a.borrow(cs);
                let b = self.phase_b.borrow(cs);
                a.set_duty_cycle(0);
                b.set_duty_cycle(0);
                self.mode.store(Mode::Off);
            });
        }
    }
    
    /// Fully disable motor including holding torque (force off)
    pub fn force_disable(&self) {
        critical_section::with(|cs| {
            let a = self.phase_a.borrow(cs);
            let b = self.phase_b.borrow(cs);
            a.set_duty_cycle(0);
            b.set_duty_cycle(0);
            self.mode.store(Mode::Off);
        });
    }
    
    /// Apply holding current at the current position
    fn apply_hold_current(&self) {
        let hold_pct = self.hold_power_pct.load(Ordering::Relaxed);
        if hold_pct == 0 {
            return;
        }
        
        let base_power = self.power.load() as i32;
        let hold_power = (base_power * hold_pct as i32 / 100) as u16;
        
        critical_section::with(|cs| {
            let stepper_pos = self.stepper_pos.borrow(cs).get();
            let (a, b) = self.stepper.get(stepper_pos as usize);
            let phase_a = self.phase_a.borrow(cs);
            let phase_b = self.phase_b.borrow(cs);
            phase_a.set_duty_cycle((a * hold_power as i32 / 32768) as i16);
            phase_b.set_duty_cycle((b * hold_power as i32 / 32768) as i16);
        });
    }

    pub fn enable(&self, reverse: bool) {
        let mode = if reverse {
            Mode::Reverse
        } else {
            Mode::Forward
        };
        self.mode.store(mode);
    }

    /// Set the peak duty cycle used for commutating
    pub fn set_power(&self, value: u16) {
        self.power.store(value);
    }
    
    /// Set holding torque as percentage of peak power (0-100).
    /// 0 = disabled, 100 = full power when stationary.
    /// Typical values: 25-50% for light holding, 50-75% for strong holding.
    pub fn set_hold_power(&self, percent: u16) {
        let percent = percent.min(100);
        self.hold_power_pct.store(percent, Ordering::Relaxed);
    }
    
    /// Get current holding power percentage
    pub fn hold_power(&self) -> u16 {
        self.hold_power_pct.load(Ordering::Relaxed)
    }
    
    /// Check if motor is in holding mode
    pub fn is_holding(&self) -> bool {
        self.mode.load() == Mode::Hold
    }

    pub fn step(&self) {
        let reverse = match self.mode.load() {
            Mode::Off | Mode::Hold => return,
            Mode::Forward => false,
            Mode::Reverse => true,
        };

        if reverse {
            self.step_count.fetch_sub(1, Ordering::Relaxed);
        } else {
            self.step_count.fetch_add(1, Ordering::Relaxed);
        }

        let mut new_pos = 0;
        critical_section::with(|cs| {
            let stepper_pos = self.stepper_pos.borrow(cs).get();
            new_pos = if reverse {
                stepper_pos.overflowing_sub(1).0
            } else {
                stepper_pos.overflowing_add(1).0
            };
            new_pos %= self.stepper.nsteps() as u16;
            self.stepper_pos.borrow(cs).set(new_pos);
        });

        let (a, b) = self.stepper.get(new_pos as usize);
        let power = self.power.load() as i32;
        critical_section::with(|cs| {
            let phase_a = self.phase_a.borrow(cs);
            let phase_b = self.phase_b.borrow(cs);
            phase_a.set_duty_cycle((a * power / 32768) as i16);
            phase_b.set_duty_cycle((b * power / 32768) as i16);
        });
    }

    pub fn step_count(&self) -> i32 {
        self.step_count.load(Ordering::Relaxed)
    }
}
