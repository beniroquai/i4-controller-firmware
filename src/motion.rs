use core::cell::Cell;

use critical_section::Mutex;
use portable_atomic::{AtomicBool, AtomicU16, Ordering};

/// True while a MOVE/SNAKE is active or either axis is still stepping.
/// Set by the USB handler when a command is queued, updated by the control task.
pub static BUSY: AtomicBool = AtomicBool::new(false);

/// Per-axis peak duty override (0 = use OBJECT3002). Not persisted.
static POWER: [AtomicU16; 2] = [AtomicU16::new(0), AtomicU16::new(0)];

/// Duty above this overflows the i16 duty in Stepper::step (LUT peak is 32767).
pub const MAX_POWER: u16 = 32767;

pub fn set_axis_power(axis: usize, value: u16) {
    POWER[axis].store(value.min(MAX_POWER), Ordering::Relaxed);
}

/// Effective peak duty for an axis given the global default.
pub fn axis_power(axis: usize, default: u16) -> u16 {
    match POWER[axis].load(Ordering::Relaxed) {
        0 => default.min(MAX_POWER),
        v => v,
    }
}

#[derive(Clone, Copy, Debug)]
pub enum MotionCommand {
    Cancel,
    MoveSteps {
        x_steps: i32,
        y_steps: i32,
        speed: u16,
    },
    SnakeScan {
        nx: u16,
        ny: u16,
        stepsx: i32,
        stepsy: i32,
        speed: u16,
        pause_ms: u32,
        /// Holding torque during pause phases (0-100% of peak power).
        /// Overrides the global OBJECT3003 setting for the duration of the scan.
        /// 0 = use global HOLD setting.
        hold_pct: u16,
    },
}

static MOTION_CMD: Mutex<Cell<Option<MotionCommand>>> = Mutex::new(Cell::new(None));

pub fn set_command(cmd: MotionCommand) {
    critical_section::with(|cs| {
        MOTION_CMD.borrow(cs).set(Some(cmd));
    });
}

pub fn take_command() -> Option<MotionCommand> {
    critical_section::with(|cs| {
        let cell = MOTION_CMD.borrow(cs);
        let cmd = cell.get();
        cell.set(None);
        cmd
    })
}
