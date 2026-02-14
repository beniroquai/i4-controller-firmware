use core::cell::Cell;

use critical_section::Mutex;

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
