//! USB CDC ACM serial interface for velocity commands.
//!
//! Provides a virtual COM port over USB for controlling XY motor velocity
//! from a host PC. Commands are newline-delimited ASCII.
//!
//! # Protocol
//!
//! | Command        | Description                             |
//! |----------------|-----------------------------------------|
//! | `V <x> <y>\n`                | Set XY velocity (i16, steps/s)     |
//! | `MOVE <x> <y> [speed]\n`      | Move relative steps at speed       |
//! | `SNAKE <nx> <ny> <sx> <sy> <speed> <pause_ms>\n` | Snake scan |
//! | `STOP\n`                      | Stop motors (equivalent to `V 0 0`)|
//! | `PING\n`                      | Returns `OK\n`                     |
//! | `HELP\n` / `?`                | Show available commands            |

use core::convert::Infallible;

use lilos::exec::Notify;
use lilos::time::Millis;
use static_cell::StaticCell;
use stm32_usbd::{UsbBus, UsbPeripheral};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::pac;
use crate::{motion, motion::MotionCommand};

// ---------------------------------------------------------------------------
// STM32G474 USB FS peripheral descriptor
// ---------------------------------------------------------------------------

/// Zero-sized type representing the STM32G474 USB Full-Speed device peripheral.
pub struct UsbPeriph;

unsafe impl Sync for UsbPeriph {}
unsafe impl Send for UsbPeriph {}

unsafe impl UsbPeripheral for UsbPeriph {
    /// USB device register block (RM0440 §44.6, base 0x4000_5C00).
    const REGISTERS: *const () = 0x4000_5C00 as *const ();

    /// The STM32G474 has an internal DP pull-up controlled via the BCDR register.
    const DP_PULL_UP_FEATURE: bool = true;

    /// Packet Buffer Memory Area base address (1024 bytes at 0x4000_6000).
    const EP_MEMORY: *const () = 0x4000_6000 as *const ();

    /// Total PMA size in bytes.
    const EP_MEMORY_SIZE: usize = 1024;

    /// STM32G4 uses 2×16 access – both 16-bit halves of each 32-bit word are usable.
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral clock on APB1
            pac::RCC.apb1enr1().modify(|w| w.set_usben(true));
            // Reset USB peripheral
            pac::RCC.apb1rstr1().modify(|w| w.set_usbrst(true));
            pac::RCC.apb1rstr1().modify(|w| w.set_usbrst(false));
        });
    }

    fn startup_delay() {
        // ≈1 µs at 128 MHz system clock
        cortex_m::asm::delay(128);
    }
}

/// Convenience type alias used throughout this module.
pub type UsbBusType = UsbBus<UsbPeriph>;

// ---------------------------------------------------------------------------
// Static allocations
// ---------------------------------------------------------------------------

static USB_BUS: StaticCell<UsbBusAllocator<UsbBusType>> = StaticCell::new();
static USB_SERIAL_BUF: StaticCell<[u8; 8]> = StaticCell::new();

// ---------------------------------------------------------------------------
// Velocity limits – must match the constants in main.rs / control loop
// ---------------------------------------------------------------------------

const MAX_STEP_FREQ: i16 = 2000;
const DEFAULT_MOVE_SPEED: i16 = 500;

// ---------------------------------------------------------------------------
// Public initialisation helpers (called from main before the task list)
// ---------------------------------------------------------------------------

/// Enable the HSI48 RC oscillator and select it as the 48 MHz USB clock source.
///
/// Must be called **before** [`init_usb_bus`].
pub fn init_usb_clock() {
    // Enable HSI48 oscillator
    pac::RCC.crrcr().modify(|w| w.set_hsi48on(true));
    // Wait until it is stable
    while !pac::RCC.crrcr().read().hsi48rdy() {}
    // Select HSI48 as the CLK48 source used by the USB peripheral
    pac::RCC
        .ccipr()
        .modify(|w| w.set_clk48sel(pac::rcc::vals::Clk48sel::HSI48));
}

/// Create the global USB bus allocator (singleton).
///
/// Returns a `&'static` reference that can be passed to [`usb_task`].
/// Must be called exactly once, after [`init_usb_clock`].
pub fn init_usb_bus() -> &'static UsbBusAllocator<UsbBusType> {
    USB_BUS.init(UsbBus::new(UsbPeriph))
}

/// Format a 32-bit value as an 8-character uppercase hex string.
///
/// The result lives in a `StaticCell` and is suitable for
/// [`UsbDeviceBuilder::serial_number`].
pub fn init_serial_string(serial: u32) -> &'static str {
    let buf = USB_SERIAL_BUF.init([0u8; 8]);
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for i in 0..8 {
        buf[i] = HEX[((serial >> (28 - 4 * i)) & 0xF) as usize];
    }
    // Safety: the buffer only contains ASCII hex digits.
    core::str::from_utf8(buf).unwrap()
}

// ---------------------------------------------------------------------------
// USB serial task
// ---------------------------------------------------------------------------

/// Write a byte slice to the USB serial port, silently ignoring errors
/// (e.g. buffer full, device not configured yet).
fn write_response(serial: &mut SerialPort<UsbBusType>, msg: &[u8]) {
    let _ = serial.write(msg);
}

/// Parse and execute one command line, sending the response over USB serial.
fn handle_command(
    line: &[u8],
    serial: &mut SerialPort<UsbBusType>,
    control_notify: &Notify,
) {
    let line_str = match core::str::from_utf8(line) {
        Ok(s) => s.trim(),
        Err(_) => {
            write_response(serial, b"ERR invalid utf8\n");
            return;
        }
    };

    if line_str.is_empty() {
        return;
    }

    let mut parts = line_str.split_ascii_whitespace();
    let cmd = match parts.next() {
        Some(c) => c,
        None => return,
    };

    if cmd.eq_ignore_ascii_case("PING") {
        write_response(serial, b"OK\n");
    } else if cmd.eq_ignore_ascii_case("STOP") {
        crate::zencan::OBJECT3101.set(0, 0i16).ok();
        crate::zencan::OBJECT3101.set(1, 0i16).ok();
        motion::set_command(MotionCommand::Cancel);
        control_notify.notify();
        write_response(serial, b"OK\n");
    } else if cmd.eq_ignore_ascii_case("HELP") || cmd.eq_ignore_ascii_case("?") {
        write_response(serial, b"Commands:\n");
        write_response(serial, b"  V <x> <y>                       - set XY velocity (steps/s, i16)\n");
        write_response(serial, b"  MOVE <x> <y> [speed]            - move relative steps (default 500)\n");
        write_response(serial, b"  SNAKE <nx> <ny> <sx> <sy> <speed> <pause_ms> - snake scan\n");
        write_response(serial, b"  STOP                            - stop motors (V 0 0)\n");
        write_response(serial, b"  PING                            - returns OK\n");
        write_response(serial, b"  HELP / ?                        - show this help\n");
    } else if cmd.eq_ignore_ascii_case("V") {
            let x_str = match parts.next() {
                Some(s) => s,
                None => {
                    write_response(serial, b"ERR missing x\n");
                    return;
                }
            };
            let y_str = match parts.next() {
                Some(s) => s,
                None => {
                    write_response(serial, b"ERR missing y\n");
                    return;
                }
            };

            let x: i16 = match x_str.parse() {
                Ok(v) => v,
                Err(_) => {
                    write_response(serial, b"ERR bad x\n");
                    return;
                }
            };
            let y: i16 = match y_str.parse() {
                Ok(v) => v,
                Err(_) => {
                    write_response(serial, b"ERR bad y\n");
                    return;
                }
            };

            // Clamp to firmware limits (same as CAN path)
            let x = x.clamp(-MAX_STEP_FREQ, MAX_STEP_FREQ);
            let y = y.clamp(-MAX_STEP_FREQ, MAX_STEP_FREQ);

            crate::zencan::OBJECT3101.set(0, x).ok();
            crate::zencan::OBJECT3101.set(1, y).ok();
            motion::set_command(MotionCommand::Cancel);
            control_notify.notify();

            write_response(serial, b"OK\n");
    } else if cmd.eq_ignore_ascii_case("MOVE") || cmd.eq_ignore_ascii_case("M") {
        let x_steps: i32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => {
                write_response(serial, b"ERR bad x\n");
                return;
            }
        };
        let y_steps: i32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => {
                write_response(serial, b"ERR bad y\n");
                return;
            }
        };
        let speed: i32 = match parts.next() {
            Some(s) => match s.parse() {
                Ok(v) => v,
                Err(_) => {
                    write_response(serial, b"ERR bad speed\n");
                    return;
                }
            },
            None => DEFAULT_MOVE_SPEED as i32,
        };

        if speed <= 0 {
            write_response(serial, b"ERR bad speed\n");
            return;
        }

        let speed = (speed as i16).clamp(1, MAX_STEP_FREQ) as u16;
        motion::set_command(MotionCommand::MoveSteps {
            x_steps,
            y_steps,
            speed,
        });
        control_notify.notify();
        write_response(serial, b"OK\n");
    } else if cmd.eq_ignore_ascii_case("SNAKE") || cmd.eq_ignore_ascii_case("SCAN") {
        let nx: u16 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) if v > 0 => v,
            _ => {
                write_response(serial, b"ERR bad nx\n");
                return;
            }
        };
        let ny: u16 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) if v > 0 => v,
            _ => {
                write_response(serial, b"ERR bad ny\n");
                return;
            }
        };
        let stepsx: i32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) if v > 0 => v,
            _ => {
                write_response(serial, b"ERR bad stepsx\n");
                return;
            }
        };
        let stepsy: i32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) if v > 0 => v,
            _ => {
                write_response(serial, b"ERR bad stepsy\n");
                return;
            }
        };
        let speed: i32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) if v > 0 => v,
            _ => {
                write_response(serial, b"ERR bad speed\n");
                return;
            }
        };
        let pause_ms: u32 = match parts.next().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => {
                write_response(serial, b"ERR bad pause\n");
                return;
            }
        };

        let speed = (speed as i16).clamp(1, MAX_STEP_FREQ) as u16;
        motion::set_command(MotionCommand::SnakeScan {
            nx,
            ny,
            stepsx,
            stepsy,
            speed,
            pause_ms,
        });
        control_notify.notify();
        write_response(serial, b"OK\n");
    } else {
        write_response(serial, b"ERR unknown cmd\n");
    }
}

/// Async task: USB CDC ACM serial interface.
///
/// Polls the USB stack at 1 ms intervals, reads incoming bytes, assembles
/// complete lines (delimited by `\n`), and dispatches commands to the motor
/// control path via the zencan object dictionary.
///
/// This task never returns.
pub async fn usb_task(
    usb_bus: &'static UsbBusAllocator<UsbBusType>,
    control_notify: &Notify,
    serial_str: &'static str,
) -> Infallible {
    let mut serial = SerialPort::new(usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("i4")
            .product("i4-controller")
            .serial_number(serial_str)])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    // Line assembly buffer (64 bytes is plenty for the simple protocol)
    let mut line_buf = [0u8; 64];
    let mut line_pos: usize = 0;
    let mut overflow = false;

    // Poll at 1 ms – matches the USB FS SOF period
    let mut poll_gate = lilos::time::PeriodicGate::new_shift(Millis(1), Millis(0));

    loop {
        poll_gate.next_time().await;

        // Drive the USB state machine (enumeration, control transfers, data)
        usb_dev.poll(&mut [&mut serial]);

        // Try to read bytes the host may have sent
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Ok(count) => {
                for &byte in &buf[..count] {
                    if byte == b'\n' {
                        if overflow {
                            write_response(&mut serial, b"ERR line too long\n");
                            overflow = false;
                        } else {
                            handle_command(
                                &line_buf[..line_pos],
                                &mut serial,
                                control_notify,
                            );
                        }
                        line_pos = 0;
                    } else if byte == b'\r' {
                        // Ignore carriage returns
                    } else if overflow {
                        // Discard bytes until the next newline
                    } else if line_pos < line_buf.len() {
                        line_buf[line_pos] = byte;
                        line_pos += 1;
                    } else {
                        // Line buffer full – flag overflow
                        overflow = true;
                    }
                }
            }
            Err(_) => {
                // UsbError::WouldBlock (no data) or other transient error – nothing to do
            }
        }
    }
}
