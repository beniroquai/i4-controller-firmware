# i4-controller-firmware

Embedded firmware for the i4 controller, an STM32G474-based dual-axis stepper motor controller with CAN bus communication.

## Overview

This firmware controls two stepper motors (X and Y axes) using microstepping with bipolar current control. It features:

- **Dual-axis stepper motor control** with configurable velocity and acceleration
- **CAN bus interface** using CANopen-like protocol (zencan)
- **PWM-based current control** for smooth microstepping operation
- **Multiple operating modes**: direct duty cycle control or velocity-based stepping
- **Real-time operation** using the `lilos` async RTOS

## Hardware Platform

- **MCU**: STM32G474RE (ARM Cortex-M4F @ 128 MHz)
- **Communication**: CAN FD (500 kbps classic CAN mode)
- **Motor drivers**: 4-channel H-bridge with PWM current control
- **Debug**: RTT (Real-Time Transfer) for logging

## Prerequisites

### Required Tools

1. **Rust toolchain** (2024 edition):
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```

2. **ARM Cortex-M target**:
   ```bash
   rustup target add thumbv7em-none-eabihf
   ```

3. **flip-link** (linker wrapper for embedded):
   ```bash
   cargo install flip-link
   ```

4. **probe-rs** (for flashing and debugging):
   ```bash
   curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
   ```

   This will install:
   - `probe-rs` - CLI tool for flashing and debugging
   - `cargo-flash` - Cargo subcommand for flashing
   - `cargo-embed` - Cargo subcommand for development workflow

### Dependencies

This project depends on the `zencan` library for CAN bus communication. You need to have the `zencan` repository cloned in a sibling directory:

```bash
cd ..
git clone https://github.com/mcbridejc/zencan.git
cd i4-controller-firmware
```

The expected directory structure:
```
parent/
├── zencan/
│   └── zencan/
│       ├── zencan-node/
│       └── zencan-build/
└── i4-controller-firmware/
```

## Building

### Standard Build

```bash
cargo build --release
```

The compiled binary will be located at:
```
target/thumbv7em-none-eabihf/release/i4-controller
```

### Development Build

For faster iteration with debug symbols:
```bash
cargo build
```

Note: Even debug builds use optimization level `z` for size optimization (configured in Cargo.toml).

## Flashing

### Using probe-rs

```bash
probe-rs run --chip STM32G474RETx --release
```

### Using cargo-embed

If you have an `Embed.toml` configuration file:
```bash
cargo embed --release
```

## Debugging

### RTT Logging

The firmware uses RTT for real-time logging. To view logs:

```bash
probe-rs attach --chip STM32G474RETx
```

Or use `cargo-embed` which automatically displays RTT output.

### GDB Debugging

```bash
# Terminal 1: Start probe-rs gdb server
probe-rs gdb --chip STM32G474RETx target/thumbv7em-none-eabihf/release/i4-controller

# Terminal 2: Connect with GDB
arm-none-eabi-gdb target/thumbv7em-none-eabihf/release/i4-controller
(gdb) target remote :1337
(gdb) load
(gdb) continue
```

## Architecture

### Code Structure

```
src/
├── main.rs              # Main application, task orchestration, CAN setup
├── stepper.rs           # Stepper motor control with microstepping
├── current_control.rs   # H-bridge current control logic
├── pwm.rs              # PWM timer configuration and management
├── step_timer.rs       # Timer-based step generation
├── adc.rs              # ADC and op-amp configuration (for current sensing)
└── gpio.rs             # GPIO abstraction layer
```

### Core Components

#### 1. **Stepper Motor Control** ([stepper.rs](src/stepper.rs))

- **BipolarMicrostepper**: Implements 16-step microstepping using sine/cosine lookup tables
- Symmetric table optimization reduces flash usage
- Dynamic current scaling for power control
- Step position tracking with atomic operations

#### 2. **Current Control** ([current_control.rs](src/current_control.rs))

- **IChannel**: Controls one H-bridge phase (2 PWM channels)
- Bidirectional current control using complementary PWM
- Duty cycle conversion for signed current values

#### 3. **PWM Generation** ([pwm.rs](src/pwm.rs))

- Configures STM32 timers (TIM1, TIM3, TIM8) for PWM output
- 100 kHz PWM frequency for quiet motor operation
- 16-bit resolution with prescaler calculation

#### 4. **Step Timing** ([step_timer.rs](src/step_timer.rs))

- Uses TIM2 and TIM5 for precise step timing
- Interrupt-driven step execution
- Configurable step frequency (8-2000 Hz)

### Task Architecture

The firmware uses the `lilos` cooperative multitasking RTOS with three async tasks:

#### 1. **ADC Task** ([main.rs](src/main.rs#L369))
- Reads current sense ADC values every 5ms
- Updates CANopen objects with sensor data
- Currently disabled due to hardware design issues

#### 2. **CAN Task** ([main.rs](src/main.rs#L383))
- Processes incoming CAN messages
- Handles CANopen protocol (NMT, PDO, SDO)
- Transmits outgoing messages
- Updates operational state flag

#### 3. **Control Task** ([main.rs](src/main.rs#L482))
- Main motor control loop (5ms cycle)
- Implements two operating modes:
  - **Mode 0**: Direct duty cycle control
  - **Mode 1**: Velocity control with acceleration limiting
- Updates step timers and stepper positions

### CAN Communication

The firmware implements a CANopen-like protocol using the `zencan` library:

- **Node ID**: Configurable (default: 11)
- **Serial Number**: Derived from STM32 unique ID
- **Baud Rate**: 500 kbps (classic CAN)
- **Objects**:
  - `0x1018`: Device Identity
  - `0x2000`: Current sense ADC values
  - `0x3000`: Operating mode selection
  - `0x3001`: Acceleration limit
  - `0x3002`: Motor power/current scaling
  - `0x3100`: Direct duty cycle commands (mode 0)
  - `0x3101`: Velocity commands (mode 1)

### Operating Modes

#### Mode 0: Direct Duty Cycle Control

Directly sets H-bridge PWM duty cycles for each motor phase:
- `OBJECT3100[0]`: X motor phase A duty
- `OBJECT3100[1]`: X motor phase B duty
- `OBJECT3100[2]`: Y motor phase A duty
- `OBJECT3100[3]`: Y motor phase B duty

#### Mode 1: Velocity Control

Closed-loop velocity control with acceleration limiting:
- `OBJECT3101[0]`: X velocity command (steps/sec)
- `OBJECT3101[1]`: Y velocity command (steps/sec)
- `OBJECT3001`: Acceleration limit (steps/sec²)
- `OBJECT3002`: Power scaling (0-65535)

Velocity range: ±2000 steps/sec (limited by MIN_STEP_FREQ=8, MAX_STEP_FREQ=2000)

## Clock Configuration

- **System Clock**: 128 MHz (HSI16 × 8 PLL)
- **APB1/APB2**: 128 MHz (no prescaler)
- **PWM Frequency**: 100 kHz
- **Control Loop**: 200 Hz (5ms period)

## Interrupt Priorities

- **TIM2/TIM5** (step interrupts): High priority
- **FDCAN2_IT0** (CAN RX): Medium priority
- **SysTick** (RTOS scheduler): Default priority

## Memory Layout

See [memory.x](memory.x) for linker script:
- **Flash**: 512 KB @ 0x08000000
- **RAM**: 128 KB @ 0x20000000
- **CCMRAM**: 32 KB @ 0x10000000 (not used)

## Configuration Files

- **Cargo.toml**: Rust dependencies and build profiles
- **Embed.toml**: probe-rs embedding configuration
- **rust-toolchain.toml**: Rust toolchain version
- **zencan_config.toml**: CAN object dictionary configuration
- **memory.x**: Linker memory layout
- **build.rs**: Build-time code generation for CAN protocol

## Power Optimization

The firmware is optimized for code size:
- **LTO**: Fat link-time optimization enabled in release
- **Optimization level**: -Oz (size optimization)
- **Codegen units**: 1 (better optimization)

## Safety Considerations

This firmware uses `unsafe` code in several places:
1. Static peripheral initialization (one-time setup)
2. Interrupt access to shared stepper instances
3. Atomic operations for thread-safe communication

All `unsafe` usage is carefully documented and follows Rust embedded best practices.

### Troubleshooting

### Build Errors

**Problem**: Missing `zencan` dependency
```
error: failed to load source for dependency `zencan-node`
```
**Solution**: Clone zencan repository in parent directory
```bash
cd /Users/bene/Downloads  # or your workspace parent directory
git clone https://github.com/mcbridejc/zencan.git
cd i4-controller-firmware
```

**Problem**: Wrong Rust edition
```
Solution: Ensure Rust toolchain is up to date
rustup update
```

**Problem**: Missing `flip-link`
```
error: linker `flip-link` not found
```
**Solution**: Install flip-link
```bash
cargo install flip-link
```

**Problem**: `probe-rs-tools` installation fails due to Rust version mismatch
```
error: rustc 1.85.1 is not supported
```
**Solution**: Use the official installer script instead:
```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

### Runtime Issues

**Problem**: Motors not moving
- Check NMT state is "Operational" (send NMT start command)
- Verify velocity commands are within range (8-2000 steps/sec)
- Check H-bridge enable pins are configured

**Problem**: CAN communication not working
- Verify CAN baud rate matches bus (500 kbps)
- Check termination resistors on CAN bus
- Use RTT logs to debug message reception

## License

See repository for license information.

## Contributing

This is a hardware-specific embedded project. For contributions:
1. Ensure code follows existing style
2. Test on actual hardware
3. Document any hardware dependencies
4. Keep unsafe code to a minimum


## Flashing 

```
probe-rs list
probe-rs run --chip STM32G474RETx target/thumbv7em-none-eabihf/release/i4-controller

# Alternatively, if you have an Embed.toml configuration:
cargo install cargo-binutils
rustup component add llvm-tools
cargo objcopy --release -- -O binary i4-controller.bin
# Then flashing works with:
cargo embed --release
```

## Author

Based on the repository: mcbridejc/i4-controller-firmware
