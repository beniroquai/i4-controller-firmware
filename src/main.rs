#![no_std]
#![no_main]
use core::{
    convert::Infallible,
    pin::pin,
    sync::atomic::Ordering,
    time::Duration,
};

use grounded::uninit::GroundedCell;
use lilos::{exec::Notify, time::Millis};
use pac::{
    RCC,
    timer::{TimAdv, TimGp16},
};
use portable_atomic::AtomicBool;
use pwm::PwmTimer;
use static_cell::StaticCell;
use stm32_metapac::{self as pac, interrupt};
use zencan_node::{
    common::{nmt::NmtState, NodeId}, Callbacks, Node
};

use crate::{gpio::DynamicPin, step_timer::StepTimer};

use cortex_m_rt as _;
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};

mod adc;
mod can;
mod current_control;
mod gpio;
mod motion;
mod pwm;
mod step_timer;
mod stepper;
mod usb_serial;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

use stepper::Stepper;

use gpio::Pin;

fn get_serial() -> u32 {
    let mut ctx = md5::Context::new();
    ctx.consume(pac::UID.uid(0).read().to_le_bytes());
    ctx.consume(pac::UID.uid(1).read().to_le_bytes());
    ctx.consume(pac::UID.uid(2).read().to_le_bytes());
    let digest = ctx.compute();
    u32::from_le_bytes(digest.0[0..4].try_into().unwrap())
}

static X_STEPPER: GroundedCell<Stepper> = GroundedCell::uninit();
static Y_STEPPER: GroundedCell<Stepper> = GroundedCell::uninit();

/// Notify for NodeMbox notify callback to access
static CAN_NOTIFY: Notify = Notify::new();

static TIMER1: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();
static TIMER3: StaticCell<PwmTimer<TimGp16>> = StaticCell::new();
static TIMER8: StaticCell<PwmTimer<TimAdv>> = StaticCell::new();

const MIN_STEP_FREQ: i32 = 16;
const MAX_STEP_FREQ: i32 = 2000;

fn notify_can_task() {
    CAN_NOTIFY.notify();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                name: "defmt",
            }
        }
    };

    set_defmt_channel(channels.up.0);

    // Setup PLL to generate clock of 16MHz * 8
    RCC.cr().write(|w| w.set_hsion(true));
    RCC.pllcfgr().write(|w| {
        w.set_plln(pac::rcc::vals::Plln::MUL16);
        w.set_pllm(pac::rcc::vals::Pllm::DIV1);
        w.set_pllsrc(pac::rcc::vals::Pllsrc::HSI);
        w.set_pllr(pac::rcc::vals::Pllr::DIV2);
        w.set_pllren(true);
    });
    // Enable the PLL
    RCC.cr().modify(|w| w.set_pllon(true));
    while !RCC.cr().read().pllrdy() {}

    const CLK_FREQ: u32 = 128_000_000;
    const PWM_FREQ: u32 = 100_000;

    pac::PWR.cr5().modify(|w| w.set_r1mode(true));
    pac::FLASH
        .acr()
        .modify(|w| w.set_latency(pac::flash::vals::Latency::WS4));
    RCC.cfgr().modify(|w| {
        w.set_sw(pac::rcc::vals::Sw::PLL1_R);
        w.set_hpre(pac::rcc::vals::Hpre::DIV1);
        w.set_ppre1(pac::rcc::vals::Ppre::DIV1);
        w.set_ppre2(pac::rcc::vals::Ppre::DIV1);
    });

    RCC.apb1enr1().modify(|w| {
        w.set_tim2en(true);
        w.set_tim3en(true);
        w.set_tim5en(true);
        w.set_fdcanen(true);
    });
    RCC.ccipr()
        .write(|w| w.set_fdcansel(pac::rcc::vals::Fdcansel::PCLK1));
    RCC.apb2enr().modify(|w| {
        w.set_tim1en(true);
        w.set_tim8en(true);
    });
    RCC.ahb2enr().modify(|w| {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpioeen(true);
        w.set_gpiofen(true);
        w.set_gpiogen(true);
    });

    let gpios = gpio::gpios();

    // Configure timer output GPIOS to the appropriate AF for the timer channels
    let io_ch1_in1 = gpios.PC3;
    let io_ch1_in2 = gpios.PA6;
    let io_ch2_in1 = gpios.PA8;
    let io_ch2_in2 = gpios.PA7;
    let io_ch3_in1 = gpios.PA9;
    let io_ch3_in2 = gpios.PA15;
    let io_ch4_in1 = gpios.PB8;
    let io_ch4_in2 = gpios.PB9;
    io_ch1_in1.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch1_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch2_in1.set_as_af(6, gpio::AFType::OutputPushPull);
    io_ch2_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch3_in1.set_as_af(6, gpio::AFType::OutputPushPull);
    io_ch3_in2.set_as_af(2, gpio::AFType::OutputPushPull);
    io_ch4_in1.set_as_af(10, gpio::AFType::OutputPushPull);
    io_ch4_in2.set_as_af(10, gpio::AFType::OutputPushPull);

    let io_ch1_en1 = gpios.PA4;
    let io_ch1_en2 = gpios.PC5;
    let io_ch2_en1 = gpios.PC6;
    let io_ch2_en2 = gpios.PC4;
    let io_ch3_en1 = gpios.PA10;
    let io_ch3_en2 = gpios.PC10;
    let io_ch4_en1 = gpios.PC12;
    let io_ch4_en2 = gpios.PD2;
    let enable_ios: &mut [DynamicPin] = &mut [
        io_ch1_en1.into(),
        io_ch1_en2.into(),
        io_ch2_en1.into(),
        io_ch2_en2.into(),
        io_ch3_en1.into(),
        io_ch3_en2.into(),
        io_ch4_en1.into(),
        io_ch4_en2.into(),
    ];

    for io in enable_ios {
        io.set_high();
        io.set_as_output(gpio::Speed::Low);
    }

    // adc::configure_adcs();
    // adc::configure_op_amps();

    let timer1 = TIMER1.init(pwm::PwmTimer::<TimAdv>::new(pac::TIM1, PWM_FREQ, CLK_FREQ));
    let timer3 = TIMER3.init(pwm::PwmTimer::<TimGp16>::new(pac::TIM3, PWM_FREQ, CLK_FREQ));
    let timer8 = TIMER8.init(pwm::PwmTimer::<TimAdv>::new(pac::TIM8, PWM_FREQ, CLK_FREQ));

    let ch1_in1 = timer1.ch4();
    let ch1_in2 = timer3.ch1();
    let ch2_in1 = timer1.ch1();
    let ch2_in2 = timer3.ch2();
    let ch3_in1 = timer1.ch2();
    let ch3_in2 = timer8.ch1();
    let ch4_in1 = timer8.ch2();
    let ch4_in2 = timer8.ch3();

    let current1 = current_control::IChannel::new(ch1_in1, ch1_in2);
    let current2 = current_control::IChannel::new(ch2_in1, ch2_in2);
    let current3 = current_control::IChannel::new(ch3_in1, ch3_in2);
    let current4 = current_control::IChannel::new(ch4_in1, ch4_in2);
    current1.set_duty_cycle(0);
    current2.set_duty_cycle(0);
    current3.set_duty_cycle(0);
    current4.set_duty_cycle(0);

    // Safety: This is the only time we will mutate these statics
    let (x_stepper, y_stepper) = unsafe {
        *X_STEPPER.get() = Stepper::new(current1, current2);
        *Y_STEPPER.get() = Stepper::new(current3, current4);
        (&*X_STEPPER.get(), &*Y_STEPPER.get())
    };

    let mut x_step_timer = StepTimer::new(pac::TIM2, CLK_FREQ);
    let mut y_step_timer = StepTimer::new(pac::TIM5, CLK_FREQ);
    x_step_timer.enable_irq();
    y_step_timer.enable_irq();

    timer1.enable();
    timer3.enable();
    timer8.enable();

    let can_rx_pin = gpios.PB12;
    let mut can_tx_pin = gpios.PB13;
    can_tx_pin.set_high();
    can_tx_pin.set_as_output(gpio::Speed::High);
    can_rx_pin.set_as_af(9, gpio::AFType::Input);
    can_tx_pin.set_as_af(9, gpio::AFType::OutputPushPull);

    can::init_can();

    // Register a hook for notification when messages are received that are awaiting handling by
    // node process call. This will be called in the CAN IRQ context, and will set a notify to cause
    // the CAN task to promptly execute.
    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);
    zencan::NODE_MBOX.set_transmit_notify_callback(&can::transmit_notify_handler);

    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_sleep(true);
        w.set_dbg_stop(true);
    });
    // Have to enable DMA1 clock to keep RAM accessible for RTT during debug
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));

    // Initialize USB CDC ACM serial interface
    usb_serial::init_usb_clock();
    let usb_bus = usb_serial::init_usb_bus();
    let usb_serial_str = usb_serial::init_serial_string(get_serial());

    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Configure the systick timer for 1kHz ticks at 16MHz.
    lilos::time::initialize_sys_tick(&mut cp.SYST, CLK_FREQ);

    // Use the UID register to set a unique serial number
    zencan::OBJECT1018.set_serial(get_serial());

    // Node ID is hardcoded for now
    let callbacks = Callbacks::default();
    let node = Node::new(
        NodeId::new(11).unwrap(),
        callbacks,
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );

    unsafe {
        cortex_m::interrupt::enable();
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::FDCAN2_IT0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM5);
    }


    // Create a flag to communicate status between CAN task and control task
    let operational_flag = portable_atomic::AtomicBool::new(false);
    let control_notify = Notify::new();

    defmt::info!("Starting tasks");

    lilos::exec::run_tasks(
        &mut [
            pin!(adc_task()),
            pin!(can_task(
                node,
                &CAN_NOTIFY,
                &control_notify,
                &operational_flag
            )),
            pin!(control_task(
                x_step_timer,
                y_step_timer,
                x_stepper,
                y_stepper,
                &control_notify,
                &operational_flag
            )),
            pin!(usb_serial::usb_task(
                usb_bus,
                &control_notify,
                usb_serial_str
            )),
        ],
        lilos::exec::ALL_TASKS,
    );
}


/// Task to read the ISENSE adc channels
///
/// Note that these are basically junk due to design issues with the current sensing.
/// It's here waiting for a future hardware rev, and for supporting experiments.
async fn adc_task() -> Infallible {
    const ADC_INTERVAL: Millis = Millis(5);
    let mut periodic_gate = lilos::time::PeriodicGate::new_shift(ADC_INTERVAL, Millis(0));
    loop {
        periodic_gate.next_time().await;

        let adc_values = adc::read_isense();

        for (i, value) in adc_values.iter().enumerate() {
            zencan::OBJECT2000.set(i, *value).ok();
        }
    }
}

/// Task to run the zencan process
async fn can_task(
    mut node: Node<'static>,
    process_notify: &Notify,
    control_notify: &Notify,
    operational_flag: &AtomicBool,
) -> Infallible {
    let epoch = lilos::time::TickTime::now();
    loop {
        lilos::time::with_timeout(Duration::from_millis(10), process_notify.until_next()).await;
        let time_us = epoch.elapsed().0 * 1000;
        let objects_updated = node.process(time_us);
        if objects_updated {
            control_notify.notify();
        }
        operational_flag.store(node.nmt_state() == NmtState::Operational, Ordering::Relaxed);
    }
}

fn run_duty_cycle_mode(x_stepper: &Stepper<'static>, y_stepper: &Stepper<'static>) {
    x_stepper.set_duty_cycles(
        zencan::OBJECT3100.get(0).unwrap(),
        zencan::OBJECT3100.get(1).unwrap(),
    );
    y_stepper.set_duty_cycles(
        zencan::OBJECT3100.get(2).unwrap(),
        zencan::OBJECT3100.get(3).unwrap(),
    );
}

fn run_stepper_mode(
    x_vel: &mut i32,
    y_vel: &mut i32,
    delta_t_ms: u32,
    x_step_timer: &mut StepTimer,
    y_step_timer: &mut StepTimer,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
) {
    let x_vel_cmd = zencan::OBJECT3101.get(0).unwrap() as i32;
    let y_vel_cmd = zencan::OBJECT3101.get(1).unwrap() as i32;
    let accel = zencan::OBJECT3001.get_value();
    let power = zencan::OBJECT3002.get_value();
    if accel == 0 {
        *x_vel = x_vel_cmd;
        *y_vel = y_vel_cmd;
    } else {
        // Compute the maximum change in velocity, but never round to 0.
        let max_delta = (((delta_t_ms * accel as u32) / 1000) as i32).max(1);
        let mut x_delta = x_vel_cmd - *x_vel;
        if x_delta.abs() > max_delta {
            x_delta = x_delta.signum() * max_delta;
        }
        let mut y_delta = y_vel_cmd - *y_vel;
        if y_delta.abs() > max_delta {
            y_delta = y_delta.signum() * max_delta;
        }
        *x_vel += x_delta;
        *y_vel += y_delta;
    }

    if x_vel.abs() > MAX_STEP_FREQ {
        *x_vel = x_vel.signum() * MAX_STEP_FREQ;
    }
    if y_vel.abs() > MAX_STEP_FREQ {
        *y_vel = y_vel.signum() * MAX_STEP_FREQ;
    }

    x_stepper.set_power(power);
    y_stepper.set_power(power);

    if x_vel.abs() > MIN_STEP_FREQ || y_vel.abs() > MIN_STEP_FREQ {
        // Set the timer frequencies for the timer IRQs which will trigger steps
        let x_reverse = *x_vel < 0;
        let y_reverse = *y_vel < 0;

        x_step_timer.set_overflow_freq(x_vel.unsigned_abs());
        y_step_timer.set_overflow_freq(y_vel.unsigned_abs());
        // Set the appropriate directions
        x_stepper.enable(x_reverse);
        y_stepper.enable(y_reverse);
    } else {
        // If the step frequency is too low, just turn the current off
        x_step_timer.set_overflow_freq(0);
        y_step_timer.set_overflow_freq(0);
        x_stepper.disable();
        y_stepper.disable();
    }
}

#[derive(Clone, Copy, Debug)]
struct MoveState {
    x_target: i32,
    y_target: i32,
    x_dir: i32,
    y_dir: i32,
    speed: i32,
}

#[derive(Clone, Copy, Debug)]
enum SnakePhase {
    Pause { remaining_ms: u32 },
    Move(MoveState),
}

#[derive(Clone, Copy, Debug)]
struct SnakeState {
    nx: u16,
    ny: u16,
    stepsx: i32,
    stepsy: i32,
    speed: i32,
    pause_ms: u32,
    row: u16,
    col: u16,
    dir: i32,
    phase: SnakePhase,
}

#[derive(Clone, Copy, Debug)]
enum MotionState {
    Idle,
    Move(MoveState),
    Snake(SnakeState),
}

fn set_axis_velocity(index: usize, vel: i32) {
    let vel = vel.clamp(-MAX_STEP_FREQ, MAX_STEP_FREQ) as i16;
    zencan::OBJECT3101.set(index, vel).ok();
}

fn build_move_state(
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
    x_steps: i32,
    y_steps: i32,
    speed: i32,
) -> MoveState {
    let speed = speed.clamp(MIN_STEP_FREQ, MAX_STEP_FREQ);
    let x_dir = x_steps.signum();
    let y_dir = y_steps.signum();
    let x_target = x_stepper.step_count().saturating_add(x_steps);
    let y_target = y_stepper.step_count().saturating_add(y_steps);

    MoveState {
        x_target,
        y_target,
        x_dir,
        y_dir,
        speed,
    }
}

fn update_move_state(
    state: &MoveState,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
) -> bool {
    let x_now = x_stepper.step_count();
    let y_now = y_stepper.step_count();

    let x_done = if state.x_dir == 0 {
        true
    } else if state.x_dir > 0 {
        x_now >= state.x_target
    } else {
        x_now <= state.x_target
    };

    let y_done = if state.y_dir == 0 {
        true
    } else if state.y_dir > 0 {
        y_now >= state.y_target
    } else {
        y_now <= state.y_target
    };

    let x_vel = if x_done { 0 } else { state.x_dir * state.speed };
    let y_vel = if y_done { 0 } else { state.y_dir * state.speed };

    set_axis_velocity(0, x_vel);
    set_axis_velocity(1, y_vel);

    x_done && y_done
}

fn is_snake_complete(state: &SnakeState) -> bool {
    if state.nx == 0 || state.ny == 0 {
        return true;
    }
    if state.row != state.ny - 1 {
        return false;
    }
    if state.dir > 0 {
        state.col == state.nx - 1
    } else {
        state.col == 0
    }
}

fn snake_build_next_move(
    state: &mut SnakeState,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
) -> Option<MoveState> {
    if state.dir > 0 {
        if state.col + 1 < state.nx {
            state.col += 1;
            Some(build_move_state(
                x_stepper,
                y_stepper,
                state.stepsx,
                0,
                state.speed,
            ))
        } else if state.row + 1 < state.ny {
            state.row += 1;
            state.dir = -1;
            Some(build_move_state(
                x_stepper,
                y_stepper,
                0,
                state.stepsy,
                state.speed,
            ))
        } else {
            None
        }
    } else if state.col > 0 {
        state.col -= 1;
        Some(build_move_state(
            x_stepper,
            y_stepper,
            -state.stepsx,
            0,
            state.speed,
        ))
    } else if state.row + 1 < state.ny {
        state.row += 1;
        state.dir = 1;
        Some(build_move_state(
            x_stepper,
            y_stepper,
            0,
            state.stepsy,
            state.speed,
        ))
    } else {
        None
    }
}

fn update_snake_state(
    state: &mut SnakeState,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
    delta_t_ms: u32,
) -> bool {
    match &mut state.phase {
        SnakePhase::Move(move_state) => {
            if update_move_state(move_state, x_stepper, y_stepper) {
                state.phase = SnakePhase::Pause {
                    remaining_ms: state.pause_ms,
                };
            }
            false
        }
        SnakePhase::Pause { remaining_ms } => {
            set_axis_velocity(0, 0);
            set_axis_velocity(1, 0);

            if *remaining_ms > delta_t_ms {
                *remaining_ms -= delta_t_ms;
                return false;
            }
            *remaining_ms = 0;

            if is_snake_complete(state) {
                return true;
            }

            if let Some(next_move) = snake_build_next_move(state, x_stepper, y_stepper) {
                state.phase = SnakePhase::Move(next_move);
                false
            } else {
                true
            }
        }
    }
}

async fn control_task(
    mut x_step_timer: StepTimer,
    mut y_step_timer: StepTimer,
    x_stepper: &Stepper<'static>,
    y_stepper: &Stepper<'static>,
    control_notify: &Notify,
    operational_flag: &AtomicBool,
) -> Infallible {
    const CONTROL_INTERVAL: Duration = Duration::from_millis(5);

    let sleep = || lilos::time::with_timeout(CONTROL_INTERVAL, control_notify.until_next());

    let mut motion_state = MotionState::Idle;
    let mut previous_time = lilos::time::TickTime::now();

    loop {
        // Do nothing until we enter operational mode
        while !operational_flag.load(Ordering::Relaxed) {
            sleep().await;
        }

        // Mode is latched once when transitioning to operational, and changes will not take effect
        // until you transition to PreOp and back.
        let mode = zencan::OBJECT3000.get_value();

        defmt::info!("Starting with mode = {}", mode);
        let mut x_vel = 0;
        let mut y_vel = 0;
        loop {
            if mode == 0 {
                run_duty_cycle_mode(x_stepper, y_stepper);
            } else if mode == 1 {
                let now = lilos::time::TickTime::now();
                let delta_t_ms = now.millis_since(previous_time).0 as u32;
                previous_time = now;

                if let Some(cmd) = motion::take_command() {
                    match cmd {
                        motion::MotionCommand::Cancel => {
                            motion_state = MotionState::Idle;
                        }
                        motion::MotionCommand::MoveSteps {
                            x_steps,
                            y_steps,
                            speed,
                        } => {
                            let move_state = build_move_state(
                                x_stepper,
                                y_stepper,
                                x_steps,
                                y_steps,
                                speed as i32,
                            );
                            motion_state = MotionState::Move(move_state);
                        }
                        motion::MotionCommand::SnakeScan {
                            nx,
                            ny,
                            stepsx,
                            stepsy,
                            speed,
                            pause_ms,
                        } => {
                            let speed = (speed as i32).clamp(MIN_STEP_FREQ, MAX_STEP_FREQ);
                            let snake_state = SnakeState {
                                nx,
                                ny,
                                stepsx,
                                stepsy,
                                speed,
                                pause_ms,
                                row: 0,
                                col: 0,
                                dir: 1,
                                phase: SnakePhase::Pause {
                                    remaining_ms: pause_ms,
                                },
                            };
                            motion_state = MotionState::Snake(snake_state);
                        }
                    }
                }

                match &mut motion_state {
                    MotionState::Idle => {}
                    MotionState::Move(move_state) => {
                        if update_move_state(move_state, x_stepper, y_stepper) {
                            motion_state = MotionState::Idle;
                        }
                    }
                    MotionState::Snake(snake_state) => {
                        if update_snake_state(snake_state, x_stepper, y_stepper, delta_t_ms) {
                            motion_state = MotionState::Idle;
                            set_axis_velocity(0, 0);
                            set_axis_velocity(1, 0);
                        }
                    }
                }
                run_stepper_mode(
                    &mut x_vel,
                    &mut y_vel,
                    delta_t_ms,
                    &mut x_step_timer,
                    &mut y_step_timer,
                    x_stepper,
                    y_stepper,
                );
            }
            sleep().await;
            if !operational_flag.load(Ordering::Relaxed) {
                x_stepper.disable();
                y_stepper.disable();
                motion_state = MotionState::Idle;
                set_axis_velocity(0, 0);
                set_axis_velocity(1, 0);
                break;
            }
        }
    }
}

#[pac::interrupt]
fn TIM2() {
    // Clear all interrupt flags
    pac::TIM2.sr().write(|w| w.0 = 0);

    unsafe {
        (*X_STEPPER.get()).step();
    }
}

#[pac::interrupt]
fn TIM5() {
    // Clear all interrupt flags
    pac::TIM5.sr().write(|w| w.0 = 0);

    unsafe {
        (*Y_STEPPER.get()).step();
    }
}
