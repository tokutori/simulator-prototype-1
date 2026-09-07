#![no_std]
#![no_main]

mod sensors;
mod telemetry;

use core::f32::consts::PI;
use cortex_m_rt::entry;
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c,
    pwm::SetDutyCycle,
};
use fbw_control_core::{ControllerInput, ControllerState};
use fbw_input_core::{ButtonPair, RawPilotInput, blend, decode};
use fbw_safety_core::{SafetyConfig, SafetyMode, SurfaceCommands, SurfaceSafetyState};
use fugit::RateExtU32;
use panic_halt as _;
use qx18_fbw_config::{QX18_FAILSAFE_ELEVATOR_RAD, QX18_TRAINING_CONTROLLER};
use rp2040_hal::{self as hal, Clock};
use sensors::{MeasurementFrame, SensorState};

#[unsafe(link_section = ".boot2")]
#[used]
#[unsafe(no_mangle)]
static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const CONTROL_PERIOD_US: u32 = 10_000;
const SERVO_PWM_TOP: u16 = 19_999;
const SERVO_CENTER_US: f32 = 1_500.0;
const SERVO_US_PER_RAD: f32 = 500.0 / (10.0 * PI / 180.0);
const SURFACE_LIMIT_RAD: f32 = 10.0 * PI / 180.0;
const SENSOR_REINITIALIZE_AFTER_FAILURES: u8 = 4;
const SENSOR_RETRY_SAMPLES: u16 = 10;
const SENSOR_SETTLE_SAMPLES: u16 = 1;
const SAFETY_CONFIG: SafetyConfig = SafetyConfig {
    startup_valid_samples: 10,
    hold_invalid_samples: 2,
    recovery_valid_samples: 20,
    failsafe_command_rad: QX18_FAILSAFE_ELEVATOR_RAD,
};

enum SensorRuntime {
    Active {
        state: SensorState,
        consecutive_failures: u8,
    },
    Settling {
        state: SensorState,
        samples_remaining: u16,
    },
    RetryAfter {
        retained_launch_pressure_pa: Option<f32>,
        samples_remaining: u16,
    },
}

enum SensorPoll {
    Measurement(MeasurementFrame),
    Unavailable,
}

impl SensorRuntime {
    fn initialize<I: I2c, D: DelayNs>(i2c: &mut I, delay: &mut D) -> Self {
        match SensorState::initialize(i2c, delay, None) {
            Ok(state) => Self::Active {
                state,
                consecutive_failures: 0,
            },
            Err(_) => Self::RetryAfter {
                retained_launch_pressure_pa: None,
                samples_remaining: SENSOR_RETRY_SAMPLES,
            },
        }
    }

    fn poll<I: I2c, D: DelayNs>(&mut self, i2c: &mut I, delay: &mut D) -> SensorPoll {
        let previous = core::mem::replace(
            self,
            Self::RetryAfter {
                retained_launch_pressure_pa: None,
                samples_remaining: SENSOR_RETRY_SAMPLES,
            },
        );
        match previous {
            Self::Active {
                mut state,
                consecutive_failures,
            } => {
                let retained_launch_pressure_pa = state.launch_pressure_pa();
                match state.read(i2c) {
                    Ok(frame) => {
                        *self = Self::Active {
                            state,
                            consecutive_failures: 0,
                        };
                        SensorPoll::Measurement(frame)
                    }
                    Err(_) => {
                        let failures = consecutive_failures.saturating_add(1);
                        *self = if failures >= SENSOR_REINITIALIZE_AFTER_FAILURES {
                            Self::RetryAfter {
                                retained_launch_pressure_pa,
                                samples_remaining: SENSOR_RETRY_SAMPLES,
                            }
                        } else {
                            Self::Active {
                                state,
                                consecutive_failures: failures,
                            }
                        };
                        SensorPoll::Unavailable
                    }
                }
            }
            Self::Settling {
                state,
                samples_remaining,
            } => {
                *self = if samples_remaining > 1 {
                    Self::Settling {
                        state,
                        samples_remaining: samples_remaining - 1,
                    }
                } else {
                    Self::Active {
                        state,
                        consecutive_failures: 0,
                    }
                };
                SensorPoll::Unavailable
            }
            Self::RetryAfter {
                retained_launch_pressure_pa,
                mut samples_remaining,
            } => {
                if samples_remaining > 1 {
                    samples_remaining -= 1;
                    *self = Self::RetryAfter {
                        retained_launch_pressure_pa,
                        samples_remaining,
                    };
                    return SensorPoll::Unavailable;
                }
                match SensorState::initialize(i2c, delay, retained_launch_pressure_pa) {
                    Ok(state) => {
                        *self = Self::Settling {
                            state,
                            samples_remaining: SENSOR_SETTLE_SAMPLES,
                        };
                    }
                    Err(_) => {
                        *self = Self::RetryAfter {
                            retained_launch_pressure_pa,
                            samples_remaining: SENSOR_RETRY_SAMPLES,
                        };
                    }
                }
                SensorPoll::Unavailable
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let watchdog_reset = pac.WATCHDOG.reason().read().bits() & 0x03 != 0;
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    if watchdog_reset {
        // A reset in flight must not silently re-zero pressure and re-arm.
        // Power-cycle/manual recovery is required; no servo pins are enabled.
        watchdog.disable();
        loop {
            core::hint::spin_loop();
        }
    }
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let sda = pins.gpio4.reconfigure();
    let scl = pins.gpio5.reconfigure();
    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda,
        scl,
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    let mut adc = hal::adc::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut elevator_axis = hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let mut rudder_axis = hal::adc::AdcPin::new(pins.gpio27.into_floating_input()).unwrap();
    let mut authority_axis = hal::adc::AdcPin::new(pins.gpio28.into_floating_input()).unwrap();
    let mut elevator_negative = pins.gpio10.into_pull_up_input();
    let mut elevator_positive = pins.gpio11.into_pull_up_input();
    let mut rudder_negative = pins.gpio12.into_pull_up_input();
    let mut rudder_positive = pins.gpio13.into_pull_up_input();

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut servo_pwm = pwm_slices.pwm0;
    servo_pwm.set_div_int(125);
    servo_pwm.set_div_frac(0);
    servo_pwm.set_top(SERVO_PWM_TOP);
    let _servo_pin = servo_pwm.channel_a.output_to(pins.gpio16);
    let _rudder_servo_pin = servo_pwm.channel_b.output_to(pins.gpio17);
    servo_pwm.channel_a.set_enabled(true);
    servo_pwm.channel_b.set_enabled(true);
    servo_pwm.enable();
    // GPIO21 high means unarmed/failsafe; GPIO18 high marks an invalid live sample.
    // GPIO19 toggles once per control update; GPIO20 marks a missed 10 ms period.
    let mut safety_fault = pins.gpio21.into_push_pull_output();
    let mut sensor_invalid = pins.gpio18.into_push_pull_output();
    let mut control_tick = pins.gpio19.into_push_pull_output();
    let mut deadline_missed = pins.gpio20.into_push_pull_output();
    let _ = safety_fault.set_high();
    let _ = sensor_invalid.set_low();
    let _ = control_tick.set_low();
    let _ = deadline_missed.set_low();
    let mut control_tick_high = false;

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    // UART1 GPIO8/9 is a production flight-recorder interface, not a simulator hook.
    let recorder = hal::uart::UartPeripheral::new(
        pac.UART1,
        (
            pins.gpio8.into_function::<hal::gpio::FunctionUart>(),
            pins.gpio9.into_function::<hal::gpio::FunctionUart>(),
        ),
        &mut pac.RESETS,
    )
    .enable(
        hal::uart::UartConfig::new(
            1_000_000.Hz(),
            hal::uart::DataBits::Eight,
            None,
            hal::uart::StopBits::One,
        ),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();
    let mut record_sequence = 0_u32;
    let mut controller = ControllerState::default();
    let mut barometric_sample_sequence = 0_u32;
    let mut barometric_sample_time_us = 0_u32;
    let mut safety = SurfaceSafetyState::default();
    // BNO055 specifies 650 ms from reset to configuration mode.
    timer.delay_ms(650);
    // Covers unbounded HAL polling as well as a stalled control loop.
    watchdog.start(fugit::MicrosDurationU32::micros(40_000));
    let mut sensor_runtime = SensorRuntime::initialize(&mut i2c, &mut timer);
    // Covers BNO055 operation-mode transition and first SDP810/DPS310 samples.
    timer.delay_ms(20);
    let mut previous_update_us = timer.get_counter_low().wrapping_sub(CONTROL_PERIOD_US);

    loop {
        let update_start_us = timer.get_counter_low();
        let update_dt_s = update_start_us.wrapping_sub(previous_update_us) as f32 * 1.0e-6;
        previous_update_us = update_start_us;
        control_tick_high = !control_tick_high;
        if control_tick_high {
            let _ = control_tick.set_high();
        } else {
            let _ = control_tick.set_low();
        }
        if safety.controller_should_start_clean() {
            controller.reset_feedback();
        }
        let measurement_frame = match sensor_runtime.poll(&mut i2c, &mut timer) {
            SensorPoll::Measurement(frame) => Some(frame),
            SensorPoll::Unavailable => None,
        };
        let all_sensors_valid = measurement_frame
            .as_ref()
            .is_some_and(|frame| frame.all_sensors_valid);
        if measurement_frame
            .as_ref()
            .is_some_and(|frame| frame.pressure_sample_fresh)
        {
            barometric_sample_sequence = barometric_sample_sequence.wrapping_add(1);
            barometric_sample_time_us = timer.get_counter_low();
        }
        let automatic_elevator = measurement_frame.map(|frame| {
            let measurement = frame.measurements;
            controller
                .step(
                    &QX18_TRAINING_CONTROLLER,
                    ControllerInput {
                        pitch_rad: measurement.pitch_rad,
                        pitch_rate_rad_s: measurement.pitch_rate_rad_s,
                        airspeed_mps: measurement.airspeed_mps,
                        airspeed_valid: frame.all_sensors_valid,
                        alpha_rad: measurement.alpha_rad,
                        barometric_altitude_m: measurement.barometric_altitude_m,
                        barometric_sample_sequence,
                        barometric_sample_time_us,
                    },
                    update_dt_s,
                )
                .elevator_command_rad
        });
        let automatic_rudder = measurement_frame.map_or(0.0, |frame| {
            let measurement = frame.measurements;
            (-0.30 * measurement.roll_rad - 0.45 * measurement.roll_rate_rad_s
                + 0.35 * measurement.yaw_rate_rad_s)
                .clamp(-SURFACE_LIMIT_RAD, SURFACE_LIMIT_RAD)
        });
        let safe = safety.step(
            &SAFETY_CONFIG,
            0.0,
            automatic_elevator.map(|elevator_rad| SurfaceCommands {
                elevator_rad,
                rudder_rad: automatic_rudder,
            }),
        );
        if safe.reset_controller {
            controller.reset_feedback();
        }
        let pilot = decode(RawPilotInput {
            elevator_adc: adc.read(&mut elevator_axis).unwrap_or(2048),
            rudder_adc: adc.read(&mut rudder_axis).unwrap_or(2048),
            authority_adc: adc.read(&mut authority_axis).unwrap_or(4095),
            elevator_buttons: ButtonPair::from_pressed(
                elevator_negative.is_low().unwrap_or(false),
                elevator_positive.is_low().unwrap_or(false),
            ),
            rudder_buttons: ButtonPair::from_pressed(
                rudder_negative.is_low().unwrap_or(false),
                rudder_positive.is_low().unwrap_or(false),
            ),
        });
        let elevator_command = blend(
            pilot.elevator * SURFACE_LIMIT_RAD,
            safe.commands.elevator_rad,
            pilot.autonomy,
            SURFACE_LIMIT_RAD,
        );
        // QX-18's reconstructed Cn_delta_r is negative: a right-yaw demand uses negative rudder.
        let rudder_command = blend(
            -pilot.rudder * SURFACE_LIMIT_RAD,
            safe.commands.rudder_rad,
            pilot.autonomy,
            SURFACE_LIMIT_RAD,
        );
        let elevator_pulse_us =
            (SERVO_CENTER_US + elevator_command * SERVO_US_PER_RAD).clamp(1_000.0, 2_000.0);
        let rudder_pulse_us =
            (SERVO_CENTER_US + rudder_command * SERVO_US_PER_RAD).clamp(1_000.0, 2_000.0);
        let _ = servo_pwm.channel_a.set_duty_cycle(elevator_pulse_us as u16);
        let _ = servo_pwm.channel_b.set_duty_cycle(rudder_pulse_us as u16);
        if matches!(safe.mode, SafetyMode::Arming | SafetyMode::Failsafe) {
            let _ = safety_fault.set_high();
        } else {
            let _ = safety_fault.set_low();
        }
        if all_sensors_valid {
            let _ = sensor_invalid.set_low();
        } else {
            let _ = sensor_invalid.set_high();
        }
        recorder.write_full_blocking(
            &telemetry::ControlRecord {
                sequence: record_sequence,
                time_us: update_start_us,
                automatic_valid: automatic_elevator.is_some(),
                values: [
                    pilot.elevator,
                    pilot.rudder,
                    pilot.autonomy,
                    automatic_elevator.unwrap_or(0.0),
                    automatic_rudder,
                    safe.commands.elevator_rad,
                    elevator_command,
                    rudder_command,
                    safe.commands.rudder_rad,
                ],
            }
            .encode(),
        );
        record_sequence = record_sequence.wrapping_add(1);
        watchdog.feed();
        let elapsed_us = timer.get_counter_low().wrapping_sub(update_start_us);
        if elapsed_us < CONTROL_PERIOD_US {
            let _ = deadline_missed.set_low();
            timer.delay_us(CONTROL_PERIOD_US - elapsed_us);
        } else {
            let _ = deadline_missed.set_high();
        }
    }
}
