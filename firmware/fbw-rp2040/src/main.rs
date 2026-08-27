#![no_std]
#![no_main]

mod sensors;

use core::f32::consts::PI;
use cortex_m_rt::entry;
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}, pwm::SetDutyCycle};
use fbw_control_core::{ControllerInput, ControllerState};
use fbw_input_core::{RawPilotInput, blend, decode};
use fbw_safety_core::{SafetyConfig, SafetyMode, SafetyState};
use fugit::RateExtU32;
use panic_halt as _;
use qx18_fbw_config::{QX18_FAILSAFE_ELEVATOR_RAD, QX18_TRAINING_CONTROLLER};
use rp2040_hal::{self as hal, Clock};
use sensors::SensorState;

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
const SENSOR_RETRY_SAMPLES: u16 = 100;
const SAFETY_CONFIG: SafetyConfig = SafetyConfig {
    startup_valid_samples: 10,
    hold_invalid_samples: 2,
    recovery_valid_samples: 20,
    failsafe_command_rad: QX18_FAILSAFE_ELEVATOR_RAD,
};

#[entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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
    let mut controller = ControllerState::default();
    let mut safety = SafetyState::default();
    // BNO055 specifies 650 ms from reset to configuration mode.
    timer.delay_ms(650);
    let mut sensor_state = SensorState::initialize(&mut i2c).ok();
    let mut sensor_retry_samples = SENSOR_RETRY_SAMPLES;
    // Covers BNO055 operation-mode transition and first SDP810/DPS310 samples.
    timer.delay_ms(20);

    loop {
        let update_start_us = timer.get_counter_low();
        control_tick_high = !control_tick_high;
        if control_tick_high {
            let _ = control_tick.set_high();
        } else {
            let _ = control_tick.set_low();
        }
        if sensor_state.is_none() {
            if sensor_retry_samples == 0 {
                sensor_state = SensorState::initialize(&mut i2c).ok();
                sensor_retry_samples = SENSOR_RETRY_SAMPLES;
                if sensor_state.is_some() {
                    timer.delay_ms(20);
                }
            } else {
                sensor_retry_samples -= 1;
            }
        }
        if safety.controller_should_start_clean() {
            controller = ControllerState::default();
        }
        let measurement_frame = sensor_state
            .as_mut()
            .and_then(|state| state.read(&mut i2c).ok());
        let all_sensors_valid = measurement_frame
            .as_ref()
            .is_some_and(|frame| frame.all_sensors_valid);
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
                    },
                    CONTROL_PERIOD_US as f32 * 1.0e-6,
                )
                .elevator_command_rad
        });
        let automatic_rudder = measurement_frame.map_or(0.0, |frame| {
            let measurement = frame.measurements;
            (-0.30 * measurement.roll_rad - 0.45 * measurement.roll_rate_rad_s
                + 0.35 * measurement.yaw_rate_rad_s)
                .clamp(-SURFACE_LIMIT_RAD, SURFACE_LIMIT_RAD)
        });
        let safe = safety.step(&SAFETY_CONFIG, automatic_elevator);
        if safe.reset_controller {
            controller = ControllerState::default();
        }
        let pilot = decode(RawPilotInput {
            elevator_adc: adc.read(&mut elevator_axis).unwrap_or(2048),
            rudder_adc: adc.read(&mut rudder_axis).unwrap_or(2048),
            authority_adc: adc.read(&mut authority_axis).unwrap_or(4095),
            elevator_negative: elevator_negative.is_low().unwrap_or(false),
            elevator_positive: elevator_positive.is_low().unwrap_or(false),
            rudder_negative: rudder_negative.is_low().unwrap_or(false),
            rudder_positive: rudder_positive.is_low().unwrap_or(false),
        });
        let elevator_command = blend(
            pilot.elevator * SURFACE_LIMIT_RAD,
            safe.command_rad,
            pilot.autonomy,
            SURFACE_LIMIT_RAD,
        );
        // QX-18's reconstructed Cn_delta_r is negative: a right-yaw demand uses negative rudder.
        let rudder_command = blend(
            -pilot.rudder * SURFACE_LIMIT_RAD,
            automatic_rudder,
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
        let elapsed_us = timer.get_counter_low().wrapping_sub(update_start_us);
        if elapsed_us < CONTROL_PERIOD_US {
            let _ = deadline_missed.set_low();
            timer.delay_us(CONTROL_PERIOD_US - elapsed_us);
        } else {
            let _ = deadline_missed.set_high();
        }
    }
}
