#![no_std]
#![forbid(unsafe_code)]
//! Compiled FBW profile corresponding to the QX-18 training JSON.

use core::f32::consts::PI;
use fbw_control_core::ControllerConfig;

/// Training-only fixed nose-down command after sustained sensor loss.
///
/// A deterministic open-loop sweep selected the smallest tested command that
/// avoided re-ascent for sensor-loss times 0.5, 1, 2, 3, 5 and 7 seconds.
pub const QX18_FAILSAFE_ELEVATOR_RAD: f32 = 0.75 * PI / 180.0;

/// Training-only controller profile compiled into the RP2040 firmware.
///
/// A host regression test compares every field with
/// `models/qx18-br-training-envelope.json` to detect configuration drift.
pub const QX18_TRAINING_CONTROLLER: ControllerConfig = ControllerConfig {
    pull_out_start_airspeed_mps: 6.0,
    pull_out_full_airspeed_mps: 7.5,
    launch_target_alpha_rad: 6.0 * PI / 180.0,
    launch_elevator_feedforward_rad: 0.0,
    launch_alpha_gain: 1.5,
    glide_target_flight_path_rad: -1.7 * PI / 180.0,
    flight_path_gain: 0.9,
    flight_path_lookahead_s: 0.1,
    degraded_flight_path_lookahead_s: 0.25,
    degraded_pull_out_start_altitude_loss_m: 0.75,
    degraded_pull_out_full_altitude_loss_m: 2.0,
    climb_limit_flight_path_rad: -1.5 * PI / 180.0,
    climb_suppression_gain: 6.0,
    ground_climb_limit_mps: -0.25,
    ground_climb_suppression_gain_rad_per_mps: 0.9,
    vertical_speed_filter_time_constant_s: 0.25,
    alpha_limit_rad: 7.5 * PI / 180.0,
    alpha_limit_gain: 3.0,
    launch_pitch_rate_gain_s: 0.05,
    glide_pitch_rate_gain_s: 0.6,
    glide_damping_enable_flight_path_rad: -3.0 * PI / 180.0,
    glide_damping_transition_time_s: 0.25,
    automatic_elevator_limit_rad: 10.0 * PI / 180.0,
    automatic_elevator_rate_limit_rad_s: 352.941_2 * PI / 180.0,
    automatic_elevator_filter_time_constant_s: 0.015,
};
