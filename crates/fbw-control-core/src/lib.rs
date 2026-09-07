#![no_std]
#![forbid(unsafe_code)]
//! Platform-independent FBW reference controller.
//!
//! The host simulator and the RP2040 firmware use this exact state machine.
//! All angles are radians and the positive elevator convention is nose-down.

/// Calibrated controller parameters in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerConfig {
    /// Airspeed where the launch-to-glide transition starts.
    pub pull_out_start_airspeed_mps: f32,
    /// Airspeed where the transition is complete.
    pub pull_out_full_airspeed_mps: f32,
    /// Low-drag angle of attack held just after release.
    pub launch_target_alpha_rad: f32,
    /// Launch elevator trim.
    pub launch_elevator_feedforward_rad: f32,
    /// Launch angle-of-attack feedback gain.
    pub launch_alpha_gain: f32,
    /// Desired descending glide-path angle.
    pub glide_target_flight_path_rad: f32,
    /// Glide-path feedback gain.
    pub flight_path_gain: f32,
    /// Pitch-rate look-ahead horizon.
    pub flight_path_lookahead_s: f32,
    /// Longer pitch-rate look-ahead used while pitot airspeed is unavailable.
    pub degraded_flight_path_lookahead_s: f32,
    /// Relative altitude loss where the no-pitot pull-out backup starts.
    pub degraded_pull_out_start_altitude_loss_m: f32,
    /// Relative altitude loss where the no-pitot pull-out backup is complete.
    pub degraded_pull_out_full_altitude_loss_m: f32,
    /// Highest permitted flight-path angle, normally non-positive.
    pub climb_limit_flight_path_rad: f32,
    /// Nose-down suppression gain above the climb limit.
    pub climb_suppression_gain: f32,
    /// Highest permitted ground-referenced vertical speed.
    pub ground_climb_limit_mps: f32,
    /// Nose-down suppression gain for upward vertical speed.
    pub ground_climb_suppression_gain_rad_per_mps: f32,
    /// First-order filter time constant for barometric vertical speed.
    pub vertical_speed_filter_time_constant_s: f32,
    /// Angle of attack where protection begins.
    pub alpha_limit_rad: f32,
    /// Nose-down protection gain above the angle-of-attack limit.
    pub alpha_limit_gain: f32,
    /// Pitch-rate damping gain before the pull-out schedule starts.
    pub launch_pitch_rate_gain_s: f32,
    /// Pitch-rate damping gain after the pull-out schedule completes.
    pub glide_pitch_rate_gain_s: f32,
    /// Flight-path angle at which the recovered-glide damping phase can latch.
    pub glide_damping_enable_flight_path_rad: f32,
    /// First-order transition time from launch to glide pitch-rate damping.
    pub glide_damping_transition_time_s: f32,
    /// Absolute limit applied to the automatic elevator demand before pilot blending.
    pub automatic_elevator_limit_rad: f32,
    /// Maximum automatic elevator-command slew rate.
    pub automatic_elevator_rate_limit_rad_s: f32,
    /// First-order automatic elevator-command conditioning time constant.
    pub automatic_elevator_filter_time_constant_s: f32,
}

/// One synchronized set of measurements used by the controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerInput {
    /// Fused pitch attitude.
    pub pitch_rad: f32,
    /// Body-axis pitch rate.
    pub pitch_rate_rad_s: f32,
    /// Pitot-derived airspeed.
    pub airspeed_mps: f32,
    /// Whether airspeed is live rather than held from an earlier sample.
    pub airspeed_valid: bool,
    /// Vane-derived angle of attack.
    pub alpha_rad: f32,
    /// Barometric altitude above the launch surface.
    pub barometric_altitude_m: f32,
    /// Acquisition identity, changed only for a fresh pressure measurement (including equal values).
    pub barometric_sample_sequence: u32,
    /// Monotonic acquisition time in microseconds, held with the sample; wraps at `u32::MAX`.
    pub barometric_sample_time_us: u32,
}

/// Controller output and diagnostic estimates for telemetry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerOutput {
    /// Elevator command; positive is nose-down.
    pub elevator_command_rad: f32,
    /// Pitch minus angle of attack.
    pub estimated_flight_path_rad: f32,
    /// Filtered altitude derivative.
    pub estimated_vertical_speed_mps: f32,
    /// False until two distinct post-initial altitude samples establish a time base.
    pub vertical_speed_estimate_valid: bool,
    /// Smooth launch-to-glide blend in the inclusive range zero to one.
    pub pull_out_blend: f32,
    /// Latched pitch-rate damping blend in the inclusive range zero to one.
    pub glide_damping_blend: f32,
}

/// Persistent estimator state. It contains no platform I/O or clock access.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerState {
    launch_barometric_altitude_m: Option<f32>,
    previous_barometric_altitude_m: Option<f32>,
    previous_barometric_sample_sequence: Option<u32>,
    previous_barometric_sample_time_us: u32,
    barometric_sample_primed: bool,
    vertical_speed_estimate_valid: bool,
    filtered_vertical_speed_mps: f32,
    glide_damping_armed: bool,
    glide_damping_blend: f32,
    previous_elevator_tracking_command_rad: Option<f32>,
    previous_elevator_command_rad: Option<f32>,
}

impl ControllerState {
    /// Clears transient feedback after a sensor fault without moving the launch datum.
    /// Construct a new default state only when beginning a genuinely new flight.
    pub fn reset_feedback(&mut self) {
        *self = Self {
            launch_barometric_altitude_m: self.launch_barometric_altitude_m,
            ..Self::default()
        };
    }

    /// Advances the controller by one measurement interval.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // f32 microsecond intervals match MCU arithmetic.
    pub fn step(
        &mut self,
        config: &ControllerConfig,
        input: ControllerInput,
        dt_s: f32,
    ) -> ControllerOutput {
        let launch_barometric_altitude_m = *self
            .launch_barometric_altitude_m
            .get_or_insert(input.barometric_altitude_m);
        let clock_delta = input
            .barometric_sample_time_us
            .wrapping_sub(self.previous_barometric_sample_time_us);
        // Acquisition intervals must be shorter than half the wrapping clock range.
        // A backwards/restarted timestamp must not become a huge positive interval.
        let sample_elapsed_s = if clock_delta < (1 << 31) {
            clock_delta as f32 * 1.0e-6
        } else {
            0.0
        };
        match self.previous_barometric_altitude_m {
            None => {
                self.previous_barometric_altitude_m = Some(input.barometric_altitude_m);
                self.previous_barometric_sample_sequence = Some(input.barometric_sample_sequence);
                self.previous_barometric_sample_time_us = input.barometric_sample_time_us;
            }
            Some(previous_altitude_m)
                if self.previous_barometric_sample_sequence
                    != Some(input.barometric_sample_sequence)
                    && sample_elapsed_s > 0.0 =>
            {
                if self.barometric_sample_primed {
                    let raw_vertical_speed_mps =
                        (input.barometric_altitude_m - previous_altitude_m) / sample_elapsed_s;
                    let response_fraction = sample_elapsed_s
                        / (config.vertical_speed_filter_time_constant_s + sample_elapsed_s);
                    self.filtered_vertical_speed_mps += response_fraction
                        * (raw_vertical_speed_mps - self.filtered_vertical_speed_mps);
                    self.vertical_speed_estimate_valid = true;
                } else {
                    // The first fresh sample may follow an arbitrarily long
                    // frozen preflight interval, so it only primes the time base.
                    self.barometric_sample_primed = true;
                }
                self.previous_barometric_altitude_m = Some(input.barometric_altitude_m);
                self.previous_barometric_sample_sequence = Some(input.barometric_sample_sequence);
                self.previous_barometric_sample_time_us = input.barometric_sample_time_us;
            }
            Some(_) => {}
        }

        let estimated_flight_path_rad = input.pitch_rad - input.alpha_rad;
        if (input.airspeed_mps >= config.pull_out_full_airspeed_mps || !input.airspeed_valid)
            && estimated_flight_path_rad >= config.glide_damping_enable_flight_path_rad
        {
            self.glide_damping_armed = true;
        }
        if self.glide_damping_armed && dt_s.is_finite() && dt_s > 0.0 {
            let response_fraction = dt_s / (config.glide_damping_transition_time_s + dt_s);
            self.glide_damping_blend += response_fraction * (1.0 - self.glide_damping_blend);
        }

        let unconditioned = command_with_vertical_speed(
            config,
            input,
            self.filtered_vertical_speed_mps,
            self.vertical_speed_estimate_valid,
            self.glide_damping_blend,
            (launch_barometric_altitude_m - input.barometric_altitude_m).max(0.0),
        );
        let target = unconditioned.tracking_command_rad.clamp(
            -config.automatic_elevator_limit_rad,
            config.automatic_elevator_limit_rad,
        );
        let previous = self
            .previous_elevator_tracking_command_rad
            .unwrap_or_else(|| {
                config.launch_elevator_feedforward_rad.clamp(
                    -config.automatic_elevator_limit_rad,
                    config.automatic_elevator_limit_rad,
                )
            });
        let valid_dt = dt_s.is_finite() && dt_s > 0.0;
        let response_fraction = if valid_dt {
            dt_s / (config.automatic_elevator_filter_time_constant_s + dt_s)
        } else {
            0.0
        };
        let filtered_target = previous + response_fraction * (target - previous);
        self.previous_elevator_tracking_command_rad = Some(filtered_target);
        // Envelope protection bypasses the tracking low-pass. Delaying a
        // nose-down alpha/climb intervention made the uncertainty corner leave
        // the aerodynamic table even though the nominal trace looked smoother.
        // The combined command is still amplitude- and slew-limited below.
        let mut output = unconditioned.output;
        let target_command = (filtered_target + unconditioned.protection_command_rad).clamp(
            -config.automatic_elevator_limit_rad,
            config.automatic_elevator_limit_rad,
        );
        let previous_command = self.previous_elevator_command_rad.unwrap_or_else(|| {
            config.launch_elevator_feedforward_rad.clamp(
                -config.automatic_elevator_limit_rad,
                config.automatic_elevator_limit_rad,
            )
        });
        let maximum_delta = if valid_dt {
            config.automatic_elevator_rate_limit_rad_s * dt_s
        } else {
            0.0
        };
        let command = previous_command
            + (target_command - previous_command).clamp(-maximum_delta, maximum_delta);
        self.previous_elevator_command_rad = Some(command);
        output.elevator_command_rad = command;
        output
    }
}

struct UnconditionedControllerOutput {
    output: ControllerOutput,
    tracking_command_rad: f32,
    protection_command_rad: f32,
}

fn command_with_vertical_speed(
    config: &ControllerConfig,
    input: ControllerInput,
    estimated_vertical_speed_mps: f32,
    vertical_speed_estimate_valid: bool,
    glide_damping_blend: f32,
    altitude_loss_m: f32,
) -> UnconditionedControllerOutput {
    let estimated_flight_path_rad = input.pitch_rad - input.alpha_rad;
    let flight_path_lookahead_s = if input.airspeed_valid {
        config.flight_path_lookahead_s
    } else {
        config.degraded_flight_path_lookahead_s
    };
    let predicted_flight_path_rad =
        estimated_flight_path_rad + flight_path_lookahead_s * input.pitch_rate_rad_s;

    // Release speed is below level-flight speed. Hold the low-drag launch angle
    // instead of commanding an immediate, energy-losing pull-up.
    let launch_command = config.launch_elevator_feedforward_rad
        + config.launch_alpha_gain * (input.alpha_rad - config.launch_target_alpha_rad);

    let glide_command =
        config.flight_path_gain * (predicted_flight_path_rad - config.glide_target_flight_path_rad);
    let airspeed_pull_out_blend = smoothstep(
        config.pull_out_start_airspeed_mps,
        config.pull_out_full_airspeed_mps,
        input.airspeed_mps,
    );
    // With invalid pitot data, staying in launch-alpha control after pull-out
    // conflicts with climb suppression. Blend over the final approach to the
    // recovered-path threshold, then retain the latched damping transition.
    let degraded_path_blend = smoothstep(
        2.0 * config.glide_damping_enable_flight_path_rad,
        config.glide_damping_enable_flight_path_rad,
        estimated_flight_path_rad,
    );
    let degraded_altitude_blend = smoothstep(
        config.degraded_pull_out_start_altitude_loss_m,
        config.degraded_pull_out_full_altitude_loss_m,
        altitude_loss_m,
    );
    let pull_out_blend = if input.airspeed_valid {
        airspeed_pull_out_blend
    } else {
        airspeed_pull_out_blend
            .max(degraded_path_blend)
            .max(degraded_altitude_blend)
            .max(glide_damping_blend)
    };
    let scheduled_command =
        launch_command * (1.0 - pull_out_blend) + glide_command * pull_out_blend;

    // These envelope terms can only add a nose-down command. In particular,
    // the controller never requests a climb merely to hold altitude.
    let climb_suppression = config.climb_suppression_gain
        * (predicted_flight_path_rad - config.climb_limit_flight_path_rad).max(0.0);
    let ground_climb_suppression = if vertical_speed_estimate_valid {
        config.ground_climb_suppression_gain_rad_per_mps
            * (estimated_vertical_speed_mps - config.ground_climb_limit_mps).max(0.0)
    } else {
        0.0
    };
    let alpha_protection =
        config.alpha_limit_gain * (input.alpha_rad - config.alpha_limit_rad).max(0.0);
    let pitch_rate_gain_s = config.launch_pitch_rate_gain_s * (1.0 - glide_damping_blend)
        + config.glide_pitch_rate_gain_s * glide_damping_blend;
    let pitch_rate_damping = pitch_rate_gain_s * input.pitch_rate_rad_s;

    let tracking_command_rad = scheduled_command + pitch_rate_damping;
    let protection_command_rad = climb_suppression + ground_climb_suppression + alpha_protection;
    UnconditionedControllerOutput {
        output: ControllerOutput {
            elevator_command_rad: tracking_command_rad + protection_command_rad,
            estimated_flight_path_rad,
            estimated_vertical_speed_mps,
            vertical_speed_estimate_valid,
            pull_out_blend,
            glide_damping_blend,
        },
        tracking_command_rad,
        protection_command_rad,
    }
}

fn smoothstep(start: f32, end: f32, value: f32) -> f32 {
    let normalized = ((value - start) / (end - start)).clamp(0.0, 1.0);
    normalized * normalized * (3.0 - 2.0 * normalized)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> ControllerConfig {
        ControllerConfig {
            pull_out_start_airspeed_mps: 8.0,
            pull_out_full_airspeed_mps: 9.0,
            launch_target_alpha_rad: 2.0_f32.to_radians(),
            launch_elevator_feedforward_rad: 0.0,
            launch_alpha_gain: 0.5,
            glide_target_flight_path_rad: -1.7_f32.to_radians(),
            flight_path_gain: 0.5,
            flight_path_lookahead_s: 1.5,
            degraded_flight_path_lookahead_s: 2.0,
            degraded_pull_out_start_altitude_loss_m: 0.75,
            degraded_pull_out_full_altitude_loss_m: 2.0,
            climb_limit_flight_path_rad: 0.0,
            climb_suppression_gain: 1.0,
            ground_climb_limit_mps: 0.0,
            ground_climb_suppression_gain_rad_per_mps: 0.5,
            vertical_speed_filter_time_constant_s: 0.15,
            alpha_limit_rad: 7.0_f32.to_radians(),
            alpha_limit_gain: 1.0,
            launch_pitch_rate_gain_s: 0.05,
            glide_pitch_rate_gain_s: 0.2,
            glide_damping_enable_flight_path_rad: -3.0_f32.to_radians(),
            glide_damping_transition_time_s: 0.25,
            automatic_elevator_limit_rad: 10.0_f32.to_radians(),
            automatic_elevator_rate_limit_rad_s: 60.0_f32.to_radians(),
            automatic_elevator_filter_time_constant_s: 0.05,
        }
    }

    fn input(airspeed_mps: f32, pitch_deg: f32, alpha_deg: f32) -> ControllerInput {
        ControllerInput {
            pitch_rad: pitch_deg.to_radians(),
            pitch_rate_rad_s: 0.0,
            airspeed_mps,
            airspeed_valid: true,
            alpha_rad: alpha_deg.to_radians(),
            barometric_altitude_m: 10.0,
            barometric_sample_sequence: 0,
            barometric_sample_time_us: 0,
        }
    }

    #[test]
    fn release_phase_holds_launch_alpha_without_pull_out() {
        let output = ControllerState::default().step(&config(), input(5.0, -1.0, 2.0), 0.01);
        assert_eq!(output.pull_out_blend, 0.0);
        assert!(output.elevator_command_rad.abs() < 1.0e-6);
    }

    #[test]
    fn positive_flight_path_adds_nose_down_command() {
        let output = ControllerState::default().step(&config(), input(9.0, 4.0, 2.0), 0.01);
        assert!(output.estimated_flight_path_rad > 0.0);
        assert!(output.elevator_command_rad > 0.0);
    }

    #[test]
    fn transition_is_continuous_and_bounded() {
        let low = ControllerState::default().step(&config(), input(8.0, -4.0, 2.0), 0.01);
        let middle = ControllerState::default().step(&config(), input(8.5, -4.0, 2.0), 0.01);
        let high = ControllerState::default().step(&config(), input(9.0, -4.0, 2.0), 0.01);
        assert_eq!(low.pull_out_blend, 0.0);
        assert!((middle.pull_out_blend - 0.5).abs() < 1.0e-6);
        assert_eq!(high.pull_out_blend, 1.0);
    }

    #[test]
    fn filtered_barometric_climb_adds_nose_down_command() {
        let mut state = ControllerState::default();
        let first = state.step(&config(), input(9.0, -1.0, 2.0), 0.01);
        let mut climbed = input(9.0, -1.0, 2.0);
        climbed.barometric_altitude_m = 10.01;
        climbed.barometric_sample_sequence = 1;
        climbed.barometric_sample_time_us = 10_000;
        let second = state.step(&config(), climbed, 0.01);
        climbed.barometric_altitude_m = 10.02;
        climbed.barometric_sample_sequence = 2;
        climbed.barometric_sample_time_us = 20_000;
        let third = state.step(&config(), climbed, 0.01);

        assert!(!first.vertical_speed_estimate_valid);
        assert!(!second.vertical_speed_estimate_valid);
        assert!(third.vertical_speed_estimate_valid);
        assert!(third.estimated_vertical_speed_mps > 0.0);
        assert!(third.elevator_command_rad > first.elevator_command_rad);
    }

    #[test]
    fn held_barometric_sample_does_not_decay_vertical_speed_estimate() {
        let mut state = ControllerState::default();
        let mut sample = input(9.0, -1.0, 2.0);
        let _ = state.step(&config(), sample, 0.01);
        sample.barometric_altitude_m = 10.01;
        sample.barometric_sample_sequence = 1;
        sample.barometric_sample_time_us = 30_000;
        let _ = state.step(&config(), sample, 0.03);
        sample.barometric_altitude_m = 10.02;
        sample.barometric_sample_sequence = 2;
        sample.barometric_sample_time_us = 60_000;
        let updated = state.step(&config(), sample, 0.03);
        let held = state.step(&config(), sample, 0.01);

        assert!(updated.vertical_speed_estimate_valid);
        assert_eq!(
            held.estimated_vertical_speed_mps,
            updated.estimated_vertical_speed_mps
        );
    }

    #[test]
    fn fresh_equal_altitudes_decay_climb_but_held_samples_do_not() {
        let mut state = ControllerState::default();
        let mut sample = input(9.0, -1.0, 2.0);
        let _ = state.step(&config(), sample, 0.01);
        sample.barometric_sample_sequence = 1;
        sample.barometric_sample_time_us = 30_000;
        let _ = state.step(&config(), sample, 0.03);
        sample.barometric_sample_sequence = 2;
        sample.barometric_sample_time_us = 60_000;
        sample.barometric_altitude_m = 10.03;
        let climbed = state.step(&config(), sample, 0.03);
        let held = state.step(&config(), sample, 0.01);
        assert_eq!(
            climbed.estimated_vertical_speed_mps,
            held.estimated_vertical_speed_mps
        );
        sample.barometric_sample_sequence = 3;
        sample.barometric_sample_time_us = 90_000;
        let level = state.step(&config(), sample, 0.02);
        assert!(level.vertical_speed_estimate_valid);
        assert!(level.estimated_vertical_speed_mps < held.estimated_vertical_speed_mps);
        for sequence in 4..100 {
            sample.barometric_sample_sequence = sequence;
            sample.barometric_sample_time_us = sequence * 30_000;
            let _ = state.step(&config(), sample, 0.03);
        }
        assert!(state.filtered_vertical_speed_mps.abs() < 1.0e-6);
    }

    #[test]
    fn sequence_wrap_is_a_fresh_measurement() {
        let mut state = ControllerState::default();
        let mut sample = input(9.0, -1.0, 2.0);
        sample.barometric_sample_sequence = u32::MAX - 1;
        sample.barometric_sample_time_us = u32::MAX - 30_000;
        let _ = state.step(&config(), sample, 0.03);
        sample.barometric_sample_sequence = u32::MAX;
        sample.barometric_sample_time_us = u32::MAX;
        let _ = state.step(&config(), sample, 0.03);
        sample.barometric_sample_sequence = 0;
        sample.barometric_sample_time_us = 29_999;
        assert!(
            state
                .step(&config(), sample, 0.03)
                .vertical_speed_estimate_valid
        );
    }

    #[test]
    fn altitude_derivative_uses_acquisition_time_not_controller_calls() {
        let mut state = ControllerState::default();
        let mut sample = input(9.0, -1.0, 2.0);
        let _ = state.step(&config(), sample, 0.01);
        sample.barometric_sample_sequence = 1;
        sample.barometric_sample_time_us = 30_000;
        let _ = state.step(&config(), sample, 0.01);
        // No calls during 120 ms of missing measurements. A fresh sample
        // represents a 1 m/s climb, not 12 m/s at the nominal 10 ms control dt.
        sample.barometric_sample_sequence = 2;
        sample.barometric_sample_time_us = 150_000;
        sample.barometric_altitude_m = 10.12;
        let mut delayed_loop = state;
        let recovered = state.step(&config(), sample, 0.01);
        let delayed = delayed_loop.step(&config(), sample, 0.12);
        assert!((recovered.estimated_vertical_speed_mps - (0.12 / 0.27)).abs() < 1.0e-5);
        assert_eq!(
            recovered.estimated_vertical_speed_mps,
            delayed.estimated_vertical_speed_mps
        );
    }

    #[test]
    fn recovery_preserves_launch_altitude_datum() {
        let mut state = ControllerState::default();
        let mut sample = input(5.0, -8.0, 2.0);
        sample.airspeed_valid = false;
        let _ = state.step(&config(), sample, 0.01);
        state.reset_feedback();
        sample.barometric_altitude_m = 8.0;
        assert_eq!(state.step(&config(), sample, 0.01).pull_out_blend, 1.0);
    }

    #[test]
    fn pitch_rate_damping_is_scheduled_from_launch_to_glide() {
        let mut controller = config();
        controller.automatic_elevator_rate_limit_rad_s = 10_000.0;
        controller.automatic_elevator_filter_time_constant_s = 1.0e-6;
        controller.flight_path_gain = 0.0;
        let mut launch = input(5.0, -1.0, 2.0);
        launch.pitch_rate_rad_s = 0.01;
        let mut glide = launch;
        glide.airspeed_mps = 9.0;
        let launch_output = ControllerState::default().step(&controller, launch, 0.01);
        let glide_output = ControllerState::default().step(&controller, glide, 0.01);

        assert_eq!(launch_output.pull_out_blend, 0.0);
        assert_eq!(glide_output.pull_out_blend, 1.0);
        assert_eq!(launch_output.glide_damping_blend, 0.0);
        assert!(glide_output.glide_damping_blend > 0.0);
        assert!(glide_output.elevator_command_rad > launch_output.elevator_command_rad);
    }

    #[test]
    fn invalid_airspeed_blends_on_the_approach_to_recovered_path() {
        let mut state = ControllerState::default();
        let mut degraded = input(5.0, -2.5, 2.0);
        degraded.airspeed_valid = false;
        let first = state.step(&config(), degraded, 0.01);
        degraded.pitch_rad = 0.0;
        let recovered = state.step(&config(), degraded, 0.01);

        assert_eq!(first.estimated_flight_path_rad, -4.5_f32.to_radians());
        assert!((first.pull_out_blend - 0.5).abs() < 1.0e-5);
        assert_eq!(recovered.estimated_flight_path_rad, -2.0_f32.to_radians());
        assert_eq!(recovered.pull_out_blend, 1.0);
    }

    #[test]
    fn invalid_airspeed_uses_longer_pitch_rate_prediction() {
        let mut controller = config();
        controller.automatic_elevator_rate_limit_rad_s = 10_000.0;
        controller.automatic_elevator_filter_time_constant_s = 1.0e-6;
        let mut rising = input(5.0, -4.0, 2.0);
        rising.pitch_rate_rad_s = 0.1;
        let valid = ControllerState::default().step(&controller, rising, 0.01);
        rising.airspeed_valid = false;
        let degraded = ControllerState::default().step(&controller, rising, 0.01);

        assert!(degraded.elevator_command_rad > valid.elevator_command_rad);
    }

    #[test]
    fn invalid_airspeed_can_complete_pull_out_from_relative_altitude_loss() {
        let mut state = ControllerState::default();
        let mut degraded = input(5.0, -8.0, 2.0);
        degraded.airspeed_valid = false;
        let launch = state.step(&config(), degraded, 0.01);
        degraded.barometric_altitude_m = 8.0;
        let after_two_metres = state.step(&config(), degraded, 0.01);

        assert_eq!(launch.pull_out_blend, 0.0);
        assert_eq!(after_two_metres.pull_out_blend, 1.0);
    }

    #[test]
    fn automatic_elevator_command_is_amplitude_and_slew_limited() {
        let mut controller = config();
        controller.climb_suppression_gain = 0.0;
        controller.ground_climb_suppression_gain_rad_per_mps = 0.0;
        controller.alpha_limit_gain = 0.0;
        let mut state = ControllerState::default();
        let mut steep = input(9.0, 25.0, 0.0);
        steep.pitch_rate_rad_s = 1.0;

        let first = state.step(&controller, steep, 0.01);
        let second = state.step(&controller, steep, 0.01);

        assert!((first.elevator_command_rad - 0.6_f32.to_radians()).abs() < 1.0e-6);
        assert!((second.elevator_command_rad - 1.2_f32.to_radians()).abs() < 1.0e-6);
        for _ in 0..30 {
            let output = state.step(&controller, steep, 0.01);
            assert!(output.elevator_command_rad <= 10.0_f32.to_radians());
        }
    }
}
