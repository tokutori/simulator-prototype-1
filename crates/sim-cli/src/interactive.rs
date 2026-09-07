//! Pilot-in-the-loop command mixing for manual, shared, and automatic comparison.

use fbw_control_core::{ControllerInput, ControllerState};
use serde::Serialize;

use crate::{config::SimulationFile, controller::controller_config, session::PlantObservation};

/// Normalized pilot demand and automatic-control authority.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PilotCommand {
    /// Positive requests nose-down pitch control.
    pub elevator: f64,
    /// Positive requests a right-yaw response.
    pub rudder: f64,
    /// Zero is manual, one is fully automatic, and intermediate values are shared control.
    pub autonomy: f64,
}

/// Surface commands before the actuator dynamics, plus comparison telemetry.
#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct MixedControl {
    /// Normalized pilot elevator demand after validation.
    pub pilot_elevator: f64,
    /// Normalized pilot rudder demand after validation.
    pub pilot_rudder: f64,
    /// Automatic-control authority in the inclusive range zero to one.
    pub autonomy: f64,
    /// Pilot-only elevator surface command.
    pub manual_elevator_command_rad: f64,
    /// Pilot-only rudder surface command in the model sign convention.
    pub manual_rudder_command_rad: f64,
    /// Longitudinal FBW elevator command.
    pub automatic_elevator_command_rad: f64,
    /// Lateral stabilization rudder command.
    pub automatic_rudder_command_rad: f64,
    /// Elevator command after authority blending and saturation.
    pub mixed_elevator_command_rad: f64,
    /// Rudder command after authority blending and saturation.
    pub mixed_rudder_command_rad: f64,
}

/// Invalid pilot or time-step input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InteractiveControlError {
    /// A normalized command was non-finite or outside its documented range.
    InvalidCommand,
    /// The control interval was non-finite or non-positive.
    InvalidTimeStep,
}

impl core::fmt::Display for InteractiveControlError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidCommand => {
                formatter.write_str("pilot command must be finite and within its normalized range")
            }
            Self::InvalidTimeStep => {
                formatter.write_str("control time step must be finite and positive")
            }
        }
    }
}

impl std::error::Error for InteractiveControlError {}

/// Stateful controller that reuses the production longitudinal core.
pub struct InteractiveController {
    longitudinal: ControllerState,
    longitudinal_config: fbw_control_core::ControllerConfig,
    elevator_limit_rad: f64,
    rudder_limit_rad: f64,
    rudder_for_right_yaw_sign: f64,
    rudder_for_positive_roll_sign: f64,
    roll_level_gain: f64,
    roll_rate_gain_s: f64,
    yaw_rate_gain_s: f64,
}

impl InteractiveController {
    /// Derives actuator limits and lateral signs from a validated aircraft file.
    #[must_use]
    pub fn from_model(file: &SimulationFile) -> Self {
        let derivatives = &file.aerodynamics.derivatives_per_rad;
        Self {
            longitudinal: ControllerState::default(),
            longitudinal_config: controller_config(&file.reference_controller),
            elevator_limit_rad: file.actuators.elevator.max_abs_deg.to_radians(),
            rudder_limit_rad: file.actuators.rudder.max_abs_deg.to_radians(),
            // The pilot command is response-oriented, while the FDM surface sign is model data.
            rudder_for_right_yaw_sign: nonzero_sign(derivatives.cn_rudder),
            rudder_for_positive_roll_sign: nonzero_sign(derivatives.c_roll_rudder),
            // Training-only starting values. Flight-log identification is still required.
            roll_level_gain: 0.30,
            roll_rate_gain_s: 0.45,
            yaw_rate_gain_s: 0.35,
        }
    }

    /// Mixes pilot and automatic commands using an explicit authority blend.
    ///
    /// # Errors
    ///
    /// Rejects non-finite/out-of-range pilot values and invalid time steps.
    pub fn step(
        &mut self,
        observation: PlantObservation,
        pilot: PilotCommand,
        dt_s: f64,
    ) -> Result<MixedControl, InteractiveControlError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(InteractiveControlError::InvalidTimeStep);
        }
        if [pilot.elevator, pilot.rudder, pilot.autonomy]
            .iter()
            .any(|value| !value.is_finite())
            || !(-1.0..=1.0).contains(&pilot.elevator)
            || !(-1.0..=1.0).contains(&pilot.rudder)
            || !(0.0..=1.0).contains(&pilot.autonomy)
        {
            return Err(InteractiveControlError::InvalidCommand);
        }

        #[allow(clippy::cast_possible_truncation)] // Deliberately exercise MCU-width arithmetic.
        let automatic_elevator = f64::from(
            self.longitudinal
                .step(
                    &self.longitudinal_config,
                    ControllerInput {
                        pitch_rad: observation.sensor_pitch_rad as f32,
                        pitch_rate_rad_s: observation.sensor_pitch_rate_rad_s as f32,
                        airspeed_mps: observation.sensor_airspeed_mps as f32,
                        airspeed_valid: true,
                        alpha_rad: observation.sensor_alpha_rad as f32,
                        barometric_altitude_m: observation.sensor_barometric_altitude_m as f32,
                        barometric_sample_sequence: observation.sensor_barometric_sample_sequence,
                    },
                    dt_s as f32,
                )
                .elevator_command_rad,
        )
        .clamp(-self.elevator_limit_rad, self.elevator_limit_rad);

        let roll_component = -self.rudder_for_positive_roll_sign
            * (self.roll_level_gain * observation.sensor_roll_rad
                + self.roll_rate_gain_s * observation.sensor_roll_rate_rad_s);
        let yaw_component = -self.rudder_for_right_yaw_sign
            * self.yaw_rate_gain_s
            * observation.sensor_yaw_rate_rad_s;
        let automatic_rudder =
            (roll_component + yaw_component).clamp(-self.rudder_limit_rad, self.rudder_limit_rad);

        let manual_elevator = pilot.elevator * self.elevator_limit_rad;
        let manual_rudder = pilot.rudder * self.rudder_limit_rad * self.rudder_for_right_yaw_sign;
        let mixed_elevator = blend(manual_elevator, automatic_elevator, pilot.autonomy)
            .clamp(-self.elevator_limit_rad, self.elevator_limit_rad);
        let mixed_rudder = blend(manual_rudder, automatic_rudder, pilot.autonomy)
            .clamp(-self.rudder_limit_rad, self.rudder_limit_rad);

        Ok(MixedControl {
            pilot_elevator: pilot.elevator,
            pilot_rudder: pilot.rudder,
            autonomy: pilot.autonomy,
            manual_elevator_command_rad: manual_elevator,
            manual_rudder_command_rad: manual_rudder,
            automatic_elevator_command_rad: automatic_elevator,
            automatic_rudder_command_rad: automatic_rudder,
            mixed_elevator_command_rad: mixed_elevator,
            mixed_rudder_command_rad: mixed_rudder,
        })
    }
}

const fn blend(manual: f64, automatic: f64, autonomy: f64) -> f64 {
    manual * (1.0 - autonomy) + automatic * autonomy
}

const fn nonzero_sign(value: f64) -> f64 {
    if value < 0.0 { -1.0 } else { 1.0 }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use crate::{config::LoadedSimulation, session::PlantSession};

    fn setup() -> (InteractiveController, PlantObservation) {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let loaded = LoadedSimulation::load(&path).expect("model must load");
        let controller = InteractiveController::from_model(&loaded.file);
        let observation = PlantSession::new(loaded)
            .expect("plant must initialize")
            .observe(0.01)
            .expect("observation");
        (controller, observation)
    }

    #[test]
    fn manual_mode_uses_only_pilot_commands() {
        let (mut controller, observation) = setup();
        let output = controller
            .step(
                observation,
                PilotCommand {
                    elevator: 0.5,
                    rudder: 0.25,
                    autonomy: 0.0,
                },
                0.01,
            )
            .expect("valid command");
        assert_eq!(
            output.mixed_elevator_command_rad,
            output.manual_elevator_command_rad
        );
        assert_eq!(
            output.mixed_rudder_command_rad,
            output.manual_rudder_command_rad
        );
    }

    #[test]
    fn automatic_mode_is_independent_of_pilot_demand() {
        let (mut controller, observation) = setup();
        let output = controller
            .step(
                observation,
                PilotCommand {
                    elevator: -1.0,
                    rudder: 1.0,
                    autonomy: 1.0,
                },
                0.01,
            )
            .expect("valid command");
        assert_eq!(
            output.mixed_elevator_command_rad,
            output.automatic_elevator_command_rad
        );
        assert_eq!(
            output.mixed_rudder_command_rad,
            output.automatic_rudder_command_rad
        );
    }

    #[test]
    fn half_authority_is_the_arithmetic_midpoint() {
        let (mut controller, observation) = setup();
        let output = controller
            .step(
                observation,
                PilotCommand {
                    elevator: 0.6,
                    rudder: -0.4,
                    autonomy: 0.5,
                },
                0.01,
            )
            .expect("valid command");
        assert!(
            (output.mixed_elevator_command_rad
                - 0.5
                    * (output.manual_elevator_command_rad + output.automatic_elevator_command_rad))
                .abs()
                < 1.0e-12
        );
        assert!(
            (output.mixed_rudder_command_rad
                - 0.5 * (output.manual_rudder_command_rad + output.automatic_rudder_command_rad))
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn out_of_range_input_is_rejected() {
        let (mut controller, observation) = setup();
        assert_eq!(
            controller.step(
                observation,
                PilotCommand {
                    elevator: 1.01,
                    rudder: 0.0,
                    autonomy: 0.0,
                },
                0.01,
            ),
            Err(InteractiveControlError::InvalidCommand)
        );
    }

    #[test]
    fn lateral_automatic_command_opposes_roll_and_yaw_rate_for_qx18_signs() {
        let (mut roll_controller, mut roll_observation) = setup();
        roll_observation.sensor_roll_rad = 0.1;
        let roll_output = roll_controller
            .step(
                roll_observation,
                PilotCommand {
                    elevator: 0.0,
                    rudder: 0.0,
                    autonomy: 1.0,
                },
                0.01,
            )
            .expect("valid command");
        assert!(roll_output.automatic_rudder_command_rad < 0.0);

        let (mut yaw_controller, mut yaw_observation) = setup();
        yaw_observation.sensor_yaw_rate_rad_s = 0.1;
        let yaw_output = yaw_controller
            .step(
                yaw_observation,
                PilotCommand {
                    elevator: 0.0,
                    rudder: 0.0,
                    autonomy: 1.0,
                },
                0.01,
            )
            .expect("valid command");
        assert!(yaw_output.automatic_rudder_command_rad > 0.0);
    }
}
