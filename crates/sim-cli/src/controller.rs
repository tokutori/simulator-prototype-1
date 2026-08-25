//! Adapter from the simulator model file and virtual sensors to the shared FBW core.

use crate::config::ReferenceControllerFile;
pub use fbw_control_core::ControllerOutput as ControlDecision;
use fbw_control_core::{ControllerConfig, ControllerInput, ControllerState};
use flight_dynamics_core::SensorSample;

/// Host-side owner of the same persistent state used by the RP2040 firmware.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReferenceControllerState(ControllerState);

impl ReferenceControllerState {
    /// Advances the shared controller using a simulator sensor sample.
    #[allow(clippy::cast_possible_truncation)] // Deliberately exercise MCU-width arithmetic.
    pub fn step(
        &mut self,
        controller: &ReferenceControllerFile,
        sensors: SensorSample,
        dt_s: f64,
    ) -> ControlDecision {
        self.0.step(
            &controller_config(controller),
            ControllerInput {
                pitch_rad: sensors.pitch_rad as f32,
                pitch_rate_rad_s: sensors.gyro_rad_s.y as f32,
                airspeed_mps: sensors.airspeed_mps as f32,
                airspeed_valid: true,
                alpha_rad: sensors.alpha_rad as f32,
                barometric_altitude_m: sensors.barometric_altitude_m as f32,
            },
            dt_s as f32,
        )
    }
}

#[allow(clippy::cast_possible_truncation)] // Model JSON is adapted to the RP2040's f32 controller.
pub(crate) fn controller_config(file: &ReferenceControllerFile) -> ControllerConfig {
    ControllerConfig {
        pull_out_start_airspeed_mps: file.pull_out_start_airspeed_mps as f32,
        pull_out_full_airspeed_mps: file.pull_out_full_airspeed_mps as f32,
        launch_target_alpha_rad: file.launch_target_alpha_deg.to_radians() as f32,
        launch_elevator_feedforward_rad: file.launch_elevator_feedforward_deg.to_radians() as f32,
        launch_alpha_gain: file.launch_alpha_gain as f32,
        glide_target_flight_path_rad: file.glide_target_flight_path_deg.to_radians() as f32,
        flight_path_gain: file.flight_path_gain as f32,
        flight_path_lookahead_s: file.flight_path_lookahead_s as f32,
        degraded_flight_path_lookahead_s: file.degraded_flight_path_lookahead_s as f32,
        degraded_pull_out_start_altitude_loss_m: file.degraded_pull_out_start_altitude_loss_m
            as f32,
        degraded_pull_out_full_altitude_loss_m: file.degraded_pull_out_full_altitude_loss_m as f32,
        climb_limit_flight_path_rad: file.climb_limit_flight_path_deg.to_radians() as f32,
        climb_suppression_gain: file.climb_suppression_gain as f32,
        ground_climb_limit_mps: file.ground_climb_limit_mps as f32,
        ground_climb_suppression_gain_rad_per_mps: file.ground_climb_suppression_gain_rad_per_mps
            as f32,
        vertical_speed_filter_time_constant_s: file.vertical_speed_filter_time_constant_s as f32,
        alpha_limit_rad: file.alpha_limit_deg.to_radians() as f32,
        alpha_limit_gain: file.alpha_limit_gain as f32,
        launch_pitch_rate_gain_s: file.launch_pitch_rate_gain_s as f32,
        glide_pitch_rate_gain_s: file.glide_pitch_rate_gain_s as f32,
        glide_damping_enable_flight_path_rad: file.glide_damping_enable_flight_path_deg.to_radians()
            as f32,
        glide_damping_transition_time_s: file.glide_damping_transition_time_s as f32,
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use qx18_fbw_config::{QX18_FAILSAFE_ELEVATOR_RAD, QX18_TRAINING_CONTROLLER};

    use super::*;
    use crate::config::LoadedSimulation;

    #[test]
    #[allow(clippy::cast_possible_truncation)]
    fn compiled_qx18_firmware_profile_matches_model_json() {
        let model = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let loaded = LoadedSimulation::load(&model).expect("QX-18 training model must load");
        let from_json = controller_config(&loaded.file.reference_controller);
        let compiled = QX18_TRAINING_CONTROLLER;
        let pairs = [
            (
                from_json.pull_out_start_airspeed_mps,
                compiled.pull_out_start_airspeed_mps,
            ),
            (
                from_json.pull_out_full_airspeed_mps,
                compiled.pull_out_full_airspeed_mps,
            ),
            (
                from_json.launch_target_alpha_rad,
                compiled.launch_target_alpha_rad,
            ),
            (
                from_json.launch_elevator_feedforward_rad,
                compiled.launch_elevator_feedforward_rad,
            ),
            (from_json.launch_alpha_gain, compiled.launch_alpha_gain),
            (
                from_json.glide_target_flight_path_rad,
                compiled.glide_target_flight_path_rad,
            ),
            (from_json.flight_path_gain, compiled.flight_path_gain),
            (
                from_json.flight_path_lookahead_s,
                compiled.flight_path_lookahead_s,
            ),
            (
                from_json.degraded_flight_path_lookahead_s,
                compiled.degraded_flight_path_lookahead_s,
            ),
            (
                from_json.degraded_pull_out_start_altitude_loss_m,
                compiled.degraded_pull_out_start_altitude_loss_m,
            ),
            (
                from_json.degraded_pull_out_full_altitude_loss_m,
                compiled.degraded_pull_out_full_altitude_loss_m,
            ),
            (
                from_json.climb_limit_flight_path_rad,
                compiled.climb_limit_flight_path_rad,
            ),
            (
                from_json.climb_suppression_gain,
                compiled.climb_suppression_gain,
            ),
            (
                from_json.ground_climb_limit_mps,
                compiled.ground_climb_limit_mps,
            ),
            (
                from_json.ground_climb_suppression_gain_rad_per_mps,
                compiled.ground_climb_suppression_gain_rad_per_mps,
            ),
            (
                from_json.vertical_speed_filter_time_constant_s,
                compiled.vertical_speed_filter_time_constant_s,
            ),
            (from_json.alpha_limit_rad, compiled.alpha_limit_rad),
            (from_json.alpha_limit_gain, compiled.alpha_limit_gain),
            (
                from_json.launch_pitch_rate_gain_s,
                compiled.launch_pitch_rate_gain_s,
            ),
            (
                from_json.glide_pitch_rate_gain_s,
                compiled.glide_pitch_rate_gain_s,
            ),
            (
                from_json.glide_damping_enable_flight_path_rad,
                compiled.glide_damping_enable_flight_path_rad,
            ),
            (
                from_json.glide_damping_transition_time_s,
                compiled.glide_damping_transition_time_s,
            ),
        ];
        assert!(
            pairs
                .iter()
                .all(|(left, right)| (left - right).abs() < 1.0e-6)
        );
        assert!(
            (loaded
                .file
                .reference_controller
                .failsafe_elevator_deg
                .to_radians() as f32
                - QX18_FAILSAFE_ELEVATOR_RAD)
                .abs()
                < 1.0e-6
        );
    }
}
