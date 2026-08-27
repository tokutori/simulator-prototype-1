//! Stepwise aircraft plant used by external controller and virtual-MCU adapters.

use crate::config::{ConfigError, LoadedSimulation, deg_to_rad};
use flight_dynamics_core::{
    Actuator, ActuatorError, ControlSurfaceDeflection, Quaternion, RigidBodyState, SensorError,
    SensorSample, SensorSuite, StepError, Vec3, aerodynamic_loads, step_rk4,
};
use serde::Serialize;

/// A serializable plant observation expressed in SI units.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct PlantObservation {
    /// Simulation time.
    pub time_s: f64,
    /// North displacement from release.
    pub north_m: f64,
    /// East displacement from release.
    pub east_m: f64,
    /// Altitude above the water/reference surface.
    pub altitude_m: f64,
    /// True roll attitude.
    pub roll_rad: f64,
    /// True pitch attitude.
    pub pitch_rad: f64,
    /// True yaw/heading attitude.
    pub yaw_rad: f64,
    /// True flight-path angle.
    pub flight_path_rad: f64,
    /// Actual elevator deflection after servo dynamics.
    pub elevator_rad: f64,
    /// Actual rudder deflection after servo dynamics.
    pub rudder_rad: f64,
    /// Virtual BNO055 roll output.
    pub sensor_roll_rad: f64,
    /// Virtual BNO055 pitch output.
    pub sensor_pitch_rad: f64,
    /// Virtual BNO055 yaw/heading output.
    pub sensor_yaw_rad: f64,
    /// Virtual BNO055 body roll-rate output.
    pub sensor_roll_rate_rad_s: f64,
    /// Virtual BNO055 body pitch-rate output.
    pub sensor_pitch_rate_rad_s: f64,
    /// Virtual BNO055 body yaw-rate output.
    pub sensor_yaw_rate_rad_s: f64,
    /// Virtual SDP810-derived airspeed.
    pub sensor_airspeed_mps: f64,
    /// Virtual SDP810 differential pressure.
    pub sensor_differential_pressure_pa: f64,
    /// Virtual pressure-altimeter output.
    pub sensor_barometric_altitude_m: f64,
    /// Virtual angle-vane output.
    pub sensor_alpha_rad: f64,
    /// Whether the aerodynamic table covers this observation.
    pub aero_in_range: bool,
    /// Whether the aircraft has contacted the reference surface.
    pub surface_contact: bool,
}

/// Owns plant, actuator, and sensor state while an external controller closes the loop.
pub struct PlantSession {
    loaded: LoadedSimulation,
    state: RigidBodyState,
    controls: ControlSurfaceDeflection,
    elevator: Actuator,
    rudder: Actuator,
    sensors: SensorSuite,
    time_s: f64,
}

impl PlantSession {
    /// Loads a model and constructs the release state.
    ///
    /// # Errors
    ///
    /// Returns an error if the model, actuator, or sensor configuration is invalid.
    pub fn load(path: &std::path::Path) -> Result<Self, PlantSessionError> {
        let loaded = LoadedSimulation::load(path).map_err(PlantSessionError::Config)?;
        Self::new(loaded)
    }

    /// Constructs a session from a validated simulation model.
    ///
    /// # Errors
    ///
    /// Returns an error if actuator or sensor initialization fails.
    pub fn new(loaded: LoadedSimulation) -> Result<Self, PlantSessionError> {
        let initial = &loaded.file.initial_state;
        let alpha = deg_to_rad(initial.alpha_deg);
        let state = RigidBodyState {
            position_ned_m: Vec3::new(0.0, 0.0, -initial.altitude_m),
            velocity_body_mps: Vec3::new(
                initial.airspeed_mps * alpha.cos(),
                0.0,
                initial.airspeed_mps * alpha.sin(),
            ),
            attitude_body_to_ned: Quaternion::from_euler(
                deg_to_rad(initial.roll_deg),
                deg_to_rad(initial.pitch_deg),
                deg_to_rad(initial.heading_deg),
            ),
            rates_body_rad_s: Vec3::ZERO,
        };
        let launch_elevator_rad = deg_to_rad(
            loaded
                .file
                .reference_controller
                .launch_elevator_feedforward_deg,
        );
        let elevator = Actuator::new(launch_elevator_rad, loaded.elevator_config())
            .map_err(PlantSessionError::Actuator)?;
        let rudder =
            Actuator::new(0.0, loaded.rudder_config()).map_err(PlantSessionError::Actuator)?;
        let sensors = SensorSuite::new(loaded.sensor_model()).map_err(PlantSessionError::Sensor)?;
        Ok(Self {
            loaded,
            state,
            controls: ControlSurfaceDeflection {
                elevator_rad: launch_elevator_rad,
                ..ControlSurfaceDeflection::default()
            },
            elevator,
            rudder,
            sensors,
            time_s: 0.0,
        })
    }

    /// Samples virtual sensors without advancing the rigid body.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid time step, model, or sensor state.
    pub fn observe(&mut self, dt_s: f64) -> Result<PlantObservation, PlantSessionError> {
        validate_dt(dt_s)?;
        let model = self.loaded.model().map_err(PlantSessionError::Config)?;
        let environment = self
            .loaded
            .environment_at_north(self.state.position_ned_m.x);
        let loads = aerodynamic_loads(model, self.state, self.controls, environment);
        let sample = self
            .sensors
            .step(self.state, loads, environment, model.mass_kg, dt_s)
            .map_err(PlantSessionError::Sensor)?;
        Ok(observation(
            self.time_s,
            self.state,
            self.controls,
            sample,
            model.longitudinal.contains(loads.condition.alpha_rad),
        ))
    }

    /// Applies commands through servo dynamics and advances the nonlinear FDM once.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid commands, time step, or dynamic state.
    pub fn step(
        &mut self,
        elevator_command_rad: f64,
        rudder_command_rad: f64,
        dt_s: f64,
    ) -> Result<PlantObservation, PlantSessionError> {
        validate_dt(dt_s)?;
        self.controls.elevator_rad = self
            .elevator
            .step(elevator_command_rad, self.loaded.elevator_config(), dt_s)
            .map_err(PlantSessionError::Actuator)?;
        self.controls.rudder_rad = self
            .rudder
            .step(rudder_command_rad, self.loaded.rudder_config(), dt_s)
            .map_err(PlantSessionError::Actuator)?;
        let model = self.loaded.model().map_err(PlantSessionError::Config)?;
        let environment = self
            .loaded
            .environment_at_north(self.state.position_ned_m.x);
        self.state = step_rk4(model, self.state, self.controls, environment, dt_s)
            .map_err(PlantSessionError::Step)?;
        self.time_s += dt_s;
        self.observe(dt_s)
    }
}

fn observation(
    time_s: f64,
    state: RigidBodyState,
    controls: ControlSurfaceDeflection,
    sample: SensorSample,
    aero_in_range: bool,
) -> PlantObservation {
    let euler = state.attitude_body_to_ned.to_euler();
    PlantObservation {
        time_s,
        north_m: state.position_ned_m.x,
        east_m: state.position_ned_m.y,
        altitude_m: -state.position_ned_m.z,
        roll_rad: euler.x,
        pitch_rad: euler.y,
        yaw_rad: euler.z,
        flight_path_rad: flight_path_angle_rad(state),
        elevator_rad: controls.elevator_rad,
        rudder_rad: controls.rudder_rad,
        sensor_roll_rad: sample.roll_rad,
        sensor_pitch_rad: sample.pitch_rad,
        sensor_yaw_rad: sample.yaw_rad,
        sensor_roll_rate_rad_s: sample.gyro_rad_s.x,
        sensor_pitch_rate_rad_s: sample.gyro_rad_s.y,
        sensor_yaw_rate_rad_s: sample.gyro_rad_s.z,
        sensor_airspeed_mps: sample.airspeed_mps,
        sensor_differential_pressure_pa: sample.differential_pressure_pa,
        sensor_barometric_altitude_m: sample.barometric_altitude_m,
        sensor_alpha_rad: sample.alpha_rad,
        aero_in_range,
        surface_contact: state.position_ned_m.z >= 0.0,
    }
}

fn flight_path_angle_rad(state: RigidBodyState) -> f64 {
    let velocity_ned = state
        .attitude_body_to_ned
        .rotate_body_to_ned(state.velocity_body_mps);
    (-velocity_ned.z)
        .atan2((velocity_ned.x * velocity_ned.x + velocity_ned.y * velocity_ned.y).sqrt())
}

fn validate_dt(dt_s: f64) -> Result<(), PlantSessionError> {
    if dt_s.is_finite() && dt_s > 0.0 {
        Ok(())
    } else {
        Err(PlantSessionError::InvalidTimeStep)
    }
}

/// Failures that stop a stepwise plant session.
#[derive(Debug)]
pub enum PlantSessionError {
    /// Model loading or conversion failed.
    Config(ConfigError),
    /// Servo model rejected a value.
    Actuator(ActuatorError),
    /// Sensor model rejected a value.
    Sensor(SensorError),
    /// Integrator or rigid-body model rejected a value.
    Step(StepError),
    /// Time step was non-finite or non-positive.
    InvalidTimeStep,
}

impl core::fmt::Display for PlantSessionError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Config(error) => write!(formatter, "{error}"),
            Self::Actuator(error) => write!(formatter, "actuator error: {error:?}"),
            Self::Sensor(error) => write!(formatter, "sensor error: {error:?}"),
            Self::Step(error) => write!(formatter, "flight-dynamics step error: {error:?}"),
            Self::InvalidTimeStep => write!(formatter, "time step must be finite and positive"),
        }
    }
}

impl std::error::Error for PlantSessionError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn external_controller_can_advance_the_same_plant_core() {
        let model = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let mut session = PlantSession::load(&model).expect("model must load");
        let initial = session.observe(0.01).expect("initial observation");
        let next = session.step(0.0, 0.0, 0.01).expect("plant step");

        assert_eq!(initial.time_s, 0.0);
        assert!((next.time_s - 0.01).abs() < 1.0e-12);
        assert!(next.north_m > initial.north_m);
        assert_eq!(initial.east_m, 0.0);
        assert!(next.altitude_m < initial.altitude_m);
    }
}
