use std::{fs, path::Path};

use flight_dynamics_core::{
    ActuatorConfig, AeroDerivatives, AeroPoint, AeroTable, AircraftModel, Environment,
    GroundEffectModel, Inertia, ModelError, SensorModel, Vec3,
};
use serde::Deserialize;

const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulationFile {
    #[serde(rename = "$schema")]
    pub _schema: Option<String>,
    pub schema_version: String,
    pub metadata: MetadataFile,
    pub mass_properties: MassPropertiesFile,
    pub reference_geometry: ReferenceGeometryFile,
    pub aerodynamics: AerodynamicsFile,
    pub actuators: ActuatorsFile,
    pub sensors: SensorsFile,
    pub environment: EnvironmentFile,
    pub initial_state: InitialStateFile,
    pub reference_controller: ReferenceControllerFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetadataFile {
    pub name: String,
    pub provenance: String,
    pub validation_status: String,
    pub valid_for: String,
    pub sources: Vec<SourceFile>,
    pub assumptions: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceFile {
    pub id: String,
    pub url: String,
    pub applies_to: String,
    pub authority: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MassPropertiesFile {
    pub mass_kg: f64,
    pub inertia_body_kg_m2: InertiaFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InertiaFile {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixz: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceGeometryFile {
    pub area_m2: f64,
    pub span_m: f64,
    pub chord_m: f64,
    pub moment_reference: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AerodynamicsFile {
    pub out_of_range_policy: OutOfRangePolicy,
    pub longitudinal_table: Vec<AeroPointFile>,
    pub derivatives_per_rad: DerivativesFile,
    pub ground_effect: GroundEffectFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundEffectFile {
    pub enabled: bool,
    pub wing_height_offset_m: f64,
    pub induced_drag_factor: f64,
    pub minimum_induced_drag_ratio: f64,
    pub correlation_gain: f64,
    pub height_exponent: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum OutOfRangePolicy {
    Terminate,
    ClampAndFlag,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroPointFile {
    pub alpha_deg: f64,
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DerivativesFile {
    pub cl_elevator: f64,
    pub cm_elevator: f64,
    pub cm_pitch_rate: f64,
    pub cy_beta: f64,
    pub cy_roll_rate: f64,
    pub cy_yaw_rate: f64,
    pub cy_rudder: f64,
    pub c_roll_beta: f64,
    pub c_roll_roll_rate: f64,
    pub c_roll_yaw_rate: f64,
    pub c_roll_rudder: f64,
    pub cn_beta: f64,
    pub cn_roll_rate: f64,
    pub cn_yaw_rate: f64,
    pub cn_rudder: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorsFile {
    pub elevator: ActuatorFile,
    pub rudder: ActuatorFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActuatorFile {
    pub max_abs_deg: f64,
    pub max_rate_deg_s: f64,
    pub time_constant_s: f64,
    pub deadband_deg: f64,
    pub command_resolution_deg: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorsFile {
    pub imu: ImuFile,
    pub air_data: AirDataFile,
    pub alpha_bias_deg: f64,
    pub alpha_resolution_deg: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuFile {
    pub sample_rate_hz: f64,
    pub gyro_bias_deg_s: [f64; 3],
    pub gyro_resolution_deg_s: f64,
    pub euler_resolution_deg: f64,
    pub acceleration_resolution_mps2: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirDataFile {
    pub sample_rate_hz: f64,
    pub differential_pressure_range_pa: f64,
    pub differential_pressure_resolution_pa: f64,
    pub differential_pressure_bias_pa: f64,
    pub differential_pressure_time_constant_s: f64,
    pub pitot_coefficient: f64,
    pub static_pressure_resolution_pa: f64,
    pub static_pressure_bias_pa: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EnvironmentFile {
    pub density_kg_m3: f64,
    pub gravity_mps2: f64,
    pub wind_ned_mps: [f64; 3],
    pub one_minus_cosine_gust: OneMinusCosineGustFile,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OneMinusCosineGustFile {
    pub enabled: bool,
    pub start_north_m: f64,
    pub length_m: f64,
    pub peak_wind_ned_mps: [f64; 3],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitialStateFile {
    pub altitude_m: f64,
    pub airspeed_mps: f64,
    pub alpha_deg: f64,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub heading_deg: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceControllerFile {
    pub pull_out_start_airspeed_mps: f64,
    pub pull_out_full_airspeed_mps: f64,
    pub launch_target_alpha_deg: f64,
    pub launch_elevator_feedforward_deg: f64,
    pub failsafe_elevator_deg: f64,
    pub launch_alpha_gain: f64,
    pub glide_target_flight_path_deg: f64,
    pub flight_path_gain: f64,
    pub flight_path_lookahead_s: f64,
    pub degraded_flight_path_lookahead_s: f64,
    pub degraded_pull_out_start_altitude_loss_m: f64,
    pub degraded_pull_out_full_altitude_loss_m: f64,
    pub climb_limit_flight_path_deg: f64,
    pub climb_suppression_gain: f64,
    pub ground_climb_limit_mps: f64,
    pub ground_climb_suppression_gain_rad_per_mps: f64,
    pub vertical_speed_filter_time_constant_s: f64,
    pub alpha_limit_deg: f64,
    pub alpha_limit_gain: f64,
    pub launch_pitch_rate_gain_s: f64,
    pub glide_pitch_rate_gain_s: f64,
    pub glide_damping_enable_flight_path_deg: f64,
    pub glide_damping_transition_time_s: f64,
}

#[derive(Debug)]
pub enum ConfigError {
    Read(std::io::Error),
    Parse(serde_json::Error),
    UnsupportedSchema(String),
    InvalidModel(ModelError),
    InvalidMetadata,
    InvalidScenario(&'static str),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Read(error) => write!(formatter, "モデルファイルを読めません: {error}"),
            Self::Parse(error) => write!(formatter, "モデルJSONが不正です: {error}"),
            Self::UnsupportedSchema(version) => {
                write!(formatter, "未対応のschema_versionです: {version}")
            }
            Self::InvalidModel(error) => write!(formatter, "機体モデルが不正です: {error:?}"),
            Self::InvalidMetadata => write!(formatter, "モデルのmetadataが空です"),
            Self::InvalidScenario(field) => {
                write!(formatter, "scenarioの値が不正です: {field}")
            }
        }
    }
}

impl std::error::Error for ConfigError {}

pub struct LoadedSimulation {
    pub file: SimulationFile,
    longitudinal: Vec<AeroPoint>,
}

impl LoadedSimulation {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = fs::read_to_string(path).map_err(ConfigError::Read)?;
        let file: SimulationFile = serde_json::from_str(&text).map_err(ConfigError::Parse)?;
        if file.schema_version != "0.8.0" {
            return Err(ConfigError::UnsupportedSchema(file.schema_version));
        }
        if [
            &file.metadata.name,
            &file.metadata.provenance,
            &file.metadata.validation_status,
            &file.metadata.valid_for,
            &file.reference_geometry.moment_reference,
        ]
        .iter()
        .any(|value| value.trim().is_empty())
        {
            return Err(ConfigError::InvalidMetadata);
        }
        if file.metadata.sources.is_empty()
            || file.metadata.assumptions.is_empty()
            || file.metadata.sources.iter().any(|source| {
                [
                    &source.id,
                    &source.url,
                    &source.applies_to,
                    &source.authority,
                ]
                .iter()
                .any(|value| value.trim().is_empty())
            })
            || file
                .metadata
                .assumptions
                .iter()
                .any(|value| value.trim().is_empty())
        {
            return Err(ConfigError::InvalidMetadata);
        }
        validate_scenario(&file)?;
        let longitudinal = file
            .aerodynamics
            .longitudinal_table
            .iter()
            .map(|point| AeroPoint {
                alpha_rad: point.alpha_deg * DEG_TO_RAD,
                cl: point.cl,
                cd: point.cd,
                cm: point.cm,
            })
            .collect::<Vec<_>>();
        let loaded = Self { file, longitudinal };
        loaded.model()?;
        Ok(loaded)
    }

    pub fn model(&self) -> Result<AircraftModel<'_>, ConfigError> {
        let inertia = &self.file.mass_properties.inertia_body_kg_m2;
        let geometry = &self.file.reference_geometry;
        let derivatives = &self.file.aerodynamics.derivatives_per_rad;
        let ground_effect = &self.file.aerodynamics.ground_effect;
        let model = AircraftModel {
            mass_kg: self.file.mass_properties.mass_kg,
            inertia_kg_m2: Inertia {
                ixx: inertia.ixx,
                iyy: inertia.iyy,
                izz: inertia.izz,
                ixz: inertia.ixz,
            },
            reference_area_m2: geometry.area_m2,
            reference_span_m: geometry.span_m,
            reference_chord_m: geometry.chord_m,
            longitudinal: AeroTable::new(&self.longitudinal).map_err(ConfigError::InvalidModel)?,
            derivatives: AeroDerivatives {
                cl_elevator: derivatives.cl_elevator,
                cm_elevator: derivatives.cm_elevator,
                cm_pitch_rate: derivatives.cm_pitch_rate,
                cy_beta: derivatives.cy_beta,
                cy_roll_rate: derivatives.cy_roll_rate,
                cy_yaw_rate: derivatives.cy_yaw_rate,
                cy_rudder: derivatives.cy_rudder,
                c_roll_beta: derivatives.c_roll_beta,
                c_roll_roll_rate: derivatives.c_roll_roll_rate,
                c_roll_yaw_rate: derivatives.c_roll_yaw_rate,
                c_roll_rudder: derivatives.c_roll_rudder,
                cn_beta: derivatives.cn_beta,
                cn_roll_rate: derivatives.cn_roll_rate,
                cn_yaw_rate: derivatives.cn_yaw_rate,
                cn_rudder: derivatives.cn_rudder,
            },
            ground_effect: GroundEffectModel {
                enabled: ground_effect.enabled,
                wing_height_offset_m: ground_effect.wing_height_offset_m,
                induced_drag_factor: ground_effect.induced_drag_factor,
                minimum_induced_drag_ratio: ground_effect.minimum_induced_drag_ratio,
                correlation_gain: ground_effect.correlation_gain,
                height_exponent: ground_effect.height_exponent,
            },
        };
        model.validate().map_err(ConfigError::InvalidModel)?;
        Ok(model)
    }

    pub fn environment(&self) -> Environment {
        let environment = &self.file.environment;
        Environment {
            density_kg_m3: environment.density_kg_m3,
            gravity_mps2: environment.gravity_mps2,
            wind_ned_mps: array_to_vec3(environment.wind_ned_mps),
            ground_effect_enabled: false,
        }
    }

    pub fn environment_at_north(&self, north_m: f64) -> Environment {
        let mut environment = self.environment();
        environment.ground_effect_enabled = true;
        let gust = &self.file.environment.one_minus_cosine_gust;
        if gust.enabled
            && north_m >= gust.start_north_m
            && north_m <= gust.start_north_m + gust.length_m
        {
            let phase = (north_m - gust.start_north_m) / gust.length_m;
            let shape = one_minus_cosine_shape(phase);
            environment.wind_ned_mps += array_to_vec3(gust.peak_wind_ned_mps) * shape;
        }
        environment
    }

    pub fn sensor_model(&self) -> SensorModel {
        let sensors = &self.file.sensors;
        let imu = &sensors.imu;
        let air_data = &sensors.air_data;
        SensorModel {
            imu_sample_period_s: 1.0 / imu.sample_rate_hz,
            air_data_sample_period_s: 1.0 / air_data.sample_rate_hz,
            gyro_bias_rad_s: array_to_vec3(imu.gyro_bias_deg_s) * DEG_TO_RAD,
            gyro_resolution_rad_s: imu.gyro_resolution_deg_s * DEG_TO_RAD,
            euler_resolution_rad: imu.euler_resolution_deg * DEG_TO_RAD,
            acceleration_resolution_mps2: imu.acceleration_resolution_mps2,
            differential_pressure_range_pa: air_data.differential_pressure_range_pa,
            differential_pressure_resolution_pa: air_data.differential_pressure_resolution_pa,
            differential_pressure_bias_pa: air_data.differential_pressure_bias_pa,
            differential_pressure_time_constant_s: air_data.differential_pressure_time_constant_s,
            pitot_coefficient: air_data.pitot_coefficient,
            static_pressure_resolution_pa: air_data.static_pressure_resolution_pa,
            static_pressure_bias_pa: air_data.static_pressure_bias_pa,
            alpha_bias_rad: sensors.alpha_bias_deg * DEG_TO_RAD,
            alpha_resolution_rad: sensors.alpha_resolution_deg * DEG_TO_RAD,
        }
    }

    pub fn elevator_config(&self) -> ActuatorConfig {
        actuator_config(&self.file.actuators.elevator)
    }

    pub fn rudder_config(&self) -> ActuatorConfig {
        actuator_config(&self.file.actuators.rudder)
    }
}

fn validate_scenario(file: &SimulationFile) -> Result<(), ConfigError> {
    let environment = &file.environment;
    if !environment.density_kg_m3.is_finite() || environment.density_kg_m3 < 0.0 {
        return Err(ConfigError::InvalidScenario("environment.density_kg_m3"));
    }
    if !environment.gravity_mps2.is_finite() || environment.gravity_mps2 < 0.0 {
        return Err(ConfigError::InvalidScenario("environment.gravity_mps2"));
    }
    if environment
        .wind_ned_mps
        .iter()
        .chain(environment.one_minus_cosine_gust.peak_wind_ned_mps.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ConfigError::InvalidScenario("environment.wind_ned_mps"));
    }
    let gust = &environment.one_minus_cosine_gust;
    if !gust.start_north_m.is_finite()
        || !gust.length_m.is_finite()
        || (gust.enabled && gust.length_m <= 0.0)
    {
        return Err(ConfigError::InvalidScenario(
            "environment.one_minus_cosine_gust",
        ));
    }
    let initial = &file.initial_state;
    if !initial.altitude_m.is_finite() || initial.altitude_m < 0.0 {
        return Err(ConfigError::InvalidScenario("initial_state.altitude_m"));
    }
    if !initial.airspeed_mps.is_finite() || initial.airspeed_mps < 0.0 {
        return Err(ConfigError::InvalidScenario("initial_state.airspeed_mps"));
    }
    if [
        initial.alpha_deg,
        initial.roll_deg,
        initial.pitch_deg,
        initial.heading_deg,
        file.reference_controller.pull_out_start_airspeed_mps,
        file.reference_controller.pull_out_full_airspeed_mps,
        file.reference_controller.launch_target_alpha_deg,
        file.reference_controller.launch_elevator_feedforward_deg,
        file.reference_controller.failsafe_elevator_deg,
        file.reference_controller.launch_alpha_gain,
        file.reference_controller.glide_target_flight_path_deg,
        file.reference_controller.flight_path_gain,
        file.reference_controller.flight_path_lookahead_s,
        file.reference_controller.degraded_flight_path_lookahead_s,
        file.reference_controller
            .degraded_pull_out_start_altitude_loss_m,
        file.reference_controller
            .degraded_pull_out_full_altitude_loss_m,
        file.reference_controller.climb_limit_flight_path_deg,
        file.reference_controller.climb_suppression_gain,
        file.reference_controller.ground_climb_limit_mps,
        file.reference_controller
            .ground_climb_suppression_gain_rad_per_mps,
        file.reference_controller
            .vertical_speed_filter_time_constant_s,
        file.reference_controller.alpha_limit_deg,
        file.reference_controller.alpha_limit_gain,
        file.reference_controller.launch_pitch_rate_gain_s,
        file.reference_controller.glide_pitch_rate_gain_s,
        file.reference_controller
            .glide_damping_enable_flight_path_deg,
        file.reference_controller.glide_damping_transition_time_s,
        file.sensors.alpha_bias_deg,
        file.sensors.alpha_resolution_deg,
        file.sensors.imu.sample_rate_hz,
        file.sensors.imu.gyro_resolution_deg_s,
        file.sensors.imu.euler_resolution_deg,
        file.sensors.imu.acceleration_resolution_mps2,
        file.sensors.air_data.sample_rate_hz,
        file.sensors.air_data.differential_pressure_range_pa,
        file.sensors.air_data.differential_pressure_resolution_pa,
        file.sensors.air_data.differential_pressure_bias_pa,
        file.sensors.air_data.differential_pressure_time_constant_s,
        file.sensors.air_data.pitot_coefficient,
        file.sensors.air_data.static_pressure_resolution_pa,
        file.sensors.air_data.static_pressure_bias_pa,
    ]
    .iter()
    .chain(file.sensors.imu.gyro_bias_deg_s.iter())
    .any(|value| !value.is_finite())
    {
        return Err(ConfigError::InvalidScenario(
            "angles, controller, or sensor bias",
        ));
    }
    validate_reference_controller(&file.reference_controller)?;
    if file.reference_controller.failsafe_elevator_deg.abs() > file.actuators.elevator.max_abs_deg {
        return Err(ConfigError::InvalidScenario(
            "reference_controller.failsafe_elevator_deg",
        ));
    }
    Ok(())
}

fn validate_reference_controller(controller: &ReferenceControllerFile) -> Result<(), ConfigError> {
    if controller.pull_out_start_airspeed_mps <= 0.0
        || controller.pull_out_full_airspeed_mps <= controller.pull_out_start_airspeed_mps
        || controller.launch_alpha_gain < 0.0
        || controller.flight_path_gain < 0.0
        || controller.flight_path_lookahead_s < 0.0
        || controller.degraded_flight_path_lookahead_s < controller.flight_path_lookahead_s
        || controller.degraded_pull_out_start_altitude_loss_m < 0.0
        || controller.degraded_pull_out_full_altitude_loss_m
            <= controller.degraded_pull_out_start_altitude_loss_m
        || controller.climb_suppression_gain < 0.0
        || controller.ground_climb_limit_mps > 0.0
        || controller.ground_climb_suppression_gain_rad_per_mps < 0.0
        || controller.vertical_speed_filter_time_constant_s <= 0.0
        || controller.alpha_limit_gain < 0.0
        || controller.launch_pitch_rate_gain_s < 0.0
        || controller.glide_pitch_rate_gain_s < 0.0
        || controller.glide_damping_enable_flight_path_deg > 0.0
        || controller.glide_damping_transition_time_s <= 0.0
        || controller.glide_target_flight_path_deg > controller.climb_limit_flight_path_deg
        || controller.climb_limit_flight_path_deg > 0.0
    {
        return Err(ConfigError::InvalidScenario(
            "reference_controller envelope",
        ));
    }
    Ok(())
}

fn actuator_config(file: &ActuatorFile) -> ActuatorConfig {
    ActuatorConfig {
        max_abs_rad: file.max_abs_deg * DEG_TO_RAD,
        max_rate_rad_s: file.max_rate_deg_s * DEG_TO_RAD,
        time_constant_s: file.time_constant_s,
        deadband_rad: file.deadband_deg * DEG_TO_RAD,
        command_resolution_rad: file.command_resolution_deg * DEG_TO_RAD,
    }
}

const fn array_to_vec3(value: [f64; 3]) -> Vec3 {
    Vec3::new(value[0], value[1], value[2])
}

fn one_minus_cosine_shape(phase: f64) -> f64 {
    0.5 * (1.0 - (2.0 * std::f64::consts::PI * phase).cos())
}

pub const fn deg_to_rad(value: f64) -> f64 {
    value * DEG_TO_RAD
}

pub const fn rad_to_deg(value: f64) -> f64 {
    value / DEG_TO_RAD
}

#[cfg(test)]
mod tests {
    use super::one_minus_cosine_shape;

    #[test]
    fn full_one_minus_cosine_pulse_has_zero_endpoints_and_unit_peak() {
        assert!(one_minus_cosine_shape(0.0).abs() < 1.0e-12);
        assert!((one_minus_cosine_shape(0.5) - 1.0).abs() < 1.0e-12);
        assert!(one_minus_cosine_shape(1.0).abs() < 1.0e-12);
    }
}
