//! Deterministic sample-and-hold models for the flight sensor suite.

use crate::{AeroLoads, Environment, RigidBodyState, Vec3};

const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;
const SEA_LEVEL_TEMPERATURE_K: f64 = 288.15;
const TEMPERATURE_LAPSE_K_M: f64 = 0.0065;
const PRESSURE_EXPONENT: f64 = 5.255_879_7;

/// Sensor configuration in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorModel {
    /// BNO055 fusion-mode output period.
    pub imu_sample_period_s: f64,
    /// SDP810 differential-pressure register acquisition period.
    pub differential_pressure_sample_period_s: f64,
    /// DPS310 absolute-pressure register acquisition period.
    pub static_pressure_sample_period_s: f64,
    /// AS5600 angle-of-attack register acquisition period.
    pub alpha_sample_period_s: f64,
    /// Constant gyroscope bias in body axes.
    pub gyro_bias_rad_s: Vec3,
    /// Gyroscope register quantum.
    pub gyro_resolution_rad_s: f64,
    /// Euler-angle register quantum.
    pub euler_resolution_rad: f64,
    /// Accelerometer register quantum.
    pub acceleration_resolution_mps2: f64,
    /// Differential-pressure full-scale magnitude.
    pub differential_pressure_range_pa: f64,
    /// Differential-pressure register quantum.
    pub differential_pressure_resolution_pa: f64,
    /// Constant differential-pressure offset.
    pub differential_pressure_bias_pa: f64,
    /// First-order pressure-port and sensor response time constant.
    pub differential_pressure_time_constant_s: f64,
    /// Pitot calibration coefficient, `dp = coefficient * q`.
    pub pitot_coefficient: f64,
    /// Absolute-pressure register quantum.
    pub static_pressure_resolution_pa: f64,
    /// Constant absolute-pressure offset.
    pub static_pressure_bias_pa: f64,
    /// Constant angle-of-attack bias.
    pub alpha_bias_rad: f64,
    /// Angle-of-attack output quantum.
    pub alpha_resolution_rad: f64,
}

/// Invalid sensor-model input.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorError {
    /// A configured period, range, resolution, or coefficient is invalid.
    InvalidConfig,
    /// Time step, mass, density, or sensor input is invalid.
    InvalidInput,
}

/// Sampled sensor outputs held between configured update instants.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorSample {
    /// Quantized fused roll angle.
    pub roll_rad: f64,
    /// Quantized fused pitch angle.
    pub pitch_rad: f64,
    /// Quantized fused yaw/heading angle.
    pub yaw_rad: f64,
    /// Quantized, biased angular rates in body axes.
    pub gyro_rad_s: Vec3,
    /// Specific force at the centre of gravity, excluding gravity.
    pub acceleration_body_mps2: Vec3,
    /// Airspeed reconstructed from differential pressure.
    pub airspeed_mps: f64,
    /// Quantized differential pressure.
    pub differential_pressure_pa: f64,
    /// Barometric altitude relative to standard sea-level pressure.
    pub barometric_altitude_m: f64,
    /// Wrapping acquisition identity; advances even when quantized pressure is unchanged.
    pub barometric_sample_sequence: u32,
    /// Quantized angle of attack.
    pub alpha_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum SampleClockState {
    Initial,
    Running { elapsed_s: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SampleClock {
    period_s: f64,
    state: SampleClockState,
}

impl SampleClock {
    const fn new(period_s: f64) -> Self {
        Self {
            period_s,
            state: SampleClockState::Initial,
        }
    }

    fn advance(&mut self, dt_s: f64) -> bool {
        match self.state {
            SampleClockState::Initial => {
                self.state = SampleClockState::Running { elapsed_s: 0.0 };
                true
            }
            SampleClockState::Running { elapsed_s } => {
                let mut next_elapsed_s = elapsed_s + dt_s;
                if next_elapsed_s + f64::EPSILON < self.period_s {
                    self.state = SampleClockState::Running {
                        elapsed_s: next_elapsed_s,
                    };
                    return false;
                }
                while next_elapsed_s + f64::EPSILON >= self.period_s {
                    next_elapsed_s = (next_elapsed_s - self.period_s).max(0.0);
                }
                self.state = SampleClockState::Running {
                    elapsed_s: next_elapsed_s,
                };
                true
            }
        }
    }
}

/// Explicit state for deterministic sensor replay.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorSuite {
    model: SensorModel,
    imu_clock: SampleClock,
    differential_pressure_clock: SampleClock,
    static_pressure_clock: SampleClock,
    alpha_clock: SampleClock,
    filtered_differential_pressure_pa: f64,
    differential_pressure_initialized: bool,
    sample: SensorSample,
}

impl SensorSuite {
    /// Creates a suite whose first call to [`Self::step`] captures every channel.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidConfig`] for non-finite or non-positive
    /// periods, ranges, resolutions, time constants, and coefficients.
    pub fn new(model: SensorModel) -> Result<Self, SensorError> {
        validate_model(model)?;
        Ok(Self {
            model,
            imu_clock: SampleClock::new(model.imu_sample_period_s),
            differential_pressure_clock: SampleClock::new(
                model.differential_pressure_sample_period_s,
            ),
            static_pressure_clock: SampleClock::new(model.static_pressure_sample_period_s),
            alpha_clock: SampleClock::new(model.alpha_sample_period_s),
            filtered_differential_pressure_pa: 0.0,
            differential_pressure_initialized: false,
            sample: SensorSample::default(),
        })
    }

    /// Advances response dynamics and discrete sample-and-hold registers.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::InvalidInput`] when the step or physical inputs
    /// are outside their finite positive domain.
    pub fn step(
        &mut self,
        state: RigidBodyState,
        loads: AeroLoads,
        environment: Environment,
        mass_kg: f64,
        dt_s: f64,
    ) -> Result<SensorSample, SensorError> {
        if !dt_s.is_finite()
            || dt_s <= 0.0
            || !mass_kg.is_finite()
            || mass_kg <= 0.0
            || !environment.density_kg_m3.is_finite()
            || environment.density_kg_m3 <= 0.0
            || !loads.condition.dynamic_pressure_pa.is_finite()
        {
            return Err(SensorError::InvalidInput);
        }

        let target_dp = self.model.pitot_coefficient * loads.condition.dynamic_pressure_pa;
        if self.differential_pressure_initialized {
            let response_fraction =
                dt_s / (self.model.differential_pressure_time_constant_s + dt_s);
            self.filtered_differential_pressure_pa +=
                response_fraction * (target_dp - self.filtered_differential_pressure_pa);
        } else {
            // The scenario begins at platform release. The pitot has already
            // been exposed to the launch run, so avoid inventing a power-on
            // transient from zero differential pressure.
            self.filtered_differential_pressure_pa = target_dp;
            self.differential_pressure_initialized = true;
        }

        if self.imu_clock.advance(dt_s) {
            self.capture_imu(state, loads, mass_kg);
        }

        if self.differential_pressure_clock.advance(dt_s) {
            self.capture_differential_pressure(environment);
        }

        if self.static_pressure_clock.advance(dt_s) {
            self.capture_static_pressure(state);
        }

        if self.alpha_clock.advance(dt_s) {
            self.capture_alpha(loads);
        }
        Ok(self.sample)
    }

    fn capture_imu(&mut self, state: RigidBodyState, loads: AeroLoads, mass_kg: f64) {
        let euler = state.attitude_body_to_ned.to_euler();
        self.sample.roll_rad = quantize(euler.x, self.model.euler_resolution_rad);
        self.sample.pitch_rad = quantize(euler.y, self.model.euler_resolution_rad);
        self.sample.yaw_rad = quantize(euler.z, self.model.euler_resolution_rad);
        self.sample.gyro_rad_s = quantize_vec3(
            state.rates_body_rad_s + self.model.gyro_bias_rad_s,
            self.model.gyro_resolution_rad_s,
        );
        self.sample.acceleration_body_mps2 = quantize_vec3(
            loads.force_body_n / mass_kg,
            self.model.acceleration_resolution_mps2,
        );
    }

    fn capture_differential_pressure(&mut self, environment: Environment) {
        let biased_dp =
            self.filtered_differential_pressure_pa + self.model.differential_pressure_bias_pa;
        self.sample.differential_pressure_pa = quantize(
            biased_dp.clamp(
                -self.model.differential_pressure_range_pa,
                self.model.differential_pressure_range_pa,
            ),
            self.model.differential_pressure_resolution_pa,
        );
        let nonnegative_dp = self.sample.differential_pressure_pa.max(0.0);
        self.sample.airspeed_mps = libm::sqrt(
            2.0 * nonnegative_dp / (environment.density_kg_m3 * self.model.pitot_coefficient),
        );
    }

    fn capture_static_pressure(&mut self, state: RigidBodyState) {
        self.sample.barometric_sample_sequence =
            self.sample.barometric_sample_sequence.wrapping_add(1);
        let altitude_m = -state.position_ned_m.z;
        let static_pressure =
            pressure_from_altitude(altitude_m) + self.model.static_pressure_bias_pa;
        let measured_pressure = quantize(static_pressure, self.model.static_pressure_resolution_pa);
        self.sample.barometric_altitude_m = altitude_from_pressure(measured_pressure);
    }

    fn capture_alpha(&mut self, loads: AeroLoads) {
        self.sample.alpha_rad = quantize(
            loads.condition.alpha_rad + self.model.alpha_bias_rad,
            self.model.alpha_resolution_rad,
        );
    }
}

fn validate_model(model: SensorModel) -> Result<(), SensorError> {
    let positive = [
        model.imu_sample_period_s,
        model.differential_pressure_sample_period_s,
        model.static_pressure_sample_period_s,
        model.alpha_sample_period_s,
        model.gyro_resolution_rad_s,
        model.euler_resolution_rad,
        model.acceleration_resolution_mps2,
        model.differential_pressure_range_pa,
        model.differential_pressure_resolution_pa,
        model.differential_pressure_time_constant_s,
        model.pitot_coefficient,
        model.static_pressure_resolution_pa,
        model.alpha_resolution_rad,
    ];
    let finite = [
        model.gyro_bias_rad_s.x,
        model.gyro_bias_rad_s.y,
        model.gyro_bias_rad_s.z,
        model.differential_pressure_bias_pa,
        model.static_pressure_bias_pa,
        model.alpha_bias_rad,
    ];
    if positive
        .iter()
        .all(|value| value.is_finite() && *value > 0.0)
        && finite.iter().all(|value| value.is_finite())
    {
        Ok(())
    } else {
        Err(SensorError::InvalidConfig)
    }
}

fn quantize(value: f64, resolution: f64) -> f64 {
    libm::round(value / resolution) * resolution
}

fn quantize_vec3(value: Vec3, resolution: f64) -> Vec3 {
    Vec3::new(
        quantize(value.x, resolution),
        quantize(value.y, resolution),
        quantize(value.z, resolution),
    )
}

fn pressure_from_altitude(altitude_m: f64) -> f64 {
    let base =
        (1.0 - TEMPERATURE_LAPSE_K_M * altitude_m / SEA_LEVEL_TEMPERATURE_K).clamp(0.01, 2.0);
    SEA_LEVEL_PRESSURE_PA * libm::pow(base, PRESSURE_EXPONENT)
}

fn altitude_from_pressure(pressure_pa: f64) -> f64 {
    let ratio = (pressure_pa / SEA_LEVEL_PRESSURE_PA).max(0.01);
    SEA_LEVEL_TEMPERATURE_K / TEMPERATURE_LAPSE_K_M
        * (1.0 - libm::pow(ratio, 1.0 / PRESSURE_EXPONENT))
}

#[cfg(test)]
mod tests {
    use super::{SensorModel, SensorSuite};
    use crate::{AeroLoads, Environment, FlightCondition, Quaternion, RigidBodyState, Vec3};

    fn model() -> SensorModel {
        SensorModel {
            imu_sample_period_s: 0.01,
            differential_pressure_sample_period_s: 0.02,
            static_pressure_sample_period_s: 0.04,
            alpha_sample_period_s: 0.03,
            gyro_bias_rad_s: Vec3::new(0.001, 0.0, 0.0),
            gyro_resolution_rad_s: 0.001,
            euler_resolution_rad: 0.001,
            acceleration_resolution_mps2: 0.01,
            differential_pressure_range_pa: 500.0,
            differential_pressure_resolution_pa: 0.1,
            differential_pressure_bias_pa: 0.0,
            differential_pressure_time_constant_s: 0.003,
            pitot_coefficient: 1.0,
            static_pressure_resolution_pa: 0.1,
            static_pressure_bias_pa: 0.0,
            alpha_bias_rad: 0.0,
            alpha_resolution_rad: 0.001,
        }
    }

    #[test]
    fn air_data_channels_are_sampled_and_held_on_independent_clocks() {
        let initial_state = RigidBodyState {
            position_ned_m: Vec3::new(0.0, 0.0, -10.0),
            velocity_body_mps: Vec3::new(10.0, 0.0, 0.0),
            attitude_body_to_ned: Quaternion::IDENTITY,
            rates_body_rad_s: Vec3::ZERO,
        };
        let mut loads = AeroLoads {
            condition: FlightCondition {
                airspeed_mps: 10.0,
                dynamic_pressure_pa: 50.0,
                ..FlightCondition::default()
            },
            ..AeroLoads::default()
        };
        let environment = Environment {
            density_kg_m3: 1.0,
            gravity_mps2: 9.81,
            wind_ned_mps: Vec3::ZERO,
            ground_effect_enabled: false,
        };
        let mut suite = SensorSuite::new(model()).expect("valid model");
        let initial = suite
            .step(initial_state, loads, environment, 10.0, 0.01)
            .expect("valid sample");
        loads.condition.dynamic_pressure_pa = 200.0;
        loads.condition.alpha_rad = 0.2;
        let changed_state = RigidBodyState {
            position_ned_m: Vec3::new(0.0, 0.0, -20.0),
            ..initial_state
        };
        let held = suite
            .step(changed_state, loads, environment, 10.0, 0.01)
            .expect("valid sample");
        assert_eq!(held, initial);

        let differential_pressure_updated = suite
            .step(changed_state, loads, environment, 10.0, 0.01)
            .expect("valid sample");
        assert!(
            differential_pressure_updated.differential_pressure_pa > held.differential_pressure_pa
        );
        assert_eq!(
            differential_pressure_updated.barometric_altitude_m,
            held.barometric_altitude_m
        );
        assert_eq!(differential_pressure_updated.alpha_rad, held.alpha_rad);

        let alpha_updated = suite
            .step(changed_state, loads, environment, 10.0, 0.01)
            .expect("valid sample");
        assert!(alpha_updated.alpha_rad > held.alpha_rad);
        assert_eq!(
            alpha_updated.barometric_altitude_m,
            held.barometric_altitude_m
        );

        let static_pressure_updated = suite
            .step(changed_state, loads, environment, 10.0, 0.01)
            .expect("valid sample");
        assert!(static_pressure_updated.barometric_altitude_m > 19.9);
        assert_eq!(
            static_pressure_updated.barometric_sample_sequence,
            initial.barometric_sample_sequence + 1
        );
        let mut equal_pressure = static_pressure_updated;
        for _ in 0..4 {
            equal_pressure = suite
                .step(changed_state, loads, environment, 10.0, 0.01)
                .expect("fresh equal pressure");
        }
        assert_eq!(
            equal_pressure.barometric_altitude_m,
            static_pressure_updated.barometric_altitude_m
        );
        assert_eq!(
            equal_pressure.barometric_sample_sequence,
            static_pressure_updated.barometric_sample_sequence + 1
        );
    }
}
