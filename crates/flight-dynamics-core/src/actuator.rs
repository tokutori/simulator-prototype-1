//! Deterministic servo position, lag, rate, and saturation model.

/// Servo model parameters in radians and seconds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorConfig {
    /// Symmetric mechanical travel limit.
    pub max_abs_rad: f64,
    /// Maximum position rate.
    pub max_rate_rad_s: f64,
    /// First-order time constant.
    pub time_constant_s: f64,
    /// Position error inside which the servo holds its current position.
    pub deadband_rad: f64,
    /// Smallest representable command increment.
    pub command_resolution_rad: f64,
}

/// Invalid actuator inputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActuatorError {
    /// Configuration is not finite and strictly positive.
    InvalidConfig,
    /// Time step is not finite and strictly positive.
    InvalidTimeStep,
    /// Command is not finite.
    InvalidCommand,
}

/// Stateful servo position. All environment interaction remains outside core.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Actuator {
    position_rad: f64,
}

impl Actuator {
    /// Creates an actuator at a clamped initial position.
    ///
    /// # Errors
    ///
    /// Returns [`ActuatorError::InvalidConfig`] for non-positive parameters or
    /// [`ActuatorError::InvalidCommand`] for a non-finite initial position.
    pub fn new(initial_position_rad: f64, config: ActuatorConfig) -> Result<Self, ActuatorError> {
        validate_config(config)?;
        if !initial_position_rad.is_finite() {
            return Err(ActuatorError::InvalidCommand);
        }
        Ok(Self {
            position_rad: initial_position_rad.clamp(-config.max_abs_rad, config.max_abs_rad),
        })
    }

    /// Advances under a held command and returns the new position.
    ///
    /// Integrates the rate-limited first-order response analytically: constant
    /// speed until the unsaturated region, then exponential decay. Motion stops
    /// at the deadband boundary. Positive time steps cannot overshoot a target
    /// or create numerical oscillation, even when larger than the time constant.
    ///
    /// # Errors
    ///
    /// Returns an [`ActuatorError`] when the configuration, command, or time
    /// step is outside its documented finite positive domain.
    pub fn step(
        &mut self,
        command_rad: f64,
        config: ActuatorConfig,
        dt_s: f64,
    ) -> Result<f64, ActuatorError> {
        validate_config(config)?;
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(ActuatorError::InvalidTimeStep);
        }
        if !command_rad.is_finite() {
            return Err(ActuatorError::InvalidCommand);
        }
        let quantized_command = libm::round(command_rad / config.command_resolution_rad)
            * config.command_resolution_rad;
        let target = quantized_command.clamp(-config.max_abs_rad, config.max_abs_rad);
        let error = target - self.position_rad;
        let magnitude = error.abs();
        if magnitude <= config.deadband_rad {
            return Ok(self.position_rad);
        }
        let linear_duration = (magnitude / config.max_rate_rad_s - config.time_constant_s).max(0.0);
        let displacement = if dt_s <= linear_duration {
            config.max_rate_rad_s * dt_s
        } else {
            let linear_displacement = config.max_rate_rad_s * linear_duration;
            let remaining = (magnitude - linear_displacement).max(0.0);
            linear_displacement
                + remaining * -libm::expm1(-(dt_s - linear_duration) / config.time_constant_s)
        };
        let displacement = displacement.min(magnitude - config.deadband_rad);
        self.position_rad = (self.position_rad + error.signum() * displacement)
            .clamp(-config.max_abs_rad, config.max_abs_rad);
        Ok(self.position_rad)
    }

    /// Returns the current position.
    #[must_use]
    pub const fn position_rad(self) -> f64 {
        self.position_rad
    }
}

fn validate_config(config: ActuatorConfig) -> Result<(), ActuatorError> {
    if [
        config.max_abs_rad,
        config.max_rate_rad_s,
        config.time_constant_s,
        config.command_resolution_rad,
    ]
    .iter()
    .all(|value| value.is_finite() && *value > 0.0)
        && config.deadband_rad.is_finite()
        && config.deadband_rad >= 0.0
    {
        Ok(())
    } else {
        Err(ActuatorError::InvalidConfig)
    }
}

#[cfg(test)]
mod tests {
    use super::{Actuator, ActuatorConfig};

    #[test]
    fn rate_and_travel_are_both_limited() {
        let config = ActuatorConfig {
            max_abs_rad: 0.2,
            max_rate_rad_s: 0.1,
            time_constant_s: 0.01,
            deadband_rad: 0.001,
            command_resolution_rad: 0.000_1,
        };
        let mut actuator = Actuator::new(0.0, config).expect("valid actuator");
        assert_eq!(actuator.step(1.0, config, 0.5), Ok(0.05));
        for _ in 0..10 {
            actuator.step(1.0, config, 0.5).expect("valid step");
        }
        assert!((actuator.position_rad() - 0.199).abs() < 1e-12);
    }

    #[test]
    fn quantization_and_deadband_hold_small_commands() {
        let config = ActuatorConfig {
            max_abs_rad: 1.0,
            max_rate_rad_s: 10.0,
            time_constant_s: 0.01,
            deadband_rad: 0.02,
            command_resolution_rad: 0.01,
        };
        let mut actuator = Actuator::new(0.0, config).expect("valid actuator");
        assert_eq!(actuator.step(0.014, config, 0.01), Ok(0.0));
        assert!((actuator.step(0.026, config, 0.01).unwrap() - 0.01).abs() < 1e-12);
    }

    #[test]
    fn large_steps_converge_without_reversing_or_overshooting() {
        let config = ActuatorConfig {
            max_abs_rad: 0.2,
            max_rate_rad_s: 10.0,
            time_constant_s: 0.06,
            deadband_rad: 0.0,
            command_resolution_rad: 0.0001,
        };
        for target in [-0.01_f64, 0.01] {
            let mut actuator = Actuator::new(0.0, config).unwrap();
            let mut previous_error = target.abs();
            for _ in 0..20 {
                let position = actuator.step(target, config, 0.2).unwrap();
                assert!(position.abs() <= target.abs());
                let error = (target - position).abs();
                assert!(error <= previous_error);
                previous_error = error;
            }
            assert!(previous_error < 1e-12);
        }
    }

    #[test]
    fn held_command_is_independent_of_step_partition_through_rate_limit_and_deadband() {
        let config = ActuatorConfig {
            max_abs_rad: 1.0,
            max_rate_rad_s: 0.4,
            time_constant_s: 0.1,
            deadband_rad: 0.005,
            command_resolution_rad: 0.001,
        };
        for duration in [0.01, 0.5, 1.95, 2.1, 4.0] {
            let mut whole = Actuator::new(-0.4, config).unwrap();
            let mut divided = whole;
            whole.step(0.4, config, duration).unwrap();
            for _ in 0..100 {
                divided.step(0.4, config, duration / 100.0).unwrap();
            }
            assert!((whole.position_rad() - divided.position_rad()).abs() < 1e-12);
        }
    }
}
