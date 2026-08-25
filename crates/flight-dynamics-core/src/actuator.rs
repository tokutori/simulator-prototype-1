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

    /// Advances one fixed step and returns the new position.
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
        let mut target = quantized_command.clamp(-config.max_abs_rad, config.max_abs_rad);
        if (target - self.position_rad).abs() <= config.deadband_rad {
            target = self.position_rad;
        }
        let unconstrained_rate = (target - self.position_rad) / config.time_constant_s;
        let rate = unconstrained_rate.clamp(-config.max_rate_rad_s, config.max_rate_rad_s);
        self.position_rad =
            (self.position_rad + rate * dt_s).clamp(-config.max_abs_rad, config.max_abs_rad);
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
        assert_eq!(actuator.position_rad(), 0.2);
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
        assert_eq!(actuator.step(0.026, config, 0.01), Ok(0.03));
    }
}
