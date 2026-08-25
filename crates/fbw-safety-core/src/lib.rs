#![no_std]
#![forbid(unsafe_code)]
//! Platform-independent command gate for sensor faults.

/// Sensor-validity state exposed to diagnostics and tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SafetyMode {
    /// The configured failsafe command remains active until enough valid samples arrive.
    Arming,
    /// Valid sensor data controls the actuator.
    Active,
    /// A short dropout holds the last valid command.
    HoldLast,
    /// A sustained fault applies the configured command until a valid recovery window passes.
    Failsafe,
}

/// Timing policy expressed in controller samples.
#[derive(Clone, Copy, Debug)]
pub struct SafetyConfig {
    /// Consecutive valid samples required at startup.
    pub startup_valid_samples: u16,
    /// Invalid samples for which the last command may be held.
    pub hold_invalid_samples: u16,
    /// Consecutive valid samples required after a sustained fault.
    pub recovery_valid_samples: u16,
    /// Aircraft-specific command used while unarmed or in failsafe.
    pub failsafe_command_rad: f32,
}

impl SafetyConfig {
    /// Returns whether sample counts form a usable policy.
    #[must_use]
    pub const fn is_valid(self) -> bool {
        self.startup_valid_samples > 0 && self.recovery_valid_samples > 0
    }
}

/// Result of one safety-gate update.
#[derive(Clone, Copy, Debug)]
pub struct SafetyOutput {
    /// Actuator command after safety policy is applied.
    pub command_rad: f32,
    /// Current gate state.
    pub mode: SafetyMode,
    /// True on entry to failsafe or return to active control.
    pub reset_controller: bool,
}

/// Stateful sensor-validity and command gate.
#[derive(Clone, Copy, Debug)]
pub struct SafetyState {
    mode: SafetyMode,
    valid_streak: u16,
    invalid_streak: u16,
    last_valid_command_rad: f32,
}

impl Default for SafetyState {
    fn default() -> Self {
        Self {
            mode: SafetyMode::Arming,
            valid_streak: 0,
            invalid_streak: 0,
            last_valid_command_rad: 0.0,
        }
    }
}

impl SafetyState {
    /// Current state without advancing the gate.
    #[must_use]
    pub const fn mode(&self) -> SafetyMode {
        self.mode
    }

    /// Whether controller state should be rebuilt before evaluating a valid sample.
    #[must_use]
    pub const fn controller_should_start_clean(&self) -> bool {
        matches!(self.mode, SafetyMode::Arming | SafetyMode::Failsafe)
    }

    /// Applies the validity policy to an optional finite command.
    ///
    /// `None` and non-finite values are both invalid sensor/control samples.
    #[must_use]
    pub fn step(&mut self, config: &SafetyConfig, candidate_rad: Option<f32>) -> SafetyOutput {
        debug_assert!(config.is_valid());
        let candidate_rad = candidate_rad.filter(|value| value.is_finite());
        match (self.mode, candidate_rad) {
            (SafetyMode::Arming, Some(command)) => {
                self.last_valid_command_rad = command;
                self.valid_streak = self.valid_streak.saturating_add(1);
                if self.valid_streak >= config.startup_valid_samples {
                    self.mode = SafetyMode::Active;
                    self.invalid_streak = 0;
                    self.output(command, true)
                } else {
                    self.output(config.failsafe_command_rad, false)
                }
            }
            (SafetyMode::Arming | SafetyMode::Failsafe, None) => {
                self.valid_streak = 0;
                self.output(config.failsafe_command_rad, false)
            }
            (SafetyMode::Active | SafetyMode::HoldLast, Some(command)) => {
                self.mode = SafetyMode::Active;
                self.invalid_streak = 0;
                self.valid_streak = 0;
                self.last_valid_command_rad = command;
                self.output(command, false)
            }
            (SafetyMode::Active | SafetyMode::HoldLast, None) => {
                self.invalid_streak = self.invalid_streak.saturating_add(1);
                if self.invalid_streak <= config.hold_invalid_samples {
                    self.mode = SafetyMode::HoldLast;
                    self.output(self.last_valid_command_rad, false)
                } else {
                    self.mode = SafetyMode::Failsafe;
                    self.valid_streak = 0;
                    self.output(config.failsafe_command_rad, true)
                }
            }
            (SafetyMode::Failsafe, Some(command)) => {
                self.last_valid_command_rad = command;
                self.valid_streak = self.valid_streak.saturating_add(1);
                if self.valid_streak >= config.recovery_valid_samples {
                    self.mode = SafetyMode::Active;
                    self.invalid_streak = 0;
                    self.output(command, true)
                } else {
                    self.output(config.failsafe_command_rad, false)
                }
            }
        }
    }

    const fn output(&self, command_rad: f32, reset_controller: bool) -> SafetyOutput {
        SafetyOutput {
            command_rad,
            mode: self.mode,
            reset_controller,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{SafetyConfig, SafetyMode, SafetyState};

    const CONFIG: SafetyConfig = SafetyConfig {
        startup_valid_samples: 3,
        hold_invalid_samples: 2,
        recovery_valid_samples: 4,
        failsafe_command_rad: 0.0,
    };

    #[test]
    fn startup_requires_a_consecutive_valid_window() {
        let mut state = SafetyState::default();
        assert_eq!(state.step(&CONFIG, Some(0.1)).mode, SafetyMode::Arming);
        assert_eq!(state.step(&CONFIG, None).mode, SafetyMode::Arming);
        assert_eq!(state.step(&CONFIG, Some(0.1)).mode, SafetyMode::Arming);
        assert_eq!(state.step(&CONFIG, Some(0.1)).mode, SafetyMode::Arming);
        let armed = state.step(&CONFIG, Some(0.1));
        assert_eq!(armed.mode, SafetyMode::Active);
        assert_eq!(armed.command_rad, 0.1);
        assert!(armed.reset_controller);
    }

    #[test]
    fn transient_dropout_holds_last_command_without_failsafe() {
        let mut state = active_state();
        assert_eq!(state.step(&CONFIG, None).command_rad, 0.1);
        assert_eq!(state.mode(), SafetyMode::HoldLast);
        assert_eq!(state.step(&CONFIG, None).command_rad, 0.1);
        let recovered = state.step(&CONFIG, Some(0.2));
        assert_eq!(recovered.mode, SafetyMode::Active);
        assert_eq!(recovered.command_rad, 0.2);
        assert!(!recovered.reset_controller);
    }

    #[test]
    fn sustained_fault_applies_failsafe_and_requires_recovery_window() {
        let mut state = active_state();
        let _ = state.step(&CONFIG, None);
        let _ = state.step(&CONFIG, None);
        let failed = state.step(&CONFIG, None);
        assert_eq!(failed.mode, SafetyMode::Failsafe);
        assert_eq!(failed.command_rad, 0.0);
        assert!(failed.reset_controller);
        for _ in 0..3 {
            let recovering = state.step(&CONFIG, Some(0.3));
            assert_eq!(recovering.mode, SafetyMode::Failsafe);
            assert_eq!(recovering.command_rad, 0.0);
        }
        let recovered = state.step(&CONFIG, Some(0.3));
        assert_eq!(recovered.mode, SafetyMode::Active);
        assert_eq!(recovered.command_rad, 0.3);
        assert!(recovered.reset_controller);
    }

    #[test]
    fn non_finite_command_is_invalid() {
        let mut state = active_state();
        assert_eq!(
            state.step(&CONFIG, Some(f32::NAN)).mode,
            SafetyMode::HoldLast
        );
    }

    fn active_state() -> SafetyState {
        let mut state = SafetyState::default();
        for _ in 0..3 {
            let _ = state.step(&CONFIG, Some(0.1));
        }
        state
    }
}
