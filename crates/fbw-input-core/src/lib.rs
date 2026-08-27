#![no_std]
#![forbid(unsafe_code)]
//! Platform-independent decoding and authority blending for pilot controls.

/// Raw RP2040 pilot controls sampled in one control update.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RawPilotInput {
    /// Elevator joystick ADC counts.
    pub elevator_adc: u16,
    /// Rudder joystick ADC counts.
    pub rudder_adc: u16,
    /// Automatic-control authority ADC counts.
    pub authority_adc: u16,
    /// Elevator button pair state.
    pub elevator_buttons: ButtonPair,
    /// Rudder button pair state.
    pub rudder_buttons: ButtonPair,
}

/// Complete, mutually exclusive state of one opposing button pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ButtonPair {
    /// Neither button is active.
    Released,
    /// Only the negative-direction button is active.
    Negative,
    /// Only the positive-direction button is active.
    Positive,
    /// Both buttons are active; decoded safely as neutral.
    Conflicting,
}

impl ButtonPair {
    /// Constructs the one valid pair state from electrical pressed levels.
    #[must_use]
    pub const fn from_pressed(negative: bool, positive: bool) -> Self {
        match (negative, positive) {
            (false, false) => Self::Released,
            (true, false) => Self::Negative,
            (false, true) => Self::Positive,
            (true, true) => Self::Conflicting,
        }
    }
}

/// Normalized pilot demand and automatic-control authority.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PilotDemand {
    /// Elevator demand in -1 through 1; positive is nose-down.
    pub elevator: f32,
    /// Rudder response demand in -1 through 1; positive requests right yaw.
    pub rudder: f32,
    /// Automatic authority in 0 through 1.
    pub autonomy: f32,
}

/// Converts 12-bit ADC axes and active-low button states into bounded demand.
#[must_use]
pub fn decode(raw: RawPilotInput) -> PilotDemand {
    PilotDemand {
        elevator: button_or_axis(raw.elevator_buttons, axis(raw.elevator_adc)),
        rudder: button_or_axis(raw.rudder_buttons, axis(raw.rudder_adc)),
        autonomy: f32::from(raw.authority_adc.min(4095)) / 4095.0,
    }
}

/// Blends manual and automatic surface commands using explicit authority.
#[must_use]
pub fn blend(manual_rad: f32, automatic_rad: f32, autonomy: f32, limit_rad: f32) -> f32 {
    let authority = autonomy.clamp(0.0, 1.0);
    (manual_rad * (1.0 - authority) + automatic_rad * authority).clamp(-limit_rad, limit_rad)
}

fn axis(counts: u16) -> f32 {
    const CENTER: f32 = 2047.5;
    const DEAD_ZONE: f32 = 164.0; // Four percent of full scale around centre.
    let centered = f32::from(counts.min(4095)) - CENTER;
    let magnitude = centered.abs();
    if magnitude <= DEAD_ZONE {
        0.0
    } else {
        centered.signum() * ((magnitude - DEAD_ZONE) / (CENTER - DEAD_ZONE)).min(1.0)
    }
}

const fn button_or_axis(buttons: ButtonPair, analog: f32) -> f32 {
    match buttons {
        ButtonPair::Negative => -1.0,
        ButtonPair::Positive => 1.0,
        ButtonPair::Conflicting => 0.0,
        ButtonPair::Released => analog,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centred_stick_is_neutral_and_endpoints_are_full_scale() {
        assert_eq!(axis(2048), 0.0);
        assert_eq!(axis(0), -1.0);
        assert_eq!(axis(4095), 1.0);
    }

    #[test]
    fn buttons_override_axis_and_opposites_cancel() {
        assert_eq!(button_or_axis(ButtonPair::Negative, 0.4), -1.0);
        assert_eq!(button_or_axis(ButtonPair::Positive, -0.4), 1.0);
        assert_eq!(button_or_axis(ButtonPair::Conflicting, 0.8), 0.0);
    }

    #[test]
    fn authority_blend_has_manual_shared_and_auto_endpoints() {
        assert_eq!(blend(-0.1, 0.2, 0.0, 1.0), -0.1);
        assert!((blend(-0.1, 0.2, 0.5, 1.0) - 0.05).abs() < 1.0e-6);
        assert_eq!(blend(-0.1, 0.2, 1.0, 1.0), 0.2);
    }
}
