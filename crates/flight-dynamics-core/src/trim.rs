//! Deterministic steady, unpowered longitudinal trim calculations.

use crate::{AeroCoefficients, AircraftModel, Environment};

/// A steady unpowered glide solution at a fixed elevator deflection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteadyGlideTrim {
    /// Body angle of attack.
    pub alpha_rad: f64,
    /// Air-relative flight-path angle; a glide has a negative value.
    pub flight_path_rad: f64,
    /// Pitch attitude for zero wind in wings-level flight.
    pub pitch_rad: f64,
    /// True airspeed satisfying normal force equilibrium.
    pub airspeed_mps: f64,
    /// Elevator deflection used for the solution.
    pub elevator_rad: f64,
    /// Static coefficients at the solution (`q = 0`).
    pub coefficients: AeroCoefficients,
}

/// Failure to find a physical steady-glide solution inside the aero table.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrimError {
    /// Density, gravity, or elevator input is non-finite or non-positive where required.
    InvalidInput,
    /// Pitching moment does not cross zero inside the available angle-of-attack table.
    PitchMomentNotBracketed,
    /// The zero-moment solution has non-positive lift or invalid drag.
    NonPhysicalForce,
}

/// Solves a wings-level, unpowered steady glide at a fixed elevator deflection.
///
/// The solution assumes zero angular rate, no thrust, and no ground effect. It
/// first finds `Cm(alpha, elevator) = 0`, then balances lift and drag against
/// weight. It does not claim that the resulting equilibrium is dynamically
/// stable or inside a flight-test-validated envelope.
///
/// # Errors
///
/// Returns [`TrimError`] when inputs are invalid, the pitching-moment root is
/// not bracketed by the aerodynamic table, or the resulting force coefficients
/// cannot represent an unpowered glide.
pub fn steady_glide_trim(
    model: AircraftModel<'_>,
    environment: Environment,
    elevator_rad: f64,
) -> Result<SteadyGlideTrim, TrimError> {
    if !environment.density_kg_m3.is_finite()
        || environment.density_kg_m3 <= 0.0
        || !environment.gravity_mps2.is_finite()
        || environment.gravity_mps2 <= 0.0
        || !elevator_rad.is_finite()
    {
        return Err(TrimError::InvalidInput);
    }
    let (mut lower_alpha, mut upper_alpha) = model.longitudinal.alpha_bounds();
    let mut lower_cm = pitch_coefficient(model, lower_alpha, elevator_rad)?;
    let upper_cm = pitch_coefficient(model, upper_alpha, elevator_rad)?;
    let alpha_rad = if lower_cm == 0.0 {
        lower_alpha
    } else if upper_cm == 0.0 {
        upper_alpha
    } else {
        if lower_cm.is_sign_positive() == upper_cm.is_sign_positive() {
            return Err(TrimError::PitchMomentNotBracketed);
        }
        for _ in 0..64 {
            let middle_alpha = (lower_alpha + upper_alpha) * 0.5;
            let middle_cm = pitch_coefficient(model, middle_alpha, elevator_rad)?;
            if lower_cm.is_sign_positive() == middle_cm.is_sign_positive() {
                lower_alpha = middle_alpha;
                lower_cm = middle_cm;
            } else {
                upper_alpha = middle_alpha;
            }
        }
        (lower_alpha + upper_alpha) * 0.5
    };

    let base = model
        .longitudinal
        .sample(alpha_rad)
        .map_err(|_| TrimError::InvalidInput)?;
    let lift = base.cl + model.derivatives.cl_elevator * elevator_rad;
    let drag = base.cd;
    if !lift.is_finite() || lift <= 0.0 || !drag.is_finite() || drag < 0.0 {
        return Err(TrimError::NonPhysicalForce);
    }
    let flight_path_rad = -libm::atan2(drag, lift);
    let required_lift_n = model.mass_kg * environment.gravity_mps2 * libm::cos(flight_path_rad);
    let airspeed_mps = libm::sqrt(
        2.0 * required_lift_n / (environment.density_kg_m3 * model.reference_area_m2 * lift),
    );
    Ok(SteadyGlideTrim {
        alpha_rad,
        flight_path_rad,
        pitch_rad: alpha_rad + flight_path_rad,
        airspeed_mps,
        elevator_rad,
        coefficients: AeroCoefficients {
            lift,
            drag,
            pitch: pitch_coefficient(model, alpha_rad, elevator_rad)?,
            ..AeroCoefficients::default()
        },
    })
}

fn pitch_coefficient(
    model: AircraftModel<'_>,
    alpha_rad: f64,
    elevator_rad: f64,
) -> Result<f64, TrimError> {
    let result = model
        .longitudinal
        .sample(alpha_rad)
        .map_err(|_| TrimError::InvalidInput)?
        .cm
        + model.derivatives.cm_elevator * elevator_rad;
    if result.is_finite() {
        Ok(result)
    } else {
        Err(TrimError::InvalidInput)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AeroDerivatives, AeroPoint, AeroTable, GroundEffectModel, Inertia, Vec3};

    fn model(points: &[AeroPoint]) -> AircraftModel<'_> {
        AircraftModel {
            force_coefficient_basis: crate::ForceCoefficientBasis::WindAxes,
            mass_kg: 100.0,
            inertia_kg_m2: Inertia {
                ixx: 10.0,
                iyy: 20.0,
                izz: 30.0,
                ixz: 0.0,
            },
            reference_area_m2: 20.0,
            reference_span_m: 20.0,
            reference_chord_m: 1.0,
            longitudinal: AeroTable::new(points).expect("valid table"),
            derivatives: AeroDerivatives::default(),
            ground_effect: GroundEffectModel::default(),
        }
    }

    #[test]
    fn solves_known_unpowered_equilibrium() {
        let points = [
            AeroPoint {
                alpha_rad: -0.1,
                cl: 0.8,
                cd: 0.04,
                cm: 0.1,
            },
            AeroPoint {
                alpha_rad: 0.1,
                cl: 1.2,
                cd: 0.06,
                cm: -0.1,
            },
        ];
        let environment = Environment {
            density_kg_m3: 1.0,
            gravity_mps2: 10.0,
            wind_ned_mps: Vec3::ZERO,
            ground_effect_enabled: false,
        };
        let trim = steady_glide_trim(model(&points), environment, 0.0).expect("trim exists");
        let expected_gamma = -libm::atan2(0.05, 1.0);
        let expected_speed = libm::sqrt(100.0 * libm::cos(expected_gamma));
        assert!(trim.alpha_rad.abs() < 1.0e-12);
        assert!((trim.flight_path_rad - expected_gamma).abs() < 1.0e-12);
        assert!((trim.airspeed_mps - expected_speed).abs() < 1.0e-12);
        assert!(trim.coefficients.pitch.abs() < 1.0e-12);
    }

    #[test]
    fn rejects_unbracketed_pitch_moment() {
        let points = [
            AeroPoint {
                alpha_rad: -0.1,
                cl: 1.0,
                cd: 0.05,
                cm: 0.2,
            },
            AeroPoint {
                alpha_rad: 0.1,
                cl: 1.0,
                cd: 0.05,
                cm: 0.1,
            },
        ];
        let result = steady_glide_trim(
            model(&points),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 10.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
            0.0,
        );
        assert_eq!(result, Err(TrimError::PitchMomentNotBracketed));
    }
}
