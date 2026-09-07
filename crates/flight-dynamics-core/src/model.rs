//! Validated aircraft model data.

use crate::math::Vec3;

/// A longitudinal aerodynamic data point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AeroPoint {
    /// Body angle of attack in radians.
    pub alpha_rad: f64,
    /// Lift coefficient.
    pub cl: f64,
    /// Drag coefficient.
    pub cd: f64,
    /// Pitching-moment coefficient about the configured reference point.
    pub cm: f64,
}

/// Errors detected before an aircraft model can be simulated.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModelError {
    /// Mass is zero, negative, or non-finite.
    InvalidMass,
    /// A reference geometry value is zero, negative, or non-finite.
    InvalidReferenceGeometry,
    /// The supported symmetric inertia tensor is not positive definite.
    InvalidInertia,
    /// At least two longitudinal aerodynamic points are required.
    AeroTableTooShort,
    /// Angle-of-attack breakpoints are not finite and strictly increasing.
    AeroAlphaNotStrictlyIncreasing,
    /// An aerodynamic coefficient is non-finite or drag is negative.
    InvalidAeroCoefficient,
    /// A ground-effect correlation parameter is outside its physical domain.
    InvalidGroundEffect,
}

/// Borrowed, validated longitudinal coefficient table.
#[derive(Clone, Copy, Debug)]
pub struct AeroTable<'a> {
    points: &'a [AeroPoint],
}

impl<'a> AeroTable<'a> {
    /// Validates and borrows a table.
    ///
    /// Values outside the breakpoint range are clamped to the nearest endpoint;
    /// this is intentional because uncontrolled extrapolation is unsafe. The
    /// caller must separately enforce the model's documented validity range.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelError`] when fewer than two points are supplied, a
    /// breakpoint is not strictly increasing, or a coefficient is invalid.
    pub fn new(points: &'a [AeroPoint]) -> Result<Self, ModelError> {
        if points.len() < 2 {
            return Err(ModelError::AeroTableTooShort);
        }
        for point in points {
            if !point.alpha_rad.is_finite()
                || !point.cl.is_finite()
                || !point.cd.is_finite()
                || !point.cm.is_finite()
                || point.cd < 0.0
            {
                return Err(ModelError::InvalidAeroCoefficient);
            }
        }
        if points
            .windows(2)
            .any(|pair| pair[0].alpha_rad >= pair[1].alpha_rad)
        {
            return Err(ModelError::AeroAlphaNotStrictlyIncreasing);
        }
        Ok(Self { points })
    }

    /// Interpolates coefficients linearly and clamps outside the table range.
    #[must_use]
    pub fn sample(self, alpha_rad: f64) -> AeroPoint {
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        if alpha_rad <= first.alpha_rad {
            return first;
        }
        if alpha_rad >= last.alpha_rad {
            return last;
        }
        let upper = self
            .points
            .partition_point(|point| point.alpha_rad < alpha_rad);
        let lower_point = self.points[upper - 1];
        let upper_point = self.points[upper];
        let fraction =
            (alpha_rad - lower_point.alpha_rad) / (upper_point.alpha_rad - lower_point.alpha_rad);
        AeroPoint {
            alpha_rad,
            cl: lerp(lower_point.cl, upper_point.cl, fraction),
            cd: lerp(lower_point.cd, upper_point.cd, fraction),
            cm: lerp(lower_point.cm, upper_point.cm, fraction),
        }
    }

    /// Reports whether an angle is inside the supplied coefficient range.
    #[must_use]
    pub fn contains(self, alpha_rad: f64) -> bool {
        alpha_rad.is_finite()
            && alpha_rad >= self.points[0].alpha_rad
            && alpha_rad <= self.points[self.points.len() - 1].alpha_rad
    }

    /// Returns the inclusive angle-of-attack bounds of the supplied table.
    #[must_use]
    pub fn alpha_bounds(self) -> (f64, f64) {
        (
            self.points[0].alpha_rad,
            self.points[self.points.len() - 1].alpha_rad,
        )
    }
}

const fn lerp(start: f64, end: f64, fraction: f64) -> f64 {
    start + (end - start) * fraction
}

/// Symmetric body-axis inertia tensor in kg m².
///
/// The prototype supports the matrix
/// `[[ixx, 0, ixz], [0, iyy, 0], [ixz, 0, izz]]`. `ixz` is the literal matrix
/// element rather than the negated product-of-inertia convention used by some
/// aeronautical texts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertia {
    /// Roll-axis tensor element.
    pub ixx: f64,
    /// Pitch-axis tensor element.
    pub iyy: f64,
    /// Yaw-axis tensor element.
    pub izz: f64,
    /// Symmetric x-z tensor element.
    pub ixz: f64,
}

impl Inertia {
    /// Multiplies the tensor by a body-rate vector.
    #[must_use]
    pub const fn multiply(self, rates: Vec3) -> Vec3 {
        Vec3::new(
            self.ixx * rates.x + self.ixz * rates.z,
            self.iyy * rates.y,
            self.ixz * rates.x + self.izz * rates.z,
        )
    }

    /// Solves `I * x = rhs` for the supported symmetric tensor.
    #[must_use]
    pub fn solve(self, rhs: Vec3) -> Vec3 {
        let determinant_xz = self.ixx * self.izz - self.ixz * self.ixz;
        Vec3::new(
            (self.izz * rhs.x - self.ixz * rhs.z) / determinant_xz,
            rhs.y / self.iyy,
            (-self.ixz * rhs.x + self.ixx * rhs.z) / determinant_xz,
        )
    }

    const fn is_positive_definite(self) -> bool {
        self.ixx > 0.0
            && self.iyy > 0.0
            && self.izz > 0.0
            && self.ixx * self.izz - self.ixz * self.ixz > 0.0
    }
}

/// Linearized lateral/directional and control derivatives, all per radian.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AeroDerivatives {
    /// Lift change from elevator deflection.
    pub cl_elevator: f64,
    /// Pitching-moment change from elevator deflection.
    pub cm_elevator: f64,
    /// Pitch damping derivative.
    pub cm_pitch_rate: f64,
    /// Side-force derivative with sideslip.
    pub cy_beta: f64,
    /// Side-force derivative with normalized roll rate.
    pub cy_roll_rate: f64,
    /// Side-force derivative with normalized yaw rate.
    pub cy_yaw_rate: f64,
    /// Side-force derivative with rudder deflection.
    pub cy_rudder: f64,
    /// Rolling-moment derivative with sideslip.
    pub c_roll_beta: f64,
    /// Roll damping derivative.
    pub c_roll_roll_rate: f64,
    /// Rolling-moment derivative with normalized yaw rate.
    pub c_roll_yaw_rate: f64,
    /// Rolling-moment derivative with rudder deflection.
    pub c_roll_rudder: f64,
    /// Yawing-moment derivative with sideslip.
    pub cn_beta: f64,
    /// Yawing-moment derivative with normalized roll rate.
    pub cn_roll_rate: f64,
    /// Yaw damping derivative.
    pub cn_yaw_rate: f64,
    /// Yawing-moment derivative with rudder deflection.
    pub cn_rudder: f64,
}

/// Aircraft-specific induced-drag ground-effect correlation.
///
/// The free-air coefficient table is corrected by
/// `CD = CD_free + (ratio - 1) * induced_drag_factor * CL^2`. The ratio uses
/// `(minimum_ratio + gain * (h/b)^exponent) / (1 + gain * (h/b)^exponent)`.
/// This deliberately does not claim lift, downwash, or pitching-moment changes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundEffectModel {
    /// Enables the aircraft correlation when the environment also enables it.
    pub enabled: bool,
    /// Wing-height offset above the state reference point, metres.
    pub wing_height_offset_m: f64,
    /// Free-air induced-drag factor, normally `1 / (pi * e * AR)`.
    pub induced_drag_factor: f64,
    /// Limiting induced-drag ratio at zero wing height.
    pub minimum_induced_drag_ratio: f64,
    /// Correlation multiplier.
    pub correlation_gain: f64,
    /// Exponent applied to height/span.
    pub height_exponent: f64,
}

impl GroundEffectModel {
    /// Returns the induced-drag ratio for the current wing height.
    #[must_use]
    pub fn induced_drag_ratio(self, altitude_m: f64, span_m: f64, active: bool) -> f64 {
        if !self.enabled || !active {
            return 1.0;
        }
        let wing_height_m = (altitude_m + self.wing_height_offset_m).max(0.0);
        let height_to_span = wing_height_m / span_m;
        let scaled_height = self.correlation_gain * libm::pow(height_to_span, self.height_exponent);
        (self.minimum_induced_drag_ratio + scaled_height) / (1.0 + scaled_height)
    }
}

impl Default for GroundEffectModel {
    fn default() -> Self {
        Self {
            enabled: false,
            wing_height_offset_m: 0.0,
            induced_drag_factor: 0.0,
            minimum_induced_drag_ratio: 1.0,
            correlation_gain: 1.0,
            height_exponent: 1.0,
        }
    }
}

/// Coordinate contract for aerodynamic force coefficients.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ForceCoefficientBasis {
    /// CL, CD, CY resolved in the orthogonal wind frame.
    WindAxes,
    /// CL/CD in the longitudinal stability plane; CY is total body-axis side force.
    /// Explicit small-sideslip approximation used by the source BR model.
    StabilityLiftDragBodySide,
}

/// Validated rigid-aircraft and aerodynamic model.
#[derive(Clone, Copy, Debug)]
pub struct AircraftModel<'a> {
    /// Coordinate definition of supplied force coefficients.
    pub force_coefficient_basis: ForceCoefficientBasis,
    /// Total mass in kilograms.
    pub mass_kg: f64,
    /// Body-axis inertia tensor.
    pub inertia_kg_m2: Inertia,
    /// Aerodynamic reference area in square metres.
    pub reference_area_m2: f64,
    /// Aerodynamic reference span in metres.
    pub reference_span_m: f64,
    /// Aerodynamic reference chord in metres.
    pub reference_chord_m: f64,
    /// Longitudinal coefficient table.
    pub longitudinal: AeroTable<'a>,
    /// Linearized rate, sideslip, and control derivatives.
    pub derivatives: AeroDerivatives,
    /// Optional aircraft-specific induced-drag ground-effect model.
    pub ground_effect: GroundEffectModel,
}

impl AircraftModel<'_> {
    /// Validates all scalar mass and geometry fields.
    ///
    /// # Errors
    ///
    /// Returns a [`ModelError`] when mass/reference dimensions are not finite
    /// positive values, or when the inertia tensor is not positive definite.
    pub fn validate(self) -> Result<(), ModelError> {
        if !self.mass_kg.is_finite() || self.mass_kg <= 0.0 {
            return Err(ModelError::InvalidMass);
        }
        if [
            self.reference_area_m2,
            self.reference_span_m,
            self.reference_chord_m,
        ]
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(ModelError::InvalidReferenceGeometry);
        }
        let inertia = self.inertia_kg_m2;
        if ![inertia.ixx, inertia.iyy, inertia.izz, inertia.ixz]
            .iter()
            .all(|value| value.is_finite())
            || !inertia.is_positive_definite()
        {
            return Err(ModelError::InvalidInertia);
        }
        let derivatives = self.derivatives;
        if [
            derivatives.cl_elevator,
            derivatives.cm_elevator,
            derivatives.cm_pitch_rate,
            derivatives.cy_beta,
            derivatives.cy_roll_rate,
            derivatives.cy_yaw_rate,
            derivatives.cy_rudder,
            derivatives.c_roll_beta,
            derivatives.c_roll_roll_rate,
            derivatives.c_roll_yaw_rate,
            derivatives.c_roll_rudder,
            derivatives.cn_beta,
            derivatives.cn_roll_rate,
            derivatives.cn_yaw_rate,
            derivatives.cn_rudder,
        ]
        .iter()
        .any(|value| !value.is_finite())
        {
            return Err(ModelError::InvalidAeroCoefficient);
        }
        let ground_effect = self.ground_effect;
        if !ground_effect.wing_height_offset_m.is_finite()
            || !ground_effect.induced_drag_factor.is_finite()
            || !ground_effect.minimum_induced_drag_ratio.is_finite()
            || !ground_effect.correlation_gain.is_finite()
            || !ground_effect.height_exponent.is_finite()
            || (ground_effect.enabled
                && (ground_effect.induced_drag_factor <= 0.0
                    || !(0.0..=1.0).contains(&ground_effect.minimum_induced_drag_ratio)
                    || ground_effect.correlation_gain <= 0.0
                    || ground_effect.height_exponent <= 0.0))
        {
            return Err(ModelError::InvalidGroundEffect);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{AeroPoint, AeroTable, ModelError};

    #[test]
    fn interpolation_and_endpoint_clamping_are_explicit() {
        let points = [
            AeroPoint {
                alpha_rad: 0.0,
                cl: 0.0,
                cd: 0.01,
                cm: 0.1,
            },
            AeroPoint {
                alpha_rad: 1.0,
                cl: 2.0,
                cd: 0.03,
                cm: -0.1,
            },
        ];
        let table = AeroTable::new(&points).expect("valid table");
        assert_eq!(table.sample(-1.0), points[0]);
        assert_eq!(table.sample(2.0), points[1]);
        assert_eq!(table.sample(0.25).cl, 0.5);
    }

    #[test]
    fn duplicate_breakpoint_is_rejected() {
        let point = AeroPoint {
            alpha_rad: 0.0,
            cl: 0.0,
            cd: 0.01,
            cm: 0.0,
        };
        assert!(matches!(
            AeroTable::new(&[point, point]),
            Err(ModelError::AeroAlphaNotStrictlyIncreasing)
        ));
    }
}
