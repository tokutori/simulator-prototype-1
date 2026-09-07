//! Nonlinear rigid-body equations of motion and fixed-step RK4 integration.

use crate::{AircraftModel, Quaternion, Vec3};

const MIN_AIRSPEED_MPS: f64 = 1.0e-6;

/// Rigid-body state using body velocity and a body-to-NED attitude quaternion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBodyState {
    /// Position in NED coordinates, metres.
    pub position_ned_m: Vec3,
    /// Translational velocity resolved in body axes, metres per second.
    pub velocity_body_mps: Vec3,
    /// Attitude rotating body vectors into NED.
    pub attitude_body_to_ned: Quaternion,
    /// Body roll, pitch, and yaw rates, radians per second.
    pub rates_body_rad_s: Vec3,
}

/// Elevator and rudder positions in radians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlSurfaceDeflection {
    /// Elevator deflection. Positive sign follows the supplied coefficient data.
    pub elevator_rad: f64,
    /// Rudder deflection. Positive sign follows the supplied coefficient data.
    pub rudder_rad: f64,
}

/// Atmosphere and gravity inputs for one deterministic step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Environment {
    /// Air density in kilograms per cubic metre.
    pub density_kg_m3: f64,
    /// Gravitational acceleration magnitude in metres per second squared.
    pub gravity_mps2: f64,
    /// Wind velocity in NED coordinates, metres per second.
    pub wind_ned_mps: Vec3,
    /// Applies the aircraft's ground-effect correlation for this step.
    pub ground_effect_enabled: bool,
}

/// Air-relative state derived at the centre of gravity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlightCondition {
    /// True airspeed magnitude.
    pub airspeed_mps: f64,
    /// Body angle of attack.
    pub alpha_rad: f64,
    /// Body sideslip angle.
    pub beta_rad: f64,
    /// Dynamic pressure in pascals.
    pub dynamic_pressure_pa: f64,
}

/// Dimensionless aerodynamic coefficients.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AeroCoefficients {
    /// Lift coefficient.
    pub lift: f64,
    /// Drag coefficient.
    pub drag: f64,
    /// Side-force coefficient.
    pub side: f64,
    /// Rolling-moment coefficient.
    pub roll: f64,
    /// Pitching-moment coefficient.
    pub pitch: f64,
    /// Yawing-moment coefficient.
    pub yaw: f64,
}

/// Aerodynamic loads about the model reference point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AeroLoads {
    /// Force resolved in body axes, newtons.
    pub force_body_n: Vec3,
    /// Moment resolved in body axes, newton metres.
    pub moment_body_nm: Vec3,
    /// Condition used to evaluate the coefficients.
    pub condition: FlightCondition,
    /// Evaluated coefficients.
    pub coefficients: AeroCoefficients,
    /// Ratio of in-ground-effect to free-air induced drag.
    pub induced_drag_ground_effect_ratio: f64,
}

/// Invalid integration inputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepError {
    /// Time step is not finite and strictly positive.
    InvalidTimeStep,
    /// Density or gravity is negative or non-finite.
    InvalidEnvironment,
}

#[derive(Clone, Copy)]
struct StateDerivative {
    position_ned_m_s: Vec3,
    velocity_body_mps2: Vec3,
    attitude_rate: Quaternion,
    angular_acceleration_rad_s2: Vec3,
}

/// Calculates aerodynamic loads for a state without advancing time.
#[must_use]
pub fn aerodynamic_loads(
    model: AircraftModel<'_>,
    state: RigidBodyState,
    controls: ControlSurfaceDeflection,
    environment: Environment,
) -> AeroLoads {
    let wind_body_mps = state
        .attitude_body_to_ned
        .rotate_ned_to_body(environment.wind_ned_mps);
    let relative_velocity = state.velocity_body_mps - wind_body_mps;
    let airspeed = relative_velocity.norm();
    if airspeed < MIN_AIRSPEED_MPS {
        return AeroLoads::default();
    }

    let alpha = libm::atan2(relative_velocity.z, relative_velocity.x);
    let beta = libm::asin((relative_velocity.y / airspeed).clamp(-1.0, 1.0));
    let dynamic_pressure = 0.5 * environment.density_kg_m3 * airspeed * airspeed;
    let base = model.longitudinal.sample(alpha);
    let normalized_roll_rate = state.rates_body_rad_s.x * model.reference_span_m / (2.0 * airspeed);
    let normalized_pitch_rate =
        state.rates_body_rad_s.y * model.reference_chord_m / (2.0 * airspeed);
    let normalized_yaw_rate = state.rates_body_rad_s.z * model.reference_span_m / (2.0 * airspeed);
    let derivatives = model.derivatives;
    let lift = base.cl + derivatives.cl_elevator * controls.elevator_rad;
    let ground_effect_ratio = model.ground_effect.induced_drag_ratio(
        -state.position_ned_m.z,
        model.reference_span_m,
        environment.ground_effect_enabled,
    );
    let drag = base.cd
        + (ground_effect_ratio - 1.0) * model.ground_effect.induced_drag_factor * lift * lift;
    let coefficients = AeroCoefficients {
        lift,
        drag,
        pitch: base.cm
            + derivatives.cm_pitch_rate * normalized_pitch_rate
            + derivatives.cm_elevator * controls.elevator_rad,
        side: derivatives.cy_beta * beta
            + derivatives.cy_roll_rate * normalized_roll_rate
            + derivatives.cy_yaw_rate * normalized_yaw_rate
            + derivatives.cy_rudder * controls.rudder_rad,
        roll: derivatives.c_roll_beta * beta
            + derivatives.c_roll_roll_rate * normalized_roll_rate
            + derivatives.c_roll_yaw_rate * normalized_yaw_rate
            + derivatives.c_roll_rudder * controls.rudder_rad,
        yaw: derivatives.cn_beta * beta
            + derivatives.cn_roll_rate * normalized_roll_rate
            + derivatives.cn_yaw_rate * normalized_yaw_rate
            + derivatives.cn_rudder * controls.rudder_rad,
    };
    let sin_alpha = libm::sin(alpha);
    let cos_alpha = libm::cos(alpha);
    let scale = dynamic_pressure * model.reference_area_m2;
    let (longitudinal_drag, side) = match model.force_coefficient_basis {
        crate::ForceCoefficientBasis::WindAxes => {
            let sin_beta = libm::sin(beta);
            let cos_beta = libm::cos(beta);
            (
                coefficients.drag * cos_beta + coefficients.side * sin_beta,
                -coefficients.drag * sin_beta + coefficients.side * cos_beta,
            )
        }
        crate::ForceCoefficientBasis::StabilityLiftDragBodySide => {
            (coefficients.drag, coefficients.side)
        }
    };
    let force_body_n = Vec3::new(
        scale * (coefficients.lift * sin_alpha - longitudinal_drag * cos_alpha),
        scale * side,
        scale * (-coefficients.lift * cos_alpha - longitudinal_drag * sin_alpha),
    );
    let moment_body_nm = Vec3::new(
        scale * model.reference_span_m * coefficients.roll,
        scale * model.reference_chord_m * coefficients.pitch,
        scale * model.reference_span_m * coefficients.yaw,
    );
    AeroLoads {
        force_body_n,
        moment_body_nm,
        condition: FlightCondition {
            airspeed_mps: airspeed,
            alpha_rad: alpha,
            beta_rad: beta,
            dynamic_pressure_pa: dynamic_pressure,
        },
        coefficients,
        induced_drag_ground_effect_ratio: ground_effect_ratio,
    }
}

/// Advances the nonlinear rigid-body state by one fixed RK4 step.
///
/// The controls and environment are held constant during the step. Model
/// validation is deliberately separate so callers can validate once before a
/// batch rather than paying for it in every integration step.
///
/// # Errors
///
/// Returns [`StepError::InvalidTimeStep`] for a non-positive/non-finite step,
/// or [`StepError::InvalidEnvironment`] for invalid density, gravity, or wind.
pub fn step_rk4(
    model: AircraftModel<'_>,
    state: RigidBodyState,
    controls: ControlSurfaceDeflection,
    environment: Environment,
    dt_s: f64,
) -> Result<RigidBodyState, StepError> {
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(StepError::InvalidTimeStep);
    }
    if !environment.density_kg_m3.is_finite()
        || environment.density_kg_m3 < 0.0
        || !environment.gravity_mps2.is_finite()
        || environment.gravity_mps2 < 0.0
        || !environment.wind_ned_mps.x.is_finite()
        || !environment.wind_ned_mps.y.is_finite()
        || !environment.wind_ned_mps.z.is_finite()
    {
        return Err(StepError::InvalidEnvironment);
    }

    let k1 = derivative(model, state, controls, environment);
    let k2 = derivative(model, advance(state, k1, dt_s * 0.5), controls, environment);
    let k3 = derivative(model, advance(state, k2, dt_s * 0.5), controls, environment);
    let k4 = derivative(model, advance(state, k3, dt_s), controls, environment);
    let weighted = StateDerivative {
        position_ned_m_s: (k1.position_ned_m_s
            + k2.position_ned_m_s * 2.0
            + k3.position_ned_m_s * 2.0
            + k4.position_ned_m_s)
            / 6.0,
        velocity_body_mps2: (k1.velocity_body_mps2
            + k2.velocity_body_mps2 * 2.0
            + k3.velocity_body_mps2 * 2.0
            + k4.velocity_body_mps2)
            / 6.0,
        attitude_rate: (k1.attitude_rate
            + k2.attitude_rate * 2.0
            + k3.attitude_rate * 2.0
            + k4.attitude_rate)
            * (1.0 / 6.0),
        angular_acceleration_rad_s2: (k1.angular_acceleration_rad_s2
            + k2.angular_acceleration_rad_s2 * 2.0
            + k3.angular_acceleration_rad_s2 * 2.0
            + k4.angular_acceleration_rad_s2)
            / 6.0,
    };
    Ok(advance(state, weighted, dt_s))
}

fn derivative(
    model: AircraftModel<'_>,
    state: RigidBodyState,
    controls: ControlSurfaceDeflection,
    environment: Environment,
) -> StateDerivative {
    let loads = aerodynamic_loads(model, state, controls, environment);
    let gravity_body = state.attitude_body_to_ned.rotate_ned_to_body(Vec3::new(
        0.0,
        0.0,
        environment.gravity_mps2,
    ));
    let translational_acceleration = loads.force_body_n / model.mass_kg + gravity_body
        - state.rates_body_rad_s.cross(state.velocity_body_mps);
    let angular_momentum = model.inertia_kg_m2.multiply(state.rates_body_rad_s);
    let angular_acceleration = model
        .inertia_kg_m2
        .solve(loads.moment_body_nm - state.rates_body_rad_s.cross(angular_momentum));
    let omega = Quaternion::new(
        0.0,
        state.rates_body_rad_s.x,
        state.rates_body_rad_s.y,
        state.rates_body_rad_s.z,
    );
    StateDerivative {
        position_ned_m_s: state
            .attitude_body_to_ned
            .rotate_body_to_ned(state.velocity_body_mps),
        velocity_body_mps2: translational_acceleration,
        attitude_rate: state.attitude_body_to_ned.product(omega) * 0.5,
        angular_acceleration_rad_s2: angular_acceleration,
    }
}

fn advance(state: RigidBodyState, derivative: StateDerivative, dt_s: f64) -> RigidBodyState {
    RigidBodyState {
        position_ned_m: state.position_ned_m + derivative.position_ned_m_s * dt_s,
        velocity_body_mps: state.velocity_body_mps + derivative.velocity_body_mps2 * dt_s,
        attitude_body_to_ned: (state.attitude_body_to_ned + derivative.attitude_rate * dt_s)
            .normalized(),
        rates_body_rad_s: state.rates_body_rad_s + derivative.angular_acceleration_rad_s2 * dt_s,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ControlSurfaceDeflection, Environment, RigidBodyState, aerodynamic_loads, step_rk4,
    };
    use crate::{
        AeroDerivatives, AeroPoint, AeroTable, AircraftModel, GroundEffectModel, Inertia,
        Quaternion, Vec3,
    };

    fn model(points: &[AeroPoint]) -> AircraftModel<'_> {
        AircraftModel {
            force_coefficient_basis: crate::ForceCoefficientBasis::WindAxes,
            mass_kg: 10.0,
            inertia_kg_m2: Inertia {
                ixx: 2.0,
                iyy: 3.0,
                izz: 4.0,
                ixz: 0.0,
            },
            reference_area_m2: 2.0,
            reference_span_m: 3.0,
            reference_chord_m: 1.0,
            longitudinal: AeroTable::new(points).expect("valid table"),
            derivatives: AeroDerivatives::default(),
            ground_effect: GroundEffectModel::default(),
        }
    }

    #[test]
    fn wind_axis_drag_opposes_velocity_at_nonzero_alpha_and_beta() {
        let points = [
            AeroPoint {
                alpha_rad: -1.0,
                cl: 0.0,
                cd: 0.1,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 1.0,
                cl: 0.0,
                cd: 0.1,
                cm: 0.0,
            },
        ];
        for velocity in [Vec3::new(8.0, 5.0, 3.0), Vec3::new(8.0, -5.0, -3.0)] {
            let aircraft = model(&points);
            let loads = aerodynamic_loads(
                aircraft,
                RigidBodyState {
                    position_ned_m: Vec3::ZERO,
                    velocity_body_mps: velocity,
                    attitude_body_to_ned: Quaternion::IDENTITY,
                    rates_body_rad_s: Vec3::ZERO,
                },
                ControlSurfaceDeflection::default(),
                Environment {
                    density_kg_m3: 1.0,
                    gravity_mps2: 0.0,
                    wind_ned_mps: Vec3::ZERO,
                    ground_effect_enabled: false,
                },
            );
            let expected = velocity * (-0.5 * velocity.norm() * aircraft.reference_area_m2 * 0.1);
            assert!((loads.force_body_n - expected).norm() < 1.0e-12);
        }
    }

    #[test]
    fn declared_body_side_derivatives_are_not_rotated_or_counted_twice() {
        let points = [
            AeroPoint {
                alpha_rad: -1.0,
                cl: 0.0,
                cd: 0.1,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 1.0,
                cl: 0.0,
                cd: 0.1,
                cm: 0.0,
            },
        ];
        let mut aircraft = model(&points);
        aircraft.force_coefficient_basis = crate::ForceCoefficientBasis::StabilityLiftDragBodySide;
        aircraft.derivatives.cy_beta = -0.2;
        let velocity = Vec3::new(8.0, 5.0, 3.0);
        let loads = aerodynamic_loads(
            aircraft,
            RigidBodyState {
                position_ned_m: Vec3::ZERO,
                velocity_body_mps: velocity,
                attitude_body_to_ned: Quaternion::IDENTITY,
                rates_body_rad_s: Vec3::ZERO,
            },
            ControlSurfaceDeflection::default(),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 0.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
        );
        let scale = 0.5 * velocity.norm() * velocity.norm() * aircraft.reference_area_m2;
        assert!(
            (loads.force_body_n.y - scale * -0.2 * libm::asin(5.0 / velocity.norm())).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn zero_airspeed_freefall_matches_constant_gravity() {
        let points = [
            AeroPoint {
                alpha_rad: -1.0,
                cl: 0.0,
                cd: 0.0,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 1.0,
                cl: 0.0,
                cd: 0.0,
                cm: 0.0,
            },
        ];
        let state = RigidBodyState {
            position_ned_m: Vec3::ZERO,
            velocity_body_mps: Vec3::ZERO,
            attitude_body_to_ned: Quaternion::IDENTITY,
            rates_body_rad_s: Vec3::ZERO,
        };
        let next = step_rk4(
            model(&points),
            state,
            ControlSurfaceDeflection::default(),
            Environment {
                density_kg_m3: 1.225,
                gravity_mps2: 9.81,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
            0.1,
        )
        .expect("valid step");
        assert!((next.velocity_body_mps.z - 0.981).abs() < 1.0e-12);
        assert!((next.position_ned_m.z - 0.049_05).abs() < 1.0e-12);
    }

    #[test]
    fn aerodynamic_force_uses_forward_right_down_signs() {
        let points = [
            AeroPoint {
                alpha_rad: -0.1,
                cl: 1.0,
                cd: 0.1,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 0.1,
                cl: 1.0,
                cd: 0.1,
                cm: 0.0,
            },
        ];
        let loads = aerodynamic_loads(
            model(&points),
            RigidBodyState {
                position_ned_m: Vec3::ZERO,
                velocity_body_mps: Vec3::new(10.0, 0.0, 0.0),
                attitude_body_to_ned: Quaternion::IDENTITY,
                rates_body_rad_s: Vec3::ZERO,
            },
            ControlSurfaceDeflection::default(),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 0.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
        );
        assert_eq!(loads.force_body_n, Vec3::new(-10.0, 0.0, -100.0));
    }

    #[test]
    fn ground_effect_changes_only_the_induced_drag_component() {
        let points = [
            AeroPoint {
                alpha_rad: -0.1,
                cl: 1.0,
                cd: 0.1,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 0.1,
                cl: 1.0,
                cd: 0.1,
                cm: 0.0,
            },
        ];
        let mut aircraft = model(&points);
        aircraft.ground_effect = GroundEffectModel {
            enabled: true,
            wing_height_offset_m: 0.0,
            induced_drag_factor: 0.05,
            minimum_induced_drag_ratio: 0.2,
            correlation_gain: 33.0,
            height_exponent: 1.5,
        };
        let loads = aerodynamic_loads(
            aircraft,
            RigidBodyState {
                position_ned_m: Vec3::ZERO,
                velocity_body_mps: Vec3::new(10.0, 0.0, 0.0),
                attitude_body_to_ned: Quaternion::IDENTITY,
                rates_body_rad_s: Vec3::ZERO,
            },
            ControlSurfaceDeflection::default(),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 0.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: true,
            },
        );
        assert!((loads.induced_drag_ground_effect_ratio - 0.2).abs() < 1.0e-12);
        assert!((loads.coefficients.drag - 0.06).abs() < 1.0e-12);
        assert!((loads.force_body_n.x + 6.0).abs() < 1.0e-12);
        assert_eq!(loads.force_body_n.z, -100.0);
    }

    #[test]
    fn quaternion_remains_normalized_after_rotation() {
        let points = [
            AeroPoint {
                alpha_rad: -1.0,
                cl: 0.0,
                cd: 0.0,
                cm: 0.0,
            },
            AeroPoint {
                alpha_rad: 1.0,
                cl: 0.0,
                cd: 0.0,
                cm: 0.0,
            },
        ];
        let next = step_rk4(
            model(&points),
            RigidBodyState {
                position_ned_m: Vec3::ZERO,
                velocity_body_mps: Vec3::ZERO,
                attitude_body_to_ned: Quaternion::IDENTITY,
                rates_body_rad_s: Vec3::new(0.1, -0.2, 0.3),
            },
            ControlSurfaceDeflection::default(),
            Environment {
                density_kg_m3: 0.0,
                gravity_mps2: 0.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
            0.01,
        )
        .expect("valid step");
        let q = next.attitude_body_to_ned;
        let norm = libm::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        assert!((norm - 1.0).abs() < 1.0e-12);
    }
}
