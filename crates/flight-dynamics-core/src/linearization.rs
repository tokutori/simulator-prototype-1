//! Allocation-free longitudinal linearization around a steady glide.

use crate::{
    AircraftModel, ControlSurfaceDeflection, Environment, Quaternion, RigidBodyState,
    SteadyGlideTrim, TrimError, Vec3, aerodynamic_loads, steady_glide_trim,
};

const VELOCITY_STEP_MPS: f64 = 1.0e-3;
const RATE_STEP_RAD_S: f64 = 1.0e-5;
const ANGLE_STEP_RAD: f64 = 1.0e-5;
const CONTROL_STEP_RAD: f64 = 1.0e-5;

/// State order used by [`LongitudinalLinearization`]: `[u, w, q, theta]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalState {
    /// Forward body-axis velocity, metres per second.
    pub forward_velocity_mps: f64,
    /// Down body-axis velocity, metres per second.
    pub down_velocity_mps: f64,
    /// Body pitch rate, radians per second.
    pub pitch_rate_rad_s: f64,
    /// Pitch attitude in the NED frame, radians.
    pub pitch_rad: f64,
}

/// Time derivative order: `[u_dot, w_dot, q_dot, theta_dot]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalStateDerivative {
    /// Forward acceleration, metres per second squared.
    pub forward_acceleration_mps2: f64,
    /// Down acceleration, metres per second squared.
    pub down_acceleration_mps2: f64,
    /// Pitch angular acceleration, radians per second squared.
    pub pitch_acceleration_rad_s2: f64,
    /// Pitch attitude rate, radians per second.
    pub pitch_rate_rad_s: f64,
}

/// Finite-difference linear model `delta_x_dot = A delta_x + B delta_elevator`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LongitudinalLinearization {
    /// Steady-glide operating point used for the linearization.
    pub trim: SteadyGlideTrim,
    /// Absolute trim state in `[u, w, q, theta]` coordinates.
    pub trim_state: LongitudinalState,
    /// Nonlinear residual at the nominal trim point.
    pub trim_residual: LongitudinalStateDerivative,
    /// State Jacobian, with rows and columns ordered `[u, w, q, theta]`.
    pub state_matrix: [[f64; 4]; 4],
    /// Elevator input Jacobian, with output order `[u_dot, w_dot, q_dot, theta_dot]`.
    pub elevator_input: [f64; 4],
}

/// Failure to construct a steady-glide longitudinal linear model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinearizationError {
    /// Numerical dynamics rejected a non-finite state or load.
    Dynamics(crate::StepError),
    /// The steady-glide trim solve failed.
    Trim(TrimError),
    /// The trim helper is defined only for a still-air operating point.
    NonZeroWind,
}

/// Linearizes the nonlinear longitudinal equations around a steady unpowered glide.
///
/// The returned fixed-size matrices are allocation-free and use central finite
/// differences. The operating point is wings-level in still air; gust and wind
/// linearizations must use a separately defined air-relative equilibrium.
///
/// # Errors
///
/// Returns [`LinearizationError::NonZeroWind`] when wind is non-zero, or wraps
/// a [`TrimError`] if no physical steady glide exists at the requested elevator.
pub fn linearize_steady_glide(
    model: AircraftModel<'_>,
    environment: Environment,
    elevator_rad: f64,
) -> Result<LongitudinalLinearization, LinearizationError> {
    if environment.wind_ned_mps != Vec3::ZERO {
        return Err(LinearizationError::NonZeroWind);
    }
    let trim =
        steady_glide_trim(model, environment, elevator_rad).map_err(LinearizationError::Trim)?;
    let trim_state = LongitudinalState {
        forward_velocity_mps: trim.airspeed_mps * libm::cos(trim.alpha_rad),
        down_velocity_mps: trim.airspeed_mps * libm::sin(trim.alpha_rad),
        pitch_rate_rad_s: 0.0,
        pitch_rad: trim.pitch_rad,
    };
    let trim_residual = longitudinal_derivative(
        model,
        environment,
        trim_state,
        ControlSurfaceDeflection {
            elevator_rad,
            rudder_rad: 0.0,
        },
    );
    let trim_residual = trim_residual.map_err(LinearizationError::Dynamics)?;
    let steps = [
        VELOCITY_STEP_MPS,
        VELOCITY_STEP_MPS,
        RATE_STEP_RAD_S,
        ANGLE_STEP_RAD,
    ];
    let mut state_matrix = [[0.0; 4]; 4];
    for (column, step) in steps.iter().copied().enumerate() {
        let plus = perturb(trim_state, column, step);
        let minus = perturb(trim_state, column, -step);
        let plus_derivative = longitudinal_derivative(
            model,
            environment,
            plus,
            ControlSurfaceDeflection {
                elevator_rad,
                rudder_rad: 0.0,
            },
        );
        let minus_derivative = longitudinal_derivative(
            model,
            environment,
            minus,
            ControlSurfaceDeflection {
                elevator_rad,
                rudder_rad: 0.0,
            },
        );
        let column_values = derivative_array(
            plus_derivative.map_err(LinearizationError::Dynamics)?,
            minus_derivative.map_err(LinearizationError::Dynamics)?,
            step,
        );
        for row in 0..4 {
            state_matrix[row][column] = column_values[row];
        }
    }
    let plus_control = longitudinal_derivative(
        model,
        environment,
        trim_state,
        ControlSurfaceDeflection {
            elevator_rad: elevator_rad + CONTROL_STEP_RAD,
            rudder_rad: 0.0,
        },
    );
    let minus_control = longitudinal_derivative(
        model,
        environment,
        trim_state,
        ControlSurfaceDeflection {
            elevator_rad: elevator_rad - CONTROL_STEP_RAD,
            rudder_rad: 0.0,
        },
    );
    Ok(LongitudinalLinearization {
        trim,
        trim_state,
        trim_residual,
        state_matrix,
        elevator_input: derivative_array(
            plus_control.map_err(LinearizationError::Dynamics)?,
            minus_control.map_err(LinearizationError::Dynamics)?,
            CONTROL_STEP_RAD,
        ),
    })
}

fn longitudinal_derivative(
    model: AircraftModel<'_>,
    environment: Environment,
    state: LongitudinalState,
    controls: ControlSurfaceDeflection,
) -> Result<LongitudinalStateDerivative, crate::StepError> {
    let rigid_state = RigidBodyState {
        position_ned_m: Vec3::ZERO,
        velocity_body_mps: Vec3::new(state.forward_velocity_mps, 0.0, state.down_velocity_mps),
        attitude_body_to_ned: Quaternion::from_euler(0.0, state.pitch_rad, 0.0),
        rates_body_rad_s: Vec3::new(0.0, state.pitch_rate_rad_s, 0.0),
    };
    let loads = aerodynamic_loads(model, rigid_state, controls, environment)?;
    let gravity_body = rigid_state
        .attitude_body_to_ned
        .rotate_ned_to_body(Vec3::new(0.0, 0.0, environment.gravity_mps2));
    let acceleration = loads.force_body_n / model.mass_kg + gravity_body
        - rigid_state
            .rates_body_rad_s
            .cross(rigid_state.velocity_body_mps);
    Ok(LongitudinalStateDerivative {
        forward_acceleration_mps2: acceleration.x,
        down_acceleration_mps2: acceleration.z,
        pitch_acceleration_rad_s2: loads.moment_body_nm.y / model.inertia_kg_m2.iyy,
        pitch_rate_rad_s: state.pitch_rate_rad_s,
    })
}

fn perturb(state: LongitudinalState, index: usize, delta: f64) -> LongitudinalState {
    let mut values = [
        state.forward_velocity_mps,
        state.down_velocity_mps,
        state.pitch_rate_rad_s,
        state.pitch_rad,
    ];
    values[index] += delta;
    LongitudinalState {
        forward_velocity_mps: values[0],
        down_velocity_mps: values[1],
        pitch_rate_rad_s: values[2],
        pitch_rad: values[3],
    }
}

fn derivative_array(
    plus: LongitudinalStateDerivative,
    minus: LongitudinalStateDerivative,
    half_width: f64,
) -> [f64; 4] {
    let denominator = 2.0 * half_width;
    [
        (plus.forward_acceleration_mps2 - minus.forward_acceleration_mps2) / denominator,
        (plus.down_acceleration_mps2 - minus.down_acceleration_mps2) / denominator,
        (plus.pitch_acceleration_rad_s2 - minus.pitch_acceleration_rad_s2) / denominator,
        (plus.pitch_rate_rad_s - minus.pitch_rate_rad_s) / denominator,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AeroDerivatives, AeroPoint, AeroTable, GroundEffectModel, Inertia};

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
            derivatives: AeroDerivatives {
                cl_elevator: 0.2,
                cm_elevator: -1.0,
                cm_pitch_rate: -5.0,
                ..AeroDerivatives::default()
            },
            ground_effect: GroundEffectModel::default(),
        }
    }

    #[test]
    fn steady_glide_has_small_residual_and_exact_pitch_kinematics() {
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
        let linear = linearize_steady_glide(
            model(&points),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 10.0,
                wind_ned_mps: Vec3::ZERO,
                ground_effect_enabled: false,
            },
            0.0,
        )
        .expect("linearization exists");
        assert!(linear.trim_residual.forward_acceleration_mps2.abs() < 1.0e-12);
        assert!(linear.trim_residual.down_acceleration_mps2.abs() < 1.0e-12);
        assert!(linear.trim_residual.pitch_acceleration_rad_s2.abs() < 1.0e-12);
        assert_eq!(linear.state_matrix[3], [0.0, 0.0, 1.0, 0.0]);
        assert_eq!(linear.elevator_input[3], 0.0);
        assert!(
            linear
                .state_matrix
                .iter()
                .flatten()
                .all(|value| value.is_finite())
        );
    }

    #[test]
    fn rejects_wind_because_trim_is_still_air_only() {
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
        let result = linearize_steady_glide(
            model(&points),
            Environment {
                density_kg_m3: 1.0,
                gravity_mps2: 10.0,
                wind_ned_mps: Vec3::new(1.0, 0.0, 0.0),
                ground_effect_enabled: false,
            },
            0.0,
        );
        assert_eq!(result, Err(LinearizationError::NonZeroWind));
    }
}
