#![no_std]
#![forbid(unsafe_code)]

//! Deterministic, platform-independent flight-dynamics primitives.
//!
//! The crate owns no clock, filesystem, random source, network connection, or
//! device. Callers provide every input explicitly and receive the next state as
//! a return value. Body axes are `x` forward, `y` right, and `z` down. The
//! navigation frame is North-East-Down (NED), and SI units are used throughout.
//!
//! # Example
//!
//! ```
//! use flight_dynamics_core::Vec3;
//!
//! let forward_velocity_body_mps = Vec3::new(9.0, 0.0, 0.0);
//! assert_eq!(forward_velocity_body_mps.norm(), 9.0);
//! ```

pub mod actuator;
pub mod dynamics;
pub mod linearization;
pub mod math;
pub mod model;
pub mod sensor;
pub mod trim;

pub use actuator::{Actuator, ActuatorConfig, ActuatorError};
pub use dynamics::{
    AeroCoefficients, AeroLoads, ControlSurfaceDeflection, Environment, FlightCondition,
    RigidBodyState, StepError, aerodynamic_loads, step_rk4,
};
pub use linearization::{
    LinearizationError, LongitudinalLinearization, LongitudinalState, LongitudinalStateDerivative,
    linearize_steady_glide,
};
pub use math::{Quaternion, Vec3};
pub use model::{
    AeroDerivatives, AeroPoint, AeroTable, AircraftModel, ForceCoefficientBasis, GroundEffectModel,
    Inertia, ModelError,
};
pub use sensor::{SensorError, SensorModel, SensorSample, SensorSuite};
pub use trim::{SteadyGlideTrim, TrimError, steady_glide_trim};
