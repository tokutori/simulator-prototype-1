//! Small allocation-free vector and quaternion types.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub};

/// Three-dimensional vector in the frame documented by its field or argument.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// First component.
    pub x: f64,
    /// Second component.
    pub y: f64,
    /// Third component.
    pub z: f64,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Creates a vector.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Returns the dot product.
    #[must_use]
    pub const fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Returns the right-handed cross product.
    #[must_use]
    pub const fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Returns the Euclidean norm.
    #[must_use]
    pub fn norm(self) -> f64 {
        libm::sqrt(self.dot(self))
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        self * (1.0 / rhs)
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// Unit quaternion rotating body-frame vectors into the NED frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    /// Scalar component.
    pub w: f64,
    /// First vector component.
    pub x: f64,
    /// Second vector component.
    pub y: f64,
    /// Third vector component.
    pub z: f64,
}

impl Quaternion {
    /// Identity rotation.
    pub const IDENTITY: Self = Self::new(1.0, 0.0, 0.0, 0.0);

    /// Creates a quaternion without normalizing it.
    #[must_use]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Creates a body-to-NED quaternion from aerospace roll, pitch, and yaw.
    #[must_use]
    pub fn from_euler(roll_rad: f64, pitch_rad: f64, yaw_rad: f64) -> Self {
        let (sr, cr) = (libm::sin(roll_rad * 0.5), libm::cos(roll_rad * 0.5));
        let (sp, cp) = (libm::sin(pitch_rad * 0.5), libm::cos(pitch_rad * 0.5));
        let (sy, cy) = (libm::sin(yaw_rad * 0.5), libm::cos(yaw_rad * 0.5));
        Self::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
        .normalized()
    }

    /// Returns aerospace roll, pitch, and yaw in radians.
    #[must_use]
    pub fn to_euler(self) -> Vec3 {
        let unit = self.normalized();
        let roll = libm::atan2(
            2.0 * (unit.w * unit.x + unit.y * unit.z),
            1.0 - 2.0 * (unit.x * unit.x + unit.y * unit.y),
        );
        let pitch_argument = (2.0 * (unit.w * unit.y - unit.z * unit.x)).clamp(-1.0, 1.0);
        let pitch = libm::asin(pitch_argument);
        let yaw = libm::atan2(
            2.0 * (unit.w * unit.z + unit.x * unit.y),
            1.0 - 2.0 * (unit.y * unit.y + unit.z * unit.z),
        );
        Vec3::new(roll, pitch, yaw)
    }

    /// Returns the conjugate.
    #[must_use]
    pub const fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Returns the Hamilton product.
    #[must_use]
    pub const fn product(self, rhs: Self) -> Self {
        Self::new(
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        )
    }

    /// Rotates a body-frame vector into NED.
    #[must_use]
    pub fn rotate_body_to_ned(self, vector: Vec3) -> Vec3 {
        self.rotate(vector)
    }

    /// Rotates a NED-frame vector into the body frame.
    #[must_use]
    pub fn rotate_ned_to_body(self, vector: Vec3) -> Vec3 {
        self.conjugate().rotate(vector)
    }

    /// Returns a normalized quaternion, or identity for a zero quaternion.
    #[must_use]
    pub fn normalized(self) -> Self {
        let norm =
            libm::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if norm > 0.0 {
            self * (1.0 / norm)
        } else {
            Self::IDENTITY
        }
    }

    fn rotate(self, vector: Vec3) -> Vec3 {
        let unit = self.normalized();
        let pure = Self::new(0.0, vector.x, vector.y, vector.z);
        let rotated = unit.product(pure).product(unit.conjugate());
        Vec3::new(rotated.x, rotated.y, rotated.z)
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w + rhs.w,
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
        )
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.w * rhs, self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::{Quaternion, Vec3};

    #[test]
    fn euler_round_trip_preserves_angles() {
        let expected = Vec3::new(0.2, -0.1, 0.3);
        let actual = Quaternion::from_euler(expected.x, expected.y, expected.z).to_euler();
        assert!((actual.x - expected.x).abs() < 1.0e-12);
        assert!((actual.y - expected.y).abs() < 1.0e-12);
        assert!((actual.z - expected.z).abs() < 1.0e-12);
    }

    #[test]
    fn rotation_round_trip_preserves_vector() {
        let attitude = Quaternion::from_euler(0.3, -0.2, 0.8);
        let expected = Vec3::new(3.0, -2.0, 7.0);
        let actual = attitude.rotate_ned_to_body(attitude.rotate_body_to_ned(expected));
        assert!((actual - expected).norm() < 1.0e-12);
    }
}
