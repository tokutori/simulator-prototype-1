//! Installation contract: sensor X=right, Y=forward, Z=up; identity axis remap.
//! BNO Hamilton quaternion rotates sensor vectors to Android ENU. Aircraft uses
//! FRD body and NED world: C=[0 1 0; 1 0 0; 0 0 -1], R_body=C R_sensor C^T.

pub fn attitude(raw: [i16; 4]) -> Option<(f32, f32)> {
    let [w, sx, sy, sz] = raw.map(|value| f32::from(value) / 16384.0);
    let norm = w * w + sx * sx + sy * sy + sz * sz;
    // Quantization is small. Zero/corrupt fusion output is not a valid attitude.
    if !(0.9..=1.1).contains(&norm) {
        return None;
    }
    let (x, y, z) = (sy, sx, -sz);
    let roll = atan2(2.0 * (w * x + y * z), norm - 2.0 * (x * x + y * y));
    let pitch = asin((2.0 * (w * y - z * x) / norm).clamp(-1.0, 1.0));
    Some((roll, pitch))
}

pub fn rates(raw: [i16; 3]) -> [f32; 3] {
    let scale = core::f32::consts::PI / (180.0 * 16.0);
    [
        f32::from(raw[1]) * scale,
        f32::from(raw[0]) * scale,
        -f32::from(raw[2]) * scale,
    ]
}

#[cfg(not(test))]
fn atan2(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}
#[cfg(test)]
fn atan2(y: f32, x: f32) -> f32 {
    y.atan2(x)
}
#[cfg(not(test))]
fn asin(value: f32) -> f32 {
    libm::asinf(value)
}
#[cfg(test)]
fn asin(value: f32) -> f32 {
    value.asin()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 0.0002, "{actual} vs {expected}");
    }
    #[test]
    fn sensor_basis_rotations_follow_installation_not_euler_labels() {
        // 90deg right-handed sensor X=aircraft pitch, Y=roll, Z=-yaw.
        for (raw, expected) in [
            ([11585, 11585, 0, 0], (0.0, core::f32::consts::FRAC_PI_2)),
            ([11585, 0, 11585, 0], (core::f32::consts::FRAC_PI_2, 0.0)),
            ([11585, 0, 0, 11585], (0.0, 0.0)),
        ] {
            let (roll, pitch) = attitude(raw).unwrap();
            close(roll, expected.0);
            close(pitch, expected.1);
        }
        let actual = rates([16, 32, 48]);
        for (value, deg) in actual.into_iter().zip([2.0_f32, 1.0, -3.0]) {
            close(value, deg.to_radians());
        }
    }
    #[test]
    fn mixed_pose_and_quaternion_sign_are_consistent() {
        // Rz(60deg) Ry(-20deg) Rx(30deg), independently calculated fixture.
        for raw in [[13129, -292, 4991, -8430], [-13129, 292, -4991, 8430]] {
            let (roll, pitch) = attitude(raw).unwrap();
            close(roll, 30_f32.to_radians());
            close(pitch, -20_f32.to_radians());
        }
    }
    #[test]
    fn invalid_fusion_quaternion_is_rejected() {
        assert!(attitude([0; 4]).is_none());
        assert!(attitude([16384; 4]).is_none());
    }
}
