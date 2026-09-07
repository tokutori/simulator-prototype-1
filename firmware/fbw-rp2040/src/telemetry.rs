//! Fixed-size UART flight-recorder protocol, also usable by a physical logger.
//! Little endian: FBW2, sequence, timer_us, validity flags, nine f32 values, CRC32.

pub const FRAME_SIZE: usize = 56;

pub struct ControlRecord {
    pub sequence: u32,
    pub time_us: u32,
    pub automatic_valid: bool,
    /// Pilot elevator/rudder/authority, automatic elevator/rudder, safe elevator,
    /// mixed elevator/rudder, safe rudder. Angles are radians; pilot values are normalized.
    pub values: [f32; 9],
}

impl ControlRecord {
    pub fn encode(&self) -> [u8; FRAME_SIZE] {
        let mut frame = [0; FRAME_SIZE];
        frame[..4].copy_from_slice(b"FBW2");
        frame[4..8].copy_from_slice(&self.sequence.to_le_bytes());
        frame[8..12].copy_from_slice(&self.time_us.to_le_bytes());
        frame[12..16].copy_from_slice(&u32::from(self.automatic_valid).to_le_bytes());
        for (index, value) in self.values.iter().enumerate() {
            frame[16 + index * 4..20 + index * 4].copy_from_slice(&value.to_le_bytes());
        }
        let crc = crc32(&frame[..52]);
        frame[52..].copy_from_slice(&crc.to_le_bytes());
        frame
    }
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & (0_u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ieee_crc_check_vector() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn frame_has_explicit_validity_and_exact_field_order() {
        let bytes = ControlRecord {
            sequence: 42,
            time_us: 1234,
            automatic_valid: false,
            values: [0.0, 1.0, 0.5, -0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        }
        .encode();
        assert_eq!(&bytes[..4], b"FBW2");
        assert_eq!(&bytes[4..8], &42_u32.to_le_bytes());
        assert_eq!(&bytes[12..16], &[0; 4]);
        assert_eq!(&bytes[20..24], &1_f32.to_le_bytes());
        assert_eq!(
            u32::from_le_bytes(bytes[52..].try_into().unwrap()),
            crc32(&bytes[..52])
        );
    }
}
