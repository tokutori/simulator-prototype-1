//! Minimal datasheet-level drivers for the flight-control sensor set.

use core::f32::consts::PI;
use embedded_hal::i2c::I2c;

pub const BNO055_ADDRESS: u8 = 0x28;
pub const AS5600_ADDRESS: u8 = 0x36;
pub const SDP810_ADDRESS: u8 = 0x25;
pub const DPS310_ADDRESS: u8 = 0x77;

const BNO055_OPERATION_MODE: u8 = 0x3d;
const BNO055_NDOF_MODE: u8 = 0x0c;
const BNO055_CHIP_ID: u8 = 0x00;
const BNO055_EXPECTED_CHIP_ID: u8 = 0xa0;
const BNO055_GYRO_X_LSB: u8 = 0x14;
const BNO055_SYSTEM_STATUS: u8 = 0x39;
const BNO055_FUSION_RUNNING: u8 = 0x05;
const AS5600_STATUS: u8 = 0x0b;
const AS5600_RAW_ANGLE_HIGH: u8 = 0x0c;
const AS5600_MAGNET_DETECTED: u8 = 1 << 5;
const AS5600_MAGNET_TOO_WEAK: u8 = 1 << 4;
const AS5600_MAGNET_TOO_STRONG: u8 = 1 << 3;
const AS5600_ZERO_RAW: i32 = 2048;
const DPS310_PRESSURE_RESULT: u8 = 0x00;
const DPS310_PRESSURE_CONFIG: u8 = 0x06;
const DPS310_TEMPERATURE_CONFIG: u8 = 0x07;
const DPS310_MEASUREMENT_CONFIG: u8 = 0x08;
const DPS310_COEFFICIENTS: u8 = 0x10;
const DPS310_COEFFICIENT_SOURCE: u8 = 0x28;
const DPS310_SCALE_FACTOR_OSR_1: f32 = 524_288.0;
const DPS310_MAX_PRESSURE_HOLD_READS: u8 = 20;
const AIR_DENSITY_KG_M3: f32 = 1.164;
const GRAVITY_MPS2: f32 = 9.806_65;
const LAUNCH_ALTITUDE_M: f32 = 10.5;

/// Sensor transaction or datasheet-level validity failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorError<E> {
    /// I²C peripheral or bus transaction failed.
    Bus(E),
    /// The device at the BNO055 address did not identify as a BNO055.
    Bno055ChipId(u8),
    /// Fusion is not running or the BNO055 reports a system error.
    Bno055System { status: u8, error: u8 },
    /// AS5600 does not report one usable magnet in range.
    As5600Magnet(u8),
    /// One SDP810 response word failed its CRC-8 check.
    Sdp810Crc(u8),
    /// SDP810 returned an unusable differential-pressure scale factor.
    Sdp810Scale(u16),
    /// DPS310 initialization, coefficient data or the first pressure sample is not ready.
    Dps310NotReady(u8),
}

/// Measurements presented to the platform-independent controller.
#[derive(Clone, Copy)]
pub struct Measurements {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub roll_rate_rad_s: f32,
    pub pitch_rate_rad_s: f32,
    pub yaw_rate_rad_s: f32,
    pub airspeed_mps: f32,
    pub alpha_rad: f32,
    pub barometric_altitude_m: f32,
}

/// Controller input plus diagnostic health of non-critical redundant data.
#[derive(Clone, Copy)]
pub struct MeasurementFrame {
    pub measurements: Measurements,
    /// False when airspeed is being held from the last valid SDP810 sample.
    pub all_sensors_valid: bool,
}

/// DPS310 factory polynomial coefficients.
#[derive(Clone, Copy)]
pub struct Dps310Calibration {
    c00: f32,
    c10: f32,
    c01: f32,
    c11: f32,
    c20: f32,
    c21: f32,
    c30: f32,
}

/// Runtime sensor calibration that is established at release.
pub struct SensorState {
    dps310: Dps310Calibration,
    launch_pressure_pa: Option<f32>,
    last_pressure_pa: Option<f32>,
    pressure_hold_reads: u8,
    last_airspeed_mps: Option<f32>,
}

impl SensorState {
    pub fn initialize<I: I2c>(
        i2c: &mut I,
        retained_launch_pressure_pa: Option<f32>,
    ) -> Result<Self, SensorError<I::Error>> {
        let mut chip_id = [0_u8; 1];
        i2c.write_read(BNO055_ADDRESS, &[BNO055_CHIP_ID], &mut chip_id)
            .map_err(SensorError::Bus)?;
        if chip_id[0] != BNO055_EXPECTED_CHIP_ID {
            return Err(SensorError::Bno055ChipId(chip_id[0]));
        }
        i2c.write(BNO055_ADDRESS, &[BNO055_OPERATION_MODE, BNO055_NDOF_MODE])
            .map_err(SensorError::Bus)?;
        i2c.write(SDP810_ADDRESS, &[0x36, 0x15])
            .map_err(SensorError::Bus)?;

        let mut measurement_status = [0_u8; 1];
        i2c.write_read(
            DPS310_ADDRESS,
            &[DPS310_MEASUREMENT_CONFIG],
            &mut measurement_status,
        )
        .map_err(SensorError::Bus)?;
        require_dps310_ready(measurement_status[0])?;

        let mut coefficient_source = [0_u8; 1];
        i2c.write_read(
            DPS310_ADDRESS,
            &[DPS310_COEFFICIENT_SOURCE],
            &mut coefficient_source,
        )
        .map_err(SensorError::Bus)?;
        let temperature_source = coefficient_source[0] & 0x80;
        // PM_RATE=101 selects 32 measurements/s; PM_PRC=000 keeps OSR=1.
        i2c.write(DPS310_ADDRESS, &[DPS310_PRESSURE_CONFIG, 0x50])
            .map_err(SensorError::Bus)?;
        i2c.write(
            DPS310_ADDRESS,
            &[DPS310_TEMPERATURE_CONFIG, temperature_source | 0x50],
        )
        .map_err(SensorError::Bus)?;
        i2c.write(DPS310_ADDRESS, &[DPS310_MEASUREMENT_CONFIG, 0x07])
            .map_err(SensorError::Bus)?;

        let mut coefficients = [0_u8; 18];
        i2c.write_read(DPS310_ADDRESS, &[DPS310_COEFFICIENTS], &mut coefficients)
            .map_err(SensorError::Bus)?;
        Ok(Self {
            dps310: decode_dps310_coefficients(coefficients),
            launch_pressure_pa: retained_launch_pressure_pa,
            last_pressure_pa: None,
            pressure_hold_reads: 0,
            last_airspeed_mps: None,
        })
    }

    pub const fn launch_pressure_pa(&self) -> Option<f32> {
        self.launch_pressure_pa
    }

    pub fn read<I: I2c>(&mut self, i2c: &mut I) -> Result<MeasurementFrame, SensorError<I::Error>> {
        let mut bno_status = [0_u8; 2];
        i2c.write_read(BNO055_ADDRESS, &[BNO055_SYSTEM_STATUS], &mut bno_status)
            .map_err(SensorError::Bus)?;
        if bno_status[0] != BNO055_FUSION_RUNNING || bno_status[1] != 0 {
            return Err(SensorError::Bno055System {
                status: bno_status[0],
                error: bno_status[1],
            });
        }

        let mut bno = [0_u8; 12];
        i2c.write_read(BNO055_ADDRESS, &[BNO055_GYRO_X_LSB], &mut bno)
            .map_err(SensorError::Bus)?;
        let gyro_x_raw = i16::from_le_bytes([bno[0], bno[1]]);
        let gyro_y_raw = i16::from_le_bytes([bno[2], bno[3]]);
        let gyro_z_raw = i16::from_le_bytes([bno[4], bno[5]]);
        let roll_raw = i16::from_le_bytes([bno[8], bno[9]]);
        let pitch_raw = i16::from_le_bytes([bno[10], bno[11]]);

        let mut as5600_status = [0_u8; 1];
        i2c.write_read(AS5600_ADDRESS, &[AS5600_STATUS], &mut as5600_status)
            .map_err(SensorError::Bus)?;
        let magnet_status = as5600_status[0];
        let magnet_usable = magnet_status & AS5600_MAGNET_DETECTED != 0
            && magnet_status & (AS5600_MAGNET_TOO_WEAK | AS5600_MAGNET_TOO_STRONG) == 0;
        if !magnet_usable {
            return Err(SensorError::As5600Magnet(magnet_status));
        }

        let mut angle = [0_u8; 2];
        i2c.write_read(AS5600_ADDRESS, &[AS5600_RAW_ANGLE_HIGH], &mut angle)
            .map_err(SensorError::Bus)?;
        let raw_angle = i32::from(u16::from_be_bytes(angle) & 0x0fff);
        let wrapped_angle = wrap_12_bit(raw_angle - AS5600_ZERO_RAW);

        let (airspeed_mps, airspeed_valid) = match read_airspeed_mps(i2c) {
            Ok(airspeed_mps) => {
                self.last_airspeed_mps = Some(airspeed_mps);
                (airspeed_mps, true)
            }
            Err(error) => match self.last_airspeed_mps {
                Some(last_airspeed_mps) => (last_airspeed_mps, false),
                None => return Err(error),
            },
        };

        let pressure_pa = self.read_pressure_pa(i2c)?;
        let launch_pressure_pa = *self.launch_pressure_pa.get_or_insert(pressure_pa);
        let altitude_m = LAUNCH_ALTITUDE_M
            + (launch_pressure_pa - pressure_pa) / (AIR_DENSITY_KG_M3 * GRAVITY_MPS2);

        Ok(MeasurementFrame {
            measurements: Measurements {
                roll_rad: f32::from(roll_raw) / 16.0 * PI / 180.0,
                pitch_rad: f32::from(pitch_raw) / 16.0 * PI / 180.0,
                roll_rate_rad_s: f32::from(gyro_x_raw) / 16.0 * PI / 180.0,
                pitch_rate_rad_s: f32::from(gyro_y_raw) / 16.0 * PI / 180.0,
                yaw_rate_rad_s: f32::from(gyro_z_raw) / 16.0 * PI / 180.0,
                airspeed_mps,
                alpha_rad: wrapped_angle as f32 * (2.0 * PI / 4096.0),
                barometric_altitude_m: altitude_m,
            },
            all_sensors_valid: airspeed_valid,
        })
    }

    fn read_pressure_pa<I: I2c>(&mut self, i2c: &mut I) -> Result<f32, SensorError<I::Error>> {
        let mut status = [0_u8; 1];
        i2c.write_read(DPS310_ADDRESS, &[DPS310_MEASUREMENT_CONFIG], &mut status)
            .map_err(SensorError::Bus)?;
        require_dps310_ready(status[0])?;
        const PRESSURE_READY: u8 = 1 << 4;
        if status[0] & PRESSURE_READY == 0 {
            let last_pressure_pa = self
                .last_pressure_pa
                .ok_or(SensorError::Dps310NotReady(status[0]))?;
            if self.pressure_hold_reads >= DPS310_MAX_PRESSURE_HOLD_READS {
                return Err(SensorError::Dps310NotReady(status[0]));
            }
            self.pressure_hold_reads += 1;
            return Ok(last_pressure_pa);
        }
        let mut raw = [0_u8; 6];
        i2c.write_read(DPS310_ADDRESS, &[DPS310_PRESSURE_RESULT], &mut raw)
            .map_err(SensorError::Bus)?;
        let p = sign_extend_24(raw[0], raw[1], raw[2]) as f32 / DPS310_SCALE_FACTOR_OSR_1;
        let t = sign_extend_24(raw[3], raw[4], raw[5]) as f32 / DPS310_SCALE_FACTOR_OSR_1;
        let c = self.dps310;
        let pressure_pa =
            c.c00 + p * (c.c10 + p * (c.c20 + p * c.c30)) + t * c.c01 + t * p * (c.c11 + p * c.c21);
        self.last_pressure_pa = Some(pressure_pa);
        self.pressure_hold_reads = 0;
        Ok(pressure_pa)
    }
}

fn read_airspeed_mps<I: I2c>(i2c: &mut I) -> Result<f32, SensorError<I::Error>> {
    let mut sdp = [0_u8; 9];
    i2c.read(SDP810_ADDRESS, &mut sdp)
        .map_err(SensorError::Bus)?;
    for (word_index, offset) in [0_usize, 3, 6].into_iter().enumerate() {
        if crc8(&sdp[offset..offset + 2]) != sdp[offset + 2] {
            return Err(SensorError::Sdp810Crc(word_index as u8));
        }
    }
    let raw = f32::from(i16::from_be_bytes([sdp[0], sdp[1]]));
    let scale_raw = u16::from_be_bytes([sdp[6], sdp[7]]);
    if scale_raw == 0 {
        return Err(SensorError::Sdp810Scale(scale_raw));
    }
    let differential_pressure_pa = raw / f32::from(scale_raw);
    Ok(libm::sqrtf(
        2.0 * differential_pressure_pa.max(0.0) / AIR_DENSITY_KG_M3,
    ))
}

fn require_dps310_ready<E>(status: u8) -> Result<(), SensorError<E>> {
    const COEFFICIENTS_READY: u8 = 1 << 7;
    const SENSOR_READY: u8 = 1 << 6;
    if status & (COEFFICIENTS_READY | SENSOR_READY) == COEFFICIENTS_READY | SENSOR_READY {
        Ok(())
    } else {
        Err(SensorError::Dps310NotReady(status))
    }
}

fn wrap_12_bit(value: i32) -> i32 {
    (value + 2048).rem_euclid(4096) - 2048
}

fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0xff_u8;
    for byte in bytes {
        crc ^= *byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn decode_dps310_coefficients(bytes: [u8; 18]) -> Dps310Calibration {
    Dps310Calibration {
        c00: sign_extend(
            (i32::from(bytes[3]) << 12) | (i32::from(bytes[4]) << 4) | (i32::from(bytes[5]) >> 4),
            20,
        ) as f32,
        c10: sign_extend(
            (i32::from(bytes[5] & 0x0f) << 16) | (i32::from(bytes[6]) << 8) | i32::from(bytes[7]),
            20,
        ) as f32,
        c01: f32::from(i16::from_be_bytes([bytes[8], bytes[9]])),
        c11: f32::from(i16::from_be_bytes([bytes[10], bytes[11]])),
        c20: f32::from(i16::from_be_bytes([bytes[12], bytes[13]])),
        c21: f32::from(i16::from_be_bytes([bytes[14], bytes[15]])),
        c30: f32::from(i16::from_be_bytes([bytes[16], bytes[17]])),
    }
}

fn sign_extend_24(msb: u8, middle: u8, lsb: u8) -> i32 {
    sign_extend(
        (i32::from(msb) << 16) | (i32::from(middle) << 8) | i32::from(lsb),
        24,
    )
}

fn sign_extend(value: i32, bits: u32) -> i32 {
    let shift = 32 - bits;
    (value << shift) >> shift
}
