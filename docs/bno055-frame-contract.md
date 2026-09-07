# BNO055 quaternion / installation contract

The FBW interface no longer equates Bosch `Euler_Roll` with aircraft roll or
`Euler_Pitch` with aircraft pitch. Bosch's Euler names refer to Y and X rotation,
respectively; coupling them to gyro X and Y was inconsistent and the old virtual
device reproduced that same error.

## Explicit frames

This prototype selects the following physical installation (not a claim about an
existing aircraft's actual board mounting): sensor X points right, Y forward and
Z up. Firmware programs identity axis remap `0x24`, axis signs `0x00`, and
`UNIT_SEL=0x80` (Android convention, degrees/second gyro). Configuration writes
occur in CONFIGMODE, including its 19 ms transition wait during recovery.

The Hamilton quaternion, in w/x/y/z register order and 1/16384 units, rotates
sensor vectors to Android's East/North/Up world frame. Aircraft state uses
Forward/Right/Down body vectors and North/East/Down world vectors. With

```text
C = [0 1 0; 1 0 0; 0 0 -1]
R_body_to_NED = C R_sensor_to_ENU C^T
q_body_to_NED = (w, sensor_q_y, sensor_q_x, -sensor_q_z)
omega_body = (sensor_gyro_y, sensor_gyro_x, -sensor_gyro_z)
```

the angle and rate channels now obey one physical transform. Aircraft roll/pitch
are extracted from the normalized quaternion, not by swapping Euler fields.
Quaternion squared norm outside [0.9, 1.1] is rejected as a critical sensor error.
Magnetic yaw and true-geographic yaw differ by declination; this prototype does
not use BNO yaw for heading hold. Its simulated magnetic/true north are aligned.

The virtual device models this installation and rejects unsupported units/remaps
and configuration writes outside CONFIGMODE. Legacy Euler registers deliberately
raise an unsupported-contract error; silently fabricating aircraft Euler values
under Bosch register names would reintroduce the original defect.

## Evidence and remaining limits

Tests include three independent sensor-axis rotations, gyro basis mapping,
q/-q equivalence, invalid fusion data, and a mixed roll/pitch/yaw comparison
against an independently constructed direction-cosine matrix on all basis vectors.
Firmware math tests can run without a target board:

```powershell
rustc --test firmware/fbw-rp2040/src/bno_orientation.rs --edition=2024 -o target/bno-orientation-tests.exe
target/bno-orientation-tests.exe
npm.cmd test --prefix virtual-platform
```

The upstream sensor model still supplies sampled/quantized Euler attitude to the
virtual transducer; this is not a simulation of Bosch's proprietary fusion or its
native quaternion accuracy. Calibration, magnetic disturbance, acceleration
misinterpreted as gravity, mounting tolerances, and power/reset transition timing
require hardware characterization. Validate static basis poses AND simultaneous
gyro directions on the actual board before flight. A matching virtual roundtrip
is not hardware validation.

Sources:

- [Bosch Quick Start Guide, page 4: Android orientation and Euler axes](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf).
- [Bosch BNO055 datasheet rev. 1.8: sections 3.3, 3.4, 3.6.5.5 and registers UNIT_SEL / AXIS_MAP_CONFIG / AXIS_MAP_SIGN](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
- [Android SensorEvent: rotation vector quaternion, device axes and ENU reference frame](https://developer.android.com/reference/android/hardware/SensorEvent#values).
