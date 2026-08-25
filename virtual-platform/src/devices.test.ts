import assert from 'node:assert/strict';
import test from 'node:test';

import {
  As5600Device,
  Bno055Device,
  Dps310Device,
  Sdp810Device,
  crc8,
  type I2cDevice,
  type PlantObservation,
} from './devices.js';

function observation(overrides: Partial<PlantObservation> = {}): PlantObservation {
  return {
    time_s: 0,
    north_m: 0,
    altitude_m: 10.5,
    pitch_rad: 0,
    flight_path_rad: -3 * Math.PI / 180,
    elevator_rad: 0,
    rudder_rad: 0,
    sensor_pitch_rad: 0,
    sensor_pitch_rate_rad_s: 0,
    sensor_airspeed_mps: 5,
    sensor_differential_pressure_pa: 14.55,
    sensor_barometric_altitude_m: 10.5,
    sensor_alpha_rad: 0,
    aero_in_range: true,
    surface_contact: false,
    ...overrides,
  };
}

function readRegisters(device: I2cDevice, start: number, count: number): number[] {
  device.startWrite();
  device.writeByte(start);
  device.startRead();
  return Array.from({ length: count }, () => device.readByte());
}

test('Sensirion CRC-8 matches the datasheet example', () => {
  assert.equal(crc8(Uint8Array.from([0xbe, 0xef])), 0x92);
});

test('BNO055 exposes 1/16-degree little-endian pitch and pitch rate', () => {
  const device = new Bno055Device();
  device.update(observation({
    sensor_pitch_rad: -1.25 * Math.PI / 180,
    sensor_pitch_rate_rad_s: 2.5 * Math.PI / 180,
  }));
  assert.deepEqual(readRegisters(device, 0x16, 2), [40, 0]);
  assert.deepEqual(readRegisters(device, 0x1e, 2), [0xec, 0xff]);
  assert.deepEqual(readRegisters(device, 0x00, 1), [0xa0]);
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x05, 0x00]);
  device.update(observation(), 'bno-status');
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x01, 0x09]);
});

test('AS5600 maps zero AoA to installation midpoint and wraps 12 bits', () => {
  const device = new As5600Device();
  device.update(observation({ sensor_alpha_rad: 0 }));
  assert.deepEqual(readRegisters(device, 0x0c, 2), [0x08, 0x00]);
  device.update(observation({ sensor_alpha_rad: Math.PI }));
  assert.deepEqual(readRegisters(device, 0x0c, 2), [0x00, 0x00]);
  assert.deepEqual(readRegisters(device, 0x0b, 1), [0x20]);
  device.update(observation(), 'as5600-magnet');
  assert.deepEqual(readRegisters(device, 0x0b, 1), [0x10]);
});

test('SDP810 frame carries scale 60 and valid CRC words', () => {
  const device = new Sdp810Device();
  device.update(observation({ sensor_differential_pressure_pa: 10 }));
  device.startRead();
  const frame = Uint8Array.from({ length: 9 }, () => device.readByte());
  assert.deepEqual(Array.from(frame.subarray(0, 2)), [0x02, 0x58]);
  assert.deepEqual(Array.from(frame.subarray(6, 8)), [0x00, 0x3c]);
  assert.equal(frame[2], crc8(frame.subarray(0, 2)));
  assert.equal(frame[8], crc8(frame.subarray(6, 8)));
  device.update(observation(), 'sdp-crc');
  device.startRead();
  const invalidFrame = Uint8Array.from({ length: 9 }, () => device.readByte());
  assert.notEqual(invalidFrame[2], crc8(invalidFrame.subarray(0, 2)));
});

test('DPS310 exposes ready bits while preserving configured measurement mode', () => {
  const device = new Dps310Device();
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc0]);
  device.startWrite();
  device.writeByte(0x08);
  device.writeByte(0x07);
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xd7]);
  readRegisters(device, 0x00, 3);
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation());
  device.update(observation());
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xd7]);
  device.update(observation({ time_s: 0.08 }), 'dps-stale');
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation(), 'dps-not-ready');
  assert.deepEqual(readRegisters(device, 0x08, 1), [0x07]);
});
