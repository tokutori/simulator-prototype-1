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
    sensor_roll_rad: 0,
    sensor_yaw_rad: 0,
    sensor_roll_rate_rad_s: 0,
    sensor_pitch_rate_rad_s: 0,
    sensor_yaw_rate_rad_s: 0,
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

test('BNO055 exposes all FBW attitude and rate channels at 1/16 degree', () => {
  const device = new Bno055Device();
  device.update(observation({
    sensor_pitch_rad: -1.25 * Math.PI / 180,
    sensor_roll_rad: 3 * Math.PI / 180,
    sensor_yaw_rad: 4 * Math.PI / 180,
    sensor_roll_rate_rad_s: -1 * Math.PI / 180,
    sensor_pitch_rate_rad_s: 2.5 * Math.PI / 180,
    sensor_yaw_rate_rad_s: 1.5 * Math.PI / 180,
  }));
  assert.deepEqual(readRegisters(device, 0x14, 6), [0xf0, 0xff, 40, 0, 24, 0]);
  assert.deepEqual(readRegisters(device, 0x16, 2), [40, 0]);
  assert.deepEqual(readRegisters(device, 0x1a, 4), [64, 0, 48, 0]);
  assert.deepEqual(readRegisters(device, 0x1e, 2), [0xec, 0xff]);
  assert.deepEqual(readRegisters(device, 0x00, 1), [0xa0]);
  device.startWrite();
  device.writeByte(0x3d);
  device.writeByte(0x0c);
  assert.equal(device.configurationWriteCount, 1);
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x05, 0x00]);
  device.update(observation(), 'bno-status');
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x01, 0x09]);
  device.update(observation(), 'bno-reset');
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x01, 0x09]);
  device.startWrite();
  device.writeByte(0x3d);
  device.writeByte(0x0c);
  device.update(observation());
  assert.equal(device.configurationWriteCount, 2);
  assert.deepEqual(readRegisters(device, 0x39, 2), [0x05, 0x00]);
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
  let nowUs = 0;
  const device = new Sdp810Device(() => nowUs);
  assert.equal(device.startRead(), false);
  device.startWrite(); device.writeByte(0x36); assert.equal(device.writeByte(0x15), true);
  assert.equal(device.startRead(), false);
  nowUs = 8000;
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
  let nowUs = 0;
  const device = new Dps310Device(() => nowUs);
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc0]);
  device.update(observation());
  nowUs = 1e6;
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc0]);
  device.startWrite(); device.writeByte(0x06); device.writeByte(0x50);
  device.startWrite();
  device.writeByte(0x08);
  device.writeByte(0x07);
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  nowUs += 31250;
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xd7]);
  readRegisters(device, 0x00, 3);
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation());
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  nowUs += 31250;
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xd7]);
  device.update(observation({ time_s: 0.08 }), 'dps-stale');
  assert.deepEqual(readRegisters(device, 0x08, 1), [0xc7]);
  device.update(observation(), 'dps-not-ready');
  assert.deepEqual(readRegisters(device, 0x08, 1), [0x07]);
});

test('SDP810 rejects repeated start and requires stop recovery delay', () => {
  let nowUs = 0;
  const device = new Sdp810Device(() => nowUs);
  const command = (word: number): boolean => {
    if (!device.startWrite()) return false;
    device.writeByte(word >>> 8); return device.writeByte(word & 255);
  };
  assert.equal(command(0x3615), true);
  nowUs = 9000;
  assert.equal(command(0x3615), false);
  assert.equal(command(0x3ff9), true);
  assert.equal(device.startRead(), false);
  nowUs += 499;
  assert.equal(command(0x3615), false);
  nowUs += 1;
  assert.equal(command(0x3615), true);
  assert.equal(device.startRead(), false);
  nowUs += 8000;
  assert.equal(device.startRead(), true);
});
