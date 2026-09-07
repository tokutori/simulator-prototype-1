import assert from 'node:assert/strict';
import { test } from 'node:test';
import { FirmwareTelemetry, crc32 } from './firmware-telemetry.js';

function frame(sequence = 42, time = 1234): Uint8Array {
  const bytes = new Uint8Array(56);
  bytes.set([70, 66, 87, 50]);
  const view = new DataView(bytes.buffer);
  view.setUint32(4, sequence, true);
  view.setUint32(8, time, true);
  view.setUint32(12, 1, true);
  [0, 1, 0, -0.1, 0.2, 0.3, 0.4, 0.5, 0.6].forEach((value, index) => view.setFloat32(16 + index * 4, value, true));
  view.setUint32(52, crc32(bytes.subarray(0, 52)), true);
  return bytes;
}

test('UART records retain automatic output even under full manual authority', () => {
  const reader = new FirmwareTelemetry();
  frame().forEach(byte => reader.receive(byte));
  const record = reader.requireFresh(1240);
  assert.equal(record.sequence, 42);
  assert.equal(record.autonomy, 0);
  assert.ok(Math.abs(record.automaticElevator + 0.1) < 1e-7);
  assert.equal(record.automaticValid, true);
});

test('noise, truncation and corrupt records resynchronize without invented data', () => {
  const reader = new FirmwareTelemetry();
  [5, 70, 6, ...frame().subarray(0, 20)].forEach(byte => reader.receive(byte));
  frame(43).forEach(byte => reader.receive(byte));
  frame(44).forEach(byte => reader.receive(byte));
  assert.equal(reader.requireFresh(1240).sequence, 44);
  assert.ok(reader.rejectedFrames > 0);
  assert.throws(() => reader.requireFresh(100_000), /stale/);
});

test('CRC agrees with standard IEEE vector and timer wrap is explicit', () => {
  assert.equal(crc32(new TextEncoder().encode('123456789')), 0xcbf43926);
  const reader = new FirmwareTelemetry();
  frame(0xffff_ffff, 0xffff_fff0).forEach(byte => reader.receive(byte));
  assert.equal(reader.requireFresh(0x1_0000_0010).sequence, 0xffff_ffff);
});
