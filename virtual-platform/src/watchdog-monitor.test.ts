import assert from 'node:assert/strict';
import test from 'node:test';
import { Simulator } from 'rp2040js';
import { installWatchdogMonitor, WatchdogResetRequested } from './watchdog-monitor.js';

test('actual RP2040 watchdog feed postpones expiry and stalled firmware requests reset', () => {
  const simulator = new Simulator();
  const mcu = simulator.rp2040;
  installWatchdogMonitor(mcu);
  // HAL uses twice the microsecond period to account for RP2040-E1.
  mcu.writeUint32(0x4005802c, 0x200 | 12);
  mcu.writeUint32(0x40058004, 200000);
  mcu.writeUint32(0x40058000, 1 << 30);
  for (let feed = 0; feed < 20; feed++) {
    simulator.clock.tick(10000000);
    mcu.writeUint32(0x40058004, 200000);
  }
  simulator.clock.tick(99999000);
  assert.throws(() => simulator.clock.tick(1000), (error: unknown) => {
    assert.ok(error instanceof WatchdogResetRequested);
    assert.equal(error.atUs, 300000);
    return true;
  });
  assert.equal(mcu.readUint32(0x40058008), 1);
});
