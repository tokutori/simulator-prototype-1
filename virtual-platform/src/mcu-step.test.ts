import assert from 'node:assert/strict';
import test from 'node:test';
import { Simulator } from 'rp2040js';
import { stepMcu } from './mcu-step.js';

test('timer forced pending must be cleared as well as the raw alarm flag', () => {
  const { rp2040: mcu } = new Simulator();
  mcu.writeUint32(0x40054038, 1); // TIMER INTE
  mcu.writeUint32(0x4005403c, 1); // TIMER INTF, HAL's late-schedule path
  mcu.writeUint32(0x40054034, 1); // INTR W1C alone is insufficient
  assert.equal(mcu.readUint32(0x40054040), 1);
  mcu.writeUint32(0x4005703c, 1); // ISR's atomic INTF clear
  assert.equal(mcu.readUint32(0x40054040), 0);
});

test('sleep stops at the plant boundary without executing the next instruction', () => {
  const sim = new Simulator();
  sim.rp2040.core.waiting = true;
  const pc = sim.rp2040.core.PC;
  const events: number[] = [];
  sim.clock.createAlarm(() => events.push(sim.clock.micros)).schedule(20_000);
  assert.equal(stepMcu(sim, 8, 10), 0);
  assert.equal(sim.clock.micros, 10);
  assert.deepEqual(events, []);
  assert.equal(sim.rp2040.core.PC, pc);
  stepMcu(sim, 8, 30);
  assert.deepEqual(events, [20]);
});

test('sleep dispatches intervening peripheral events in order', () => {
  const sim = new Simulator();
  sim.rp2040.core.waiting = true;
  const events: string[] = [];
  sim.clock.createAlarm(() => events.push('PWM')).schedule(2_000);
  sim.clock.createAlarm(() => { events.push('wake'); sim.rp2040.core.waiting = false; }).schedule(8_000);
  stepMcu(sim, 8, 10);
  assert.deepEqual(events, ['PWM']);
  stepMcu(sim, 8, 10);
  assert.deepEqual(events, ['PWM', 'wake']);
  assert.equal(sim.clock.micros, 8);
  assert.equal(sim.rp2040.core.waiting, false);
});

test('sleep without alarms advances bounded time rather than executing phantom instructions', () => {
  const sim = new Simulator();
  sim.rp2040.core.waiting = true;
  assert.equal(stepMcu(sim, 8, 0.5), 0);
  assert.equal(sim.clock.micros, 0.5);
});
