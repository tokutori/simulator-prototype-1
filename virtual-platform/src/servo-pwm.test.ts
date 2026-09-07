import assert from 'node:assert/strict';
import test from 'node:test';
import { Simulator } from 'rp2040js';
import { ServoPulseDecoder, attachServoPwm } from './servo-pwm.js';

test('servo requires complete pulses, configured period and expires missing edges', () => {
  const decoder = new ServoPulseDecoder();
  assert.equal(decoder.sample(0).kind, 'missing');
  decoder.edge(true, 0); decoder.edge(false, 1500);
  assert.equal(decoder.sample(1500).kind, 'missing');
  decoder.edge(true, 20000); decoder.edge(false, 22000);
  const valid = decoder.sample(22000);
  assert.equal(valid.kind, 'valid');
  assert.ok(Math.abs(valid.commandRad - Math.PI / 18) < 1e-10);
  assert.equal(decoder.sample(83000).kind, 'lost');
});

test('actual rp2040js pin waveform requires mux, enable, divider and TOP', () => {
  for (const config of [
    { mux: 4, enabled: 1, divider: 125, top: 19999, valid: true },
    { mux: 5, enabled: 1, divider: 125, top: 19999, valid: false },
    { mux: 4, enabled: 0, divider: 125, top: 19999, valid: false },
    { mux: 4, enabled: 1, divider: 250, top: 19999, valid: false },
    { mux: 4, enabled: 1, divider: 125, top: 9999, valid: false },
  ]) {
    const simulator = new Simulator();
    const servos = attachServoPwm(simulator);
    const mcu = simulator.rp2040;
    mcu.writeUint32(0x40014084, config.mux);
    mcu.writeUint32(0x4001408c, config.mux);
    mcu.writeUint32(0x40050004, config.divider << 4);
    mcu.writeUint32(0x40050010, config.top);
    mcu.writeUint32(0x4005000c, (1500 << 16) | 1500);
    mcu.writeUint32(0x40050000, config.enabled);
    for (let us = 0; us < 90000; us++) simulator.clock.tick(1000);
    assert.equal(servos.elevator.sample(simulator.clock.micros).kind === 'valid', config.valid);
    assert.equal(servos.rudder.sample(simulator.clock.micros).kind === 'valid', config.valid);
    mcu.writeUint32(0x40050000, 0);
    simulator.clock.tick(70000000);
    assert.notEqual(servos.elevator.sample(simulator.clock.micros).kind, 'valid');
  }
});

test('wrong divider or TOP cannot produce an accepted servo command', () => {
  for (const [period, width] of [[2000, 150], [40000, 3000], [10000, 1500]]) {
    const decoder = new ServoPulseDecoder();
    decoder.edge(true, 0); decoder.edge(false, width!);
    decoder.edge(true, period!); decoder.edge(false, period! + width!);
    assert.equal(decoder.sample(50000).kind, 'missing');
  }
});
