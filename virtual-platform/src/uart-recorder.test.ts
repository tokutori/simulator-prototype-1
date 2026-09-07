import assert from 'node:assert/strict';
import test from 'node:test';
import { Simulator } from 'rp2040js';
import { attachUartRecorder } from './uart-recorder.js';

const BASE = 0x40038000;
function fixture(): { sim: Simulator; bytes: number[]; times: number[] } {
  const sim = new Simulator(), bytes: number[] = [], times: number[] = [];
  const m = sim.rp2040;
  m.writeUint32(0x40014044, 2);
  m.writeUint32(BASE + 0x24, 7); m.writeUint32(BASE + 0x28, 52);
  m.writeUint32(BASE + 0x2c, 0x70); m.writeUint32(BASE + 0x30, 0x301);
  attachUartRecorder(sim, byte => { bytes.push(byte); times.push(sim.clock.micros); });
  return { sim, bytes, times };
}
test('UARTDR without configured transmitter/mux is not transmitted telemetry', () => {
  const sim = new Simulator();
  attachUartRecorder(sim, () => assert.fail('must not receive'));
  assert.throws(() => sim.rp2040.writeUint32(BASE, 70), /UARTEN/);
  for (const [address, value, message] of [
    [0x40014044, 31, /mux/], [BASE+0x30, 0x201, /UARTEN/],
    [BASE+0x24, 65, /baud/], [BASE+0x2c, 0x72, /8N1/],
  ] as const) {
    const { sim: bad } = fixture(); bad.rp2040.writeUint32(address, value);
    assert.throws(() => bad.rp2040.writeUint32(BASE, 70), message);
  }
});
test('actual MMIO TX FIFO provides backpressure and byte completion ordering', () => {
  const { sim, bytes, times } = fixture(), m = sim.rp2040;
  for (let i=0; i<33; i++) m.writeUint32(BASE, i);
  assert.equal(m.readUint32(BASE+0x18) & 0xa8, 0x28);
  assert.throws(() => m.writeUint32(BASE, 34), /overrun/);
  sim.clock.tick(9999); assert.deepEqual(bytes, []);
  sim.clock.tick(1); assert.deepEqual(bytes, [0]);
  assert.equal(m.readUint32(BASE+0x18) & 0x20, 0);
  // The same polling/backpressure contract used by the real blocking HAL.
  for (let i=33; i<56; i++) {
    while (m.readUint32(BASE+0x18) & 0x20) sim.clock.tick(10_000);
    m.writeUint32(BASE, i);
  }
  while (m.readUint32(BASE+0x18) & 0x08) sim.clock.tick(10_000);
  assert.deepEqual(bytes, Array.from({ length: 56 }, (_, i) => i));
  assert.equal(times.at(-1), 560);
  assert.equal(m.readUint32(BASE+0x18) & 0xa8, 0x80);
});
test('transmitter reconfiguration during a byte cannot fabricate successful reception', () => {
  const { sim, bytes } = fixture();
  sim.rp2040.writeUint32(BASE, 70);
  sim.rp2040.writeUint32(0x40014044, 31);
  assert.throws(() => sim.clock.tick(10_000), /mux/);
  assert.deepEqual(bytes, []);
});

test('disabled FIFO has one holding slot besides the transmitting shift register', () => {
  const { sim, bytes } = fixture(), m = sim.rp2040;
  m.writeUint32(BASE+0x2c, 0x60);
  m.writeUint32(BASE, 1); m.writeUint32(BASE, 2);
  assert.equal(m.readUint32(BASE+0x18) & 0x20, 0x20);
  assert.throws(() => m.writeUint32(BASE, 3), /overrun/);
  sim.clock.tick(20_000);
  assert.deepEqual(bytes, [1,2]);
});

test('UART rejects IrDA and transmit interrupt configuration immediately', () => {
  for (const [offset, value] of [[0x30, 0x303], [0x30, 0x305], [0x38, 0x20]]) {
    const { sim, bytes } = fixture();
    assert.throws(() => sim.rp2040.writeUint32(BASE + offset!, value!), /outside the recorder contract/);
    assert.deepEqual(bytes, []);
  }
});

test('UART permits HAL DMA request gates but rejects an active DMA transmitter', () => {
  const { sim, bytes } = fixture(), m = sim.rp2040;
  m.writeUint32(BASE + 0x48, 3);
  m.writeUint32(BASE, 70); sim.clock.tick(10_000);
  assert.deepEqual(bytes, [70]);
  // Unpaced (DREQ0) descriptor remains pending until clock dispatch. The
  // recorder must reject it before accepting more output as supported TX.
  m.writeUint32(0x50000004, BASE);
  m.writeUint32(0x50000008, 1);
  m.writeUint32(0x5000000c, 1);
  assert.throws(() => m.writeUint32(BASE, 71), /active TX DMA/);
});
