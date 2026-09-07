/** Measure the real interactive bridge without rendering or wall-clock pacing.
 * Does not accelerate the MCU clock or substitute a host controller.
 * Run alone; concurrent emulator sessions invalidate capacity comparisons.
 */
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { fileURLToPath } from 'node:url';

const count = 300;
const child = spawn(process.execPath, ['--import', 'tsx', fileURLToPath(new URL('./web-bridge.ts', import.meta.url))],
  { stdio: ['pipe', 'pipe', 'inherit'], windowsHide: true });
const command = `${JSON.stringify({ pilot_elevator: 0, pilot_rudder: 0, autonomy: 1 })}\n`;
const samples: { mcu: number; plant: number; total: number; instructions: number }[] = [];
const timeout = setTimeout(() => child.kill(), 60_000);
let started = 0;
try {
  for await (const line of createInterface({ input: child.stdout })) {
    const value = JSON.parse(line);
    if (value.type === 'ready') {
      started = performance.now();
      child.stdin.write(command);
      continue;
    }
    assert.equal(value.type, undefined, `unexpected bridge terminal: ${line}`);
    assert.equal(value.backend, 'rp2040js-actual-uf2');
    assert.equal(value.emulation.timing_acceleration, 1);
    const e = value.emulation;
    for (const metric of [e.mcu_processing_ms, e.plant_round_trip_ms, e.processing_ms, e.instructions]) {
      assert.ok(Number.isFinite(metric) && metric >= 0, 'finite nonnegative performance evidence');
    }
    samples.push({ mcu: e.mcu_processing_ms, plant: e.plant_round_trip_ms,
      total: e.processing_ms, instructions: e.instructions });
    if (samples.length === count) break;
    child.stdin.write(command);
  }
  assert.equal(samples.length, count, 'must complete the entire measurement');
  const elapsed = performance.now() - started;
  const mean = (key: keyof typeof samples[number]): number => samples.reduce((sum, item) => sum + item[key], 0) / count;
  console.log(JSON.stringify({ samples: count, simulated_s: count * 0.01, wall_ms: elapsed,
    real_time_ratio: count * 10 / elapsed, mean_mcu_ms: mean('mcu'), mean_plant_round_trip_ms: mean('plant'),
    mean_step_ms: mean('total'), mean_instructions: mean('instructions'),
    scope: 'headless interactive bridge capacity; CPU x1; physical cycle timing unvalidated' }, null, 2));
} finally {
  clearTimeout(timeout);
  child.stdin.end();
  child.kill();
}
