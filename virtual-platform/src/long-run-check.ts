/** Runtime-capacity test, NOT a Birdman flight prediction. Uses a high-altitude
 * fixture solely so surface contact cannot hide a lifetime MCU instruction cap.
 */
import assert from 'node:assert/strict';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { spawn } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const folder = resolve(root, 'target/runtime-capacity');
mkdirSync(folder, { recursive: true });
const model = JSON.parse(readFileSync(resolve(root, 'models/qx18-br-training-envelope.json'), 'utf8'));
model.initial_state.altitude_m = 1500;
model.metadata.name = 'runtime capacity fixture - NOT an aircraft validation';
const modelPath = resolve(folder, 'model.json');
const summaryPath = resolve(folder, 'summary.json');
writeFileSync(modelPath, JSON.stringify(model), 'utf8');
const result = await new Promise<number | null>((done, reject) => {
  const child = spawn(process.execPath, ['--import', 'tsx', 'src/run.ts',
    '--model', modelPath, '--steps', '12000', '--output', resolve(folder, 'flight.csv'),
    '--summary', summaryPath], { cwd: resolve(root, 'virtual-platform'), stdio: 'inherit', windowsHide: true });
  child.once('error', reject);
  child.once('exit', done);
});
assert.equal(result, 0, 'actual-UF2 long-duration run must complete');
const summary = JSON.parse(readFileSync(summaryPath, 'utf8'));
assert.ok(summary.simulated_s >= 119.99, 'must reach 120s, not early water contact');
assert.ok(summary.instructions > 100_000_000, 'must exceed the removed lifetime cap');
console.log(JSON.stringify({ passed: true, simulated_s: summary.simulated_s, instructions: summary.instructions,
  scope: 'actual-UF2 runtime capacity only; deliberately non-Birdman launch height' }));
