/** Rebuild first. Every case executes actual UF2; summaries must identify the same
 * artifacts and nominal CPU clock. No pre-generated result is accepted as proof.
 */
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { readFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const directory = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const folder = resolve(directory, '../target/recovery-review');
mkdirSync(folder, { recursive: true });
const cases = [
  { name: 'transient', fault: 'bno-status', updates: '3', duration: '0.2', minimum: 0, maximum: 0 },
  { name: 'single-reset', fault: 'bno-reset', updates: '1', duration: '0.2', minimum: 1, maximum: 1 },
  { name: 'persistent-reset', fault: 'bno-reset', updates: '0', duration: '0.5', minimum: 2, maximum: 10 },
];
let identity = '';
for (const scenario of cases) {
  const summaryPath = resolve(folder, `${scenario.name}.json`);
  const result = spawnSync(process.execPath, ['--import', 'tsx', 'src/run.ts', '--steps', '200',
    '--timing-acceleration', '1', '--sensor-fault', scenario.fault, '--fault-start-s', '0.3',
    '--fault-duration-s', scenario.duration, '--fault-update-count', scenario.updates,
    '--summary', summaryPath, '--output', resolve(folder, `${scenario.name}.csv`)],
    { cwd: directory, encoding: 'utf8', windowsHide: true, timeout: 60_000 });
  assert.equal(result.status, 0, result.stderr);
  const summary = JSON.parse(readFileSync(summaryPath, 'utf8'));
  assert.equal(summary.evidence_version, 1);
  const current = JSON.stringify(summary.inputs);
  if (identity === '') identity = current;
  assert.equal(current, identity, 'all scenarios must use identical artifacts');
  assert.equal(summary.timing_acceleration, 1);
  assert.equal(summary.deadline_miss_activation_count, 0);
  assert.equal(summary.safety_failsafe_at_end, false);
  assert.ok(summary.sensor_reinitializations_observed >= scenario.minimum && summary.sensor_reinitializations_observed <= scenario.maximum);
  assert.equal(summary.recovery_valid_to_rearmed_control_updates, 19);
  console.log(JSON.stringify({ scenario: scenario.name, passed: true, reinitializations: summary.sensor_reinitializations_observed, inputs: summary.inputs }));
}
