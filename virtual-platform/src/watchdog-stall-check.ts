/** Actual-UF2 negative test. Successful test means an explicit watchdog abort,
 * not a recovered aircraft or emulated physical reboot.
 */
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const directory = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const result = spawnSync(process.execPath, ['--import', 'tsx', 'src/run.ts',
  '--steps', '100', '--timing-acceleration', '1',
  '--sensor-fault', 'i2c-stall', '--fault-start-s', '0.3', '--fault-duration-s', '1',
  '--output', 'target/watchdog-stall.csv'], {
  cwd: directory, encoding: 'utf8', timeout: 60000, windowsHide: true,
});
assert.equal(result.status, 1, 'stalled actual firmware must fail the experiment');
assert.match(result.stderr, /WatchdogResetRequested: RP2040 watchdog reset requested/);
assert.doesNotMatch(result.stderr, /telemetry stale|instruction watchdog reached/);
console.log('PASS: actual UF2 stalled I2C triggers the hardware watchdog; physical reboot is explicitly not simulated.');
