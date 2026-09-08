/** Real server / actual UF2 normal-end lifecycle check. No alternate backend. */
import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import WebSocket from 'ws';

const cwd = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const port = 4186;
const server = spawn(process.execPath, ['--import', 'tsx', 'server.ts'], {
  cwd, env: { ...process.env, BIRDMAN_VISUALIZER_PORT: String(port) }, windowsHide: true,
  stdio: ['ignore', 'pipe', 'pipe'],
});
let socket: WebSocket | undefined;
try {
  await new Promise<void>((done, reject) => {
    const timeout = setTimeout(() => reject(new Error('server startup timeout')), 15000);
    server.once('error', error => { clearTimeout(timeout); reject(error); });
    server.once('exit', code => { clearTimeout(timeout); reject(new Error(`server exited ${code}`)); });
    server.stdout.on('data', chunk => {
      if (String(chunk).includes(`http://127.0.0.1:${port}`)) { clearTimeout(timeout); done(); }
    });
  });
  const connected = new WebSocket(`ws://127.0.0.1:${port}/live`);
  socket = connected;
  let updates = 0;
  let contact = false;
  let terminal = false;
  await new Promise<void>((done, reject) => {
    const timeout = setTimeout(() => reject(new Error('natural contact/close timeout')), 45000);
    const fail = (error: unknown): void => { clearTimeout(timeout); reject(error); };
    connected.on('error', fail);
    connected.on('message', bytes => {
      try {
        const value = JSON.parse(String(bytes));
        if (value.type) {
          assert.equal(value.type, 'ended', JSON.stringify(value));
          assert.equal(value.message, 'surface contact');
          assert.equal(contact, true, 'final contact telemetry must precede terminal');
          terminal = true;
        } else {
          assert.equal(value.backend, 'rp2040js-actual-uf2');
          assert.equal(value.emulation.timing_acceleration, 1);
          contact = value.surface_contact;
          updates++;
        }
      } catch (error) { fail(error); }
    });
    connected.on('close', code => {
      clearTimeout(timeout);
      try {
        assert.equal(code, 1000);
        assert.equal(terminal, true, 'server closes only after explicit terminal evidence');
        assert.ok(updates > 2000);
        done();
      } catch (error) { reject(error); }
    });
  });
  console.log(JSON.stringify({ passed: true, updates, contact, terminal, serverClosedSocket: true }));
} finally {
  socket?.terminate();
  if (server.exitCode === null && server.signalCode === null) {
    if (process.platform === 'win32' && typeof server.pid === 'number') {
      spawnSync('taskkill.exe', ['/PID', String(server.pid), '/T', '/F'], { windowsHide: true, stdio: 'ignore' });
    } else server.kill();
  }
}
