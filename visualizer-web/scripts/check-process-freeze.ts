/** Windows-only real-process liveness regression; no fake MCU or plant backend.
 * Run: node --import tsx scripts/check-process-freeze.ts
 */
import assert from 'node:assert/strict';
import { spawn, execFileSync, type ChildProcessWithoutNullStreams } from 'node:child_process';
import { once } from 'node:events';
import { createServer } from 'node:net';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createInterface } from 'node:readline';
import WebSocket from 'ws';

if (process.platform !== 'win32') throw new Error('This regression requires Windows thread suspension');
const directory = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const port = 4185;
const probe = createServer();
await new Promise<void>((ok, bad) => { probe.once('error', bad); probe.listen(port, '127.0.0.1', ok); });
await new Promise<void>(ok => probe.close(() => ok()));
const server = spawn(process.execPath, ['--import', 'tsx', 'server.ts'], {
  cwd: directory, env: { ...process.env, BIRDMAN_VISUALIZER_PORT: String(port) }, windowsHide: true,
});
let log = '';
server.stdout.on('data', chunk => { log += String(chunk); });
server.stderr.on('data', chunk => { log += String(chunk); });
const delay = (ms: number): Promise<void> => new Promise(ok => setTimeout(ok, ms));
function killOwnedServer(): void {
  if (server.exitCode === null && server.signalCode === null && server.pid) {
    try { execFileSync('taskkill.exe', ['/PID', String(server.pid), '/T', '/F'], { windowsHide: true, stdio: 'ignore' }); } catch { /* already exited */ }
  }
}
async function bounded<T>(promise: Promise<T>, ms: number, label: string): Promise<T> {
  let timer: ReturnType<typeof setTimeout> | undefined;
  try { return await Promise.race([promise, new Promise<never>((_, reject) => { timer = setTimeout(() => reject(new Error(`${label}: ${log.slice(-2000)}`)), ms); })]); }
  finally { clearTimeout(timer); }
}
try {
  await bounded((async () => { while (!log.includes('Birdman visualizer:')) { if (server.exitCode !== null) throw new Error(log); await delay(30); } })(), 15000, 'server startup');
  for (const kind of ['plant', 'mcu'] as const) {
    const socket = new WebSocket(`ws://127.0.0.1:${port}/live`, { origin: `http://127.0.0.1:${port}` });
    let helper: ChildProcessWithoutNullStreams | undefined;
    let lastSampleAt = 0;
    let serverCloseCode = 0;
    socket.on('close', code => { serverCloseCode = code; });
    let receivedIdentity: unknown;
    let resolveFirst!: () => void;
    const first = new Promise<void>(ok => { resolveFirst = ok; });
    let resolveTerminal!: (value: { message: string; at: number }) => void;
    const terminal = new Promise<{ message: string; at: number }>(ok => { resolveTerminal = ok; });
    socket.on('error', error => { log += String(error); });
    socket.on('message', data => {
      const value = JSON.parse(data.toString()) as { type?: string; message?: string; backend?: string; run_identity?: unknown };
      if (value.type === 'error') resolveTerminal({ message: value.message ?? '', at: performance.now() });
      else if (value.backend === 'rp2040js-actual-uf2') { lastSampleAt = performance.now(); receivedIdentity = value.run_identity; resolveFirst(); }
    });
    try {
      await bounded(first, 15000, 'first actual UF2 sample');
      helper = spawn('powershell.exe', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
        resolve(directory, 'scripts/freeze-owned-process.ps1'), '-ServerProcessId', String(server.pid), '-Kind', kind],
      { windowsHide: true });
      let helperError = '';
      helper.stderr.on('data', chunk => { helperError += String(chunk); });
      const helperLines = createInterface({ input: helper.stdout })[Symbol.asyncIterator]();
      const next = await bounded(helperLines.next(), 10000, 'freeze helper');
      assert.equal(next.done, false, helperError);
      const evidence = JSON.parse(next.value!) as { target: number; threads: number; children: { ProcessId: number; CreationDate: unknown }[] };
      const frozenAt = performance.now();
      const ended = await bounded(terminal, 10000, 'frozen process did not produce terminal error');
      assert.match(ended.message, kind === 'plant' ? /plant response timed out after 4 seconds/ : /bridge response timed out after 5 seconds/);
      assert.ok(ended.at - lastSampleAt >= (kind === 'plant' ? 3800 : 4800), 'timeout fired prematurely');
      // Do not close the socket yet: cleanup must follow the error, not be
      // accidentally supplied by this test's socket close/finally path.
      await delay(1200);
      assert.equal(serverCloseCode, 1011, 'server must release errored WebSocket admission slots without client teardown');
      const ids = evidence.children.map(item => item.ProcessId);
      const query = `[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false); @(Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -in @(${ids.join(',')}) } | Select-Object ProcessId,CreationDate) | ConvertTo-Json -Compress`;
      const remaining = execFileSync('powershell.exe', ['-NoProfile', '-Command', query], { windowsHide: true, encoding: 'utf8' }).trim();
      assert.ok(remaining === '' || remaining === '[]', `orphaned owned MCU/plant processes: ${remaining}`);
      console.log(JSON.stringify({ pass: true, kind, frozen_pid: evidence.target, threads: evidence.threads,
        timeout_ms: ended.at - frozenAt, terminal: ended.message, cleaned_pids: ids, run_identity: receivedIdentity }));
    } finally {
      if (helper && helper.exitCode === null) { helper.stdin.end('\n'); await bounded(once(helper, 'exit'), 5000, 'resume helper exit'); }
      socket.close();
    }
  }
} finally { killOwnedServer(); }
