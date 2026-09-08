import assert from 'node:assert/strict';
import WebSocket from 'ws';

const base = process.env.BIRDMAN_VISUALIZER_URL ?? 'http://127.0.0.1:4173';
for (let attempt = 1; attempt <= 12; attempt++) {
  const start = performance.now();
  const socket = new WebSocket(base.replace(/^http/, 'ws') + '/live');
  let received = false;
  await new Promise<void>((resolve, reject) => {
    const timer = setTimeout(() => { socket.terminate(); reject(Error('restart observation timeout')); }, 20000);
    socket.on('error', error => { clearTimeout(timer); reject(error); });
    socket.on('message', raw => {
      try {
        const value = JSON.parse(String(raw));
        assert.equal(value.backend, 'rp2040js-actual-uf2', JSON.stringify(value));
        assert.equal(value.emulation.timing_acceleration, 1);
        if (!received) console.log(JSON.stringify({ attempt, firstTelemetryMs: performance.now() - start,
          releaseMcuTimeUs: value.release_mcu_time_us, runIdentity: value.run_identity }));
        received = true;
        socket.close();
      } catch (error) { clearTimeout(timer); socket.terminate(); reject(error); }
    });
    socket.on('close', () => { clearTimeout(timer); if (received) resolve(); else reject(Error('closed before telemetry')); });
  });
}
