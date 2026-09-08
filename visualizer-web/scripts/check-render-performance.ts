import { chromium } from '@playwright/test';

// Diagnostic, not a portable speed gate: hardware, GPU and host load matter.
const browser = await chromium.launch();
try {
  for (const [width, height] of [[1920, 1080], [3840, 2160]]) {
    const page = await browser.newPage({ viewport: { width: width!, height: height! } });
    const samples: { time: number; ratio: number; mcuMs: number; plantMs: number; lagMs: number }[] = [];
    const errors: string[] = [];
    page.on('pageerror', error => errors.push(String(error)));
    page.on('websocket', socket => socket.on('framereceived', ({ payload }) => {
      if (!String(payload).startsWith('{')) return;
      const value = JSON.parse(String(payload));
      if (value.type === 'error') errors.push(value.message);
      if (value.backend !== 'rp2040js-actual-uf2') return;
      if (value.emulation.timing_acceleration !== 1) throw Error('CPU clock acceleration is forbidden');
      samples.push({ time: value.time_s, ratio: value.emulation.real_time_ratio,
        mcuMs: value.emulation.mcu_processing_ms, plantMs: value.emulation.plant_round_trip_ms,
        lagMs: value.emulation.lag_ms });
    }));
    await page.goto(process.env.BIRDMAN_VISUALIZER_URL ?? 'http://127.0.0.1:4173');
    await page.locator('#mcu-performance').filter({ hasText: 'CPU ×1' }).waitFor({ timeout: 45_000 });
    const graphics = await page.evaluate(() => {
      const gl = document.querySelector('canvas')?.getContext('webgl2');
      if (!gl) return { renderer: 'unavailable' };
      const debug = gl.getExtension('WEBGL_debug_renderer_info');
      return { renderer: debug ? String(gl.getParameter(debug.UNMASKED_RENDERER_WEBGL)) : 'masked',
        drawingWidth: gl.drawingBufferWidth, drawingHeight: gl.drawingBufferHeight };
    });
    // A source string keeps tsx's function-name helper out of browser scope.
    const rendering = await page.evaluate<{ count: number; elapsedMs: number; p95Ms: number }>(`new Promise(resolve => {
      const intervals = [];
      const start = performance.now();
      let previous = start;
      function tick(now) {
        intervals.push(now - previous); previous = now;
        if (now - start < 10_000) { requestAnimationFrame(tick); return; }
        intervals.sort((a, b) => a - b);
        resolve({ count: intervals.length, elapsedMs: now - start,
          p95Ms: intervals[Math.floor(intervals.length * 0.95)] });
      }
      requestAnimationFrame(tick);
    })`);
    if (samples.length < 100 || errors.length) throw Error(JSON.stringify({ samples: samples.length, errors }));
    const settled = samples.filter(sample => sample.time >= 1);
    if (!settled.length) throw Error('Insufficient settled telemetry');
    console.log(JSON.stringify({ viewport: { width, height }, backend: 'rp2040js-actual-uf2',
      samples: samples.length, last: samples.at(-1), minSettledRatio: Math.min(...settled.map(s => s.ratio)),
      meanMcuMs: settled.reduce((sum, s) => sum + s.mcuMs, 0) / settled.length,
      meanPlantMs: settled.reduce((sum, s) => sum + s.plantMs, 0) / settled.length,
      graphics, renderedFps: rendering.count * 1000 / rendering.elapsedMs, p95FrameMs: rendering.p95Ms }));
    await page.close();
  }
} finally {
  await browser.close();
}
