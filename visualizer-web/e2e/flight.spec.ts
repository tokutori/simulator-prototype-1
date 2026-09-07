import { test, expect } from '@playwright/test';

test('late startup sample cannot replace a user-selected replay', async ({ page }) => {
  let release!: () => void;
  const gate = new Promise<void>(resolve => { release = resolve; });
  await page.route('**/sample-flight.csv', async route => {
    await gate;
    await route.fulfill({ contentType: 'text/csv', body: 'time_s,north_m,altitude_m,pitch_deg,airspeed_mps\n0,0,10,0,2\n1,10,9,0,2\n' });
  });
  await page.goto('/');
  await page.locator('#csv-file').setInputFiles({ name: 'user-flight.csv', mimeType: 'text/csv',
    buffer: Buffer.from('time_s,north_m,altitude_m,pitch_deg,airspeed_mps\n0,0,10,0,11\n1,123,9,0,11\n') });
  await expect(page.locator('#connection-status')).toHaveText('user-flight.csv');
  await expect(page.locator('#airspeed-value')).toHaveText('11.0');
  const response = page.waitForResponse('**/sample-flight.csv');
  release();
  await (await response).finished();
  await page.evaluate(() => new Promise<void>(resolve => requestAnimationFrame(() => requestAnimationFrame(() => resolve()))));
  await expect(page.locator('#connection-status')).toHaveText('user-flight.csv');
  await expect(page.locator('#airspeed-value')).toHaveText('11.0');
});

test('real UF2 live restart, replay isolation and durable analysis tab', async ({ page, context }, testInfo) => {
  const errors: string[] = [];
  const frames: Record<string, unknown>[] = [];
  page.on('websocket', socket => socket.on('framereceived', event => {
    const data = JSON.parse(String(event.payload));
    if (data.backend === 'rp2040js-actual-uf2') frames.push(data);
  }));
  page.on('pageerror', error => errors.push(String(error)));
  await page.goto('/');
  await expect(page.locator('#mcu-performance')).toContainText('CPU ×1', { timeout: 45_000 });
  await expect(page.locator('#open-analysis')).toBeEnabled();
  await page.screenshot({ path: testInfo.outputPath('fhd-chase.png') });
  await page.locator('#autonomy').fill('0');
  // Restart keeps a normal button focused; keyboard flight input must still work.
  await page.locator('#restart-live').click();
  await expect(page.locator('#mcu-performance')).toContainText('CPU ×1', { timeout: 45_000 });
  await page.screenshot({ path: testInfo.outputPath('fhd-after-restart.png') });
  await page.keyboard.down('s');
  await expect.poll(() => frames.some(frame => frame.pilot_elevator === 1 && frame.autonomy === 0), { timeout: 15_000 }).toBe(true);
  await page.keyboard.up('s');
  const manual = frames.find(frame => frame.pilot_elevator === 1 && frame.autonomy === 0)!;
  expect(manual.firmware_sequence).toBeGreaterThan(0);
  expect(manual.automatic_valid).toBe(true);
  expect(Number(manual.mixed_elevator_command_rad)).toBeCloseTo(Math.PI / 18, 5);
  await page.locator('#restart-live').click();
  await page.locator('#replay-mode').click();
  await expect(page.locator('#mode-badge')).toHaveText('REPLAY');
  await expect(page.locator('#timeline')).toBeVisible();
  await page.locator('#live-mode').click();
  await expect(page.locator('#mcu-performance')).toContainText('CPU ×1', { timeout: 45_000 });
  await expect(page.locator('#open-analysis')).toBeEnabled();
  const opened = context.waitForEvent('page');
  await page.locator('#open-analysis').click();
  const analysis = await opened;
  await expect(analysis).toHaveURL(/analysis\.html\?flight=/);
  await expect(analysis.locator('#analysis-content')).toBeVisible();
  await expect(analysis.locator('#analysis-status')).toHaveText('ACTIVE FLIGHT SNAPSHOT');
  const duration = await analysis.locator('#summary-time').textContent();
  await analysis.reload();
  await expect(analysis.locator('#analysis-content')).toBeVisible();
  await expect(analysis.locator('#summary-time')).toHaveText(duration!);
  await page.locator('#replay-mode').click();
  await expect(page.locator('#mode-badge')).toHaveText('REPLAY');
  await expect(page.locator('#previous-analysis')).toBeVisible();
  expect(errors).toEqual([]);
});

test('12001 full evidence samples survive IndexedDB and render at 4K', async ({ page }, testInfo) => {
  await page.goto('/analysis.html');
  const id = await page.evaluate(async () => {
    // Explicit storage-capacity fixture; no flight-performance claim.
    const replayModule: string = '/src/replay.ts';
    const dataModule: string = '/src/analysis-data.ts';
    const storageModule: string = '/src/analysis-storage.ts';
    const { parseFlightCsv } = await import(replayModule) as typeof import('../src/replay.ts');
    const { prepareAnalysisDataset } = await import(dataModule) as typeof import('../src/analysis-data.ts');
    const { storeAnalysis, loadAnalysis } = await import(storageModule) as typeof import('../src/analysis-storage.ts');
    const base = parseFlightCsv(await (await fetch('/sample-flight.csv')).text())[0];
    if (!base) throw Error('sample fixture unavailable');
    const identity = { uf2_sha256: 'a'.repeat(64), model_sha256: 'b'.repeat(64), plant_sha256: 'c'.repeat(64) };
    const frames = Array.from({ length: 12001 }, (_, i) => ({ ...base, timeS: i / 100, northM: i / 10,
      controlTelemetry: { tag: 'firmware' as const, sequence: i, timeUs: 800000 + i * 10000,
        releaseMcuTimeUs: 800000, plantIntervalStartS: Math.max(0, (i - 1) / 100),
        automaticValid: true, runIdentity: identity, safeElevatorCommandRad: 0, safeRudderCommandRad: 0,
        observedElevatorCommandRad: 0, observedRudderCommandRad: 0,
        elevatorPwmSampleTimeUs: 800000 + i * 10000, rudderPwmSampleTimeUs: 800000 + i * 10000 } }));
    const id = crypto.randomUUID();
    await storeAnalysis(id, prepareAnalysisDataset('120s storage fixture', frames));
    const restored = await loadAnalysis(id);
    if (restored.frames.length !== 12001) throw Error('sample loss');
    const lastEvidence = restored.frames[12000]?.controlTelemetry;
    if (lastEvidence?.tag !== 'firmware' || lastEvidence.sequence !== 12000) throw Error('evidence loss');
    return id;
  });
  await page.setViewportSize({ width: 3840, height: 2160 });
  await page.goto(`/analysis.html?flight=${id}`);
  await expect(page.locator('#analysis-content')).toBeVisible();
  await expect(page.locator('#summary-time')).toHaveText('120.00 s');
  await expect(page.locator('#summary-range')).toHaveText('1200.0 m');
  await page.screenshot({ path: testInfo.outputPath('4k-analysis.png'), fullPage: true });
  await page.reload();
  await expect(page.locator('#summary-time')).toHaveText('120.00 s');
});
