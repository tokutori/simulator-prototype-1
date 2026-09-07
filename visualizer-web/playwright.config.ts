import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './e2e', workers: 1, timeout: 90_000,
  use: { baseURL: 'http://127.0.0.1:4183', headless: true,
    viewport: { width: 1920, height: 1080 }, trace: 'retain-on-failure' },
  webServer: {
    command: 'node --import tsx server.ts', url: 'http://127.0.0.1:4183',
    env: { BIRDMAN_VISUALIZER_PORT: '4183' }, reuseExistingServer: false,
  },
});
