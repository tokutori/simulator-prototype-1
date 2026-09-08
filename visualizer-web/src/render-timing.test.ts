import { test } from 'node:test';
import assert from 'node:assert/strict';
import { presentationDeltaS, presentRenderTiming, updateRenderTiming, type RenderTiming } from './render-timing.ts';

test('render health measures real frame time independently of MCU speed', () => {
  let state: RenderTiming = { startMs: 0, frames: 0, status: { tag: 'measuring' } };
  for (let i = 1; i <= 10; i++) state = updateRenderTiming(state, { type: 'frame', nowMs: i * 100 });
  assert.deepEqual(state.status, { tag: 'measured', fps: 10, slow: true });
  assert.match(presentRenderTiming(state.status), /LOW FPS/);
  state = updateRenderTiming(state, { type: 'reset', nowMs: 10_000 });
  assert.deepEqual(state.status, { tag: 'measuring' });
  for (let i = 1; i <= 50; i++) state = updateRenderTiming(state, { type: 'frame', nowMs: 10_000 + i * 20 });
  assert.deepEqual(state.status, { tag: 'measured', fps: 50, slow: false });
});

test('low-FPS presentation does not turn real time into slow motion', () => {
  assert.equal(presentationDeltaS(0, 200), 0.2);
  assert.equal(presentationDeltaS(200, 100), 0);
});
