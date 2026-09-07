import assert from 'node:assert/strict';
import { test } from 'node:test';
import { advanceUntil } from './execution-budget.js';

test('120 seconds of healthy steps never exhaust a lifetime instruction budget', () => {
  let total = 0;
  for (let step = 0; step < 12_000; step++) {
    let progress = 0;
    total += advanceUntil(() => progress === 10, () => { progress++; }, 11);
  }
  assert.equal(total, 120_000);
});

test('a stuck step still trips the bounded progress watchdog', () => {
  assert.throws(() => advanceUntil(() => false, () => {}, 100), /insufficient progress/);
});
