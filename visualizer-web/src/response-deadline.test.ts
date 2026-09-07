import test from 'node:test';
import assert from 'node:assert/strict';
import { ResponseDeadline } from './response-deadline';

test('wall-clock liveness expires even if no simulated time or response advances', () => {
  const deadline = new ResponseDeadline(5000);
  assert.equal(deadline.expired(10000), false);
  deadline.begin(10000);
  assert.equal(deadline.expired(14999), false);
  assert.equal(deadline.expired(15000), true);
  assert.throws(() => deadline.begin(15000), /overlapping/);
  deadline.complete();
  assert.equal(deadline.expired(20000), false);
  deadline.begin(20000);
  assert.equal(deadline.expired(24999), false);
});
