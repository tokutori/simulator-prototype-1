import test from 'node:test';
import assert from 'node:assert/strict';
import { allowLiveAccess } from './live-access';

test('only same-origin local browser and explicit local CLI targets can start MCU sessions', () => {
  assert.equal(allowLiveAccess('127.0.0.1:4173', 'http://127.0.0.1:4173', 4173), true);
  assert.equal(allowLiveAccess('localhost:4173', undefined, 4173), true);
  for (const origin of ['https://untrusted.example', 'null', 'http://127.0.0.1:5173']) {
    assert.equal(allowLiveAccess('127.0.0.1:4173', origin, 4173), false);
  }
  assert.equal(allowLiveAccess('untrusted.example:4173', undefined, 4173), false);
});
