import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, mkdirSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { hashValue, virtualPlatformDigest } from './run-identity.js';

test('identity includes adapter, installed emulator and dependency contract, not checkout location', () => {
  const roots = [mkdtempSync(join(tmpdir(), 'birdman-identity-')), mkdtempSync(join(tmpdir(), 'birdman-identity-'))] as const;
  try {
    for (const root of roots) {
      for (const folder of ['src', 'node_modules/rp2040js']) mkdirSync(join(root, 'virtual-platform', folder), { recursive: true });
      for (const file of ['src/a.ts', 'node_modules/rp2040js/index.js', 'package.json', 'package-lock.json']) {
        writeFileSync(join(root, 'virtual-platform', file), '{}', 'utf8');
      }
    }
    const first = roots[0];
    assert.equal(virtualPlatformDigest(first), virtualPlatformDigest(roots[1]));
    for (const file of ['src/a.ts', 'node_modules/rp2040js/index.js', 'package-lock.json']) {
      const before = virtualPlatformDigest(first);
      writeFileSync(join(first, 'virtual-platform', file), '{"changed":true}', 'utf8');
      assert.notEqual(virtualPlatformDigest(first), before);
    }
    assert.notEqual(hashValue({ dt: 0.01 }), hashValue({ dt: 0.02 }));
  } finally {
    for (const root of roots) rmSync(root, { recursive: true, force: true });
  }
});
