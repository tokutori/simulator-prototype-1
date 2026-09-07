import { createHash } from 'node:crypto';
import { readFileSync, readdirSync } from 'node:fs';
import { join, relative } from 'node:path';

export function hashValue(value: unknown): string {
  return createHash('sha256').update(JSON.stringify(value)).digest('hex');
}

/** Fingerprint actual adapter and installed MCU implementation, not only UF2.
 * Paths are relative and sorted, so checkout location does not change identity.
 * Tests are deliberately included: conservative extra changes are acceptable,
 * whereas omitting executing code could falsely identify two different runs.
 */
export function virtualPlatformDigest(root: string): string {
  const files: string[] = [];
  const visit = (folder: string): void => {
    for (const entry of readdirSync(folder, { withFileTypes: true })) {
      const path = join(folder, entry.name);
      if (entry.isDirectory()) visit(path);
      else if (entry.isFile()) files.push(path);
      else throw new Error(`unsupported identity input: ${path}`);
    }
  };
  visit(join(root, 'virtual-platform/src'));
  visit(join(root, 'virtual-platform/node_modules/rp2040js'));
  files.push(join(root, 'virtual-platform/package.json'), join(root, 'virtual-platform/package-lock.json'));
  const entries = files.map(path => [relative(root, path).replaceAll('\\', '/'),
    createHash('sha256').update(readFileSync(path)).digest('hex')]).sort((a, b) =>
      JSON.stringify(a).localeCompare(JSON.stringify(b), 'en'));
  return hashValue({ format: 1, entries, runtime: { node: process.version,
    v8: process.versions.v8, architecture: process.arch, platform: process.platform } });
}
