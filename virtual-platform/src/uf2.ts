// Adapted from the MIT-licensed embedded-rust-playground harness in the
// sibling repository. Keeping this loader local avoids a runtime cross-repo dependency.
import { readFileSync } from 'node:fs';
import type { RP2040 } from 'rp2040js';

const UF2_BLOCK_SIZE = 512;
const UF2_MAGIC_START0 = 0x0a324655;
const UF2_MAGIC_START1 = 0x9e5d5157;
const UF2_MAGIC_END = 0x0ab16f30;
const FLASH_START = 0x10000000;

export function loadUf2(path: string, mcu: RP2040): void {
  const image = readFileSync(path);
  if (image.length === 0 || image.length % UF2_BLOCK_SIZE !== 0) {
    throw new Error(`Invalid UF2 size: ${image.length}`);
  }
  for (let offset = 0; offset < image.length; offset += UF2_BLOCK_SIZE) {
    const block = image.subarray(offset, offset + UF2_BLOCK_SIZE);
    if (
      block.readUInt32LE(0) !== UF2_MAGIC_START0 ||
      block.readUInt32LE(4) !== UF2_MAGIC_START1 ||
      block.readUInt32LE(508) !== UF2_MAGIC_END
    ) {
      throw new Error(`Invalid UF2 magic at block ${offset / UF2_BLOCK_SIZE}`);
    }
    const targetAddress = block.readUInt32LE(12);
    const payloadSize = block.readUInt32LE(16);
    const flashOffset = targetAddress - FLASH_START;
    if (flashOffset < 0 || flashOffset + payloadSize > mcu.flash.length) {
      throw new Error(`UF2 block targets unsupported address 0x${targetAddress.toString(16)}`);
    }
    mcu.flash.set(block.subarray(32, 32 + payloadSize), flashOffset);
  }
}
