import assert from "node:assert/strict";
import { test } from "node:test";

import { uiScaleForViewport } from "./ui-scale.ts";

test("UI scale follows the shortest viewport side from FHD through 4K", () => {
  assert.equal(uiScaleForViewport(1920, 1080), 1);
  assert.ok(Math.abs(uiScaleForViewport(2560, 1440) - 4 / 3) < 1e-12);
  assert.equal(uiScaleForViewport(3840, 2160), 2);
  assert.equal(uiScaleForViewport(3840, 1080), 1);
});

test("UI scale remains usable outside the target resolution range", () => {
  assert.equal(uiScaleForViewport(1280, 720), 1);
  assert.equal(uiScaleForViewport(7680, 4320), 2);
  assert.equal(uiScaleForViewport(Number.NaN, 1080), 1);
});
