import assert from "node:assert/strict";
import { test } from "node:test";

import { distanceRingSpecs } from "./distance-rings.ts";

test("distance rings use 25 m intervals and label every 50 m", () => {
  const rings = distanceRingSpecs(125, 25);
  assert.deepEqual(rings.map((ring) => ring.radiusM), [25, 50, 75, 100, 125]);
  assert.deepEqual(rings.map((ring) => ring.label), [undefined, "50 m", undefined, "100 m", undefined]);
});
