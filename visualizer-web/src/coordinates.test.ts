import assert from "node:assert/strict";
import test from "node:test";
import { Vector3 } from "three";

import { nedEulerToThreeQuaternion, nedPositionToThree } from "./coordinates.ts";

const tolerance = 1e-10;

test("NED position maps north away from the platform and altitude upward", () => {
  const mapped = nedPositionToThree(10, 3, 7);
  assert.deepEqual(mapped.toArray(), [3, 7, -10]);
});

test("zero attitude points the aircraft local forward toward north", () => {
  const quaternion = nedEulerToThreeQuaternion(0, 0, 0);
  const forward = new Vector3(0, 0, -1).applyQuaternion(quaternion);
  assert.ok(forward.distanceTo(new Vector3(0, 0, -1)) < tolerance);
});

test("positive ninety degree yaw points forward east", () => {
  const quaternion = nedEulerToThreeQuaternion(0, 0, Math.PI / 2);
  const forward = new Vector3(0, 0, -1).applyQuaternion(quaternion);
  assert.ok(forward.distanceTo(new Vector3(1, 0, 0)) < tolerance);
});

test("positive pitch raises the nose", () => {
  const quaternion = nedEulerToThreeQuaternion(0, Math.PI / 6, 0);
  const forward = new Vector3(0, 0, -1).applyQuaternion(quaternion);
  assert.ok(forward.y > 0);
  assert.ok(forward.z < 0);
});

test("positive roll lowers the right wing", () => {
  const quaternion = nedEulerToThreeQuaternion(Math.PI / 6, 0, 0);
  const right = new Vector3(1, 0, 0).applyQuaternion(quaternion);
  assert.ok(right.y < 0);
  assert.ok(right.x > 0);
});
