import assert from "node:assert/strict";
import test from "node:test";
import { Group, Quaternion, Vector3 } from "three";

import { resetXrRig, setXrCockpitRig } from "./xr-camera.ts";

const tolerance = 1e-10;

test("XR rig places the reference-space origin at the aircraft cockpit eye", () => {
  const rig = new Group();
  const aircraftPosition = new Vector3(5, 10, -30);
  const attitude = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);

  setXrCockpitRig(rig, aircraftPosition, attitude);

  const expected = new Vector3(0, 0.11, -1.18).applyQuaternion(attitude).add(aircraftPosition);
  assert.ok(rig.position.distanceTo(expected) < tolerance);
  assert.ok(1 - Math.abs(rig.quaternion.dot(attitude)) < tolerance);
});

test("leaving XR restores an identity parent for ordinary cameras", () => {
  const rig = new Group();
  setXrCockpitRig(
    rig,
    new Vector3(5, 10, -30),
    new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), 0.5),
  );

  resetXrRig(rig);

  assert.deepEqual(rig.position.toArray(), [0, 0, 0]);
  assert.deepEqual(rig.quaternion.toArray(), [0, 0, 0, 1]);
  assert.deepEqual(rig.scale.toArray(), [1, 1, 1]);
});
