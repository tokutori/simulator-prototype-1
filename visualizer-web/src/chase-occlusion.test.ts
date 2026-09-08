import assert from "node:assert/strict";
import { test } from "node:test";
import { Box3, Quaternion, Vector3 } from "three";
import { launchPlatformOpacity } from "./chase-occlusion.ts";

const bounds = new Box3(new Vector3(-5.1, 0, -0.055), new Vector3(5.1, 10.36, 15));

test("early descent fades the platform intersecting the chase view", () => {
  assert.equal(launchPlatformOpacity("chase", new Vector3(0, 13, 10), new Vector3(0, 7, -9), new Quaternion(), bounds), 0.08);
});

test("camera inside the platform and a tail behind its edge stay visible", () => {
  assert.equal(launchPlatformOpacity("chase", new Vector3(0, 9, 5), new Vector3(0, 7, -14), new Quaternion(), bounds), 0.08);
  assert.equal(launchPlatformOpacity("chase", new Vector3(0, 13, 5), new Vector3(0, 7, -4), new Quaternion(), bounds), 0.08);
});

test("cockpit/VR retain the opaque physical scene even with intersecting sight lines", () => {
  assert.equal(launchPlatformOpacity("cockpit", new Vector3(0, 13, 10), new Vector3(0, 7, -9), new Quaternion(), bounds), 1);
});

test("clear flight and an occluder behind the aircraft do not fade", () => {
  assert.equal(launchPlatformOpacity("chase", new Vector3(0, 11, -81), new Vector3(0, 5, -100), new Quaternion(), bounds), 1);
  assert.equal(launchPlatformOpacity("chase", new Vector3(0, 20, -50), new Vector3(0, 15, -30), new Quaternion(), bounds), 1);
});

test("occlusion check does not mutate the aircraft, camera, or world bounds", () => {
  const eye = new Vector3(0, 13, 10);
  const aircraft = new Vector3(0, 7, -9);
  const attitude = new Quaternion();
  const before = [eye.toArray(), aircraft.toArray(), attitude.toArray(), bounds.min.toArray(), bounds.max.toArray()];
  launchPlatformOpacity("chase", eye, aircraft, attitude, bounds);
  assert.deepEqual([eye.toArray(), aircraft.toArray(), attitude.toArray(), bounds.min.toArray(), bounds.max.toArray()], before);
});
