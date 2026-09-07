import assert from "node:assert/strict";
import { test } from "node:test";
import { Quaternion, Vector3 } from "three";
import { ChaseCamera } from "./chase-camera.ts";

test("restart/seek reset snaps eye and target to the new timeline", () => {
  const camera = new ChaseCamera();
  camera.update(new Vector3(0, 0, -232), new Quaternion(), 1 / 60);
  camera.reset();
  const pose = camera.update(new Vector3(0, 10, 0), new Quaternion(), 1 / 60);
  assert.deepEqual(pose.eye.toArray(), [0, 16, 19]);
  assert.deepEqual(pose.target.toArray(), [0, 10, -4]);
});

test("eye and target share one smoothed anchor, including a telemetry position jump", () => {
  const camera = new ChaseCamera();
  camera.update(new Vector3(), new Quaternion(), 0);
  const pose = camera.update(new Vector3(100, 10, -232), new Quaternion(), 1 / 60);
  assert.deepEqual(pose.target.clone().sub(pose.eye).toArray(), [0, -6, -23]);
  assert.ok(pose.target.x < 10);
});

test("heading change preserves orbit radius and takes the short path", () => {
  const camera = new ChaseCamera();
  camera.update(new Vector3(), new Quaternion(), 0);
  const pose = camera.update(new Vector3(), new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2), 0.1);
  assert.ok(pose.target.x < 0);
  assert.ok(Math.abs(pose.target.distanceTo(pose.eye) - Math.hypot(23, 6)) < 1e-12);
});

test("stationary-target smoothing is refresh-rate independent", () => {
  const result = (fps: number): Vector3 => {
    const camera = new ChaseCamera();
    camera.update(new Vector3(), new Quaternion(), 0);
    let eye = new Vector3();
    for (let i = 0; i < fps; i++) eye = camera.update(new Vector3(10, 0, 0), new Quaternion(), 1 / fps).eye;
    return eye;
  };
  assert.ok(result(60).distanceTo(result(240)) < 1e-10);
});
