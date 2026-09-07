import assert from "node:assert/strict";
import test from "node:test";

import { PilotInput, defaultInputSettings, sanitizeSettings, shapeAnalog } from "./input.ts";

test("analog shaping removes the dead zone and preserves endpoints", () => {
  assert.equal(shapeAnalog(0.05, 0.08, 1), 0);
  assert.equal(shapeAnalog(-0.05, 0.08, 1), 0);
  assert.equal(shapeAnalog(1, 0.08, 1.5), 1);
  assert.equal(shapeAnalog(-1, 0.08, 1.5), -1);
});

test("physical buttons press and release immediately, opposite buttons cancel", () => {
  const input = new PilotInput(structuredClone(defaultInputSettings));
  input.pressedCodes.add("KeyS");
  input.update(0.1, null);
  assert.equal(input.elevator, 1);
  input.pressedCodes.clear();
  input.update(0.001, null);
  assert.equal(input.elevator, 0);
  input.pressedCodes.add("KeyS");
  input.pressedCodes.add("KeyW");
  input.update(0.1, null);
  assert.equal(input.elevator, 0);
});

test("invalid persisted values fall back to bounded defaults", () => {
  const settings = sanitizeSettings({
    deadZone: 5,
    responseExponent: Number.NaN,
    gamepadIndex: -2,
    elevator: { source: "unknown", gamepadAxis: 99 },
  });
  assert.equal(settings.deadZone, defaultInputSettings.deadZone);
  assert.equal(settings.responseExponent, defaultInputSettings.responseExponent);
  assert.equal(settings.gamepadIndex, 0);
  assert.equal(settings.elevator.source, defaultInputSettings.elevator.source);
  assert.equal(settings.elevator.gamepadAxis, defaultInputSettings.elevator.gamepadAxis);
});
