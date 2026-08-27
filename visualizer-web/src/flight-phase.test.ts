import assert from "node:assert/strict";
import { test } from "node:test";

import { classifyFlightPhase, formatFlightTime } from "./flight-phase.ts";

test("flight phases distinguish launch, contact, and truncated replay", () => {
  assert.equal(classifyFlightPhase({ hasFrame: false, elapsedS: 0, surfaceContact: false, replayAtEnd: false }), "ready");
  assert.equal(classifyFlightPhase({ hasFrame: true, elapsedS: 0.8, surfaceContact: false, replayAtEnd: false }), "launch");
  assert.equal(classifyFlightPhase({ hasFrame: true, elapsedS: 9, surfaceContact: true, replayAtEnd: true }), "water-contact");
  assert.equal(classifyFlightPhase({ hasFrame: true, elapsedS: 9, surfaceContact: false, replayAtEnd: true }), "record-ended");
  assert.equal(classifyFlightPhase({ hasFrame: true, elapsedS: 9, surfaceContact: false, replayAtEnd: false }), "flying");
});

test("flight time uses an unambiguous T+ minute clock", () => {
  assert.equal(formatFlightTime(0), "00:00.00");
  assert.equal(formatFlightTime(65.25), "01:05.25");
  assert.equal(formatFlightTime(-1), "00:00.00");
});
