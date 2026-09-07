import assert from "node:assert/strict";
import { test } from "node:test";

import {
  downsampleFrames,
  parseAnalysisDataset,
  prepareAnalysisDataset,
  summarizeFlight,
} from "./analysis-data.ts";
import type { FlightFrame } from "./types.ts";

function frame(timeS: number, northM: number, eastM: number): FlightFrame {
  return {
    experiment: { tag: "unknown" },
    controlTelemetry: { tag: "unavailable" },
    timeS, northM, eastM, altitudeM: 10 - timeS, rollRad: timeS * 0.1,
    pitchRad: 0, yawRad: 0, flightPathRad: 0, airspeedMps: 5 + timeS,
    alphaRad: 0, elevatorRad: 0, rudderRad: 0, pilotElevator: 0, pilotRudder: 0,
    autonomy: 1, manualElevatorCommandRad: 0, manualRudderCommandRad: 0,
    automaticElevatorCommandRad: 0, automaticRudderCommandRad: 0,
    mixedElevatorCommandRad: 0, mixedRudderCommandRad: 0, surfaceContact: timeS === 2,
  };
}

test("downsampling preserves the first and last frames", () => {
  const frames = Array.from({ length: 101 }, (_, index) => frame(index, index, 0));
  const sampled = downsampleFrames(frames, 11);
  assert.equal(sampled.length, 11);
  assert.equal(sampled[0]?.timeS, 0);
  assert.equal(sampled.at(-1)?.timeS, 100);
});

test("flight summary distinguishes range and flown track", () => {
  const frames = [frame(0, 0, 0), frame(1, 3, 4), frame(2, 6, 0)];
  const summary = summarizeFlight(frames);
  assert.equal(summary.rangeM, 6);
  assert.equal(summary.trackM, 10);
  assert.equal(summary.surfaceContact, true);
  const dataset = prepareAnalysisDataset("test", frames, new Date("2026-08-22T00:00:00Z"));
  assert.equal(dataset.storedAtIso, "2026-08-22T00:00:00.000Z");
  assert.equal(parseAnalysisDataset(JSON.stringify(dataset)).frames.length, 3);
});

test("analysis parser rejects malformed persisted data", () => {
  assert.throws(() => parseAnalysisDataset('{"version":2,"frames":[]}'), /unsupported format/);
});

test("analysis preserves all long-run samples and brief extrema", () => {
  const frames = Array.from({ length: 12001 }, (_, index) => frame(index / 100, index / 10, 0));
  frames[7] = { ...frame(0.07, 0.7, 0), altitudeM: 99, rollRad: 1 };
  const dataset = prepareAnalysisDataset("long-run", frames);
  assert.equal(dataset.frames.length, 12001);
  assert.equal(summarizeFlight(dataset.frames).maximumAltitudeM, 99);
  assert.equal(parseAnalysisDataset(JSON.stringify(dataset)).frames.length, 12001);
});

test("all session outcomes and stall incidents survive storage without inferring a successful end", () => {
  const frames = [frame(0, 0, 0), frame(1, 1, 0)];
  for (const outcome of [{ tag: "active" }, { tag: "ended", reason: "surface contact" },
    { tag: "failed", reason: "aircraft model envelope violation" }, { tag: "failed", reason: "MCU disconnected" },
    { tag: "aborted", reason: "User restarted the flight" }] as const) {
    const incidents = [{ kind: "telemetry-stall" as const, wallTimeIso: "2026-09-08T00:00:00Z", sinceLastReceiptMs: 750 }];
    const restored = parseAnalysisDataset(JSON.stringify(prepareAnalysisDataset("test", frames, new Date(), outcome, incidents)));
    assert.deepEqual(restored.outcome, outcome);
    assert.deepEqual(restored.incidents, incidents);
  }
});

test("preflight failure is retained even when no telemetry exists", () => {
  const record = prepareAnalysisDataset("failed startup", [], new Date(), { tag: "failed", reason: "UF2 did not arm" });
  assert.deepEqual(parseAnalysisDataset(JSON.stringify(record)).outcome, record.outcome);
});

test("legacy analysis is explicitly unknown; malformed timing and nonmonotonic samples are rejected", () => {
  const frames = [frame(0, 0, 0), frame(1, 1, 0)];
  const old = { version: 2, name: "old", frames };
  const restored = parseAnalysisDataset(JSON.stringify(old));
  assert.equal(restored.outcome.tag, "unknown");
  assert.equal(restored.frames[0]!.experiment.tag, "unknown");
  const dataset = prepareAnalysisDataset("test", frames);
  assert.throws(() => parseAnalysisDataset(JSON.stringify({ ...dataset, frames: [frames[1], frames[0]] })), /increase strictly/);
});

test("saved firmware must contain every mandatory field; legacy partial evidence is unavailable", () => {
  const frames = [frame(0, 0, 0), frame(1, 1, 0)];
  const incomplete = { tag: "firmware", automaticValid: true, runIdentity: {
    uf2_sha256: "a".repeat(64), model_sha256: "b".repeat(64), plant_sha256: "c".repeat(64) } };
  const malformed = { ...prepareAnalysisDataset("bad", frames), frames: frames.map(value => ({ ...value, controlTelemetry: incomplete })) };
  assert.throws(() => parseAnalysisDataset(JSON.stringify(malformed)), /invalid telemetry/);
  const legacy = parseAnalysisDataset(JSON.stringify({ ...malformed, version: 2 }));
  assert.equal(legacy.frames[0]!.controlTelemetry.tag, "unavailable");
});
