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
