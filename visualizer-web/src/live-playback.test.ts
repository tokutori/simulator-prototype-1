import assert from "node:assert/strict";
import test from "node:test";

import { LivePlayback } from "./live-playback.ts";
import type { FlightFrame } from "./types.ts";

const frame = (timeS: number, northM: number): FlightFrame => ({
  experiment: { tag: "unknown" },
  controlTelemetry: { tag: "unavailable" },
  timeS,
  northM,
  eastM: 0,
  altitudeM: 10,
  rollRad: 0,
  pitchRad: 0,
  yawRad: 0,
  flightPathRad: 0,
  airspeedMps: 5,
  alphaRad: 0,
  elevatorRad: 0,
  rudderRad: 0,
  pilotElevator: 0,
  pilotRudder: 0,
  autonomy: 1,
  manualElevatorCommandRad: 0,
  manualRudderCommandRad: 0,
  automaticElevatorCommandRad: 0,
  automaticRudderCommandRad: 0,
  mixedElevatorCommandRad: 0,
  mixedRudderCommandRad: 0,
  surfaceContact: false,
});

test("slow producer arrivals cannot teleport the presentation clock", () => {
  const playback = new LivePlayback();
  let previous = 0;
  for (let now = 0; now <= 400; now++) {
    if (now % 80 === 0) playback.push(frame(now / 8000, now / 800), now);
    const position = playback.frame(now)!.northM;
    assert.ok(position >= previous);
    assert.ok(position - previous <= 0.0013, `jump at ${now}: ${position - previous}`);
    assert.ok(position <= Math.floor(now / 80) * 0.1 + 1e-12);
    previous = position;
  }
});

test("long stalls hold real evidence and eventual terminal pose is reached", () => {
  const playback = new LivePlayback();
  for (let i = 0; i < 4; i++) playback.push(frame(i * 0.01, i * 0.1), i * 10);
  playback.frame(30);
  const before = playback.frame(1000)!;
  assert.ok(before.timeS <= 0.03);
  for (let now = 1010; now <= 2000; now += 10) playback.frame(now);
  assert.equal(playback.frame(2010)!.timeS, 0.03);
});

test("terminal records shorter than the prime buffer still drain to their final pose", () => {
  const playback = new LivePlayback();
  playback.push(frame(0, 0), 0);
  playback.push(frame(0.01, 0.1), 10);
  playback.frame(10);
  playback.finish();
  assert.equal(playback.frame(30)!.timeS, 0.01);
});

test("low render rate does not accumulate an artificial simulation-clock backlog", () => {
  const playback = new LivePlayback();
  playback.push(frame(0, 0), 0);
  playback.frame(0);
  let shown = 0;
  for (let now = 10; now <= 1000; now += 10) {
    playback.push(frame(now / 1000, now / 100), now);
    if (now % 100 === 0) shown = playback.frame(now)!.timeS;
  }
  assert.ok(shown >= 0.99);
});

test("60 seconds of alternating delivery jitter has bounded lag at real-time and slow producer speeds", () => {
  for (const slowdown of [1, 8]) {
    const playback = new LivePlayback();
    let nextArrival = 0;
    let sampleIndex = 0;
    let displayed = 0;
    for (let wallMs = 0; wallMs <= 60_000; wallMs++) {
      if (wallMs === nextArrival) {
        playback.push(frame(sampleIndex / 100, sampleIndex / 10), wallMs);
        sampleIndex++;
        nextArrival += (sampleIndex % 2 === 1 ? 5 : 15) * slowdown;
      }
      const lastSimulationS = (sampleIndex - 1) / 100;
      if (wallMs % 16 === 0) {
        const next = playback.frame(wallMs)!.timeS;
        assert.ok(next >= displayed, "presentation time must never run backward");
        assert.ok(next <= lastSimulationS, "presentation must never invent a future aircraft state");
        displayed = next;
      }
      if (wallMs > 1000) assert.ok(lastSimulationS - displayed < 0.06,
        `accumulating lag at slowdown ${slowdown}, wall ${wallMs}: ${lastSimulationS - displayed}`);
    }
    assert.ok(Math.abs(displayed - 60 / slowdown) < 0.06);
  }
});

test("window-estimator transient lag recovers after a slow producer becomes real-time", () => {
  const playback = new LivePlayback();
  let nextArrival = 0;
  let sampleIndex = 0;
  let displayed = 0;
  let transientLag = 0;
  for (let wallMs = 0; wallMs <= 6000; wallMs++) {
    if (wallMs === nextArrival) {
      playback.push(frame(sampleIndex / 100, sampleIndex / 10), wallMs);
      sampleIndex++;
      nextArrival += wallMs < 2000 ? 80 : 10;
    }
    if (wallMs % 16 === 0) {
      const next = playback.frame(wallMs)!.timeS;
      assert.ok(next >= displayed && next <= (sampleIndex - 1) / 100);
      displayed = next;
    }
    if (wallMs === 3000) transientLag = (sampleIndex - 1) / 100 - displayed;
  }
  assert.ok(transientLag > 0.1, "fixture must exercise an estimator transient");
  assert.ok((sampleIndex - 1) / 100 - displayed < 0.04, "lag must recover, not persist for the rest of the flight");
});

test("a new session resets the rate window and ordered-sample contract rejects stale frames", () => {
  const previous = new LivePlayback();
  previous.push(frame(100, 1000), 1000);
  assert.throws(() => previous.push(frame(99, 990), 1001), /must be ordered/);
  assert.throws(() => previous.push(frame(101, 1010), 999), /must be ordered/);
  const restarted = new LivePlayback();
  restarted.push(frame(0, 0), 2000);
  assert.equal(restarted.frame(2000)!.timeS, 0);
});
