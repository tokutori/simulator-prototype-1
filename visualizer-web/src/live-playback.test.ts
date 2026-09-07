import assert from "node:assert/strict";
import test from "node:test";

import { LivePlayback } from "./live-playback.ts";
import type { FlightFrame } from "./types.ts";

const frame = (timeS: number, northM: number): FlightFrame => ({
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
