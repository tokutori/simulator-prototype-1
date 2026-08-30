import assert from "node:assert/strict";
import test from "node:test";

import { frameAtReceiptTime, trimReceiptBuffer, type ReceivedFlightFrame } from "./live-playback.ts";
import type { FlightFrame } from "./types.ts";

const frame = (timeS: number, northM: number): FlightFrame => ({
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

test("receipt-time playback interpolates irregular live arrivals", () => {
  const samples: ReceivedFlightFrame[] = [
    { frame: frame(0, 0), receivedAtMs: 100 },
    { frame: frame(0.01, 1), receivedAtMs: 112 },
    { frame: frame(0.02, 2), receivedAtMs: 130 },
  ];

  const between = frameAtReceiptTime(samples, 121);
  assert.ok(between);
  assert.equal(between.timeS, 0.015);
  assert.equal(between.northM, 1.5);
});

test("receipt buffer preserves a bracketing pair", () => {
  const samples: ReceivedFlightFrame[] = [
    { frame: frame(0, 0), receivedAtMs: 100 },
    { frame: frame(0.01, 1), receivedAtMs: 110 },
    { frame: frame(0.02, 2), receivedAtMs: 120 },
    { frame: frame(0.03, 3), receivedAtMs: 130 },
  ];

  trimReceiptBuffer(samples, 125);
  assert.deepEqual(samples.map((sample) => sample.receivedAtMs), [120, 130]);
});
