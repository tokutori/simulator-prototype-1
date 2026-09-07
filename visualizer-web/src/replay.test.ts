import assert from "node:assert/strict";
import test from "node:test";

import { frameFromLive, interpolateFrame, parseFlightCsv } from "./replay.ts";
import type { InteractiveObservation } from "./types.ts";

test("CSV parser accepts simulator output and interpolation is continuous", () => {
  const frames = parseFlightCsv(
    "time_s,north_m,east_m,altitude_m,roll_deg,pitch_deg,yaw_deg,flight_path_deg,airspeed_mps,alpha_deg,elevator_deg,rudder_deg,surface_contact\n" +
      "0,0,0,10,0,-3,179,-3,5,2,0,0,false\n" +
      "1,8,2,9,10,1,-179,-1,9,3,2,-1,true\n",
  );
  const middle = interpolateFrame(frames, 0.5);
  assert.equal(middle.northM, 4);
  assert.equal(middle.eastM, 1);
  assert.equal(middle.altitudeM, 9.5);
  assert.ok(Math.abs(Math.abs(middle.yawRad) - Math.PI) < 1e-12);
  assert.equal(frames[0]?.surfaceContact, false);
  assert.equal(frames[1]?.surfaceContact, true);
});

test("firmware evidence stays discrete during presentation interpolation", () => {
  const frames = parseFlightCsv(
    "time_s,north_m,altitude_m,pitch_deg,firmware_sequence,firmware_time_us,automatic_valid,elevator_pwm_sample_time_us\n" +
    "0,0,10,-3,3,123000,false,122000\n1,8,9,-2,5,143000,true,142000\n",
  );
  assert.equal(interpolateFrame(frames, 0.5).controlTelemetry.tag, "firmware");
  assert.deepEqual(interpolateFrame(frames, 0.5).controlTelemetry, frames[0]?.controlTelemetry);
  assert.equal(parseFlightCsv("time_s,north_m,altitude_m,pitch_deg\n0,0,10,-3\n1,8,9,-2\n")[0]?.controlTelemetry.tag, "unavailable");
});

test("live JSON without firmware evidence is rejected instead of plotted as real telemetry", () => {
  assert.throws(() => frameFromLive({} as InteractiveObservation), /Invalid actual-UF2/);
});

test("CSV parser rejects non-monotonic time", () => {
  assert.throws(
    () =>
      parseFlightCsv(
        "time_s,north_m,altitude_m,pitch_deg\n0,0,10,-3\n0,1,9,-2\n",
      ),
    /increase strictly/,
  );
});
