import assert from "node:assert/strict";
import test from "node:test";

import { frameFromLive, interpolateFrame, parseFlightCsv, parseCsvOutcome, parseCsvIncidents, experimentColumns, experimentCsvValues } from "./replay.ts";
import type { InteractiveObservation, ExperimentEvidence } from "./types.ts";

const firmwareHeader = "firmware_sequence,firmware_time_us,release_mcu_time_us,plant_interval_start_s,automatic_valid,safe_elevator_command_deg,safe_rudder_command_deg,observed_elevator_command_deg,observed_rudder_command_deg,elevator_pwm_sample_time_us,rudder_pwm_sample_time_us,uf2_sha256,model_sha256,plant_sha256,virtual_platform_sha256,scenario_sha256";
const firmwareRow = (sequence: number): string => [sequence, 123000, 120000, 0, sequence !== 3, 0, 0, 0, 0, 122000, 122000,
  ...Array.from({ length: 5 }, () => "a".repeat(64))].join(",");

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
    `time_s,north_m,altitude_m,pitch_deg,${firmwareHeader}\n` +
    `0,0,10,-3,${firmwareRow(3)}\n1,8,9,-2,${firmwareRow(5)}\n`,
  );
  assert.equal(interpolateFrame(frames, 0.5).controlTelemetry.tag, "firmware");
  assert.deepEqual(interpolateFrame(frames, 0.5).controlTelemetry, frames[0]?.controlTelemetry);
  assert.equal(parseFlightCsv("time_s,north_m,altitude_m,pitch_deg\n0,0,10,-3\n1,8,9,-2\n")[0]?.controlTelemetry.tag, "unavailable");
});

test("live JSON without firmware evidence is rejected instead of plotted as real telemetry", () => {
  assert.throws(() => frameFromLive({} as InteractiveObservation), /Invalid actual-UF2/);
});

test("live boundary requires unscaled CPU and carries reproducible binary identity", () => {
  const observation: InteractiveObservation = {
    aero_in_range: false,
    firmware_sequence: 1, firmware_time_us: 1_000_000, automatic_valid: true,
    release_mcu_time_us: 995000, plant_interval_start_s: 0,
    run_identity: { uf2_sha256: "a".repeat(64), model_sha256: "b".repeat(64), plant_sha256: "c".repeat(64),
      virtual_platform_sha256: "d".repeat(64), scenario_sha256: "e".repeat(64) },
    safe_elevator_command_rad: 0, safe_rudder_command_rad: 0.03,
    observed_elevator_command_rad: 0, observed_rudder_command_rad: 0,
    elevator_pwm_sample_time_us: 990000, rudder_pwm_sample_time_us: 990000,
    time_s: 0.01, north_m: 0.05, east_m: 0, altitude_m: 10, roll_rad: 0, pitch_rad: -0.05,
    yaw_rad: 0, flight_path_rad: -0.05, elevator_rad: 0, rudder_rad: 0,
    sensor_airspeed_mps: 5, sensor_alpha_rad: 0, pilot_elevator: 0, pilot_rudder: 0, autonomy: 1,
    manual_elevator_command_rad: 0, manual_rudder_command_rad: 0, automatic_elevator_command_rad: 0,
    automatic_rudder_command_rad: 0, mixed_elevator_command_rad: 0, mixed_rudder_command_rad: 0,
    surface_contact: false, backend: "rp2040js-actual-uf2",
    emulation: { wall_elapsed_ms: 50, timing_acceleration: 1, processing_ms: 50, processing_average_ms: 50, real_time_ratio: 0.2,
      lag_ms: 40, deadline_missed: false, real_time: false, timing_validated: false },
  };
  const result = frameFromLive(observation).controlTelemetry;
  assert.equal(result.tag, "firmware");
  if (result.tag === "firmware") {
    assert.deepEqual(result.runIdentity, observation.run_identity);
    assert.equal(result.safeRudderCommandRad, 0.03);
    assert.equal(result.releaseMcuTimeUs, 995000);
    assert.equal(result.plantIntervalStartS, 0);
  }
  assert.throws(() => frameFromLive({ ...observation, emulation: { ...observation.emulation, timing_acceleration: 50 } } as unknown as InteractiveObservation), /scaled CPU/);
  assert.throws(() => frameFromLive({ ...observation, run_identity: { ...observation.run_identity, uf2_sha256: "bad" } }), /run identity/);
  assert.equal(frameFromLive(observation).experiment.tag, "measured");
  assert.throws(() => frameFromLive({ ...observation, emulation: { ...observation.emulation, real_time_ratio: Number.NaN } }), /timing or model/);
  assert.throws(() => frameFromLive({ ...observation, aero_in_range: undefined } as unknown as InteractiveObservation), /timing or model/);
});

test("CSV preserves envelope, wall timing, deadline and explicit session/incident evidence", () => {
  const evidence: ExperimentEvidence = { tag: "measured", aeroInRange: false, wallElapsedMs: 500,
    processingMs: 30, processingAverageMs: 25, realTimeRatio: 0.37, lagMs: 300,
    deadlineMissed: true, realTime: false, timingValidated: false };
  const outcome = { tag: "failed", reason: "model envelope violation, alpha outside allowed range" };
  const incidents = [{ kind: "telemetry-stall", wallTimeIso: "2026-09-08T00:00:00Z", sinceLastReceiptMs: 650 }];
  const csv = `# birdman-session ${JSON.stringify(outcome)}\n# birdman-incidents ${JSON.stringify(incidents)}\n`
    + `time_s,north_m,altitude_m,pitch_deg,${firmwareHeader},${experimentColumns.join(",")}\n`
    + `0,0,10,0,${firmwareRow(1)},${experimentCsvValues(evidence).join(",")}\n1,10,9,0,${firmwareRow(2)},${experimentCsvValues(evidence).join(",")}\n`;
  assert.deepEqual(parseFlightCsv(csv)[0]!.experiment, evidence);
  assert.deepEqual(parseCsvOutcome(csv), outcome);
  assert.deepEqual(parseCsvIncidents(csv), incidents);
  assert.equal(parseCsvOutcome("time_s,north_m").tag, "unknown");
  assert.throws(() => parseFlightCsv(csv.replace("measured,false,500", "measured,false,")), /Missing experiment evidence/);
  assert.throws(() => parseFlightCsv(csv.replace("1,123000", "1,")), /Incomplete measured firmware evidence/);
});

test("incomplete legacy firmware evidence is unavailable rather than zero-filled", () => {
  const frames = parseFlightCsv("time_s,north_m,altitude_m,pitch_deg,firmware_sequence\n0,0,10,0,1\n1,10,9,0,2");
  assert.equal(frames[0]!.controlTelemetry.tag, "unavailable");
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
