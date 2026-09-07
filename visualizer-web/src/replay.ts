import type { FlightFrame, InteractiveObservation, ExperimentEvidence, SessionOutcome, SessionIncident } from "./types.ts";
import { isRunIdentity, isControlTelemetry, isExperimentEvidence, isSessionOutcome, isSessionIncident } from "./types.ts";

const degree = Math.PI / 180;

export const experimentColumns = ["experiment_tag", "aero_in_range", "wall_elapsed_ms", "processing_ms",
  "processing_average_ms", "real_time_ratio", "lag_ms", "deadline_missed", "real_time", "timing_validated"] as const;

export function experimentCsvValues(evidence: ExperimentEvidence): (string | number | boolean)[] {
  return evidence.tag === "unknown" ? ["unknown", ...Array.from({ length: 9 }, () => "")]
    : ["measured", evidence.aeroInRange, evidence.wallElapsedMs, evidence.processingMs,
      evidence.processingAverageMs, evidence.realTimeRatio, evidence.lagMs, evidence.deadlineMissed,
      evidence.realTime, evidence.timingValidated];
}

export function parseFlightCsv(text: string): FlightFrame[] {
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/).filter((line) => line.trim() !== "" && !line.startsWith("#"));
  const headerLine = lines.shift();
  if (!headerLine) {
    throw new Error("CSV is empty");
  }
  const headers = headerLine.split(",").map((header) => header.trim());
  const required = ["time_s", "north_m", "altitude_m", "pitch_deg"];
  for (const name of required) {
    if (!headers.includes(name)) {
      throw new Error(`CSV is missing required column: ${name}`);
    }
  }

  const frames = lines.map((line, index) => {
    const values = line.split(",");
    const row = new Map<string, string>();
    headers.forEach((header, column) => row.set(header, values[column]?.trim() ?? ""));
    return rowToFrame(row, index + 2);
  });
  if (frames.length < 2) {
    throw new Error("CSV must contain at least two data rows");
  }
  for (let index = 1; index < frames.length; index += 1) {
    const previous = frames[index - 1];
    const current = frames[index];
    if (!previous || !current || current.timeS <= previous.timeS) {
      throw new Error(`CSV time must increase strictly at row ${index + 2}`);
    }
  }
  return frames;
}

export function frameFromLive(observation: InteractiveObservation): FlightFrame {
  // This adapter is the untrusted JSON boundary, not merely a TypeScript assertion.
  const numericFields: readonly (keyof InteractiveObservation)[] = [
    "time_s", "north_m", "east_m", "altitude_m", "roll_rad", "pitch_rad", "yaw_rad",
    "flight_path_rad", "elevator_rad", "rudder_rad", "sensor_airspeed_mps", "sensor_alpha_rad",
    "pilot_elevator", "pilot_rudder", "autonomy", "manual_elevator_command_rad",
    "manual_rudder_command_rad", "automatic_elevator_command_rad", "automatic_rudder_command_rad",
    "mixed_elevator_command_rad", "mixed_rudder_command_rad", "firmware_sequence", "firmware_time_us",
    "safe_elevator_command_rad", "safe_rudder_command_rad", "observed_elevator_command_rad", "observed_rudder_command_rad",
    "elevator_pwm_sample_time_us", "rudder_pwm_sample_time_us",
    "release_mcu_time_us", "plant_interval_start_s",
  ];
  if (numericFields.some(key => typeof observation[key] !== "number" || !Number.isFinite(observation[key]))
      || typeof observation.automatic_valid !== "boolean" || typeof observation.surface_contact !== "boolean") {
    throw new Error("Invalid actual-UF2 telemetry fields");
  }
  if (!isRunIdentity(observation.run_identity) || !observation.run_identity.virtual_platform_sha256 || !observation.run_identity.scenario_sha256
      || observation.emulation?.timing_acceleration !== 1
      || observation.emulation?.timing_validated !== false) {
    throw new Error("Unverified run identity or scaled CPU timing rejected");
  }
  const experiment: ExperimentEvidence = {
    tag: "measured", aeroInRange: observation.aero_in_range,
    wallElapsedMs: observation.emulation.wall_elapsed_ms, processingMs: observation.emulation.processing_ms,
    processingAverageMs: observation.emulation.processing_average_ms, realTimeRatio: observation.emulation.real_time_ratio,
    lagMs: observation.emulation.lag_ms, deadlineMissed: observation.emulation.deadline_missed,
    realTime: observation.emulation.real_time, timingValidated: observation.emulation.timing_validated,
  };
  if (!isExperimentEvidence(experiment)) throw new Error("Invalid experiment timing or model validity evidence");
  const result: FlightFrame = {
    experiment,
    controlTelemetry: { tag: "firmware", sequence: observation.firmware_sequence,
      runIdentity: observation.run_identity,
      timeUs: observation.firmware_time_us, automaticValid: observation.automatic_valid,
      releaseMcuTimeUs: observation.release_mcu_time_us,
      plantIntervalStartS: observation.plant_interval_start_s,
      safeElevatorCommandRad: observation.safe_elevator_command_rad,
      safeRudderCommandRad: observation.safe_rudder_command_rad,
      observedElevatorCommandRad: observation.observed_elevator_command_rad,
      observedRudderCommandRad: observation.observed_rudder_command_rad,
      elevatorPwmSampleTimeUs: observation.elevator_pwm_sample_time_us,
      rudderPwmSampleTimeUs: observation.rudder_pwm_sample_time_us },
    timeS: observation.time_s,
    northM: observation.north_m,
    eastM: observation.east_m,
    altitudeM: observation.altitude_m,
    rollRad: observation.roll_rad,
    pitchRad: observation.pitch_rad,
    yawRad: observation.yaw_rad,
    flightPathRad: observation.flight_path_rad,
    airspeedMps: observation.sensor_airspeed_mps,
    alphaRad: observation.sensor_alpha_rad,
    elevatorRad: observation.elevator_rad,
    rudderRad: observation.rudder_rad,
    pilotElevator: observation.pilot_elevator,
    pilotRudder: observation.pilot_rudder,
    autonomy: observation.autonomy,
    manualElevatorCommandRad: observation.manual_elevator_command_rad,
    manualRudderCommandRad: observation.manual_rudder_command_rad,
    automaticElevatorCommandRad: observation.automatic_elevator_command_rad,
    automaticRudderCommandRad: observation.automatic_rudder_command_rad,
    mixedElevatorCommandRad: observation.mixed_elevator_command_rad,
    mixedRudderCommandRad: observation.mixed_rudder_command_rad,
    surfaceContact: observation.surface_contact,
  };
  if (!isControlTelemetry(result.controlTelemetry)) throw new Error("Invalid firmware evidence");
  return result;
}

export function interpolateFrame(frames: readonly FlightFrame[], timeS: number): FlightFrame {
  const first = frames[0];
  const last = frames.at(-1);
  if (!first || !last) {
    throw new Error("cannot interpolate an empty replay");
  }
  if (timeS <= first.timeS) {
    return first;
  }
  if (timeS >= last.timeS) {
    return last;
  }

  let low = 0;
  let high = frames.length - 1;
  while (low + 1 < high) {
    const middle = Math.floor((low + high) / 2);
    const frame = frames[middle];
    if (frame && frame.timeS <= timeS) {
      low = middle;
    } else {
      high = middle;
    }
  }
  const before = frames[low];
  const after = frames[high];
  if (!before || !after) {
    return last;
  }
  const fraction = (timeS - before.timeS) / (after.timeS - before.timeS);
  return interpolatePair(before, after, fraction);
}

function rowToFrame(row: Map<string, string>, lineNumber: number): FlightFrame {
  const number = (name: string, fallback = 0): number => {
    const raw = row.get(name);
    if (raw === undefined || raw === "") {
      return fallback;
    }
    const value = Number(raw);
    if (!Number.isFinite(value)) {
      throw new Error(`non-finite ${name} at CSV row ${lineNumber}`);
    }
    return value;
  };
  const boolean = (name: string): boolean => {
    const raw = row.get(name)?.toLowerCase();
    if (raw === undefined || raw === "" || raw === "0" || raw === "false") return false;
    if (raw === "1" || raw === "true") return true;
    throw new Error(`invalid boolean ${name} at CSV row ${lineNumber}`);
  };
  const identity = { uf2_sha256: row.get("uf2_sha256"), model_sha256: row.get("model_sha256"), plant_sha256: row.get("plant_sha256"),
    virtual_platform_sha256: row.get("virtual_platform_sha256") || undefined, scenario_sha256: row.get("scenario_sha256") || undefined };
  const firmwareColumns = ["firmware_sequence", "firmware_time_us", "release_mcu_time_us", "plant_interval_start_s", "automatic_valid",
    "safe_elevator_command_deg", "safe_rudder_command_deg", "observed_elevator_command_deg", "observed_rudder_command_deg",
    "elevator_pwm_sample_time_us", "rudder_pwm_sample_time_us", "uf2_sha256", "model_sha256", "plant_sha256"];
  const hasFirmware = firmwareColumns.every(column => row.get(column) !== undefined && row.get(column) !== "");
  if (hasFirmware && !isRunIdentity(identity)) throw new Error(`Missing or invalid run identity at CSV row ${lineNumber}`);
  const tag = row.get("experiment_tag");
  if (tag === "measured" && (!hasFirmware || !identity.virtual_platform_sha256 || !identity.scenario_sha256)) {
    throw new Error(`Incomplete measured firmware evidence at CSV row ${lineNumber}`);
  }
  if (tag && tag !== "unknown" && tag !== "measured") throw new Error(`Invalid experiment tag at CSV row ${lineNumber}`);
  if (tag === "measured" && experimentColumns.some(column => !row.get(column))) {
    throw new Error(`Missing experiment evidence at CSV row ${lineNumber}`);
  }
  const experiment: unknown = tag !== "measured" ? { tag: "unknown" } : {
    tag, aeroInRange: boolean("aero_in_range"), wallElapsedMs: number("wall_elapsed_ms"),
    processingMs: number("processing_ms"), processingAverageMs: number("processing_average_ms"),
    realTimeRatio: number("real_time_ratio"), lagMs: number("lag_ms"), deadlineMissed: boolean("deadline_missed"),
    realTime: boolean("real_time"), timingValidated: boolean("timing_validated"),
  };
  if (!isExperimentEvidence(experiment)) throw new Error(`Invalid experiment evidence at CSV row ${lineNumber}`);
  const result: FlightFrame = {
    experiment,
    timeS: number("time_s"),
    controlTelemetry: hasFirmware ? {
      tag: "firmware", sequence: number("firmware_sequence"), timeUs: number("firmware_time_us"),
      releaseMcuTimeUs: number("release_mcu_time_us"), plantIntervalStartS: number("plant_interval_start_s"),
      runIdentity: { uf2_sha256: identity.uf2_sha256 ?? "", model_sha256: identity.model_sha256 ?? "", plant_sha256: identity.plant_sha256 ?? "",
        virtual_platform_sha256: identity.virtual_platform_sha256, scenario_sha256: identity.scenario_sha256 },
      automaticValid: boolean("automatic_valid"),
      safeElevatorCommandRad: number("safe_elevator_command_deg") * degree,
      safeRudderCommandRad: number("safe_rudder_command_deg") * degree,
      observedElevatorCommandRad: number("observed_elevator_command_deg") * degree,
      observedRudderCommandRad: number("observed_rudder_command_deg") * degree,
      elevatorPwmSampleTimeUs: number("elevator_pwm_sample_time_us"),
      rudderPwmSampleTimeUs: number("rudder_pwm_sample_time_us"),
    } : { tag: "unavailable" },
    northM: number("north_m"),
    eastM: number("east_m"),
    altitudeM: number("altitude_m"),
    rollRad: number("roll_deg") * degree,
    pitchRad: number("pitch_deg") * degree,
    yawRad: number("yaw_deg") * degree,
    flightPathRad: number("flight_path_deg") * degree,
    airspeedMps: number("airspeed_mps", number("sensor_airspeed_mps")),
    alphaRad: number("alpha_deg", number("sensor_alpha_deg")) * degree,
    elevatorRad: number("elevator_deg") * degree,
    rudderRad: number("rudder_deg") * degree,
    pilotElevator: number("pilot_elevator"),
    pilotRudder: number("pilot_rudder"),
    autonomy: number("autonomy", 1),
    manualElevatorCommandRad: number("manual_elevator_command_deg") * degree,
    manualRudderCommandRad: number("manual_rudder_command_deg") * degree,
    automaticElevatorCommandRad:
      number("automatic_elevator_command_deg", number("elevator_command_deg")) * degree,
    automaticRudderCommandRad: number("automatic_rudder_command_deg") * degree,
    mixedElevatorCommandRad:
      number("mixed_elevator_command_deg", number("elevator_command_deg")) * degree,
    mixedRudderCommandRad:
      number("mixed_rudder_command_deg", number("rudder_command_deg")) * degree,
    surfaceContact: boolean("surface_contact"),
  };
  if (!isControlTelemetry(result.controlTelemetry)) throw new Error(`Invalid firmware evidence at CSV row ${lineNumber}`);
  return result;
}

export function interpolatePair(before: FlightFrame, after: FlightFrame, fraction: number): FlightFrame {
  const scalar = (left: number, right: number): number => left + (right - left) * fraction;
  const angle = (left: number, right: number): number => {
    const difference = Math.atan2(Math.sin(right - left), Math.cos(right - left));
    return left + difference * fraction;
  };
  return {
    experiment: fraction < 1 ? before.experiment : after.experiment,
    timeS: scalar(before.timeS, after.timeS),
    // Discrete firmware evidence must never acquire invented fractional sample IDs.
    controlTelemetry: fraction < 1 ? before.controlTelemetry : after.controlTelemetry,
    northM: scalar(before.northM, after.northM),
    eastM: scalar(before.eastM, after.eastM),
    altitudeM: scalar(before.altitudeM, after.altitudeM),
    rollRad: angle(before.rollRad, after.rollRad),
    pitchRad: angle(before.pitchRad, after.pitchRad),
    yawRad: angle(before.yawRad, after.yawRad),
    flightPathRad: angle(before.flightPathRad, after.flightPathRad),
    airspeedMps: scalar(before.airspeedMps, after.airspeedMps),
    alphaRad: angle(before.alphaRad, after.alphaRad),
    elevatorRad: scalar(before.elevatorRad, after.elevatorRad),
    rudderRad: scalar(before.rudderRad, after.rudderRad),
    pilotElevator: scalar(before.pilotElevator, after.pilotElevator),
    pilotRudder: scalar(before.pilotRudder, after.pilotRudder),
    autonomy: scalar(before.autonomy, after.autonomy),
    manualElevatorCommandRad: scalar(
      before.manualElevatorCommandRad,
      after.manualElevatorCommandRad,
    ),
    manualRudderCommandRad: scalar(before.manualRudderCommandRad, after.manualRudderCommandRad),
    automaticElevatorCommandRad: scalar(
      before.automaticElevatorCommandRad,
      after.automaticElevatorCommandRad,
    ),
    automaticRudderCommandRad: scalar(
      before.automaticRudderCommandRad,
      after.automaticRudderCommandRad,
    ),
    mixedElevatorCommandRad: scalar(
      before.mixedElevatorCommandRad,
      after.mixedElevatorCommandRad,
    ),
    mixedRudderCommandRad: scalar(
      before.mixedRudderCommandRad,
      after.mixedRudderCommandRad,
    ),
    surfaceContact: fraction < 0.5 ? before.surfaceContact : after.surfaceContact,
  };
}

export function parseCsvOutcome(text: string): SessionOutcome {
  const line = text.replace(/^\uFEFF/, "").split(/\r?\n/).find(value => value.startsWith("# birdman-session "));
  if (!line) return { tag: "unknown", reason: "Legacy CSV has no session completion evidence" };
  const value: unknown = JSON.parse(line.slice("# birdman-session ".length));
  if (!isSessionOutcome(value)) throw new Error("Invalid CSV session outcome");
  return value;
}

export function parseCsvIncidents(text: string): SessionIncident[] {
  const line = text.replace(/^\uFEFF/, "").split(/\r?\n/).find(value => value.startsWith("# birdman-incidents "));
  if (!line) return [];
  const value: unknown = JSON.parse(line.slice("# birdman-incidents ".length));
  if (!Array.isArray(value) || !value.every(isSessionIncident)) throw new Error("Invalid CSV session incidents");
  return value;
}
