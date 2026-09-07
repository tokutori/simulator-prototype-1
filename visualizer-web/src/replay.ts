import type { FlightFrame, InteractiveObservation } from "./types.ts";

const degree = Math.PI / 180;

export function parseFlightCsv(text: string): FlightFrame[] {
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/).filter((line) => line.trim() !== "");
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
    "safe_elevator_command_rad", "observed_elevator_command_rad", "observed_rudder_command_rad",
    "elevator_pwm_sample_time_us", "rudder_pwm_sample_time_us",
  ];
  if (numericFields.some(key => typeof observation[key] !== "number" || !Number.isFinite(observation[key]))
      || typeof observation.automatic_valid !== "boolean" || typeof observation.surface_contact !== "boolean") {
    throw new Error("Invalid actual-UF2 telemetry fields");
  }
  return {
    controlTelemetry: { tag: "firmware", sequence: observation.firmware_sequence,
      timeUs: observation.firmware_time_us, automaticValid: observation.automatic_valid,
      safeElevatorCommandRad: observation.safe_elevator_command_rad,
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
  return {
    timeS: number("time_s"),
    controlTelemetry: row.get("firmware_sequence") ? {
      tag: "firmware", sequence: number("firmware_sequence"), timeUs: number("firmware_time_us"),
      automaticValid: boolean("automatic_valid"),
      safeElevatorCommandRad: number("safe_elevator_command_deg") * degree,
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
}

export function interpolatePair(before: FlightFrame, after: FlightFrame, fraction: number): FlightFrame {
  const scalar = (left: number, right: number): number => left + (right - left) * fraction;
  const angle = (left: number, right: number): number => {
    const difference = Math.atan2(Math.sin(right - left), Math.cos(right - left));
    return left + difference * fraction;
  };
  return {
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
