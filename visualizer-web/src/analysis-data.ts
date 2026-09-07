import type { FlightFrame, SessionOutcome, SessionIncident } from "./types.ts";
import { isControlTelemetry, isExperimentEvidence, isSessionOutcome, isSessionIncident } from "./types.ts";

export interface FlightAnalysisDataset {
  version: 3;
  outcome: SessionOutcome;
  incidents: SessionIncident[];
  name: string;
  storedAtIso: string;
  frames: FlightFrame[];
}

export interface FlightSummary {
  durationS: number;
  rangeM: number;
  trackM: number;
  maximumAltitudeM: number;
  finalAirspeedMps: number;
  maximumAbsRollDeg: number;
  surfaceContact: boolean;
}

export function prepareAnalysisDataset(
  name: string,
  frames: readonly FlightFrame[],
  storedAt = new Date(),
  outcome: SessionOutcome = { tag: "unknown", reason: "No session completion evidence supplied" },
  incidents: readonly SessionIncident[] = [],
): FlightAnalysisDataset {
  return {
    version: 3,
    outcome,
    incidents: [...incidents],
    name,
    storedAtIso: storedAt.toISOString(),
    // Preserve the evidence: decimation aliases oscillations and changes extrema/track length.
    frames: [...frames],
  };
}

export function parseAnalysisDataset(raw: string): FlightAnalysisDataset {
  const candidate = JSON.parse(raw) as Partial<Omit<FlightAnalysisDataset, "version">> & { version?: number };
  if (!candidate || ![2, 3].includes(candidate.version ?? 0) || typeof candidate.name !== "string" || !Array.isArray(candidate.frames)) {
    throw new Error("stored flight analysis has an unsupported format");
  }
  if (candidate.version === 2) {
    candidate.version = 3;
    candidate.outcome = { tag: "unknown", reason: "Legacy recording has no session completion evidence" };
    candidate.incidents = [];
    candidate.frames = candidate.frames.map(frame => ({ ...frame, experiment: { tag: "unknown" },
      controlTelemetry: isControlTelemetry(frame?.controlTelemetry) ? frame.controlTelemetry : { tag: "unavailable" } }));
  }
  if (!isSessionOutcome(candidate.outcome)) throw new Error("Invalid session completion evidence");
  if (!Array.isArray(candidate.incidents) || !candidate.incidents.every(isSessionIncident)) throw new Error("Invalid session incidents");
  if (candidate.frames.some((frame) =>
    typeof frame !== "object"
    || frame === null
    || !Number.isFinite((frame as FlightFrame).timeS)
    || !Number.isFinite((frame as FlightFrame).northM)
    || !Number.isFinite((frame as FlightFrame).altitudeM)
    || !validFrame(frame))) {
    throw new Error("stored flight analysis contains invalid telemetry");
  }
  if (candidate.frames.some((frame, i, frames) => i > 0 && frame.timeS <= frames[i - 1]!.timeS)) {
    throw new Error("Stored flight timestamps must increase strictly");
  }
  return candidate as FlightAnalysisDataset;
}

function validFrame(value: FlightFrame): boolean {
  if (!isExperimentEvidence(value.experiment)) return false;
  const numericKeys: readonly (keyof FlightFrame)[] = ["timeS", "northM", "eastM", "altitudeM",
    "rollRad", "pitchRad", "yawRad", "flightPathRad", "airspeedMps", "alphaRad", "elevatorRad", "rudderRad",
    "pilotElevator", "pilotRudder", "autonomy", "manualElevatorCommandRad", "manualRudderCommandRad",
    "automaticElevatorCommandRad", "automaticRudderCommandRad", "mixedElevatorCommandRad", "mixedRudderCommandRad"];
  if (numericKeys.some(key => typeof value[key] !== "number" || !Number.isFinite(value[key]))
    || typeof value.surfaceContact !== "boolean") return false;
  return isControlTelemetry(value.controlTelemetry);
}

export function downsampleFrames(frames: readonly FlightFrame[], maximumFrames: number): FlightFrame[] {
  if (maximumFrames < 2) throw new Error("maximumFrames must be at least two");
  if (frames.length <= maximumFrames) return [...frames];
  const result: FlightFrame[] = [];
  const denominator = maximumFrames - 1;
  for (let index = 0; index < maximumFrames; index += 1) {
    const sourceIndex = Math.round((index * (frames.length - 1)) / denominator);
    const frame = frames[sourceIndex];
    if (frame && result.at(-1) !== frame) result.push(frame);
  }
  return result;
}

export function summarizeFlight(frames: readonly FlightFrame[]): FlightSummary {
  const first = frames[0];
  const last = frames.at(-1);
  if (!first || !last) throw new Error("cannot summarize an empty flight");
  let trackM = 0;
  let maximumAltitudeM = Number.NEGATIVE_INFINITY;
  let maximumAbsRollRad = 0;
  for (let index = 0; index < frames.length; index += 1) {
    const frame = frames[index];
    if (!frame) continue;
    maximumAltitudeM = Math.max(maximumAltitudeM, frame.altitudeM);
    maximumAbsRollRad = Math.max(maximumAbsRollRad, Math.abs(frame.rollRad));
    const previous = frames[index - 1];
    if (previous) trackM += Math.hypot(frame.northM - previous.northM, frame.eastM - previous.eastM);
  }
  return {
    durationS: last.timeS - first.timeS,
    rangeM: Math.hypot(last.northM - first.northM, last.eastM - first.eastM),
    trackM,
    maximumAltitudeM,
    finalAirspeedMps: last.airspeedMps,
    maximumAbsRollDeg: maximumAbsRollRad * 180 / Math.PI,
    surfaceContact: last.surfaceContact,
  };
}
