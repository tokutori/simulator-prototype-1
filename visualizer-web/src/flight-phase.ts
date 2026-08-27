export type FlightPhase = "ready" | "launch" | "flying" | "water-contact" | "record-ended";

export interface FlightPhaseInput {
  hasFrame: boolean;
  elapsedS: number;
  surfaceContact: boolean;
  replayAtEnd: boolean;
}

export function classifyFlightPhase(input: FlightPhaseInput): FlightPhase {
  if (!input.hasFrame) return "ready";
  if (input.surfaceContact) return "water-contact";
  if (input.elapsedS <= 1.25) return "launch";
  if (input.replayAtEnd) return "record-ended";
  return "flying";
}

export function formatFlightTime(elapsedS: number): string {
  const bounded = Math.max(0, elapsedS);
  const minutes = Math.floor(bounded / 60);
  const seconds = bounded - minutes * 60;
  return `${String(minutes).padStart(2, "0")}:${seconds.toFixed(2).padStart(5, "0")}`;
}
