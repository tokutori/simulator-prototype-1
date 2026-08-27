export interface EmulationPerformance {
  processingMs: number;
  processingAverageMs: number;
  realTimeRatio: number;
  lagMs: number;
  deadlineMissed: boolean;
  realTime: boolean;
  timingValidated: false;
}

export type AppState =
  | { tag: "replay"; sourceName: string }
  | { tag: "mcu-connecting" }
  | { tag: "mcu-running"; performance: EmulationPerformance }
  | { tag: "mcu-too-slow"; performance: EmulationPerformance }
  | { tag: "mcu-ended"; reason: string; performance?: EmulationPerformance }
  | { tag: "mcu-failed"; error: string };

export type AppMsg =
  | { type: "select-replay"; sourceName: string }
  | { type: "request-mcu" }
  | { type: "mcu-telemetry"; performance: EmulationPerformance }
  | { type: "mcu-ended"; reason: string }
  | { type: "mcu-failed"; error: string };

export type Effect = { type: "none" } | { type: "connect-mcu" } | { type: "disconnect-mcu" };

export function update(state: AppState, message: AppMsg): readonly [AppState, Effect] {
  switch (message.type) {
    case "select-replay":
      return [{ tag: "replay", sourceName: message.sourceName }, { type: "disconnect-mcu" }];
    case "request-mcu":
      return [{ tag: "mcu-connecting" }, { type: "connect-mcu" }];
    case "mcu-telemetry":
      return [message.performance.realTime && !message.performance.deadlineMissed
        ? { tag: "mcu-running", performance: message.performance }
        : { tag: "mcu-too-slow", performance: message.performance }, { type: "none" }];
    case "mcu-ended":
      return [{
        tag: "mcu-ended",
        reason: message.reason,
        performance: "performance" in state ? state.performance : undefined,
      }, { type: "none" }];
    case "mcu-failed":
      return [{ tag: "mcu-failed", error: message.error }, { type: "none" }];
  }
}

export function isInteractive(state: AppState): boolean {
  return state.tag !== "replay";
}

export interface AppPresentation {
  mode: "REPLAY" | "INTERACTIVE / RP2040JS";
  status: string;
  performance: string;
  warning?: string;
}

export function present(state: AppState): AppPresentation {
  switch (state.tag) {
    case "replay": return { mode: "REPLAY", status: state.sourceName, performance: "MCU idle" };
    case "mcu-connecting": return { mode: "INTERACTIVE / RP2040JS", status: "loading actual UF2…", performance: "measuring host capacity" };
    case "mcu-running": return runningPresentation(state.performance);
    case "mcu-too-slow": return {
      ...runningPresentation(state.performance),
      warning: "MCU EMULATION TOO SLOW — NOT REAL-TIME",
    };
    case "mcu-ended": return { mode: "INTERACTIVE / RP2040JS", status: state.reason, performance: state.performance ? formatPerformance(state.performance) : "actual UF2" };
    case "mcu-failed": return { mode: "INTERACTIVE / RP2040JS", status: "MCU bridge failed", performance: "actual UF2 unavailable", warning: state.error };
  }
}

function runningPresentation(performance: EmulationPerformance): AppPresentation {
  return {
    mode: "INTERACTIVE / RP2040JS",
    status: "actual RP2040 UF2",
    performance: formatPerformance(performance),
  };
}

function formatPerformance(value: EmulationPerformance): string {
  return `${value.realTimeRatio.toFixed(2)}× real-time · ${value.processingAverageMs.toFixed(1)} ms/update · lag ${value.lagMs.toFixed(0)} ms${value.deadlineMissed ? " · DEADLINE MISS" : ""}`;
}
