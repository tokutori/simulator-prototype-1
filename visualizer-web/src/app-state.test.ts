import assert from "node:assert/strict";
import test from "node:test";
import { present, update, type AppState, type EmulationPerformance } from "./app-state.ts";

const realtime: EmulationPerformance = {
  processingMs: 4,
  processingAverageMs: 5,
  realTimeRatio: 1,
  lagMs: 20,
  deadlineMissed: false,
  realTime: true,
  timingValidated: false,
};

test("only telemetry can construct an MCU running state", () => {
  const initial: AppState = { tag: "replay", sourceName: "sample" };
  const [connecting, effect] = update(initial, { type: "request-mcu" });
  assert.deepEqual(connecting, { tag: "mcu-connecting" });
  assert.deepEqual(effect, { type: "connect-mcu" });
  const [running] = update(connecting, { type: "mcu-telemetry", performance: realtime });
  assert.equal(running.tag, "mcu-running");
});

test("slow or deadline-missed emulation is an explicit warning state", () => {
  const [state] = update({ tag: "mcu-connecting" }, {
    type: "mcu-telemetry",
    performance: { ...realtime, realTime: false, lagMs: 300 },
  });
  assert.equal(state.tag, "mcu-too-slow");
  assert.match(present(state).warning ?? "", /NOT REAL-TIME/);
});
