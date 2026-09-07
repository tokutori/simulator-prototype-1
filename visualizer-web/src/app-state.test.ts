import assert from "node:assert/strict";
import test from "node:test";
import { present, update, type AppState, type EmulationPerformance } from "./app-state.ts";

const realtime: EmulationPerformance = {
  timingAcceleration: 1,
  processingMs: 4,
  processingAverageMs: 5,
  realTimeRatio: 1,
  lagMs: 20,
  deadlineMissed: false,
  realTime: true,
  timingValidated: false,
};

test("only telemetry can construct an MCU running state", () => {
  const initial: AppState = { tag: "replay", sessionId: 0, sourceName: "sample" };
  const [connecting, effect] = update(initial, { type: "request-mcu" });
  assert.deepEqual(connecting, { tag: "mcu-connecting", sessionId: 1 });
  assert.deepEqual(effect, { type: "connect-mcu", sessionId: 1 });
  const [running] = update(connecting, { type: "mcu-telemetry", sessionId: 1, performance: realtime });
  assert.equal(running.tag, "mcu-running");
  assert.match(present(running).performance, /CPU ×1.*cycle timing UNVALIDATED/);
});

test("slow or deadline-missed emulation is an explicit warning state", () => {
  const [state] = update({ tag: "mcu-connecting", sessionId: 1 }, {
    type: "mcu-telemetry",
    sessionId: 1,
    performance: { ...realtime, realTime: false, lagMs: 300 },
  });
  assert.equal(state.tag, "mcu-too-slow");
  assert.match(present(state).warning ?? "", /NOT REAL-TIME/);
});

test("restart rejects every old connection event without changing state", () => {
  const [restarted] = update({ tag: "mcu-running", sessionId: 3, performance: realtime }, { type: "request-mcu" });
  for (const message of [
    { type: "mcu-telemetry", sessionId: 3, performance: realtime },
    { type: "mcu-ended", sessionId: 3, reason: "water" },
    { type: "mcu-failed", sessionId: 3, error: "disconnected" },
  ] as const) {
    assert.deepEqual(update(restarted, message), [restarted, { type: "none" }]);
  }
  assert.equal(update(restarted, { type: "mcu-telemetry", sessionId: 4, performance: realtime })[0].tag, "mcu-running");
});

test("replay, ended and failed states cannot be revived by queued telemetry", () => {
  const terminal: AppState[] = [
    { tag: "replay", sessionId: 7, sourceName: "sample" },
    { tag: "mcu-ended", sessionId: 7, reason: "water" },
    { tag: "mcu-failed", sessionId: 7, error: "bad telemetry" },
  ];
  for (const state of terminal) {
    assert.equal(update(state, { type: "mcu-telemetry", sessionId: 7, performance: realtime })[0], state);
    const [restart] = update(state, { type: "request-mcu" });
    assert.equal(restart.sessionId, 8);
  }
});
