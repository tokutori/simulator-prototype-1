import assert from "node:assert/strict";
import test from "node:test";

import { presentXr, updateXr, type XrState } from "./xr-state.ts";

test("XR availability constructs ready and unavailable states", () => {
  const checking: XrState = { tag: "checking" };
  assert.deepEqual(updateXr(checking, { type: "availability", supported: true }), { tag: "ready" });
  assert.deepEqual(updateXr(checking, {
    type: "availability",
    supported: false,
    reason: "insecure-context",
  }), { tag: "unavailable", reason: "insecure-context" });
});

test("only a session event enters and leaves the presenting state", () => {
  const active = updateXr({ tag: "ready" }, { type: "session-started" });
  assert.deepEqual(active, { tag: "presenting" });
  assert.equal(presentXr(active).presenting, true);
  assert.deepEqual(updateXr(active, { type: "session-ended" }), { tag: "ready" });
});

test("late availability result cannot overwrite an active session", () => {
  const active: XrState = { tag: "presenting" };
  assert.equal(updateXr(active, {
    type: "availability",
    supported: false,
    reason: "permission-denied",
  }), active);
});

test("ending a session is ignored when no session is active", () => {
  const unavailable: XrState = { tag: "unavailable", reason: "api-unavailable" };
  assert.equal(updateXr(unavailable, { type: "session-ended" }), unavailable);
});
