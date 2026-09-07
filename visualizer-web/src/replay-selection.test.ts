import assert from "node:assert/strict";
import { test } from "node:test";
import { ReplaySelection } from "./replay-selection.ts";

test("late startup sample cannot overwrite the user's replay or error state", () => {
  const selection = new ReplaySelection();
  const sample = selection.current();
  const user = selection.select();
  assert.equal(selection.accepts(user), true);
  assert.equal(selection.accepts(sample), false);
});

test("latest file selection wins even when reads complete out of order", () => {
  const selection = new ReplaySelection();
  const first = selection.select();
  const second = selection.select();
  assert.equal(selection.accepts(first), false);
  assert.equal(selection.accepts(second), true);
});
