import { interpolateFrame } from "./replay.ts";
import type { FlightFrame } from "./types.ts";

/** Monotonic presentation clock. Never jumps to a late packet or extrapolates physics.
 * Runs at measured producer speed; CPU slowdown remains visible in the separate status.
 */
export class LivePlayback {
  private samples: ReceivedFlightFrame[] = [];
  private rateWindow: ReceivedFlightFrame[] = [];
  private cursorS: number | undefined;
  private previousWallMs: number | undefined;
  private rate = 1;
  private ended = false;

  finish(): void { this.ended = true; }

  push(frame: FlightFrame, receivedAtMs: number): void {
    const previous = this.samples.at(-1);
    if (previous && (frame.timeS <= previous.frame.timeS || receivedAtMs < previous.receivedAtMs)) {
      throw new Error("Live presentation samples must be ordered");
    }
    const sample = { frame, receivedAtMs };
    this.samples.push(sample);
    this.rateWindow.push(sample);
    // Keep one bracketing sample before the rolling one-second window. Ratio of
    // total elapsed times is unbiased by alternating short/long delivery gaps;
    // averaging individually capped instantaneous ratios was not.
    while (this.rateWindow.length > 2 && this.rateWindow[1]!.receivedAtMs <= receivedAtMs - 1000) {
      this.rateWindow.shift();
    }
    const first = this.rateWindow[0]!;
    const wallSpanMs = receivedAtMs - first.receivedAtMs;
    // Do not interpret an initial packet burst/timestamp quantization as speed.
    if (wallSpanMs >= 100) this.rate = (frame.timeS - first.frame.timeS) * 1000 / wallSpanMs;
    this.cursorS ??= frame.timeS;
  }

  frame(nowMs: number): FlightFrame | undefined {
    const first = this.samples[0];
    const last = this.samples.at(-1);
    // Capping elapsed time would accumulate an unbounded backlog below 20 FPS.
    // Visibility/timeline discontinuities are reset explicitly by the browser adapter.
    const elapsed = this.previousWallMs === undefined ? 0 : Math.max(0, nowMs - this.previousWallMs) / 1000;
    this.previousWallMs = nowMs;
    if (!first || !last || this.cursorS === undefined) return undefined;
    // Prime a two-interval buffer before starting. Under-runs hold, never invent a future pose.
    if (this.ended || this.samples.length >= 3 || this.cursorS > first.frame.timeS) {
      const previous = this.samples.at(-2);
      const samplePeriodS = previous ? last.frame.timeS - previous.frame.timeS : 0;
      const desiredLagS = Math.max(samplePeriodS * 2, this.rate * 0.03);
      // Gently recover backlog from a rate change/stall, rather than retaining a
      // permanent old view while the MCU is current. Never move the cursor in
      // push(), and cap recovery at twice measured producer speed.
      const recovery = Math.min(this.rate, Math.max(0, last.frame.timeS - this.cursorS - desiredLagS) * 2);
      this.cursorS = Math.min(last.frame.timeS, this.cursorS + elapsed * (this.rate + recovery));
    }
    const result = interpolateFrame(this.samples.map(sample => sample.frame), this.cursorS);
    while (this.samples.length > 3 && this.samples[1]!.frame.timeS < this.cursorS) this.samples.shift();
    return result;
  }
}

/** One plant frame annotated at the browser adapter boundary. */
export interface ReceivedFlightFrame {
  frame: FlightFrame;
  receivedAtMs: number;
}
