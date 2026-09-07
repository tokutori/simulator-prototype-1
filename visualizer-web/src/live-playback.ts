import { interpolateFrame } from "./replay.ts";
import type { FlightFrame } from "./types.ts";

/** Monotonic presentation clock. Never jumps to a late packet or extrapolates physics.
 * Runs at measured producer speed; CPU slowdown remains visible in the separate status.
 */
export class LivePlayback {
  private samples: ReceivedFlightFrame[] = [];
  private cursorS: number | undefined;
  private previousWallMs: number | undefined;
  private rate = 1;
  private ended = false;

  finish(): void { this.ended = true; }

  push(frame: FlightFrame, receivedAtMs: number): void {
    const previous = this.samples.at(-1);
    if (previous && frame.timeS > previous.frame.timeS && receivedAtMs > previous.receivedAtMs) {
      const measured = Math.min(1, (frame.timeS - previous.frame.timeS) * 1000 / (receivedAtMs - previous.receivedAtMs));
      this.rate = this.samples.length < 3 ? measured : this.rate * 0.8 + measured * 0.2;
    }
    this.samples.push({ frame, receivedAtMs });
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
      this.cursorS = Math.min(last.frame.timeS, this.cursorS + elapsed * this.rate);
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
