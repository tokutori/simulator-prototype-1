import { interpolatePair } from "./replay.ts";
import type { FlightFrame } from "./types.ts";

/** One plant frame annotated at the browser adapter boundary. */
export interface ReceivedFlightFrame {
  frame: FlightFrame;
  receivedAtMs: number;
}

/**
 * Produces a continuous presentation frame from irregular WebSocket arrivals.
 *
 * This adapter never feeds the interpolated value back to the MCU or plant.
 * Control and logging retain the original 100 Hz samples.
 */
export function frameAtReceiptTime(
  samples: readonly ReceivedFlightFrame[],
  presentationTimeMs: number,
): FlightFrame | undefined {
  const first = samples[0];
  const last = samples.at(-1);
  if (!first || !last) return undefined;
  if (presentationTimeMs <= first.receivedAtMs) return first.frame;
  if (presentationTimeMs >= last.receivedAtMs) return last.frame;

  let low = 0;
  let high = samples.length - 1;
  while (low + 1 < high) {
    const middle = Math.floor((low + high) / 2);
    const sample = samples[middle];
    if (sample && sample.receivedAtMs <= presentationTimeMs) low = middle;
    else high = middle;
  }
  const before = samples[low];
  const after = samples[high];
  if (!before || !after) return last.frame;
  const intervalMs = after.receivedAtMs - before.receivedAtMs;
  if (intervalMs <= 0) return after.frame;
  const fraction = (presentationTimeMs - before.receivedAtMs) / intervalMs;
  return interpolatePair(before.frame, after.frame, fraction);
}

/** Keeps only the short visual jitter buffer; the full-resolution flight log is separate. */
export function trimReceiptBuffer(
  samples: ReceivedFlightFrame[],
  presentationTimeMs: number,
): void {
  while (samples.length > 2 && (samples[1]?.receivedAtMs ?? Infinity) < presentationTimeMs) {
    samples.shift();
  }
}
