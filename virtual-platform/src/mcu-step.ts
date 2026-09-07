import type { Simulator } from 'rp2040js';

/** Run one instruction, or dispatch the next peripheral event while the core
 * is asleep. Never cross the plant coupling boundary during sleep. This is
 * event-driven idle execution, not CPU clock acceleration.
 * Returns the number of instructions actually executed.
 */
export function stepMcu(simulator: Simulator, cycleNanos: number, boundaryUs = Number.POSITIVE_INFINITY): number {
  const { clock, rp2040: mcu } = simulator;
  if (mcu.core.waiting) {
    const next = clock.nanosToNextAlarm;
    // A bounded fallback also handles a core with no registered peripheral
    // alarms, without a zero-time infinite loop. Real watchdog alarms still run.
    const delta = Math.min(next > 0 ? next : 1000, (boundaryUs - clock.micros) * 1000);
    if (delta > 0) clock.tick(delta);
    return 0;
  }
  clock.tick(mcu.core.executeInstruction() * cycleNanos);
  return 1;
}
