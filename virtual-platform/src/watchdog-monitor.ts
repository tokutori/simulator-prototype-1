import type { RP2040 } from 'rp2040js';

/** A real watchdog expiry ends the experiment; reset/reboot is not emulated. */
export class WatchdogResetRequested extends Error {
  constructor(readonly atUs: number) {
    super(`RP2040 watchdog reset requested at ${atUs.toFixed(1)} us; experiment stopped (physical reset/rearm is not simulated)`);
    this.name = 'WatchdogResetRequested';
  }
}

/** Attach to rp2040js's actual watchdog alarm, not a parallel software timeout. */
export function installWatchdogMonitor(mcu: RP2040): void {
  const peripheral = mcu.peripherals[0x40058];
  if (!peripheral || !('onWatchdogTrigger' in peripheral)
    || typeof peripheral.onWatchdogTrigger !== 'function') {
    throw new Error('rp2040js watchdog reset callback is unavailable');
  }
  peripheral.onWatchdogTrigger = () => { throw new WatchdogResetRequested(mcu.clock.nanos / 1000); };
}
