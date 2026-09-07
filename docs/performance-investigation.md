# Interactive performance investigation

## Contract

Interactive flight executes the actual UF2 in rp2040js with clock multiplier 1.
Rendering smoothness must not be confused with real-time controller capacity.
No physical cycle-accuracy claim follows from this multiplier.

## Reproduction (2026-09-08)

Run `node --import tsx src/performance-check.ts` from `virtual-platform` with no
other active flight. This drives the same interactive bridge, without rendering
or wall-clock pacing, for 300 updates. Startup is excluded. The probe enforces
the actual-UF2 backend and clock multiplier and rejects incomplete runs.

Observed on the development computer:

| Measurement | Result |
| --- | ---: |
| Simulated duration | 3 s |
| Wall duration | 13.463 s |
| Real-time ratio | 0.223 |
| Mean MCU work per 10 ms update | 42.425 ms |
| Mean plant round trip (including IPC) | 1.220 ms |
| Mean whole bridge step | 43.730 ms |
| Instructions per update | 698,100 |

The MCU accounts for about 97% of measured bridge processing time. Browser
rendering is not required to reproduce the reported slowdown. Results depend
on host load and are not a universal hardware benchmark.

Firmware currently waits for the remainder of its control period using
`Timer::delay_us`; rp2040-hal 0.12 implements this as a timer-polling loop. It
therefore executes instructions during idle time. Its exact share of the above
cost has NOT yet been profiled. A production-compatible timer interrupt and
CPU sleep implementation is a candidate, not an implemented or verified fix.
Any optimization must preserve peripheral events, watchdog expiry, control
deadlines, and the real firmware boundary; clock acceleration is not a remedy.

Per-step bridge diagnostics now include `mcu_processing_ms`,
`plant_round_trip_ms`, and `instructions`. These diagnostics are not yet retained
by the browser analysis dataset; that remains part of the open evidence review.
