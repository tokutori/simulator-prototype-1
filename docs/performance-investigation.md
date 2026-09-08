# Interactive performance investigation

## Contract

Interactive flight executes the actual UF2 in rp2040js with clock multiplier 1.
Rendering smoothness must not be confused with real-time controller capacity.
No physical cycle-accuracy claim follows from this multiplier.

## Browser-inclusive follow-up (2026-09-08)

`node --import tsx scripts/check-render-performance.ts` in `visualizer-web`
diagnoses the current `:4173` app at FHD and 4K. It reports MCU and plant work,
simulation/wall ratio, animation cadence, and WebGL renderer identity. This is
a diagnostic, not a portable machine-speed assertion. The actual open user
browser was not inspected.

Current headless Chromium uses **SwiftShader CPU rendering** on this host.
Ten-second windows observed about 7 FPS at FHD and 2–3 FPS at 4K, while MCU
progress remained roughly 0.9–1.0x. With no browser rendering, a full 23s
surface-contact flight took 22.98s wall time. Hardware-accelerated browser
performance cannot be inferred from either test.

The low-FPS investigation exposed two additional application bugs: a 50ms
presentation-delta cap made replay and chase smoothing run slowly; input was
sent only from animation callbacks, exceeding the 250ms server input lease at
3 FPS. Presentation now uses elapsed wall time, and input polling/sending has
its own 40ms timer. This does not bypass the stale-input safety policy when the
browser thread itself is blocked. DRAW FPS is distinct from MCU speed; low-FPS
windows are archived as typed session incidents and warned about in analysis.

High-DPI pixel load should be assessed separately from CSS UI sizing; see the
[Three.js responsive rendering guide](https://threejs.org/manual/en/responsive.html).
The current change does not silently reduce image quality or claim a GPU fix.

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

The baseline firmware waited for the remainder of its control period using
`Timer::delay_us`; rp2040-hal 0.12 implements this as a timer-polling loop. It
therefore executes instructions during idle time. Its exact share of the above
cost was not instruction-profiled before replacement.
Any optimization must preserve peripheral events, watchdog expiry, control
deadlines, and the real firmware boundary; clock acceleration is not a remedy.

Per-step bridge diagnostics now include `mcu_processing_ms`,
`plant_round_trip_ms`, and `instructions`. These diagnostics are not yet retained
by the browser analysis dataset; that remains part of the open evidence review.

## Timer sleep and event execution phase

The production firmware now uses TIMER alarm 0, an atomic completion flag,
WFE, and an ISR issuing SEV. The ISR clears both INTF (HAL's late-alarm force
path) and INTR. The independent review caught the need to clear INTF before
this phase was committed. This is the same firmware for real hardware and sim.

Web and batch share `stepMcu`: while the actual core is asleep, dispatch only
up to the next peripheral alarm or plant boundary. Executing instructions still
uses CPU multiplier 1. PWM and watchdog events are not skipped. This follows
[rp2040js's own idle execution approach](https://github.com/wokwi/rp2040js/blob/v1.3.3/src/simulator.ts),
with an additional coupling boundary. Alarm behavior follows the
[HAL Alarm contract](https://docs.rs/rp2040-hal/0.12.0/rp2040_hal/timer/trait.Alarm.html).

Final UF2 SHA256: `5b058379d1ee265c89dd12db307443c9abd2606c02d4e994b5a9a8f1ee2bfc00`.
Same 300-update capacity probe: 727.4 ms wall for 3 s simulated (4.12x),
mean MCU 1.204 ms, plant round trip 0.595 ms, 18,944 instructions/update.
Earlier runs of this change ranged around 2.61x; host-load sensitivity remains.
These are unpaced capacity results, not a claim that interactive flight runs
at faster-than-real-time speed.

Server pacing also used to add a minimum timer wait to every overdue step.
Overdue work now yields with setImmediate, while future deadlines use timers.
This preserves pilot-message servicing without repeatedly paying a host timer
quantum while already behind. With the new server, 80 live WebSocket updates
had no non-realtime samples, maximum lag 30.3 ms in the phase smoke test.

Validation: 20 platform unit tests; actual-UF2 transient, one-reset and
persistent-reset recovery tests (0/1/4 reinitializations); stalled I2C still
triggers the actual watchdog. Physical sleep/IRQ timing requires HIL. The
emulator's SEV opcode itself only logs, although exception entry/return sets
the event register for this ISR-based use; this is not general SEV validation.
