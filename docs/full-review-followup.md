# Follow-up to the September 8 full review

This is an open remediation record, not a claim of completed aircraft validation.
The earlier closure in `review-remediation.md` did not cover these subsequently
identified defects. Completion requires an integrated gate and another full
review, not merely passing each local test.

## Current audit status (supersedes historical pending notes below)

At `dee91da`, all five integrated browser tests pass, including actual-UF2 held
input at 3 FPS; 59 browser-domain unit tests and build/typecheck pass. Thirty
virtual-platform unit tests and its typecheck also pass. Real process freeze,
normal terminal delivery, long recording capacity, and chase platform visual
checks described below are complete within their stated software scope.

The remaining investigated issue is an intermittent 15s startup timeout seen
during SwiftShader browser testing. Twelve sequential no-render starts took
0.48–0.65s, all releasing at MCU780780.536us. Three repeated browser
restart/replay/analysis tests then passed (31.7–37.8s per whole test).
This does **not** prove the earlier timeout's root cause. Stage/PID/process-CPU
diagnostics now distinguish reached initialization stages; timeout reporting
includes the captured diagnostic instead of losing it. Diagnostic timing starts
after module evaluation, so loader/scheduler delay remains outside that clock.
No timeout threshold or production control behavior was relaxed.

Independent startup-path review found no deterministic race explaining the
observations. Host contention is a hypothesis, not an established cause.
The full audit remains open while that distinction is unresolved.

Server lifecycle logging now records spawn/ready/exit and taskkill completion.
In a further isolated 12-start run, all 12 cleanup commands exited0 (about
358–541ms); process exit events followed. The current real-process freeze gate
also passed both cases and checked that owned MCU/plant PIDs were absent.
This does not establish how a prior, uninstrumented timeout occurred.

The next independent holistic review identified a separate reproducible live
presentation defect: averaging clipped instantaneous arrival-rate ratios biases
the display clock slow. Alternating5/15ms packet intervals for a100Hz producer
left the displayed state about9.8s behind after60s. A windowed producer-clock
estimate and bounded backlog recovery are being implemented and tested.

A separate 12-start probe was then overlapped with FHD/4K SwiftShader rendering.
All starts succeeded; during rendering, first telemetry took 0.69–2.12s,
returning to about0.48s after the render probe ended. This establishes host-load
sensitivity, not the cause of the earlier15s outlier. Rendering ran about7.4FPS
at FHD and2.7FPS at4K; 4K accumulated about1s simulation lag under this combined
load. It is not acceptable to label that combined workload fully real-time.

| Finding | Current evidence / remaining work |
| --- | --- |
| MCU capacity near 0.25x | Production timer sleep and bounded peripheral-event execution, live CPU multiplier 1. 80-update smoke at 4173 reached minimum 0.96x and no non-realtime samples before the subsequent sensor changes; repeat integrated performance gate. |
| Camera discontinuities / slow receipt jitter | Shared eye/target smoothing, lifecycle resets, monotonic simulation-time playback clock and unit tests. Browser regression gate in progress. |
| Keyboard focus on ordinary UI buttons | Buttons no longer suppress pilot keys; editable controls remain protected. Restart-without-canvas-click browser regression added. |
| BNO055 Euler/gyro axis mismatch | Quaternion installation contract and independent basis tests in `bno055-frame-contract.md`; actual-UF2 recovery matrix passes. Real mounting/calibration still requires bench verification. |
| Initial airspeed versus groundspeed | Schema 0.11 discriminated velocity frame; training launch explicitly ground5m/s, downward3deg. Shared state constructor and wind/pose tests. |
| Sideslip force transformation | Explicit force basis. Wind-axis data uses full alpha/beta transform. BR data retains its documented stability-plane lift/drag plus total body CY approximation; no invented lateral coefficients or double-counted drag. |
| Servo timestep instability | Analytic held-input response across rate limit, lag and deadband; monotonic and partition-invariance tests. |
| Lost validity / termination / performance evidence | Implemented schema3 record/outcome/incidents and CSV metadata. Earlier integrated browser active-snapshot/reload test passed; additional strict malformed-evidence checks undergoing final gate. |
| Hung plant not detected by virtual-time watchdog | Plant response4s and server response5s wall-clock monitors; deterministic deadline test. Browser packet-age warning and incident retention implemented. Windows bridge termination owns its process subtree. Real hung-process integration remains to be exercised. |
| Incomplete run provenance | UF2/model/plant plus adapter, installed rp2040js, dependency lock and Node/V8 fingerprints; scenario identity includes timestep and fault conditions. CSV includes both new hashes. |
| UART callback bypasses TX configuration/time | Serial completion/FIFO/backpressure and enable/mux/framing/baud checks implemented. Negative configuration and actual-UF2 tests pass. Unsupported IrDA/TXIRQ/activeDMA and DPS modes explicitly rejected; physical waveform remains unvalidated. |

## Additional findings from the next holistic review

- Non-CG moment references could be accepted without a moment translation;
  enforce the supported reference contract rather than silently reinterpret data.
- Invalid states/extreme finite steps could panic inside aerodynamic lookup;
  structured numerical errors and application timestep policy are being added.
- Delayed initial sample loading could replace a user-selected replay;
  request-generation handling and regression coverage are being added.
- Incomplete saved firmware evidence could pass presence-only validation;
  required fields must be verified explicitly and never replaced with fake zeroes.
- Live WebSocket origin/session admission and Windows process-subtree ownership
  have been tightened; deterministic access tests pass.
- Vite updates now share each application's HTTP port, avoiding competing HMR
  connections across development and E2E servers. Two browser tests passed after
  this fix; a subsequent run overlapped UART hardening and must be repeated on
  the stable integrated tree.

## Integrated physical-model smoke

After servo, BNO frame and schema updates, actual-UF2 nominal run:
22.99s to contact, 0m maximum re-ascent, 0 positive-flight-path samples,
0 invalid-sensor duration, 0 failsafe activations and 0 control deadline misses.
This tests software integration, not a measured aircraft prediction.

UF2 `3f960cef095543be91b73f766077d6e5d9df467cc14130226df163ca8118035b`;
model `6b1b2a2f4d242fad0cf67695e879b7aa5ace1a21099bc16f7abd4f573a810ccb`;
plant `ec915a9f47ea7e3e584c537abbaf3337c8448496331ffc44b6106feee7e67ee6`.
Generated local evidence: `target/full-review-flight.json` and `.csv`.
Transient, single-reset, persistent-reset cases observed 0/1/4 reinitializations;
all rearmed. Stalled I2C still triggered the actual virtual watchdog.

## Remaining scope beyond individual defect patches

- Run integrated tests after all adapter and record-format changes settle.
- Rebuild served artifacts and refresh the actual 4173 server as necessary.
- Full independent review of complete paths, plus diff review.
- Verify live camera lifecycle and recording reload in a browser, including 4K.
- Keep physical bus timing, electrical faults, real servos, sensor calibration,
  VR headset operation and aircraft model identification explicitly unvalidated.

## Latest integrated snapshot

At `bc85541`, the additional numeric/CG, replay-selection, strict saved evidence,
and unsupported-peripheral-mode findings above are implemented. Final browser
suite: three tests pass (actual UF2 restart and keyboard without canvas focus,
delayed sample versus user selection, and 12001-row 4K analysis reload).
An earlier restart failure was a real 15s startup timeout: firmware still spun
through the 650ms sensor boot wait. Both 650ms and20ms startup waits now use the
production timer-alarm sleep. Actual UF2 recovery and watchdog tests pass again.

Visual inspection also found author CSS overriding `hidden`, exposing replay
controls in Interactive mode; a global hidden rule and browser assertions now
prevent it. Performance text no longer overlaps central camera controls at FHD.

The freshly restarted 4173 server's80-update actual-UF2 smoke measured maximum
average3.32ms/update, minimum1.03x real-time and zero non-realtime samples.
These short observations do not establish a universal host-speed guarantee.

Remaining final audit includes the longer-capacity run after schema0.12 and
the explicit frozen-process liveness test. The launch platform also occupies
much of the early chase view in captured images; evaluate camera occlusion
handling before considering the visual review closed. Physical model/HIL
limitations remain as stated above.

### Schema 0.12 duration gate

The rerun of `virtual-platform/src/long-run-check.ts` passed at the current
UF2/model/plant versions: 120s, 12000 control updates, 470877919 executed
instructions, zero deadline misses/failsafe activations/invalid sensor duration.
The CSV retains exactly 12000 data rows and the fixture passes 1000m north.
This is a high-altitude runtime-capacity fixture, not a Birdman range prediction.
The gate now asserts those properties as well as duration and the old lifetime
instruction-cap regression. Generated evidence is under `target/runtime-capacity`.

### Terminal and browser-performance follow-up

The real Windows process-freeze regression now passes for plant and MCU:
wall deadlines fire, the server closes with 1011, and owned processes disappear
before the client test performs cleanup. Natural surface contact also passes:
2300 updates, final contact telemetry, terminal reason, server close 1000.
Unified terminal teardown received a further independent read-only review.
See `windows-process-freeze-regression.md` for the supported test boundary.

The chase-platform obstruction is fixed and captured in browser screenshots.
Further low-FPS review found presentation clock clipping and RAF-coupled pilot
input. These are being checked with a new 3 FPS actual-UF2 regression; see
`performance-investigation.md`. Keep the completion audit open until the full
browser suite passes on the stable tree: an integrated run also observed an
intermittent real startup timeout. Physical HIL/VR/model limits remain unchanged.
