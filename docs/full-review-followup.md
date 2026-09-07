# Follow-up to the September 8 full review

This is an open remediation record, not a claim of completed aircraft validation.
The earlier closure in `review-remediation.md` did not cover these subsequently
identified defects. Completion requires an integrated gate and another full
review, not merely passing each local test.

| Finding | Current evidence / remaining work |
| --- | --- |
| MCU capacity near 0.25x | Production timer sleep and bounded peripheral-event execution, live CPU multiplier 1. 80-update smoke at 4173 reached minimum 0.96x and no non-realtime samples before the subsequent sensor changes; repeat integrated performance gate. |
| Camera discontinuities / slow receipt jitter | Shared eye/target smoothing, lifecycle resets, monotonic simulation-time playback clock and unit tests. Browser regression gate in progress. |
| Keyboard focus on ordinary UI buttons | Buttons no longer suppress pilot keys; editable controls remain protected. Restart-without-canvas-click browser regression added. |
| BNO055 Euler/gyro axis mismatch | Quaternion installation contract and independent basis tests in `bno055-frame-contract.md`; actual-UF2 recovery matrix passes. Real mounting/calibration still requires bench verification. |
| Initial airspeed versus groundspeed | Schema 0.11 discriminated velocity frame; training launch explicitly ground5m/s, downward3deg. Shared state constructor and wind/pose tests. |
| Sideslip force transformation | Explicit force basis. Wind-axis data uses full alpha/beta transform. BR data retains its documented stability-plane lift/drag plus total body CY approximation; no invented lateral coefficients or double-counted drag. |
| Servo timestep instability | Analytic held-input response across rate limit, lag and deadband; monotonic and partition-invariance tests. |
| Lost validity / termination / performance evidence | Web evidence schema implementation and integrated browser validation in progress. |
| Hung plant not detected by virtual-time watchdog | Plant response4s and server response5s wall-clock monitors; deterministic deadline test. Browser packet-age warning in progress. Real hung-process integration remains to be exercised. |
| Incomplete run provenance | UF2/model/plant plus adapter, installed rp2040js, dependency lock and Node/V8 fingerprints; scenario identity includes timestep and fault conditions. CSV includes both new hashes. |
| UART callback bypasses TX configuration/time | Adapter correction in progress; negative configuration and actual-UF2 tests still required. |

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
