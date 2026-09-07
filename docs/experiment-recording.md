# Experiment recording contract

The Web UI records full-resolution firmware/plant observations, not presentation-interpolated frames. IndexedDB records use schema version 3.

- Each measured frame retains aerodynamic-envelope membership, producer wall elapsed time, processing duration/average, real-time ratio, lag, deadline flag, real-time flag, and the explicit `timingValidated=false` limitation.
- Run identity retains UF2, model, plant, virtual-platform and scenario fingerprints. New live telemetry requires all five. Older records without platform/scenario hashes are not a complete reproduction manifest.
- Session outcomes are `active` (snapshot, not a completed flight), `ended` with reason, `failed` with original error reason, `aborted` by replay/restart, or `unknown` for missing/legacy evidence. Surface contact alone does not establish the termination reason.
- The browser independently checks reception age on its animation/wall clock. After 500 ms without a new observation it shows a TEA `mcu-stalled` warning. Fresh telemetry may recover that state; the stall event remains in recorded incidents. The server has a separate wall-clock response timeout.
- Restart, switching to replay, and terminal events archive the stopped session before replacing its in-memory frames. **Previous stopped flight** opens the most recent archive, including preflight failures without telemetry. The link survives page reload when localStorage is available; archive data uses IndexedDB. Storage errors are surfaced, not silently treated as a successful archive. Abrupt browser/process termination before a save completes is not guaranteed durable.

Downloaded live CSV begins with `# birdman-session <JSON>` and `# birdman-incidents <JSON>`. Data follows as ordinary comma-separated columns; timing/envelope evidence has explicit numeric/boolean columns. The replay loader preserves these comments as session metadata. External CSV tools should skip `#` comment lines. Older CSV and version-2 IndexedDB records have explicit unknown experiment/outcome evidence; missing values are never upgraded to real-time/within-envelope/successful.

Analysis shows outcome/reason and warns for out-of-envelope, slowed interaction, missed deadlines, unknown evidence and telemetry stalls. No record establishes physical HIL timing or real-aircraft validation.

Firmware evidence is accepted only when every required sequence/time/PWM/validity/identity field is present and finite. Incomplete measured CSV or version-3 stored evidence is rejected; incomplete legacy firmware records become unavailable, never fabricated zero-valued firmware samples. Replay file selection uses an intent generation, so late startup samples or older file reads cannot replace the selected data or its error state.
