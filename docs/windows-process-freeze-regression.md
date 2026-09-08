# Windows real-process freeze / liveness regression

Run from `visualizer-web/` after building the production platform:

```powershell
node --import tsx scripts/check-process-freeze.ts
```

This test binds an isolated server on **4185**, failing if that port is occupied.
It opens a real WebSocket and waits for actual-UF2 telemetry before injecting a
fault. It does not replace firmware, MCU emulation, plant, transport, or server.

The PowerShell helper enumerates the new server's PID descendants and verifies
the target's process creation time before suspension. For the plant case it
selects the unique owned `plant-bridge.exe`. For the MCU case it selects that
plant's direct parent, the actual `web-bridge.ts` Node process, not its tsx CLI
launcher. It uses OpenThread/SuspendThread on only this process's threads. Each
successfully suspended thread handle is retained and resumed/closed in finally
when the controller releases the helper or its stdin closes.

The controller requires the expected wall-clock error and verifies all owned
MCU/plant descendants have disappeared **before closing the WebSocket**. Thus
normal test teardown cannot supply the cleanup being tested. Finally it kills
only its explicitly spawned server subtree and releases the freeze helper.

## Observed result — 2026-09-08 JST

Both cases passed against production UF2
`a8bbcb86735eff1f5134bb321b3a83a4f44a014be59a7ea171a60fddfc9a480d`:

| Frozen process | Threads | Observed terminal after freeze acknowledgment | Result |
| --- | ---: | --- | --- |
| Actual plant | 4 | 3965 ms, `plant response timed out after 4 seconds (wall clock)` | MCU launcher, actual MCU and plant all removed |
| Actual MCU bridge | 18 | 5005 ms, `actual-UF2 bridge response timed out after 5 seconds (wall clock)` | MCU launcher, actual MCU and plant all removed |

The first observed duration is slightly below 4000 ms because the request timer
can start before suspension acknowledgment. The test also checks elapsed time
from the last telemetry sample to reject premature timeouts. It prints every
run's actual artifact identities and owned PIDs as JSON; the table is historical
evidence, not a promise about subsequent artifacts or host scheduling.

Plant SHA256: `ccd35dd8a7c760b59026cc58a664ed74e099e70086a769588db1954bb87e32a7`.
Virtual-platform SHA256: `117dafd888964ca17b61d1f5f41b76a1e0747bdf18ebfff91ed0d1e9065f1d88`.
Model SHA256: `d40d73bb05f07eab15f1f62a02eea1250eb8c0daab2bdf4a3a6d1b8472a979f4`.

This verifies OS-process liveness supervision and subtree cleanup on Windows,
not embedded control stability, physical watchdog timing, or electrical safety.
The debugger-style thread suspension is confined to expendable test descendants;
it must not be used as application synchronization or against an interactive
user process.

Sources: [Microsoft SuspendThread](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-suspendthread),
[Microsoft ResumeThread](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-resumethread).
