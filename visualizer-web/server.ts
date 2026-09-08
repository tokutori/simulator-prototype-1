import { createServer } from "node:http";
import { spawn, type ChildProcessWithoutNullStreams } from "node:child_process";
import { existsSync } from "node:fs";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
import { createServer as createViteServer } from "vite";
import { WebSocket, WebSocketServer } from "ws";
import { ResponseDeadline } from './src/response-deadline';
import { allowLiveAccess } from './src/live-access';

interface PilotCommand {
  pilot_elevator: number;
  pilot_rudder: number;
  autonomy: number;
  elevator_input_kind: "analog" | "buttons";
  rudder_input_kind: "analog" | "buttons";
}

const visualizerDirectory = dirname(fileURLToPath(import.meta.url));
const projectDirectory = resolve(visualizerDirectory, "..");
const tsxCliPath = resolve(projectDirectory, "virtual-platform", "node_modules", "tsx", "dist", "cli.mjs");
const webBridgePath = resolve(projectDirectory, "virtual-platform", "src", "web-bridge.ts");
const uf2Path = resolve(projectDirectory, "target", "virtual-platform", "fbw-rp2040.uf2");
const port = Number.parseInt(process.env.BIRDMAN_VISUALIZER_PORT ?? "4173", 10);
const stepPeriodMs = 10;
const inputTimeoutMs = 250;

for (const path of [tsxCliPath, webBridgePath, uf2Path]) {
  if (!existsSync(path)) throw new Error(`actual-UF2 web dependency not found: ${path}; run npm install in virtual-platform and npm run build:platform`);
}

const httpServer = createServer((request, response) => {
  vite.middlewares(request, response, () => {
    response.statusCode = 404;
    response.end("not found");
  });
});
const vite = await createViteServer({
  root: visualizerDirectory,
  server: { middlewareMode: true, ws: { server: httpServer } },
  appType: "spa",
});
const webSockets = new WebSocketServer({ noServer: true });

httpServer.on("upgrade", (request, socket, head) => {
  // Vite's listener on this same HTTP server owns its HMR connection. Keeping
  // it on the selected app port avoids cross-instance port 24678 collisions.
  if (['vite-hmr', 'vite-ping'].includes(String(request.headers['sec-websocket-protocol']))) return;
  if (request.url !== "/live") {
    socket.destroy();
    return;
  }
  if (!allowLiveAccess(request.headers.host, request.headers.origin, port)) {
    socket.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');
    return;
  }
  if (webSockets.clients.size >= 4) {
    socket.end('HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n');
    return;
  }
  webSockets.handleUpgrade(request, socket, head, (webSocket) => {
    webSockets.emit("connection", webSocket, request);
  });
});

webSockets.on("connection", (webSocket) => {
  const connectedAtMs = performance.now();
  const lifecycleLog = (stage: string): void => {
    process.stderr.write(`session pid=${bridge?.pid ?? 'pending'} ${stage} after ${(performance.now() - connectedAtMs).toFixed(1)} ms\n`);
  };
  let command: PilotCommand = {
    pilot_elevator: 0,
    pilot_rudder: 0,
    autonomy: 1,
    elevator_input_kind: "analog",
    rudder_input_kind: "analog",
  };
  let lastInputAt = Date.now();
  let bridge: ChildProcessWithoutNullStreams | undefined;
  let stepTimer: { tag: 'none' } | { tag: 'timer'; handle: ReturnType<typeof setTimeout> }
    | { tag: 'immediate'; handle: ReturnType<typeof setImmediate> } = { tag: 'none' };
  const cancelStep = (): void => {
    switch (stepTimer.tag) {
      case 'timer': clearTimeout(stepTimer.handle); break;
      case 'immediate': clearImmediate(stepTimer.handle); break;
      case 'none': break;
    }
    stepTimer = { tag: 'none' };
  };
  let nextStepAtMs = 0;
  let terminalReceived = false;
  const responseDeadline = new ResponseDeadline(5000);
  let terminationRequested = false;
  const stopBridge = (): void => {
    if (terminationRequested || !bridge) return;
    terminationRequested = true;
    terminateBridge(bridge);
  };
  try {
    bridge = spawn(process.execPath, [tsxCliPath, webBridgePath], { cwd: projectDirectory, windowsHide: true });
    bridge.once('spawn', () => lifecycleLog('process spawned'));
  } catch (error) {
    sendJson(webSocket, { type: "error", message: String(error) });
    webSocket.close();
    return;
  }

  const lines = createInterface({ input: bridge.stdout });
  lines.on("line", (line) => {
    if (terminalReceived) return;
    try {
      const status = JSON.parse(line) as { type?: string; backend?: string; message?: string };
      if (status.type === "ended" || status.type === "error") {
        finishSession(status.type, status.message ?? 'bridge supplied no terminal detail');
        return;
      }
      if (status.type === "ready") {
        if (status.backend !== "rp2040js-actual-uf2") throw new Error("unexpected MCU backend");
        clearTimeout(startupTimer);
        lifecycleLog('ready');
        nextStepAtMs = performance.now();
        scheduleStep();
        return;
      }
    } catch (error) {
      finishSession('error', `invalid MCU bridge output: ${String(error)}`);
      return;
    }
    if (webSocket.readyState === WebSocket.OPEN) {
      webSocket.send(line);
    }
    responseDeadline.complete();
    scheduleStep();
  });
  let errorText = "";
  bridge.stderr.setEncoding("utf8");
  bridge.stderr.on("data", (chunk: string) => {
    errorText = (errorText + chunk).slice(-8192);
  });
  bridge.on('error', error => finishSession('error', `MCU bridge process error: ${String(error)}`));
  bridge.stdin.on('error', error => finishSession('error', `MCU bridge input error: ${String(error)}`));
  bridge.on("exit", (code) => {
    lifecycleLog(`process exited code=${code}`);
    clearInterval(responseMonitor);
    cancelStep();
    clearTimeout(startupTimer);
    if (!terminalReceived && webSocket.readyState === WebSocket.OPEN) {
      finishSession('error', errorText.trim() || `bridge exited with ${code} without a terminal reason`);
    }
  });

  webSocket.on("message", (data) => {
    try {
      const parsed = JSON.parse(data.toString()) as Partial<PilotCommand>;
      if (
        isNormalized(parsed.pilot_elevator) &&
        isNormalized(parsed.pilot_rudder) &&
        typeof parsed.autonomy === "number" &&
        Number.isFinite(parsed.autonomy) &&
        parsed.autonomy >= 0 &&
        parsed.autonomy <= 1 &&
        isInputKind(parsed.elevator_input_kind) &&
        isInputKind(parsed.rudder_input_kind)
      ) {
        command = {
          pilot_elevator: parsed.pilot_elevator,
          pilot_rudder: parsed.pilot_rudder,
          autonomy: parsed.autonomy,
          elevator_input_kind: parsed.elevator_input_kind,
          rudder_input_kind: parsed.rudder_input_kind,
        };
        lastInputAt = Date.now();
      }
    } catch {
      finishSession('error', 'invalid control JSON');
    }
  });

  const sendStep = (): void => {
    if (bridge?.stdin.destroyed || bridge?.stdin.writableEnded) {
      cancelStep();
      return;
    }
    const safeCommand =
      Date.now() - lastInputAt > inputTimeoutMs
        ? { ...command, pilot_elevator: 0, pilot_rudder: 0 }
        : command;
    responseDeadline.begin(performance.now());
    bridge.stdin.write(`${JSON.stringify(safeCommand)}\n`);
  };
  const scheduleStep = (): void => {
    if (terminalReceived) return;
    cancelStep();
    nextStepAtMs += stepPeriodMs;
    const delay = nextStepAtMs - performance.now();
    // An overdue step must not pay the host's minimum timer quantum again.
    // Yield through the I/O loop so new pilot input is still serviced.
    stepTimer = delay > 0
      ? { tag: 'timer', handle: setTimeout(sendStep, delay) }
      : { tag: 'immediate', handle: setImmediate(sendStep) };
  };
  const startupTimer = setTimeout(() => {
    finishSession('error', `actual UF2 did not arm in rp2040js within 15 seconds${errorText.trim() ? `; ${errorText.trim()}` : '; no bridge startup diagnostic received'}`);
  }, 15_000);

  // This timer lives outside the CPU-emulation/plant process: a blocked plant
  // read cannot freeze the watchdog that supervises it.
  const responseMonitor = setInterval(() => {
    if (terminalReceived || !responseDeadline.expired(performance.now())) return;
    finishSession('error', 'actual-UF2 bridge response timed out after 5 seconds (wall clock)');
  }, 250);

  function finishSession(type: 'ended' | 'error', message: string): void {
    if (terminalReceived) return;
    terminalReceived = true;
    lifecycleLog(`terminal ${type}`);
    responseDeadline.complete();
    cancelStep();
    clearTimeout(startupTimer);
    clearInterval(responseMonitor);
    sendJson(webSocket, { type, message });
    // The server owns teardown, including clients which do not voluntarily
    // disconnect after an error. Close follows previously queued telemetry.
    webSocket.close(type === 'ended' ? 1000 : 1011, 'flight finished');
    stopBridge();
  }

  webSocket.on("close", () => {
    clearInterval(responseMonitor);
    cancelStep();
    clearTimeout(startupTimer);
    lines.close();
    if (bridge && bridge.exitCode === null) {
      stopBridge();
    }
  });
});

httpServer.listen(port, "127.0.0.1", () => {
  process.stdout.write(`Birdman visualizer: http://127.0.0.1:${port}\n`);
});

function isNormalized(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value) && value >= -1 && value <= 1;
}

function isInputKind(value: unknown): value is PilotCommand["elevator_input_kind"] {
  return value === "analog" || value === "buttons";
}

function sendJson(webSocket: WebSocket, value: unknown): void {
  if (webSocket.readyState === WebSocket.OPEN) {
    webSocket.send(JSON.stringify(value));
  }
}

/** The bridge owns a plant child. On Windows, killing only its parent can
 * leave a wedged plant alive; terminate this known spawned subtree as a unit.
 */
function terminateBridge(child: ChildProcessWithoutNullStreams): void {
  if (child.exitCode !== null || child.signalCode !== null) return;
  const pid = child.pid;
  if (process.platform === 'win32' && typeof pid === 'number') {
    const cleanupStartedMs = performance.now();
    const cleanup = spawn('taskkill.exe', ['/PID', String(pid), '/T', '/F'],
      { windowsHide: true, stdio: 'ignore' });
    cleanup.on('error', error => {
      process.stderr.write(`failed to terminate simulator process tree: ${String(error)}\n`);
      child.kill();
    });
    cleanup.on('exit', (code, signal) => {
      // A nonzero result may mean a naturally exiting child won the race. Keep
      // the result visible instead of silently claiming subtree cleanup worked.
      process.stderr.write(`cleanup pid=${pid} code=${code} signal=${signal ?? 'none'} elapsed=${(performance.now() - cleanupStartedMs).toFixed(1)} ms parentExited=${child.exitCode !== null || child.signalCode !== null}\n`);
    });
  } else {
    // Cooperative EOF lets the bridge close its plant, with an upper bound if
    // the bridge itself is stuck. The plant also has a four-second read bound.
    child.stdin.end();
    const timeout = setTimeout(() => child.kill(), 5000);
    child.once('exit', () => clearTimeout(timeout));
  }
}
