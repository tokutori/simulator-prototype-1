import { createServer } from "node:http";
import { spawn, type ChildProcessWithoutNullStreams } from "node:child_process";
import { existsSync } from "node:fs";
import { createInterface } from "node:readline";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
import { createServer as createViteServer } from "vite";
import { WebSocket, WebSocketServer } from "ws";

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

const vite = await createViteServer({
  root: visualizerDirectory,
  server: { middlewareMode: true },
  appType: "spa",
});
const httpServer = createServer((request, response) => {
  vite.middlewares(request, response, () => {
    response.statusCode = 404;
    response.end("not found");
  });
});
const webSockets = new WebSocketServer({ noServer: true });

httpServer.on("upgrade", (request, socket, head) => {
  if (request.url !== "/live") {
    socket.destroy();
    return;
  }
  webSockets.handleUpgrade(request, socket, head, (webSocket) => {
    webSockets.emit("connection", webSocket, request);
  });
});

webSockets.on("connection", (webSocket) => {
  let command: PilotCommand = {
    pilot_elevator: 0,
    pilot_rudder: 0,
    autonomy: 1,
    elevator_input_kind: "analog",
    rudder_input_kind: "analog",
  };
  let lastInputAt = Date.now();
  let bridge: ChildProcessWithoutNullStreams | undefined;
  let stepTimer: ReturnType<typeof setTimeout> | undefined;
  let nextStepAtMs = 0;
  try {
    bridge = spawn(process.execPath, [tsxCliPath, webBridgePath], { cwd: projectDirectory, windowsHide: true });
  } catch (error) {
    sendJson(webSocket, { type: "error", message: String(error) });
    webSocket.close();
    return;
  }

  const lines = createInterface({ input: bridge.stdout });
  lines.on("line", (line) => {
    try {
      const status = JSON.parse(line) as { type?: string; backend?: string };
      if (status.type === "ready") {
        if (status.backend !== "rp2040js-actual-uf2") throw new Error("unexpected MCU backend");
        clearTimeout(startupTimer);
        nextStepAtMs = performance.now();
        scheduleStep();
        return;
      }
    } catch (error) {
      sendJson(webSocket, { type: "error", message: `invalid MCU bridge output: ${String(error)}` });
      return;
    }
    if (webSocket.readyState === WebSocket.OPEN) {
      webSocket.send(line);
    }
    scheduleStep();
  });
  let errorText = "";
  bridge.stderr.setEncoding("utf8");
  bridge.stderr.on("data", (chunk: string) => {
    errorText += chunk;
  });
  bridge.on("exit", (code) => {
    if (stepTimer) clearTimeout(stepTimer);
    clearTimeout(startupTimer);
    if (webSocket.readyState === WebSocket.OPEN) {
      sendJson(webSocket, {
        type: code === 0 ? "ended" : "error",
        message: code === 0 ? "surface contact" : errorText.trim() || `bridge exited with ${code}`,
      });
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
      sendJson(webSocket, { type: "error", message: "invalid control JSON" });
    }
  });

  const sendStep = (): void => {
    if (bridge?.stdin.destroyed || bridge?.stdin.writableEnded) {
      if (stepTimer) clearTimeout(stepTimer);
      return;
    }
    const safeCommand =
      Date.now() - lastInputAt > inputTimeoutMs
        ? { ...command, pilot_elevator: 0, pilot_rudder: 0 }
        : command;
    bridge.stdin.write(`${JSON.stringify(safeCommand)}\n`);
  };
  const scheduleStep = (): void => {
    if (stepTimer) clearTimeout(stepTimer);
    nextStepAtMs += stepPeriodMs;
    stepTimer = setTimeout(sendStep, Math.max(0, nextStepAtMs - performance.now()));
  };
  const startupTimer = setTimeout(() => {
    sendJson(webSocket, { type: "error", message: "actual UF2 did not arm in rp2040js within 15 seconds" });
    bridge?.kill();
  }, 15_000);

  webSocket.on("close", () => {
    if (stepTimer) clearTimeout(stepTimer);
    clearTimeout(startupTimer);
    lines.close();
    if (bridge && bridge.exitCode === null) {
      bridge.kill();
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
