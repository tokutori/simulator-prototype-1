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
}

const visualizerDirectory = dirname(fileURLToPath(import.meta.url));
const projectDirectory = resolve(visualizerDirectory, "..");
const executableName = process.platform === "win32" ? "interactive-bridge.exe" : "interactive-bridge";
const bridgePath = resolve(projectDirectory, "target", "debug", executableName);
const modelPath = resolve(projectDirectory, "models", "qx18-br-training-envelope.json");
const port = Number.parseInt(process.env.BIRDMAN_VISUALIZER_PORT ?? "4173", 10);
const stepPeriodMs = 10;
const inputTimeoutMs = 250;

if (!existsSync(bridgePath)) {
  throw new Error(`interactive bridge not found: ${bridgePath}; run npm run build:bridge`);
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
  let command: PilotCommand = { pilot_elevator: 0, pilot_rudder: 0, autonomy: 1 };
  let lastInputAt = Date.now();
  let bridge: ChildProcessWithoutNullStreams | undefined;
  try {
    bridge = spawn(
      bridgePath,
      ["--model", modelPath, "--dt", String(stepPeriodMs / 1000)],
      { cwd: projectDirectory, windowsHide: true },
    );
  } catch (error) {
    sendJson(webSocket, { type: "error", message: String(error) });
    webSocket.close();
    return;
  }

  const lines = createInterface({ input: bridge.stdout });
  lines.on("line", (line) => {
    if (webSocket.readyState === WebSocket.OPEN) {
      webSocket.send(line);
    }
  });
  let errorText = "";
  bridge.stderr.setEncoding("utf8");
  bridge.stderr.on("data", (chunk: string) => {
    errorText += chunk;
  });
  bridge.on("exit", (code) => {
    clearInterval(stepTimer);
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
        parsed.autonomy <= 1
      ) {
        command = {
          pilot_elevator: parsed.pilot_elevator,
          pilot_rudder: parsed.pilot_rudder,
          autonomy: parsed.autonomy,
        };
        lastInputAt = Date.now();
      }
    } catch {
      sendJson(webSocket, { type: "error", message: "invalid control JSON" });
    }
  });

  const stepTimer = setInterval(() => {
    if (bridge?.stdin.destroyed || bridge?.stdin.writableEnded) {
      clearInterval(stepTimer);
      return;
    }
    const safeCommand =
      Date.now() - lastInputAt > inputTimeoutMs
        ? { ...command, pilot_elevator: 0, pilot_rudder: 0 }
        : command;
    bridge.stdin.write(`${JSON.stringify(safeCommand)}\n`);
  }, stepPeriodMs);

  webSocket.on("close", () => {
    clearInterval(stepTimer);
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

function sendJson(webSocket: WebSocket, value: unknown): void {
  if (webSocket.readyState === WebSocket.OPEN) {
    webSocket.send(JSON.stringify(value));
  }
}
