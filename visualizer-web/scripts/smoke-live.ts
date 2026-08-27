import WebSocket from "ws";

const baseUrl = process.env.BIRDMAN_VISUALIZER_URL ?? "http://127.0.0.1:4173";
const indexResponse = await fetch(`${baseUrl}/`);
if (!indexResponse.ok || !(await indexResponse.text()).includes("Birdman Flight Visualizer")) {
  throw new Error("visualizer index is unavailable");
}
const sampleResponse = await fetch(`${baseUrl}/sample-flight.csv`);
if (!sampleResponse.ok || !(await sampleResponse.text()).startsWith("time_s,north_m")) {
  throw new Error("sample replay is unavailable");
}
const analysisResponse = await fetch(`${baseUrl}/analysis.html`);
if (!analysisResponse.ok || !(await analysisResponse.text()).includes("Birdman Flight Analysis")) {
  throw new Error("post-flight analysis is unavailable");
}

const socket = new WebSocket(baseUrl.replace(/^http/, "ws") + "/live");
let observations = 0;
let previousTime = -1;
const timeout = setTimeout(() => {
  socket.close();
  throw new Error("live smoke test timed out");
}, 10_000);

await new Promise<void>((resolve, reject) => {
  socket.on("open", () => {
    socket.send(JSON.stringify({ pilot_elevator: 0.3, pilot_rudder: -0.4, autonomy: 0 }));
  });
  socket.on("message", (data) => {
    try {
      const value = JSON.parse(data.toString()) as Record<string, unknown>;
      if ("type" in value) {
        throw new Error(`unexpected server message: ${JSON.stringify(value)}`);
      }
      const time = finite(value.time_s, "time_s");
      finite(value.roll_rad, "roll_rad");
      finite(value.yaw_rad, "yaw_rad");
      finite(value.mixed_elevator_command_rad, "mixed_elevator_command_rad");
      finite(value.mixed_rudder_command_rad, "mixed_rudder_command_rad");
      if (time < previousTime) throw new Error("live time moved backwards");
      previousTime = time;
      observations += 1;
      if (observations === 12) {
        socket.send(JSON.stringify({ pilot_elevator: -0.2, pilot_rudder: 0.25, autonomy: 0.5 }));
      } else if (observations === 24) {
        socket.send(JSON.stringify({ pilot_elevator: 1, pilot_rudder: 1, autonomy: 1 }));
      } else if (observations >= 36) {
        resolve();
        socket.close();
      }
    } catch (error) {
      reject(error);
      socket.close();
    }
  });
  socket.on("error", reject);
});

clearTimeout(timeout);
process.stdout.write(`Validated HTTP replay, analysis page, and ${observations} live observations\n`);

function finite(value: unknown, name: string): number {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(`${name} is not finite`);
  }
  return value;
}
