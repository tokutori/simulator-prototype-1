import {
  AmbientLight,
  BoxGeometry,
  BufferGeometry,
  Color,
  DirectionalLight,
  Fog,
  Group,
  HemisphereLight,
  Line,
  LineBasicMaterial,
  Mesh,
  MeshStandardMaterial,
  PCFSoftShadowMap,
  PerspectiveCamera,
  Scene,
  Vector3,
  WebGLRenderer,
} from "three";

import "./style.css";
import { createAircraft, setControlSurfaces } from "./aircraft.ts";
import { nedEulerToThreeQuaternion, nedPositionToThree } from "./coordinates.ts";
import { createDistanceRings } from "./distance-rings.ts";
import {
  PilotInput,
  defaultInputSettings,
  sanitizeSettings,
  type AxisBinding,
  type InputSettings,
} from "./input.ts";
import { frameFromLive, interpolateFrame, parseFlightCsv } from "./replay.ts";
import { createAnimatedWater, type AnimatedWater } from "./water.ts";
import type {
  AppMode,
  CameraMode,
  FlightFrame,
  InteractiveObservation,
  PilotCommandMessage,
} from "./types.ts";

const settingsKey = "birdman-visualizer-input-v1";
const radiansToDegrees = 180 / Math.PI;
const canvas = element<HTMLCanvasElement>("flight-view");
const renderer = new WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = PCFSoftShadowMap;

const scene = new Scene();
scene.background = new Color(0x86b7cd);
scene.fog = new Fog(0x86b7cd, 180, 950);
const camera = new PerspectiveCamera(55, 1, 0.08, 1800);
const aircraft = createAircraft();
scene.add(aircraft.root);
const environment = buildEnvironment(scene);

let mode: AppMode = "replay";
let cameraMode: CameraMode = "chase";
let replayFrames: FlightFrame[] = [];
let liveFrames: FlightFrame[] = [];
let liveFrame: FlightFrame | undefined;
let playbackTimeS = 0;
let replayPlaying = true;
let playbackSpeed = 1;
let previousAnimationMs = performance.now();
let lastCommandSentMs = 0;
let socket: WebSocket | undefined;
let chaseInitialized = false;
let trajectory: Line | undefined;
let settings = loadSettings();
const pilotInput = new PilotInput(settings);
let captureBinding: { axis: "elevator" | "rudder"; polarity: "negative" | "positive" } | undefined;

bindControls();
syncSettingsForm();
setCameraMode("chase");
await loadDefaultReplay();
renderer.setAnimationLoop(animate);
window.addEventListener("resize", resize);
resize();

function animate(nowMs: number): void {
  const deltaS = Math.min(0.05, Math.max(0, (nowMs - previousAnimationMs) / 1000));
  previousAnimationMs = nowMs;
  const gamepad = navigator.getGamepads?.()[settings.gamepadIndex] ?? null;
  element("gamepad-status").textContent = gamepad
    ? `Gamepad ${gamepad.index}: ${gamepad.id}`
    : "No gamepad detected";

  if (mode === "live") {
    pilotInput.update(deltaS, gamepad);
    if (socket?.readyState === WebSocket.OPEN && nowMs - lastCommandSentMs >= 40) {
      const command: PilotCommandMessage = {
        pilot_elevator: pilotInput.elevator,
        pilot_rudder: pilotInput.rudder,
        autonomy: Number(element<HTMLInputElement>("autonomy").value) / 100,
      };
      socket.send(JSON.stringify(command));
      lastCommandSentMs = nowMs;
    }
  }

  let frame: FlightFrame | undefined;
  if (mode === "replay" && replayFrames.length > 0) {
    const last = replayFrames.at(-1);
    if (replayPlaying && last) {
      playbackTimeS = Math.min(last.timeS, playbackTimeS + deltaS * playbackSpeed);
      if (playbackTimeS >= last.timeS) {
        replayPlaying = false;
        updatePlayButton();
      }
    }
    frame = interpolateFrame(replayFrames, playbackTimeS);
    updateTimeline();
  } else if (mode === "live") {
    frame = liveFrame;
  }

  if (frame) {
    displayFrame(frame, deltaS);
  }
  environment.update(nowMs / 1000, aircraft.root.position);
  renderer.render(scene, camera);
}

function displayFrame(frame: FlightFrame, deltaS: number): void {
  const position = nedPositionToThree(frame.northM, frame.eastM, frame.altitudeM);
  const attitude = nedEulerToThreeQuaternion(frame.rollRad, frame.pitchRad, frame.yawRad);
  aircraft.root.position.copy(position);
  aircraft.root.quaternion.copy(attitude);
  setControlSurfaces(aircraft, frame.elevatorRad, frame.rudderRad);

  if (cameraMode === "cockpit") {
    const eye = new Vector3(0, 0.11, -1.18).applyQuaternion(attitude).add(position);
    camera.position.copy(eye);
    camera.quaternion.copy(attitude);
    camera.fov = 72;
    camera.updateProjectionMatrix();
  } else {
    const forward = new Vector3(0, 0, -1).applyQuaternion(attitude);
    const horizontalForward = new Vector3(forward.x, 0, forward.z);
    if (horizontalForward.lengthSq() < 1e-8) {
      horizontalForward.set(0, 0, -1);
    } else {
      horizontalForward.normalize();
    }
    const desired = position.clone().addScaledVector(horizontalForward, -19).add(new Vector3(0, 6, 0));
    if (!chaseInitialized) {
      camera.position.copy(desired);
      chaseInitialized = true;
    } else {
      const smoothing = 1 - Math.exp(-Math.max(deltaS, 1 / 120) * 4.5);
      camera.position.lerp(desired, smoothing);
    }
    camera.lookAt(position.clone().addScaledVector(horizontalForward, 4));
    camera.fov = 55;
    camera.updateProjectionMatrix();
  }
  updateHud(frame);
}

interface FlightEnvironment {
  update(timeS: number, focus: Vector3): void;
}

function buildEnvironment(target: Scene): FlightEnvironment {
  target.add(new HemisphereLight(0xd9f3ff, 0x355d59, 2.2));
  target.add(new AmbientLight(0xffffff, 0.25));
  const sun = new DirectionalLight(0xfff3d7, 2.4);
  const sunOffset = new Vector3(-80, 130, 40);
  sun.position.copy(sunOffset);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -42;
  sun.shadow.camera.right = 42;
  sun.shadow.camera.top = 42;
  sun.shadow.camera.bottom = -42;
  sun.shadow.camera.near = 55;
  sun.shadow.camera.far = 230;
  sun.shadow.bias = -0.00012;
  sun.shadow.normalBias = 0.035;
  target.add(sun);
  target.add(sun.target);

  const water: AnimatedWater = createAnimatedWater();
  target.add(water.mesh);
  target.add(createDistanceRings());

  const platform = new Group();
  const concrete = new MeshStandardMaterial({ color: 0xd5d2c7, roughness: 0.9 });
  const deck = new Mesh(new BoxGeometry(10, 10.2, 15), concrete);
  deck.position.set(0, 5.1, 7.5);
  deck.castShadow = true;
  deck.receiveShadow = true;
  platform.add(deck);
  const edge = new Mesh(
    new BoxGeometry(10.2, 0.18, 0.35),
    new MeshStandardMaterial({ color: 0xe6a44c, roughness: 0.8 }),
  );
  edge.position.set(0, 10.27, 0.12);
  platform.add(edge);
  target.add(platform);

  const buoyMaterial = new MeshStandardMaterial({ color: 0xf06e3e, roughness: 0.7 });
  for (let north = 50; north <= 900; north += 50) {
    const buoy = new Mesh(new BoxGeometry(0.4, 0.55, 0.4), buoyMaterial);
    buoy.position.set(north % 100 === 0 ? 18 : -18, 0.28, -north);
    target.add(buoy);
  }

  return {
    update(timeS: number, focus: Vector3): void {
      water.update(timeS);
      // The directional-light shadow camera is finite even though it models
      // sunlight. Tracking the aircraft prevents a hard cutoff after launch.
      sun.target.position.set(focus.x, 0, focus.z);
      sun.position.copy(sun.target.position).add(sunOffset);
    },
  };
}

function bindControls(): void {
  element<HTMLButtonElement>("cockpit-camera").addEventListener("click", () => setCameraMode("cockpit"));
  element<HTMLButtonElement>("chase-camera").addEventListener("click", () => setCameraMode("chase"));
  element<HTMLButtonElement>("replay-mode").addEventListener("click", () => setMode("replay"));
  element<HTMLButtonElement>("live-mode").addEventListener("click", () => setMode("live"));
  element<HTMLButtonElement>("restart-live").addEventListener("click", connectLive);
  element<HTMLButtonElement>("play-pause").addEventListener("click", () => {
    replayPlaying = !replayPlaying;
    if (replayPlaying && playbackTimeS >= (replayFrames.at(-1)?.timeS ?? 0)) {
      playbackTimeS = replayFrames[0]?.timeS ?? 0;
    }
    updatePlayButton();
  });
  element<HTMLInputElement>("timeline").addEventListener("input", (event) => {
    const target = event.currentTarget as HTMLInputElement;
    const first = replayFrames[0];
    const last = replayFrames.at(-1);
    if (first && last) {
      playbackTimeS = first.timeS + Number(target.value) * (last.timeS - first.timeS);
    }
  });
  element<HTMLSelectElement>("playback-speed").addEventListener("change", (event) => {
    playbackSpeed = Number((event.currentTarget as HTMLSelectElement).value);
  });
  element<HTMLInputElement>("csv-file").addEventListener("change", async (event) => {
    const file = (event.currentTarget as HTMLInputElement).files?.[0];
    if (!file) return;
    try {
      loadReplay(parseFlightCsv(await file.text()), file.name);
    } catch (error) {
      showMessage(String(error));
    }
  });
  element<HTMLInputElement>("autonomy").addEventListener("input", updateAutonomyLabel);
  element<HTMLButtonElement>("download-live").addEventListener("click", downloadLiveLog);
  element<HTMLButtonElement>("reset-input").addEventListener("click", () => {
    settings = structuredClone(defaultInputSettings);
    pilotInput.settings = settings;
    persistSettings();
    syncSettingsForm();
  });

  bindSettingsInputs();
  bindKeyCapture("elevator-negative-key", "elevator", "negative");
  bindKeyCapture("elevator-positive-key", "elevator", "positive");
  bindKeyCapture("rudder-negative-key", "rudder", "negative");
  bindKeyCapture("rudder-positive-key", "rudder", "positive");

  window.addEventListener("keydown", (event) => {
    if (captureBinding) {
      event.preventDefault();
      const binding = settings[captureBinding.axis];
      if (captureBinding.polarity === "negative") binding.negativeKey = event.code;
      else binding.positiveKey = event.code;
      captureBinding = undefined;
      persistSettings();
      syncSettingsForm();
      return;
    }
    if (!isFormTarget(event.target)) {
      if (!isControlCode(event.code) && event.code === "KeyV" && !event.repeat) {
        setCameraMode(cameraMode === "cockpit" ? "chase" : "cockpit");
      } else if (!isControlCode(event.code) && event.code === "KeyH" && !event.repeat) {
        element("hud").toggleAttribute("hidden");
      } else if (
        !isControlCode(event.code) &&
        event.code === "Space" &&
        mode === "replay" &&
        !event.repeat
      ) {
        event.preventDefault();
        element<HTMLButtonElement>("play-pause").click();
      }
      pilotInput.pressedCodes.add(event.code);
    }
  });
  window.addEventListener("keyup", (event) => pilotInput.pressedCodes.delete(event.code));
  window.addEventListener("blur", () => pilotInput.clear());
  window.addEventListener("gamepaddisconnected", () => pilotInput.clear());
}

function bindSettingsInputs(): void {
  const ids = [
    "elevator-source", "elevator-axis", "elevator-buttons", "elevator-invert",
    "rudder-source", "rudder-axis", "rudder-buttons", "rudder-invert",
    "gamepad-index", "dead-zone", "response-exponent", "button-rise", "button-return",
  ];
  for (const id of ids) {
    element<HTMLInputElement | HTMLSelectElement>(id).addEventListener("change", readSettingsForm);
  }
}

function bindKeyCapture(
  id: string,
  axis: "elevator" | "rudder",
  polarity: "negative" | "positive",
): void {
  element<HTMLButtonElement>(id).addEventListener("click", (event) => {
    document.querySelectorAll(".key-binding").forEach((node) => node.classList.remove("listening"));
    captureBinding = { axis, polarity };
    (event.currentTarget as HTMLButtonElement).classList.add("listening");
    (event.currentTarget as HTMLButtonElement).textContent = "Press a key…";
  });
}

async function loadDefaultReplay(): Promise<void> {
  try {
    const response = await fetch("/sample-flight.csv");
    if (!response.ok) throw new Error(`sample flight HTTP ${response.status}`);
    loadReplay(parseFlightCsv(await response.text()), "sample-flight.csv");
  } catch (error) {
    showMessage(`Default replay could not be loaded: ${String(error)}`);
  }
}

function loadReplay(frames: FlightFrame[], name: string): void {
  replayFrames = frames;
  playbackTimeS = frames[0]?.timeS ?? 0;
  replayPlaying = true;
  updatePlayButton();
  buildTrajectory(frames);
  element("connection-status").textContent = name;
  setMode("replay");
  hideMessage();
}

function buildTrajectory(frames: readonly FlightFrame[]): void {
  if (trajectory) {
    scene.remove(trajectory);
    trajectory.geometry.dispose();
  }
  const points = frames
    .filter((_, index) => index % 4 === 0)
    .map((frame) => nedPositionToThree(frame.northM, frame.eastM, frame.altitudeM));
  trajectory = new Line(
    new BufferGeometry().setFromPoints(points),
    new LineBasicMaterial({ color: 0xf2c879, transparent: true, opacity: 0.58 }),
  );
  scene.add(trajectory);
}

function setMode(next: AppMode): void {
  mode = next;
  element<HTMLButtonElement>("replay-mode").classList.toggle("active", next === "replay");
  element<HTMLButtonElement>("live-mode").classList.toggle("active", next === "live");
  element("replay-controls").toggleAttribute("hidden", next !== "replay");
  element("live-controls").toggleAttribute("hidden", next !== "live");
  element("restart-live").toggleAttribute("hidden", next !== "live");
  element("mode-badge").textContent = next.toUpperCase();
  if (trajectory) trajectory.visible = next === "replay" && cameraMode === "chase";
  if (next === "live") {
    connectLive();
  } else {
    disconnectLive();
    element("connection-status").textContent = "sample / loaded replay";
  }
}

function connectLive(): void {
  disconnectLive();
  liveFrame = undefined;
  liveFrames = [];
  pilotInput.clear();
  element<HTMLButtonElement>("download-live").disabled = true;
  element("connection-status").textContent = "connecting…";
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/live`);
  socket.addEventListener("open", () => {
    element("connection-status").textContent = "live Rust FDM";
    hideMessage();
  });
  socket.addEventListener("message", (event) => {
    try {
      const message = JSON.parse(String(event.data)) as InteractiveObservation | { type: string; message: string };
      if ("type" in message) {
        element("connection-status").textContent = message.type === "ended" ? "surface contact" : "bridge error";
        if (message.type === "error") showMessage(message.message);
        return;
      }
      liveFrame = frameFromLive(message);
      liveFrames.push(liveFrame);
      element<HTMLButtonElement>("download-live").disabled = liveFrames.length < 2;
    } catch (error) {
      showMessage(`Live telemetry error: ${String(error)}`);
    }
  });
  socket.addEventListener("close", () => {
    pilotInput.clear();
    if (mode === "live" && element("connection-status").textContent === "live Rust FDM") {
      element("connection-status").textContent = "disconnected";
    }
  });
  socket.addEventListener("error", () => showMessage("Interactive server is unavailable. Run npm run dev."));
}

function disconnectLive(): void {
  if (socket) {
    socket.close();
    socket = undefined;
  }
  pilotInput.clear();
}

function setCameraMode(next: CameraMode): void {
  cameraMode = next;
  chaseInitialized = false;
  element<HTMLButtonElement>("cockpit-camera").setAttribute("aria-pressed", String(next === "cockpit"));
  element<HTMLButtonElement>("chase-camera").setAttribute("aria-pressed", String(next === "chase"));
  if (trajectory) trajectory.visible = mode === "replay" && next === "chase";
}

function updateHud(frame: FlightFrame): void {
  value("airspeed-value", frame.airspeedMps, 2);
  value("altitude-value", frame.altitudeM, 2);
  value("gamma-value", frame.flightPathRad * radiansToDegrees, 1);
  value("alpha-value", frame.alphaRad * radiansToDegrees, 1);
  value("roll-value", frame.rollRad * radiansToDegrees, 1);
  value("time-value", frame.timeS, 2);
  element("elevator-value").textContent = `${(frame.elevatorRad * radiansToDegrees).toFixed(1)} deg`;
  element("rudder-value").textContent = `${(frame.rudderRad * radiansToDegrees).toFixed(1)} deg`;
  element<HTMLMeterElement>("pilot-elevator-meter").value = mode === "live" ? pilotInput.elevator : frame.pilotElevator;
  element<HTMLMeterElement>("pilot-rudder-meter").value = mode === "live" ? pilotInput.rudder : frame.pilotRudder;
}

function updateTimeline(): void {
  const first = replayFrames[0];
  const last = replayFrames.at(-1);
  if (!first || !last || last.timeS === first.timeS) return;
  element<HTMLInputElement>("timeline").value = String(
    (playbackTimeS - first.timeS) / (last.timeS - first.timeS),
  );
}

function updatePlayButton(): void {
  const button = element<HTMLButtonElement>("play-pause");
  button.textContent = replayPlaying ? "Pause" : "Play";
  button.setAttribute("aria-label", replayPlaying ? "Pause replay" : "Play replay");
}

function updateAutonomyLabel(): void {
  const percent = Number(element<HTMLInputElement>("autonomy").value);
  element("autonomy-value").textContent = `${percent}%`;
  element("control-mode-label").textContent =
    percent === 0 ? "FULL MANUAL" : percent === 100 ? "FULL AUTO" : `SHARED ${percent}%`;
}

function loadSettings(): InputSettings {
  try {
    return sanitizeSettings(JSON.parse(localStorage.getItem(settingsKey) ?? "null"));
  } catch {
    return structuredClone(defaultInputSettings);
  }
}

function persistSettings(): void {
  localStorage.setItem(settingsKey, JSON.stringify(settings));
  pilotInput.settings = settings;
}

function syncSettingsForm(): void {
  syncAxisForm("elevator", settings.elevator);
  syncAxisForm("rudder", settings.rudder);
  element<HTMLInputElement>("gamepad-index").value = String(settings.gamepadIndex);
  element<HTMLInputElement>("dead-zone").value = String(settings.deadZone);
  element<HTMLInputElement>("response-exponent").value = String(settings.responseExponent);
  element<HTMLInputElement>("button-rise").value = String(settings.buttonRisePerSecond);
  element<HTMLInputElement>("button-return").value = String(settings.buttonReturnPerSecond);
  document.querySelectorAll(".key-binding").forEach((node) => node.classList.remove("listening"));
}

function syncAxisForm(axis: "elevator" | "rudder", binding: AxisBinding): void {
  element<HTMLSelectElement>(`${axis}-source`).value = binding.source;
  element<HTMLButtonElement>(`${axis}-negative-key`).textContent = binding.negativeKey;
  element<HTMLButtonElement>(`${axis}-positive-key`).textContent = binding.positiveKey;
  element<HTMLInputElement>(`${axis}-axis`).value = String(binding.gamepadAxis);
  element<HTMLInputElement>(`${axis}-buttons`).value = `${binding.negativeButton}, ${binding.positiveButton}`;
  element<HTMLInputElement>(`${axis}-invert`).checked = binding.invert;
}

function readSettingsForm(): void {
  const candidate = {
    elevator: readAxisForm("elevator", settings.elevator),
    rudder: readAxisForm("rudder", settings.rudder),
    gamepadIndex: Number(element<HTMLInputElement>("gamepad-index").value),
    deadZone: Number(element<HTMLInputElement>("dead-zone").value),
    responseExponent: Number(element<HTMLInputElement>("response-exponent").value),
    buttonRisePerSecond: Number(element<HTMLInputElement>("button-rise").value),
    buttonReturnPerSecond: Number(element<HTMLInputElement>("button-return").value),
  };
  settings = sanitizeSettings(candidate);
  persistSettings();
  syncSettingsForm();
}

function readAxisForm(axis: "elevator" | "rudder", previous: AxisBinding): AxisBinding {
  const buttons = element<HTMLInputElement>(`${axis}-buttons`).value
    .split(/[,/ ]+/)
    .filter(Boolean)
    .map(Number);
  return {
    source: element<HTMLSelectElement>(`${axis}-source`).value as AxisBinding["source"],
    negativeKey: previous.negativeKey,
    positiveKey: previous.positiveKey,
    gamepadAxis: Number(element<HTMLInputElement>(`${axis}-axis`).value),
    negativeButton: buttons[0] ?? previous.negativeButton,
    positiveButton: buttons[1] ?? previous.positiveButton,
    invert: element<HTMLInputElement>(`${axis}-invert`).checked,
  };
}

function downloadLiveLog(): void {
  if (liveFrames.length < 2) return;
  const header = [
    "time_s", "north_m", "east_m", "altitude_m", "roll_deg", "pitch_deg", "yaw_deg",
    "flight_path_deg", "airspeed_mps", "alpha_deg", "pilot_elevator", "pilot_rudder", "autonomy",
    "manual_elevator_command_deg", "automatic_elevator_command_deg", "mixed_elevator_command_deg",
    "elevator_deg", "manual_rudder_command_deg", "automatic_rudder_command_deg",
    "mixed_rudder_command_deg", "rudder_deg", "surface_contact",
  ];
  const rows = liveFrames.map((frame) => [
    frame.timeS, frame.northM, frame.eastM, frame.altitudeM,
    frame.rollRad * radiansToDegrees, frame.pitchRad * radiansToDegrees,
    frame.yawRad * radiansToDegrees, frame.flightPathRad * radiansToDegrees,
    frame.airspeedMps, frame.alphaRad * radiansToDegrees, frame.pilotElevator, frame.pilotRudder,
    frame.autonomy, frame.manualElevatorCommandRad * radiansToDegrees,
    frame.automaticElevatorCommandRad * radiansToDegrees,
    frame.mixedElevatorCommandRad * radiansToDegrees, frame.elevatorRad * radiansToDegrees,
    frame.manualRudderCommandRad * radiansToDegrees,
    frame.automaticRudderCommandRad * radiansToDegrees,
    frame.mixedRudderCommandRad * radiansToDegrees, frame.rudderRad * radiansToDegrees,
    frame.surfaceContact ? 1 : 0,
  ].join(","));
  const blob = new Blob([[header.join(","), ...rows].join("\n") + "\n"], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `birdman-live-${new Date().toISOString().replaceAll(":", "-")}.csv`;
  anchor.click();
  URL.revokeObjectURL(url);
}

function resize(): void {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(1, height);
  camera.updateProjectionMatrix();
}

function showMessage(text: string): void {
  const message = element("message");
  message.textContent = text;
  message.hidden = false;
}

function hideMessage(): void {
  element("message").hidden = true;
}

function value(id: string, numeric: number, digits: number): void {
  element(id).textContent = Number.isFinite(numeric) ? numeric.toFixed(digits) : "--";
}

function isFormTarget(target: EventTarget | null): boolean {
  return target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLButtonElement;
}

function isControlCode(code: string): boolean {
  return [
    settings.elevator.negativeKey,
    settings.elevator.positiveKey,
    settings.rudder.negativeKey,
    settings.rudder.positiveKey,
  ].includes(code);
}

function element<T extends HTMLElement = HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (!found) throw new Error(`missing element #${id}`);
  return found as T;
}
