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
  PCFShadowMap,
  PerspectiveCamera,
  Scene,
  Vector3,
  WebGLRenderer,
} from "three";
import { VRButton } from "three/addons/webxr/VRButton.js";

import "./style.css";
import { prepareAnalysisDataset } from "./analysis-data.ts";
import { storeAnalysis } from "./analysis-storage.ts";
import {
  acceptsSessionEvent,
  isInteractive,
  present,
  update as updateApp,
  type AppMsg,
  type AppState,
  type Effect,
} from "./app-state.ts";
import { createAircraft, setControlSurfaces } from "./aircraft.ts";
import { ChaseCamera } from "./chase-camera.ts";
import { nedEulerToThreeQuaternion, nedPositionToThree } from "./coordinates.ts";
import { createDistanceRings } from "./distance-rings.ts";
import { classifyFlightPhase, formatFlightTime, type FlightPhase } from "./flight-phase.ts";
import {
  PilotInput,
  defaultInputSettings,
  sanitizeSettings,
  type AxisBinding,
  type InputSettings,
} from "./input.ts";
import { LivePlayback } from "./live-playback.ts";
import { ReplaySelection } from "./replay-selection.ts";
import { frameFromLive, interpolateFrame, parseFlightCsv, parseCsvOutcome, parseCsvIncidents, experimentColumns, experimentCsvValues } from "./replay.ts";
import { createAnimatedWater, type AnimatedWater } from "./water.ts";
import type {
  CameraMode,
  FlightFrame,
  InteractiveObservation,
  PilotCommandMessage,
  SessionOutcome,
  SessionIncident,
} from "./types.ts";
import { applyUiScale } from "./ui-scale.ts";
import { resetXrRig, setXrCockpitRig } from "./xr-camera.ts";
import {
  presentXr,
  updateXr,
  type XrMsg,
  type XrState,
} from "./xr-state.ts";

const settingsKey = "birdman-visualizer-input-v1";
const radiansToDegrees = 180 / Math.PI;
const canvas = element<HTMLCanvasElement>("flight-view");
applyUiScale(window.innerWidth, window.innerHeight);
const renderer = new WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = PCFShadowMap;
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType("local");

const scene = new Scene();
scene.background = new Color(0x86b7cd);
scene.fog = new Fog(0x86b7cd, 180, 950);
const camera = new PerspectiveCamera(55, 1, 0.08, 1800);
const xrRig = new Group();
scene.add(xrRig);
xrRig.add(camera);
const aircraft = createAircraft();
scene.add(aircraft.root);
const environment = buildEnvironment(scene);

let appState: AppState = { tag: "mcu-connecting", sessionId: 0 };
let xrState: XrState = { tag: "checking" };
let cameraMode: CameraMode = "chase";
let cameraModeBeforeXr: CameraMode = "chase";
let replayFrames: FlightFrame[] = [];
let replayName = "sample-flight.csv";
const replaySelection = new ReplaySelection();
let replayOutcome: SessionOutcome = { tag: "unknown", reason: "No session completion evidence" };
let liveOutcome: SessionOutcome = { tag: "active" };
let liveIncidents: SessionIncident[] = [];
let replayIncidents: SessionIncident[] = [];
let lastTelemetryReceiptMs = performance.now();
let lastWatchdogCheckMs = 0;
let archiveSequence = 0;
const previousAnalysis = document.createElement("a");
previousAnalysis.id = "previous-analysis";
previousAnalysis.textContent = "Previous stopped flight";
previousAnalysis.target = "_blank";
previousAnalysis.rel = "noopener";
previousAnalysis.hidden = true;
element("open-analysis").after(previousAnalysis);
try {
  const id = localStorage.getItem("birdman-last-stopped-flight");
  if (id) { previousAnalysis.href = `/analysis.html?flight=${encodeURIComponent(id)}`; previousAnalysis.hidden = false; }
} catch { /* Archiving itself uses IndexedDB; blocked optional last-link persistence is harmless. */ }
let liveFrames: FlightFrame[] = [];
let liveFrame: FlightFrame | undefined;
let livePlayback = new LivePlayback();
let playbackTimeS = 0;
let replayPlaying = true;
let playbackSpeed = 1;
let previousAnimationMs = performance.now();
let lastCommandSentMs = 0;
let socket: WebSocket | undefined;
const chaseCamera = new ChaseCamera();
let trajectory: Line | undefined;
let settings = loadSettings();
const pilotInput = new PilotInput(settings);
let captureBinding: { axis: "elevator" | "rudder"; polarity: "negative" | "positive" } | undefined;

bindControls();
initializeWebXr();
syncSettingsForm();
setCameraMode("chase");
renderAppState();
runEffect({ type: "connect-mcu", sessionId: appState.sessionId });
void loadDefaultReplay(false);
renderer.setAnimationLoop(animate);
window.addEventListener("resize", resize);
document.addEventListener("visibilitychange", () => {
  if (document.visibilityState !== "visible") return;
  // A hidden tab is a presentation discontinuity, not a backlog to replay while flying.
  previousAnimationMs = performance.now();
  chaseCamera.reset();
  livePlayback = new LivePlayback();
  if (liveFrame) livePlayback.push(liveFrame, previousAnimationMs);
});
resize();

function animate(nowMs: number): void {
  const deltaS = Math.min(0.05, Math.max(0, (nowMs - previousAnimationMs) / 1000));
  previousAnimationMs = nowMs;
  if (nowMs - lastWatchdogCheckMs >= 250) {
    lastWatchdogCheckMs = nowMs;
    if ((appState.tag === "mcu-running" || appState.tag === "mcu-too-slow" || appState.tag === "mcu-stalled")
      && nowMs - lastTelemetryReceiptMs >= 500) {
      if (appState.tag !== "mcu-stalled") liveIncidents.push({ kind: "telemetry-stall", wallTimeIso: new Date().toISOString(),
        sinceLastReceiptMs: nowMs - lastTelemetryReceiptMs });
      dispatch({ type: "mcu-stale", sessionId: appState.sessionId, ageMs: nowMs - lastTelemetryReceiptMs });
    }
  }
  const gamepad = navigator.getGamepads?.()[settings.gamepadIndex] ?? null;
  element("gamepad-status").textContent = gamepad
    ? `Gamepad ${gamepad.index}: ${gamepad.id}`
    : "No gamepad detected";

  if (isInteractive(appState)) {
    if (document.hasFocus() && document.visibilityState === "visible") pilotInput.update(deltaS, gamepad);
    else pilotInput.clear();
    if (socket?.readyState === WebSocket.OPEN && nowMs - lastCommandSentMs >= 40) {
      const command: PilotCommandMessage = {
        pilot_elevator: pilotInput.elevator,
        pilot_rudder: pilotInput.rudder,
        autonomy: Number(element<HTMLInputElement>("autonomy").value) / 100,
        elevator_input_kind: settings.elevator.source === "gamepad-axis" ? "analog" : "buttons",
        rudder_input_kind: settings.rudder.source === "gamepad-axis" ? "analog" : "buttons",
      };
      socket.send(JSON.stringify(command));
      lastCommandSentMs = nowMs;
    }
  }

  let frame: FlightFrame | undefined;
  if (!isInteractive(appState) && replayFrames.length > 0) {
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
  } else if (isInteractive(appState)) {
    frame = livePlayback.frame(nowMs) ?? liveFrame;
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

  if (renderer.xr.isPresenting) {
    setXrCockpitRig(xrRig, position, attitude);
  } else if (cameraMode === "cockpit") {
    resetXrRig(xrRig);
    const eye = new Vector3(0, 0.11, -1.18).applyQuaternion(attitude).add(position);
    camera.position.copy(eye);
    camera.quaternion.copy(attitude);
    camera.fov = 72;
    camera.updateProjectionMatrix();
  } else {
    resetXrRig(xrRig);
    const pose = chaseCamera.update(position, attitude, deltaS);
    camera.position.copy(pose.eye);
    camera.lookAt(pose.target);
    camera.fov = 55;
    camera.updateProjectionMatrix();
  }
  updateHud(frame);
  updateFlightPhase(frame);
}

function initializeWebXr(): void {
  const vrButton = VRButton.createButton(renderer);
  vrButton.classList.add("xr-button");
  element("xr-controls").append(vrButton);
  renderXrState();

  renderer.xr.addEventListener("sessionstart", () => {
    cameraModeBeforeXr = cameraMode;
    setCameraMode("cockpit");
    dispatchXr({ type: "session-started" });
  });
  renderer.xr.addEventListener("sessionend", () => {
    resetXrRig(xrRig);
    dispatchXr({ type: "session-ended" });
    setCameraMode(cameraModeBeforeXr);
  });

  const xr = navigator.xr;
  if (xr === undefined) {
    dispatchXr({
      type: "availability",
      supported: false,
      reason: window.isSecureContext ? "api-unavailable" : "insecure-context",
    });
    return;
  }
  void xr.isSessionSupported("immersive-vr").then((supported) => {
    dispatchXr(supported
      ? { type: "availability", supported: true }
      : { type: "availability", supported: false, reason: "immersive-vr-unsupported" });
  }).catch(() => dispatchXr({
    type: "availability",
    supported: false,
    reason: "permission-denied",
  }));
}

function dispatchXr(message: XrMsg): void {
  xrState = updateXr(xrState, message);
  renderXrState();
}

function renderXrState(): void {
  const presentation = presentXr(xrState);
  const status = element("xr-status");
  status.textContent = presentation.label;
  status.title = presentation.detail;
  const vrButton = document.querySelector<HTMLElement>("#xr-controls > #VRButton");
  vrButton?.classList.toggle("active", presentation.presenting);
  vrButton?.setAttribute("aria-pressed", String(presentation.presenting));
  element<HTMLButtonElement>("cockpit-camera").disabled = presentation.presenting;
  element<HTMLButtonElement>("chase-camera").disabled = presentation.presenting;
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
  element<HTMLButtonElement>("restart-live").addEventListener("click", () => dispatch({ type: "request-mcu" }));
  element<HTMLButtonElement>("open-analysis").addEventListener("click", openCurrentAnalysis);
  element<HTMLButtonElement>("flight-event-analysis").addEventListener("click", openCurrentAnalysis);
  element<HTMLButtonElement>("play-pause").addEventListener("click", () => {
    replayPlaying = !replayPlaying;
    if (replayPlaying && playbackTimeS >= (replayFrames.at(-1)?.timeS ?? 0)) {
      playbackTimeS = replayFrames[0]?.timeS ?? 0;
      chaseCamera.reset();
    }
    updatePlayButton();
  });
  element<HTMLInputElement>("timeline").addEventListener("input", (event) => {
    const target = event.currentTarget as HTMLInputElement;
    const first = replayFrames[0];
    const last = replayFrames.at(-1);
    if (first && last) {
      playbackTimeS = first.timeS + Number(target.value) * (last.timeS - first.timeS);
      chaseCamera.reset();
    }
  });
  element<HTMLSelectElement>("playback-speed").addEventListener("change", (event) => {
    playbackSpeed = Number((event.currentTarget as HTMLSelectElement).value);
  });
  element<HTMLInputElement>("csv-file").addEventListener("change", async (event) => {
    const file = (event.currentTarget as HTMLInputElement).files?.[0];
    if (!file) return;
    const selection = replaySelection.select();
    try {
      const text = await file.text();
      if (!replaySelection.accepts(selection)) return;
      loadReplay(parseFlightCsv(text), file.name, true, parseCsvOutcome(text), parseCsvIncidents(text));
    } catch (error) {
      if (replaySelection.accepts(selection)) showMessage(String(error));
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
      if (isControlCode(event.code)) event.preventDefault();
      if (!isControlCode(event.code) && event.code === "KeyV" && !event.repeat) {
        setCameraMode(cameraMode === "cockpit" ? "chase" : "cockpit");
      } else if (!isControlCode(event.code) && event.code === "KeyH" && !event.repeat) {
        element("hud").toggleAttribute("hidden");
      } else if (
        !isControlCode(event.code) &&
        event.code === "Space" &&
        !isInteractive(appState) &&
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
    "gamepad-index", "dead-zone", "response-exponent",
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

async function loadDefaultReplay(selectMode: boolean): Promise<void> {
  const selection = replaySelection.current();
  try {
    const response = await fetch("/sample-flight.csv");
    if (!response.ok) throw new Error(`sample flight HTTP ${response.status}`);
    const text = await response.text();
    if (!replaySelection.accepts(selection)) return;
    loadReplay(parseFlightCsv(text), "sample-flight.csv", selectMode, parseCsvOutcome(text), parseCsvIncidents(text));
  } catch (error) {
    if (replaySelection.accepts(selection)) showMessage(`Default replay could not be loaded: ${String(error)}`);
  }
}

function loadReplay(frames: FlightFrame[], name: string, selectMode = true,
  outcome: SessionOutcome = { tag: "unknown", reason: "No session completion evidence" }, incidents: SessionIncident[] = []): void {
  replayFrames = frames;
  replayName = name;
  replayOutcome = outcome;
  replayIncidents = incidents;
  playbackTimeS = frames[0]?.timeS ?? 0;
  replayPlaying = true;
  updatePlayButton();
  buildTrajectory(frames);
  if (selectMode) {
    chaseCamera.reset();
    element<HTMLButtonElement>("open-analysis").disabled = frames.length < 2;
    dispatch({ type: "select-replay", sourceName: name });
    hideMessage();
  }
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

function setMode(next: "replay" | "live"): void {
  chaseCamera.reset();
  if (next === "live") dispatch({ type: "request-mcu" });
  else {
    element<HTMLButtonElement>("open-analysis").disabled = replayFrames.length < 2;
    dispatch({ type: "select-replay", sourceName: replayName });
  }
}

function dispatch(message: AppMsg): void {
  const wasActive = acceptsSessionEvent(appState, appState.sessionId);
  if (wasActive && (message.type === "select-replay" || message.type === "request-mcu")) {
    liveOutcome = { tag: "aborted", reason: message.type === "request-mcu" ? "User restarted the flight" : "User switched to replay" };
    archiveStoppedSession();
  }
  const [next, effect] = updateApp(appState, message);
  appState = next;
  if (wasActive && (next.tag === "mcu-ended" || next.tag === "mcu-failed")) {
    liveOutcome = next.tag === "mcu-ended" ? { tag: "ended", reason: next.reason } : { tag: "failed", reason: next.error };
    archiveStoppedSession();
  }
  if (appState.tag === "mcu-ended" || appState.tag === "mcu-failed") livePlayback.finish();
  renderAppState();
  runEffect(effect);
}

function renderAppState(): void {
  const interactive = isInteractive(appState);
  const presentation = present(appState);
  element<HTMLButtonElement>("replay-mode").classList.toggle("active", !interactive);
  element<HTMLButtonElement>("live-mode").classList.toggle("active", interactive);
  element("replay-controls").toggleAttribute("hidden", interactive);
  element("live-controls").toggleAttribute("hidden", !interactive);
  element("restart-live").toggleAttribute("hidden", !interactive);
  element("mode-badge").textContent = presentation.mode;
  element("connection-status").textContent = presentation.status;
  element("mcu-performance").textContent = presentation.performance;
  const warning = element("realtime-warning");
  warning.textContent = presentation.warning ?? "";
  warning.hidden = presentation.warning === undefined;
  if (trajectory) trajectory.visible = !interactive && cameraMode === "chase";
}

function runEffect(effect: Effect): void {
  switch (effect.type) {
    case "none": break;
    case "connect-mcu": connectLive(effect.sessionId); break;
    case "disconnect-mcu": disconnectLive(); break;
  }
}

function connectLive(sessionId: number): void {
  disconnectLive();
  liveFrame = undefined;
  liveFrames = [];
  liveOutcome = { tag: "active" };
  liveIncidents = [];
  lastTelemetryReceiptMs = performance.now();
  livePlayback = new LivePlayback();
  chaseCamera.reset();
  pilotInput.clear();
  element<HTMLButtonElement>("download-live").disabled = true;
  element<HTMLButtonElement>("open-analysis").disabled = true;
  setFlightEvent("ACTUAL UF2", "MCU START", "Loading production RP2040 firmware in rp2040js");
  setPhaseBadge("ready");
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const connection = new WebSocket(`${protocol}//${location.host}/live`);
  socket = connection;
  const isCurrent = (): boolean => socket === connection && acceptsSessionEvent(appState, sessionId);
  connection.addEventListener("open", () => {
    if (!isCurrent()) return;
    hideMessage();
  });
  connection.addEventListener("message", (event) => {
    if (!isCurrent()) return;
    try {
      const message = JSON.parse(String(event.data)) as InteractiveObservation | { type: string; message: string };
      if ("type" in message) {
        dispatch(message.type === "ended"
          ? { type: "mcu-ended", sessionId, reason: message.message }
          : { type: "mcu-failed", sessionId, error: message.message });
        return;
      }
      if (message.backend !== "rp2040js-actual-uf2") {
        dispatch({ type: "mcu-failed", sessionId, error: "Rejected telemetry that did not come from actual RP2040 UF2" });
        return;
      }
      const nextFrame = frameFromLive(message);
      lastTelemetryReceiptMs = performance.now();
      dispatch({
        type: "mcu-telemetry",
        sessionId,
        performance: {
          timingAcceleration: message.emulation.timing_acceleration,
          processingMs: message.emulation.processing_ms,
          processingAverageMs: message.emulation.processing_average_ms,
          realTimeRatio: message.emulation.real_time_ratio,
          lagMs: message.emulation.lag_ms,
          deadlineMissed: message.emulation.deadline_missed,
          realTime: message.emulation.real_time,
          timingValidated: false,
        },
      });
      liveFrame = nextFrame;
      liveFrames.push(liveFrame);
      livePlayback.push(liveFrame, performance.now());
      element<HTMLButtonElement>("download-live").disabled = liveFrames.length < 2;
      element<HTMLButtonElement>("open-analysis").disabled = liveFrames.length < 2;
    } catch (error) {
      dispatch({ type: "mcu-failed", sessionId, error: `Live telemetry error: ${String(error)}` });
    }
  });
  connection.addEventListener("close", () => {
    if (!isCurrent()) return;
    pilotInput.clear();
    if (isInteractive(appState) && appState.tag !== "mcu-ended" && appState.tag !== "mcu-failed") {
      dispatch({ type: "mcu-failed", sessionId, error: "actual-UF2 bridge disconnected" });
    }
  });
  connection.addEventListener("error", () => {
    if (!isCurrent()) return;
    dispatch({ type: "mcu-failed", sessionId,
      error: "Interactive actual-UF2 server is unavailable. Run npm run dev." });
  });
}

function disconnectLive(): void {
  if (socket) {
    const previous = socket;
    socket = undefined;
    previous.close();
  }
  pilotInput.clear();
}

function setCameraMode(next: CameraMode): void {
  if (renderer.xr.isPresenting && next !== "cockpit") return;
  cameraMode = next;
  chaseCamera.reset();
  element<HTMLButtonElement>("cockpit-camera").setAttribute("aria-pressed", String(next === "cockpit"));
  element<HTMLButtonElement>("chase-camera").setAttribute("aria-pressed", String(next === "chase"));
  if (trajectory) trajectory.visible = !isInteractive(appState) && next === "chase";
}

function updateHud(frame: FlightFrame): void {
  value("airspeed-value", frame.airspeedMps, 1);
  value("altitude-value", frame.altitudeM, 1);
  value("gamma-value", frame.flightPathRad * radiansToDegrees, 1);
  value("alpha-value", frame.alphaRad * radiansToDegrees, 1);
  value("roll-value", frame.rollRad * radiansToDegrees, 1);
  const frames = isInteractive(appState) ? liveFrames : replayFrames;
  const elapsedS = frame.timeS - (frames[0]?.timeS ?? frame.timeS);
  element("time-value").textContent = formatFlightTime(elapsedS);
  const elevatorDeg = frame.elevatorRad * radiansToDegrees;
  const rudderDeg = frame.rudderRad * radiansToDegrees;
  element("elevator-value").textContent = `${elevatorDeg.toFixed(1)} deg`;
  element("rudder-value").textContent = `${rudderDeg.toFixed(1)} deg`;
  setSurfaceTrack("elevator-track", elevatorDeg, frame.mixedElevatorCommandRad * radiansToDegrees);
  setSurfaceTrack("rudder-track", rudderDeg, frame.mixedRudderCommandRad * radiansToDegrees);
  element("altitude-instrument").classList.toggle("caution", frame.altitudeM < 2 && !frame.surfaceContact);
  element<HTMLMeterElement>("pilot-elevator-meter").value = isInteractive(appState) ? pilotInput.elevator : frame.pilotElevator;
  element<HTMLMeterElement>("pilot-rudder-meter").value = isInteractive(appState) ? pilotInput.rudder : frame.pilotRudder;
}

function updateFlightPhase(frame: FlightFrame): void {
  const frames = isInteractive(appState) ? liveFrames : replayFrames;
  const first = frames[0] ?? frame;
  const last = frames.at(-1) ?? frame;
  const elapsedS = Math.max(0, frame.timeS - first.timeS);
  const replayAtEnd = !isInteractive(appState)
    && !replayPlaying
    && Math.abs(frame.timeS - last.timeS) < 1e-6;
  const phase = classifyFlightPhase({
    hasFrame: true,
    elapsedS,
    surfaceContact: frame.surfaceContact,
    replayAtEnd,
  });
  setPhaseBadge(phase);
  const rangeM = Math.hypot(frame.northM - first.northM, frame.eastM - first.eastM);
  if (phase === "launch") {
    setFlightEvent("T+ 00:00", "LAUNCH", `IAS ${frame.airspeedMps.toFixed(1)} m/s · flight clock started`);
  } else if (phase === "water-contact") {
    setFlightEvent(
      "FLIGHT COMPLETE",
      "WATER CONTACT",
      `T+ ${formatFlightTime(elapsedS)} · RANGE ${rangeM.toFixed(1)} m · IAS ${frame.airspeedMps.toFixed(1)} m/s`,
      "Review flight",
    );
  } else if (phase === "record-ended") {
    setFlightEvent(
      "DATA STATUS",
      "END OF RECORDING",
      `T+ ${formatFlightTime(elapsedS)} · ALT ${frame.altitudeM.toFixed(1)} m · no water contact in this record`,
      "Review record",
    );
  } else {
    element("flight-event").hidden = true;
  }
}

function setPhaseBadge(phase: FlightPhase): void {
  element("phase-badge").textContent = phase.replaceAll("-", " ").toUpperCase();
}

function setFlightEvent(kicker: string, title: string, detail: string, actionLabel?: string): void {
  element("flight-event-kicker").textContent = kicker;
  element("flight-event-title").textContent = title;
  element("flight-event-detail").textContent = detail;
  const event = element("flight-event");
  const action = element<HTMLButtonElement>("flight-event-analysis");
  action.textContent = actionLabel ?? "";
  action.hidden = actionLabel === undefined;
  event.classList.toggle("has-actions", actionLabel !== undefined);
  event.hidden = false;
}

async function openCurrentAnalysis(): Promise<void> {
  const frames = isInteractive(appState) ? liveFrames : replayFrames;
  if (frames.length < 2) return;
  const name = isInteractive(appState) ? "interactive actual-UF2 flight" : replayName;
  const tab = window.open("about:blank", "_blank");
  if (!tab) { showMessage("Allow pop-ups to open Flight analysis. The raw CSV can still be downloaded."); return; }
  tab.opener = null;
  tab.document.body.textContent = "Saving complete flight data for analysis…";
  try {
    const id = crypto.randomUUID();
    await storeAnalysis(id, prepareAnalysisDataset(name, frames, new Date(), isInteractive(appState) ? liveOutcome : replayOutcome,
      isInteractive(appState) ? liveIncidents : replayIncidents));
    tab.location.replace(`/analysis.html?flight=${encodeURIComponent(id)}`);
  } catch (error) {
    tab.close();
    showMessage(`Flight analysis could not be opened: ${String(error)}`);
  }
}

function archiveStoppedSession(): void {
  const sequence = ++archiveSequence;
  const id = crypto.randomUUID();
  const dataset = prepareAnalysisDataset("Stopped actual-UF2 flight", liveFrames, new Date(), liveOutcome, liveIncidents);
  void storeAnalysis(id, dataset).then(() => {
    if (sequence !== archiveSequence) return;
    previousAnalysis.href = `/analysis.html?flight=${encodeURIComponent(id)}`;
    previousAnalysis.hidden = false;
    try { localStorage.setItem("birdman-last-stopped-flight", id); } catch { /* Link remains available in this tab. */ }
  }).catch(error => showMessage(`Stopped flight could not be archived: ${String(error)}`));
}

function setSurfaceTrack(id: string, actualDeg: number, commandDeg: number): void {
  const position = (degrees: number): number => 50 + Math.max(-1, Math.min(1, degrees / 10)) * 46;
  const track = element(id);
  track.style.setProperty("--actual-position", `${position(actualDeg)}%`);
  track.style.setProperty("--command-position", `${position(commandDeg)}%`);
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
    "firmware_sequence", "firmware_time_us", "automatic_valid", "safe_elevator_command_deg",
    "observed_elevator_command_deg", "observed_rudder_command_deg",
    "elevator_pwm_sample_time_us", "rudder_pwm_sample_time_us",
    "safe_rudder_command_deg", "uf2_sha256", "model_sha256", "plant_sha256",
    "release_mcu_time_us", "plant_interval_start_s",
    ...experimentColumns,
    "virtual_platform_sha256", "scenario_sha256",
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
    ...(frame.controlTelemetry.tag === "firmware" ? [
      frame.controlTelemetry.sequence, frame.controlTelemetry.timeUs, frame.controlTelemetry.automaticValid,
      frame.controlTelemetry.safeElevatorCommandRad * radiansToDegrees,
      frame.controlTelemetry.observedElevatorCommandRad * radiansToDegrees,
      frame.controlTelemetry.observedRudderCommandRad * radiansToDegrees,
      frame.controlTelemetry.elevatorPwmSampleTimeUs, frame.controlTelemetry.rudderPwmSampleTimeUs,
      frame.controlTelemetry.safeRudderCommandRad * radiansToDegrees,
      frame.controlTelemetry.runIdentity.uf2_sha256, frame.controlTelemetry.runIdentity.model_sha256,
      frame.controlTelemetry.runIdentity.plant_sha256,
      frame.controlTelemetry.releaseMcuTimeUs, frame.controlTelemetry.plantIntervalStartS,
    ] : Array.from({ length: 14 }, () => "")),
    ...experimentCsvValues(frame.experiment),
    frame.controlTelemetry.tag === "firmware" ? frame.controlTelemetry.runIdentity.virtual_platform_sha256 ?? "" : "",
    frame.controlTelemetry.tag === "firmware" ? frame.controlTelemetry.runIdentity.scenario_sha256 ?? "" : "",
  ].join(","));
  const blob = new Blob([[`# birdman-session ${JSON.stringify(liveOutcome)}`, `# birdman-incidents ${JSON.stringify(liveIncidents)}`,
    header.join(","), ...rows].join("\n") + "\n"], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `birdman-live-${new Date().toISOString().replaceAll(":", "-")}.csv`;
  anchor.click();
  URL.revokeObjectURL(url);
}

function resize(): void {
  applyUiScale(window.innerWidth, window.innerHeight);
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
  // Ordinary buttons must not steal flight keys after Restart/Interactive clicks.
  // Editable controls retain their native keyboard interaction.
  return target instanceof HTMLInputElement || target instanceof HTMLSelectElement
    || target instanceof HTMLTextAreaElement || (target instanceof HTMLElement && target.isContentEditable);
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
