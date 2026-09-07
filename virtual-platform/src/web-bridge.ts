import { spawn } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { createInterface } from 'node:readline';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { ConsoleLogger, GPIOPinState, I2CMode, LogLevel, Simulator } from 'rp2040js';

import { As5600Device, Bno055Device, Dps310Device, Sdp810Device, type I2cDevice, type PlantObservation } from './devices.js';
import { loadUf2 } from './uf2.js';
import { FirmwareTelemetry } from './firmware-telemetry.js';
import { attachServoPwm } from './servo-pwm.js';
import { advanceUntil } from './execution-budget.js';
import { installWatchdogMonitor } from './watchdog-monitor.js';
import { stepMcu } from './mcu-step.js';
import { hashValue, virtualPlatformDigest } from './run-identity.js';

interface PilotCommand {
  pilot_elevator: number;
  pilot_rudder: number;
  autonomy: number;
  elevator_input_kind?: 'analog' | 'buttons';
  rudder_input_kind?: 'analog' | 'buttons';
}

const root = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const executable = process.platform === 'win32' ? 'plant-bridge.exe' : 'plant-bridge';
const bridgePath = resolve(root, 'target', 'debug', executable);
const uf2Path = resolve(root, 'target', 'virtual-platform', 'fbw-rp2040.uf2');
const modelPath = resolve(root, 'models', 'qx18-br-training-envelope.json');
const digest = (path: string): string => createHash('sha256').update(readFileSync(path)).digest('hex');
const runIdentity = { uf2_sha256: digest(uf2Path), model_sha256: digest(modelPath), plant_sha256: digest(bridgePath),
  virtual_platform_sha256: virtualPlatformDigest(root),
  scenario_sha256: hashValue({ backend: 'interactive', dt_s: 0.01, timing_acceleration: 1,
    coupling: 'held-start-inputs', pilot: 'recorded-live-inputs' }) };
for (const path of [bridgePath, uf2Path, modelPath]) {
  if (!existsSync(path)) throw new Error(`required actual-UF2 input is missing: ${path}`);
}

const plant = spawn(bridgePath, ['--model', modelPath, '--dt', '0.01'], {
  cwd: root,
  windowsHide: true,
  stdio: ['pipe', 'pipe', 'pipe'],
});
process.on('exit', () => { plant.kill(); });
let plantError = '';
plant.stderr.setEncoding('utf8');
plant.stderr.on('data', (chunk: string) => { plantError = (plantError + chunk).slice(-4096); });
const plantLines = createInterface({ input: plant.stdout })[Symbol.asyncIterator]();
const readPlant = async (): Promise<PlantObservation> => {
  const next = await new Promise<IteratorResult<string>>((resolve, reject) => {
    const timeout = setTimeout(() => {
      plant.kill();
      reject(new Error('plant response timed out after 4 seconds (wall clock)'));
    }, 4000);
    void plantLines.next().then(value => {
      clearTimeout(timeout);
      resolve(value);
    }, error => {
      clearTimeout(timeout);
      reject(error);
    });
  });
  if (next.done) throw new Error(plantError.trim() || 'plant bridge stopped');
  return JSON.parse(next.value) as PlantObservation;
};

const simulator = new Simulator();
const mcu = simulator.rp2040;
installWatchdogMonitor(mcu);
const recorder = new FirmwareTelemetry();
const uart = mcu.uart[1];
if (!uart) throw new Error('required UART1 recorder is missing');
uart.onByte = byte => recorder.receive(byte);
const servos = attachServoPwm(simulator);
mcu.logger = new ConsoleLogger(LogLevel.Error, false);
loadUf2(uf2Path, mcu);
const vectorTable = 0x10000100;
mcu.core.VTOR = vectorTable;
mcu.core.SP = mcu.readUint32(vectorTable);
mcu.core.PC = mcu.readUint32(vectorTable + 4) & 0xffff_fffe;

let observation = await readPlant();
const bno = new Bno055Device();
const angle = new As5600Device();
const sdp = new Sdp810Device(() => simulator.clock.micros);
const dps = new Dps310Device(() => simulator.clock.micros);
const devices = new Map<number, I2cDevice>([bno, angle, sdp, dps].map(device => [device.address, device]));
const updateDevices = (): void => {
  bno.update(observation);
  angle.update(observation);
  sdp.update(observation);
  dps.update(observation);
};
updateDevices();

const i2c = mcu.i2c[0];
if (!i2c) throw new Error('rp2040js has no I2C0');
let connected: I2cDevice | undefined;
i2c.onStart = () => i2c.completeStart();
i2c.onConnect = (address, mode) => {
  connected = devices.get(address);
  const acknowledged = connected ? (mode === I2CMode.Write ? connected.startWrite() : connected.startRead()) : false;
  i2c.completeConnect(acknowledged);
};
i2c.onWriteByte = value => { i2c.completeWrite(connected ? connected.writeByte(value) : false); };
i2c.onReadByte = () => i2c.completeRead(connected?.readByte() ?? 0xff);
i2c.onStop = () => { connected = undefined; i2c.completeStop(); };

for (const pin of [10, 11, 12, 13]) mcu.gpio[pin]?.setInputValue(true);
applyPilot({ pilot_elevator: 0, pilot_rudder: 0, autonomy: 1 });
const safetyPin = mcu.gpio[21];
const deadlinePin = mcu.gpio[20];
if (!safetyPin || !deadlinePin) throw new Error('required rp2040js GPIO is missing');
let safetyFailsafe = true;
let deadlineMissed = false;
safetyPin.addListener(state => { if (state === GPIOPinState.High) safetyFailsafe = true; else if (state === GPIOPinState.Low) safetyFailsafe = false; });
deadlinePin.addListener(state => { if (state === GPIOPinState.High) deadlineMissed = true; else if (state === GPIOPinState.Low) deadlineMissed = false; });

const cycleNanos = 1e9 / 125_000_000;
let instructions = 0;
advanceUntil(() => !safetyFailsafe && recorder.state.tag === 'received'
  && servos.elevator.sample(simulator.clock.micros).kind === 'valid'
  && servos.rudder.sample(simulator.clock.micros).kind === 'valid', executeOne, 100_000_000);

const initialSimulationS = observation.time_s;
const releaseMcuTimeUs = simulator.clock.micros;
let initialWallMs: number | undefined;
let nextFirmwareTickUs = simulator.clock.micros + 10_000;
let processingAverageMs = 0;
process.stdout.write(`${JSON.stringify({ type: 'ready', backend: 'rp2040js-actual-uf2' })}\n`);
const input = createInterface({ input: process.stdin });
let stepChain = Promise.resolve();
let session: 'active' | 'ended' = 'active';
input.on('line', line => {
  stepChain = stepChain.then(() => session === 'active' ? step(line) : undefined).catch(error => {
    process.stderr.write(`actual-UF2 web bridge: ${String(error)}\n`);
    process.exitCode = 1;
    finish('error', String(error));
  });
});
input.on('close', () => {
  void stepChain.finally(() => {
    plant.stdin.end();
    if (plant.exitCode === null) plant.kill();
  });
});

async function step(line: string): Promise<void> {
  const started = performance.now();
  initialWallMs ??= started;
  const command = parseCommand(line);
  // Snapshot the completed firmware record and physical PWM before applying the
  // next input. They are labelled separately from the end-of-interval plant state.
  const record = recorder.requireFresh(simulator.clock.micros);
  const elevator = servos.elevator.sample(simulator.clock.micros);
  const rudder = servos.rudder.sample(simulator.clock.micros);
  if (elevator.kind !== 'valid' || rudder.kind !== 'valid') throw new Error('servo PWM missing or invalid');
  const intervalStart = observation.time_s;
  applyPilot(command);
  const elevatorCommandRad = elevator.commandRad;
  const rudderCommandRad = rudder.commandRad;
  // Advance both subsystems with held inputs from the interval start. Never
  // expose future plant observations to firmware executing this interval.
  const mcuStarted = performance.now();
  const instructionsBefore = instructions;
  advanceUntil(() => simulator.clock.micros >= nextFirmwareTickUs,
    () => { instructions += stepMcu(simulator, cycleNanos, nextFirmwareTickUs); }, 1_000_000);
  const mcuProcessingMs = performance.now() - mcuStarted;
  nextFirmwareTickUs += 10_000;
  const plantStarted = performance.now();
  plant.stdin.write(`${JSON.stringify({ elevator_command_rad: elevatorCommandRad, rudder_command_rad: rudderCommandRad })}\n`);
  observation = await readPlant();
  const plantRoundTripMs = performance.now() - plantStarted;
  updateDevices();

  const pilotElevator = record.pilotElevator;
  const pilotRudder = record.pilotRudder;
  const autonomy = record.autonomy;
  const manualElevator = pilotElevator * 10 * Math.PI / 180;
  const manualRudder = -pilotRudder * 10 * Math.PI / 180;
  const processingMs = performance.now() - started;
  processingAverageMs = processingAverageMs === 0 ? processingMs : processingAverageMs * 0.9 + processingMs * 0.1;
  const wallElapsedMs = performance.now() - initialWallMs;
  const simulatedElapsedMs = (observation.time_s - initialSimulationS) * 1000;
  const realTimeRatio = simulatedElapsedMs / Math.max(1, wallElapsedMs);
  const lagMs = wallElapsedMs - simulatedElapsedMs;
  const ratioSettled = simulatedElapsedMs >= 500;
  process.stdout.write(`${JSON.stringify({
    ...observation,
    pilot_elevator: pilotElevator,
    pilot_rudder: pilotRudder,
    autonomy,
    manual_elevator_command_rad: manualElevator,
    manual_rudder_command_rad: manualRudder,
    automatic_elevator_command_rad: record.automaticElevator,
    automatic_rudder_command_rad: record.automaticRudder,
    mixed_elevator_command_rad: record.mixedElevator,
    mixed_rudder_command_rad: record.mixedRudder,
    firmware_sequence: record.sequence,
    firmware_time_us: record.timeUs,
    automatic_valid: record.automaticValid,
    safe_elevator_command_rad: record.safeElevator,
    safe_rudder_command_rad: record.safeRudder,
    run_identity: runIdentity,
    observed_elevator_command_rad: elevatorCommandRad,
    observed_rudder_command_rad: rudderCommandRad,
    elevator_pwm_sample_time_us: elevator.atUs,
    rudder_pwm_sample_time_us: rudder.atUs,
    plant_interval_start_s: intervalStart,
    release_mcu_time_us: releaseMcuTimeUs,
    backend: 'rp2040js-actual-uf2',
    emulation: {
      wall_elapsed_ms: wallElapsedMs,
      mcu_processing_ms: mcuProcessingMs,
      plant_round_trip_ms: plantRoundTripMs,
      instructions: instructions - instructionsBefore,
      processing_ms: processingMs,
      processing_average_ms: processingAverageMs,
      real_time_ratio: realTimeRatio,
      lag_ms: lagMs,
      deadline_missed: deadlineMissed,
      real_time: processingAverageMs <= 10 && lagMs <= 100 && (!ratioSettled || realTimeRatio >= 0.9),
      timing_validated: false,
      timing_acceleration: 1,
    },
  })}\n`);
  if (observation.surface_contact) finish('ended', 'surface contact');
}

function finish(type: 'ended' | 'error', message: string): void {
  if (session === 'ended') return;
  session = 'ended';
  process.stdout.write(`${JSON.stringify({ type, message })}\n`);
  input.close();
  process.stdin.pause();
}

function executeOne(): void {
  instructions += stepMcu(simulator, cycleNanos);
}

function applyPilot(command: PilotCommand): void {
  setAxisOrButtons(0, 10, 11, command.pilot_elevator, command.elevator_input_kind);
  setAxisOrButtons(1, 12, 13, command.pilot_rudder, command.rudder_input_kind);
  mcu.adc.channelValues[2] = Math.round(clamp(command.autonomy, 0, 1) * 4095);
}

function setAxisOrButtons(channel: number, negativePin: number, positivePin: number, value: number, kind = 'analog'): void {
  const normalized = clamp(value, -1, 1);
  const buttons = kind === 'buttons';
  mcu.gpio[negativePin]?.setInputValue(!(buttons && normalized < -0.05));
  mcu.gpio[positivePin]?.setInputValue(!(buttons && normalized > 0.05));
  mcu.adc.channelValues[channel] = buttons ? 2048 : Math.round((normalized + 1) * 0.5 * 4095);
}

function clamp(value: number, minimum: number, maximum: number): number { return Math.min(maximum, Math.max(minimum, value)); }
function parseCommand(line: string): PilotCommand {
  const value = JSON.parse(line) as PilotCommand;
  if (![value.pilot_elevator, value.pilot_rudder, value.autonomy].every(Number.isFinite)) throw new Error('invalid pilot command');
  return value;
}
