import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { createInterface } from 'node:readline';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { ConsoleLogger, GPIOPinState, I2CMode, LogLevel, Simulator } from 'rp2040js';

import { As5600Device, Bno055Device, Dps310Device, Sdp810Device, type I2cDevice, type PlantObservation } from './devices.js';
import { loadUf2 } from './uf2.js';

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
for (const path of [bridgePath, uf2Path, modelPath]) {
  if (!existsSync(path)) throw new Error(`required actual-UF2 input is missing: ${path}`);
}

const plant = spawn(bridgePath, ['--model', modelPath, '--dt', '0.01'], {
  cwd: root,
  windowsHide: true,
  stdio: ['pipe', 'pipe', 'inherit'],
});
const plantLines = createInterface({ input: plant.stdout })[Symbol.asyncIterator]();
const readPlant = async (): Promise<PlantObservation> => {
  const next = await plantLines.next();
  if (next.done) throw new Error('plant bridge stopped');
  return JSON.parse(next.value) as PlantObservation;
};

const simulator = new Simulator();
const mcu = simulator.rp2040;
mcu.logger = new ConsoleLogger(LogLevel.Error, false);
loadUf2(uf2Path, mcu);
const vectorTable = 0x10000100;
mcu.core.VTOR = vectorTable;
mcu.core.SP = mcu.readUint32(vectorTable);
mcu.core.PC = mcu.readUint32(vectorTable + 4) & 0xffff_fffe;

let observation = await readPlant();
const bno = new Bno055Device();
const angle = new As5600Device();
const sdp = new Sdp810Device();
const dps = new Dps310Device();
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
  if (connected) mode === I2CMode.Write ? connected.startWrite() : connected.startRead();
  i2c.completeConnect(connected !== undefined);
};
i2c.onWriteByte = value => { connected?.writeByte(value); i2c.completeWrite(connected !== undefined); };
i2c.onReadByte = () => i2c.completeRead(connected?.readByte() ?? 0xff);
i2c.onStop = () => { connected = undefined; i2c.completeStop(); };

for (const pin of [10, 11, 12, 13]) mcu.gpio[pin]?.setInputValue(true);
applyPilot({ pilot_elevator: 0, pilot_rudder: 0, autonomy: 1 });
const safetyPin = mcu.gpio[21];
const deadlinePin = mcu.gpio[20];
const pwmCandidate = mcu.pwm.channels[0];
if (!safetyPin || !deadlinePin || !pwmCandidate) throw new Error('required rp2040js GPIO/PWM is missing');
const pwm = pwmCandidate;
let safetyFailsafe = true;
let deadlineMissed = false;
safetyPin.addListener(state => { if (state === GPIOPinState.High) safetyFailsafe = true; else if (state === GPIOPinState.Low) safetyFailsafe = false; });
deadlinePin.addListener(state => { if (state === GPIOPinState.High) deadlineMissed = true; else if (state === GPIOPinState.Low) deadlineMissed = false; });

const cycleNanos = 1e9 / 125_000_000 * 50;
let instructions = 0;
while (((pwm.cc & 0xffff) === 0 || safetyFailsafe) && instructions < 100_000_000) executeOne();
if ((pwm.cc & 0xffff) === 0 || safetyFailsafe) throw new Error('actual UF2 did not arm in rp2040js');

const initialSimulationS = observation.time_s;
let initialWallMs: number | undefined;
let nextFirmwareTickUs = simulator.clock.micros + 10_000;
let processingAverageMs = 0;
process.stdout.write(`${JSON.stringify({ type: 'ready', backend: 'rp2040js-actual-uf2' })}\n`);
const input = createInterface({ input: process.stdin });
let stepChain = Promise.resolve();
input.on('line', line => {
  stepChain = stepChain.then(() => step(line)).catch(error => {
    process.stderr.write(`actual-UF2 web bridge: ${String(error)}\n`);
    process.exitCode = 1;
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
  applyPilot(command);
  const elevatorCommandRad = pulseToRad(pwm.cc & 0xffff);
  const rudderCommandRad = pulseToRad((pwm.cc >>> 16) & 0xffff);
  plant.stdin.write(`${JSON.stringify({ elevator_command_rad: elevatorCommandRad, rudder_command_rad: rudderCommandRad })}\n`);
  observation = await readPlant();
  updateDevices();
  while (simulator.clock.micros < nextFirmwareTickUs && instructions < 100_000_000) executeOne();
  if (instructions >= 100_000_000) throw new Error('rp2040js instruction limit reached');
  nextFirmwareTickUs += 10_000;

  const pilotElevator = decodedAxis(command.pilot_elevator, command.elevator_input_kind);
  const pilotRudder = decodedAxis(command.pilot_rudder, command.rudder_input_kind);
  const autonomy = Math.round(command.autonomy * 4095) / 4095;
  const manualElevator = pilotElevator * 10 * Math.PI / 180;
  const manualRudder = -pilotRudder * 10 * Math.PI / 180;
  const automaticElevator = inferAutomatic(elevatorCommandRad, manualElevator, autonomy);
  const automaticRudder = inferAutomatic(rudderCommandRad, manualRudder, autonomy);
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
    automatic_elevator_command_rad: automaticElevator,
    automatic_rudder_command_rad: automaticRudder,
    mixed_elevator_command_rad: elevatorCommandRad,
    mixed_rudder_command_rad: rudderCommandRad,
    backend: 'rp2040js-actual-uf2',
    emulation: {
      processing_ms: processingMs,
      processing_average_ms: processingAverageMs,
      real_time_ratio: realTimeRatio,
      lag_ms: lagMs,
      deadline_missed: deadlineMissed,
      real_time: processingAverageMs <= 10 && lagMs <= 100 && (!ratioSettled || realTimeRatio >= 0.9),
      timing_validated: false,
    },
  })}\n`);
  if (observation.surface_contact) input.close();
}

function executeOne(): void {
  const cycles = mcu.core.executeInstruction();
  simulator.clock.tick(cycles * cycleNanos);
  instructions += 1;
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

function decodedAxis(value: number, kind = 'analog'): number {
  if (kind === 'buttons') return value < -0.05 ? -1 : value > 0.05 ? 1 : 0;
  const counts = Math.round((clamp(value, -1, 1) + 1) * 0.5 * 4095);
  const centered = counts - 2047.5;
  const magnitude = Math.abs(centered);
  return magnitude <= 164 ? 0 : Math.sign(centered) * Math.min(1, (magnitude - 164) / (2047.5 - 164));
}

function inferAutomatic(mixed: number, manual: number, autonomy: number): number {
  return autonomy < 1e-4 ? 0 : clamp((mixed - manual * (1 - autonomy)) / autonomy, -10 * Math.PI / 180, 10 * Math.PI / 180);
}

function pulseToRad(pulseUs: number): number { return (pulseUs - 1500) * (10 * Math.PI / 180) / 500; }
function clamp(value: number, minimum: number, maximum: number): number { return Math.min(maximum, Math.max(minimum, value)); }
function parseCommand(line: string): PilotCommand {
  const value = JSON.parse(line) as PilotCommand;
  if (![value.pilot_elevator, value.pilot_rudder, value.autonomy].every(Number.isFinite)) throw new Error('invalid pilot command');
  return value;
}

process.on('exit', () => { plant.kill(); });
