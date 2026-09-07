import { spawn } from 'node:child_process';
import { stepMcu } from './mcu-step.js';
import { createWriteStream, existsSync, mkdirSync, writeFileSync, readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { createInterface } from 'node:readline';
import { once } from 'node:events';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { ConsoleLogger, GPIOPinState, I2CMode, LogLevel, Simulator } from 'rp2040js';
import {
  As5600Device,
  Bno055Device,
  Dps310Device,
  Sdp810Device,
  type I2cDevice,
  type PlantObservation,
  type SensorFaultKind,
} from './devices.js';
import { loadUf2 } from './uf2.js';
import { attachServoPwm } from './servo-pwm.js';
import { installWatchdogMonitor } from './watchdog-monitor.js';
import { FirmwareTelemetry } from './firmware-telemetry.js';

const commandLine = process.argv.slice(2);
if (commandLine.includes('--help') || commandLine.includes('-h')) {
  console.log(`birdman-fbw-virtual-platform

Options:
  --uf2 PATH                 production UF2 image
  --bridge PATH              Rust plant-bridge executable
  --model PATH               aircraft model JSON
  --output PATH              flight-log CSV
  --summary PATH             optional JSON summary
  --dt SECONDS               plant integration step
  --steps COUNT              maximum plant steps
  --timing-acceleration N    MCU clock acceleration; timing is not validated
  --gust-{north,east,down}-mps V
  --sensor-fault KIND        none, bno-status, bno-reset, as5600-magnet, sdp-crc, sdp-nack, dps-stale, dps-not-ready, i2c-stall
  --fault-start-s SECONDS    fault start in simulated plant time
  --fault-duration-s SECONDS fault duration
  --fault-update-count COUNT exact consecutive firmware control updates; overrides duration
  -h, --help`);
  process.exit(0);
}
const args = parseArgs(commandLine);
const repositoryRoot = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const bridgePath = resolve(repositoryRoot, args.bridge);
const modelPath = resolve(repositoryRoot, args.model);
const uf2Path = resolve(repositoryRoot, args.uf2);
for (const path of [bridgePath, modelPath, uf2Path]) {
  if (!existsSync(path)) throw new Error(`required input does not exist: ${path}`);
}
const inputIdentity = {
  uf2_sha256: createHash('sha256').update(readFileSync(uf2Path)).digest('hex'),
  model_sha256: createHash('sha256').update(readFileSync(modelPath)).digest('hex'),
  plant_sha256: createHash('sha256').update(readFileSync(bridgePath)).digest('hex'),
};
const bridgeArguments = [
  '--model', modelPath,
  '--dt', String(args.dtS),
];
if (args.gustNorthMps !== 0) bridgeArguments.push('--gust-north-mps', String(args.gustNorthMps));
if (args.gustEastMps !== 0) bridgeArguments.push('--gust-east-mps', String(args.gustEastMps));
if (args.gustDownMps !== 0) bridgeArguments.push('--gust-down-mps', String(args.gustDownMps));
const bridge = spawn(bridgePath, bridgeArguments, { stdio: ['pipe', 'pipe', 'inherit'] });
const lines = createInterface({ input: bridge.stdout })[Symbol.asyncIterator]();

async function readObservation(): Promise<PlantObservation> {
  const next = await lines.next();
  if (next.done) throw new Error('plant bridge ended before returning an observation');
  return JSON.parse(next.value) as PlantObservation;
}

async function main(): Promise<void> {
  let observation = await readObservation();
  const simulator = new Simulator();
  const mcu = simulator.rp2040;
  installWatchdogMonitor(mcu);
  const recorder = new FirmwareTelemetry();
  const uart = mcu.uart[1];
  if (!uart) throw new Error('required UART1 recorder is missing');
  uart.onByte = byte => recorder.receive(byte);
  mcu.logger = new ConsoleLogger(LogLevel.Error, false);
  loadUf2(uf2Path, mcu);
  const vectorTable = 0x10000100;
  mcu.core.VTOR = vectorTable;
  mcu.core.SP = mcu.readUint32(vectorTable);
  mcu.core.PC = mcu.readUint32(vectorTable + 4) & 0xffff_fffe;

  const bno = new Bno055Device();
  const angle = new As5600Device();
  const sdp = new Sdp810Device(() => simulator.clock.micros);
  const dps = new Dps310Device(() => simulator.clock.micros);
  const devices = new Map<number, I2cDevice>([bno, angle, sdp, dps].map(device => [device.address, device]));
  let faultUpdatesInjected = 0;
  let updateFaultActive = false;
  let faultTestStarted = false;
  const activeSensorFault = (): SensorFaultKind => (
    args.faultUpdateCount === 0
      && args.sensorFault !== 'none'
      && observation.time_s >= args.faultStartS
      && observation.time_s < args.faultStartS + args.faultDurationS
      ? args.sensorFault
      : 'none'
  );
  const updateDevicesWithFault = (fault: SensorFaultKind): void => {
    bno.update(observation, fault);
    angle.update(observation, fault);
    sdp.update(observation, fault);
    dps.update(observation, fault);
  };
  const updateDevices = (): void => updateDevicesWithFault(
    args.faultUpdateCount > 0 && updateFaultActive ? args.sensorFault : activeSensorFault(),
  );
  updateDevices();

  const i2c = mcu.i2c[0];
  if (!i2c) throw new Error('rp2040js has no I2C0 peripheral');
  let connected: I2cDevice | undefined;
  const i2cTrace: string[] = [];
  i2c.onStart = () => i2c.completeStart();
  i2c.onConnect = (address, mode) => {
    // Model a transaction that never completes, not an immediate NACK. The
    // actual firmware must rely on its hardware watchdog to bound this stall.
    if (activeSensorFault() === 'i2c-stall' || (updateFaultActive && args.sensorFault === 'i2c-stall')) return;
    const sdpNack = args.sensorFault === 'sdp-nack'
      && address === sdp.address
      && (activeSensorFault() === 'sdp-nack' || updateFaultActive);
    connected = sdpNack ? undefined : devices.get(address);
    const ack = connected !== undefined && (mode === I2CMode.Write ? connected.startWrite() : connected.startRead());
    if (!ack) connected = undefined;
    i2c.completeConnect(ack);
    i2cTrace.push(`${address.toString(16)}:${mode === I2CMode.Write ? 'w' : 'r'}:${connected ? 'ack' : 'nack'}`);
    if (i2cTrace.length > 20) i2cTrace.shift();
  };
  i2c.onWriteByte = value => {
    i2c.completeWrite(connected?.writeByte(value) ?? false);
  };
  i2c.onReadByte = () => i2c.completeRead(connected?.readByte() ?? 0xff);
  i2c.onStop = () => {
    connected = undefined;
    i2c.completeStop();
  };

  const cycleNanos = 1e9 / 125_000_000 * args.timingAcceleration;
  const pwm = mcu.pwm.channels[0];
  const servos = attachServoPwm(simulator);
  const servoReady = (): boolean => servos.elevator.sample(simulator.clock.micros).kind === 'valid'
    && servos.rudder.sample(simulator.clock.micros).kind === 'valid';
  if (!pwm) throw new Error('rp2040js has no PWM slice 0');
  // Released active-low buttons and centred analog stick with full auto authority.
  for (const pinNumber of [10, 11, 12, 13]) mcu.gpio[pinNumber]?.setInputValue(true);
  mcu.adc.channelValues[0] = 2048;
  mcu.adc.channelValues[1] = 2048;
  mcu.adc.channelValues[2] = 4095;
  const safetyPin = mcu.gpio[21];
  const invalidPin = mcu.gpio[18];
  const controlTickPin = mcu.gpio[19];
  const deadlinePin = mcu.gpio[20];
  if (!safetyPin || !invalidPin || !controlTickPin || !deadlinePin) {
    throw new Error('rp2040js has no FBW diagnostic GPIO');
  }
  let safetyFailsafe = true;
  let sensorSampleInvalid = false;
  let controlUpdateCount = 0;
  let deadlineMissed = false;
  let hasBeenArmed = false;
  let failsafeActivationCount = 0;
  let firstFailsafeTimeS: number | null = null;
  let firstInvalidControlUpdate: number | null = null;
  let firstFailsafeControlUpdate: number | null = null;
  let firstRecoveryValidControlUpdate: number | null = null;
  let firstRearmedControlUpdate: number | null = null;
  let deadlineMissActivationCount = 0;
  safetyPin.addListener(state => {
    if (state === GPIOPinState.High) {
      if (!safetyFailsafe && hasBeenArmed && faultTestStarted) {
        failsafeActivationCount += 1;
        if (firstFailsafeTimeS === null) firstFailsafeTimeS = observation.time_s;
        if (firstFailsafeControlUpdate === null) firstFailsafeControlUpdate = controlUpdateCount;
      }
      safetyFailsafe = true;
    } else if (state === GPIOPinState.Low) {
      if (safetyFailsafe && firstFailsafeControlUpdate !== null && firstRearmedControlUpdate === null) {
        firstRearmedControlUpdate = controlUpdateCount;
      }
      safetyFailsafe = false;
      hasBeenArmed = true;
    }
  });
  invalidPin.addListener(state => {
    if (state === GPIOPinState.High) {
      if (!sensorSampleInvalid && hasBeenArmed && faultTestStarted && firstInvalidControlUpdate === null) {
        firstInvalidControlUpdate = controlUpdateCount;
      }
      sensorSampleInvalid = true;
    } else if (state === GPIOPinState.Low) {
      if (sensorSampleInvalid && firstInvalidControlUpdate !== null
        && firstRecoveryValidControlUpdate === null) {
        firstRecoveryValidControlUpdate = controlUpdateCount;
      }
      sensorSampleInvalid = false;
    }
  });
  controlTickPin.addListener(state => {
    if (state === GPIOPinState.High || state === GPIOPinState.Low) {
      controlUpdateCount += 1;
      // Sensor conversion clocks continue while the aircraft plant is frozen before release.
      if (!hasBeenArmed) updateDevicesWithFault('none');
      if (args.sensorFault !== 'none' && observation.time_s >= args.faultStartS) {
        faultTestStarted = true;
      }
      if (args.faultUpdateCount > 0 && observation.time_s >= args.faultStartS) {
        updateFaultActive = faultUpdatesInjected < args.faultUpdateCount;
        updateDevicesWithFault(updateFaultActive ? args.sensorFault : 'none');
        if (updateFaultActive) faultUpdatesInjected += 1;
      }
    }
  });
  deadlinePin.addListener(state => {
    if (state === GPIOPinState.High) {
      if (!deadlineMissed) deadlineMissActivationCount += 1;
      deadlineMissed = true;
    }
    else if (state === GPIOPinState.Low) deadlineMissed = false;
  });
  let instructions = 0;
  const instructionLimit = 100_000_000;
  let lastFlashPc = mcu.core.PC;
  let invalidPc: number | undefined;
  let startupIterations = 0;
  while ((!servoReady() || safetyFailsafe) && startupIterations++ < instructionLimit) {
    if (mcu.core.PC >= 0x1000_0000 && mcu.core.PC < 0x1020_0000) lastFlashPc = mcu.core.PC;
    else if (pwm.top === 19_999 && mcu.core.PC > 0x0000_4000) {
      invalidPc = mcu.core.PC;
      break;
    }
    instructions += stepMcu(simulator, cycleNanos);
  }
  if (!servoReady() || safetyFailsafe) {
    throw new Error(
      `firmware did not reach preflight arm: pc=0x${mcu.core.PC.toString(16)} ` +
      `last_flash_pc=0x${lastFlashPc.toString(16)} invalid_pc=${invalidPc?.toString(16) ?? 'none'} ` +
      `clock_us=${simulator.clock.micros.toFixed(1)} pwm_top=${pwm.top} instructions=${instructions} ` +
      `i2c=${i2cTrace.join(',')}`,
    );
  }
  const preflightBnoConfigurationWrites = bno.configurationWriteCount;
  const releaseMcuTimeUs = simulator.clock.micros;
  // Preflight runs against a frozen plant; release-time metrics start at zero.
  controlUpdateCount = 0;
  deadlineMissActivationCount = 0;

  const outputPath = resolve(repositoryRoot, args.output);
  mkdirSync(dirname(outputPath), { recursive: true });
  const output = createWriteStream(outputPath, { encoding: 'utf8' });
  output.write('time_s,north_m,altitude_m,flight_path_deg,pitch_deg,airspeed_mps,alpha_deg,elevator_command_deg,elevator_actual_deg,rudder_command_deg,rudder_actual_deg,aero_in_range,surface_contact,sensor_fault_injected,sensor_sample_invalid,safety_failsafe,deadline_missed,firmware_sequence,firmware_time_us,automatic_valid,automatic_elevator_command_deg,automatic_rudder_command_deg,safe_elevator_command_deg,mixed_elevator_command_deg,mixed_rudder_command_deg,elevator_pwm_sample_time_us,rudder_pwm_sample_time_us,plant_interval_start_s,safe_rudder_command_deg,observed_elevator_command_deg,observed_rudder_command_deg,uf2_sha256,model_sha256,plant_sha256,release_mcu_time_us,elevator_deg,rudder_deg,pilot_elevator,pilot_rudder,autonomy,manual_elevator_command_deg,manual_rudder_command_deg,east_m,roll_deg,yaw_deg\n');
  let nextFirmwareTickUs = simulator.clock.micros + args.dtS * 1e6;
  let runningMinimumAltitude = observation.altitude_m;
  let maximumReascent = 0;
  let maximumFlightPathDeg = observation.flight_path_rad * 180 / Math.PI;
  let positiveFlightPathSamples = 0;
  let failsafeDurationS = 0;
  let invalidSampleDurationS = 0;
  let deadlineMissDurationS = 0;

  for (let step = 0; step < args.steps && !observation.surface_contact; step += 1) {
    const record = recorder.requireFresh(simulator.clock.micros);
    const elevator = servos.elevator.sample(simulator.clock.micros);
    const rudder = servos.rudder.sample(simulator.clock.micros);
    if (elevator.kind !== 'valid' || rudder.kind !== 'valid') throw new Error('servo PWM missing or invalid');
    const elevatorCommandRad = elevator.commandRad;
    const rudderCommandRad = rudder.commandRad;
    const intervalStart = observation.time_s;
    // Causal held-input coupling: firmware sees only interval-start sensors.
    let stepIterations = 0;
    while (simulator.clock.micros < nextFirmwareTickUs && stepIterations++ < instructionLimit) {
      instructions += stepMcu(simulator, cycleNanos, nextFirmwareTickUs);
    }
    if (stepIterations >= instructionLimit) throw new Error('virtual MCU per-step execution watchdog reached');
    nextFirmwareTickUs += args.dtS * 1e6;
    bridge.stdin.write(`${JSON.stringify({ elevator_command_rad: elevatorCommandRad, rudder_command_rad: rudderCommandRad })}\n`);
    observation = await readObservation();
    updateDevices();
    const flightPathDeg = observation.flight_path_rad * 180 / Math.PI;
    const pitchDeg = observation.pitch_rad * 180 / Math.PI;
    const alphaDeg = observation.sensor_alpha_rad * 180 / Math.PI;
    const commandDeg = elevatorCommandRad * 180 / Math.PI;
    const actualDeg = observation.elevator_rad * 180 / Math.PI;
    const rudderCommandDeg = rudderCommandRad * 180 / Math.PI;
    const rudderActualDeg = observation.rudder_rad * 180 / Math.PI;
    const faultInjected = activeSensorFault() !== 'none' || updateFaultActive;
    output.write([
      observation.time_s.toFixed(5), observation.north_m.toFixed(6), observation.altitude_m.toFixed(6),
      flightPathDeg.toFixed(6), pitchDeg.toFixed(6), observation.sensor_airspeed_mps.toFixed(6),
      alphaDeg.toFixed(6), commandDeg.toFixed(6), actualDeg.toFixed(6),
      rudderCommandDeg.toFixed(6), rudderActualDeg.toFixed(6),
      Number(observation.aero_in_range), Number(observation.surface_contact), Number(faultInjected),
      Number(sensorSampleInvalid), Number(safetyFailsafe), Number(deadlineMissed),
      record.sequence, record.timeUs, Number(record.automaticValid),
      record.automaticElevator * 180 / Math.PI, record.automaticRudder * 180 / Math.PI,
      record.safeElevator * 180 / Math.PI, record.mixedElevator * 180 / Math.PI, record.mixedRudder * 180 / Math.PI,
      elevator.atUs, rudder.atUs, intervalStart,
      record.safeRudder * 180 / Math.PI, commandDeg, rudderCommandDeg,
      inputIdentity.uf2_sha256, inputIdentity.model_sha256, inputIdentity.plant_sha256, releaseMcuTimeUs,
      actualDeg, rudderActualDeg, record.pilotElevator, record.pilotRudder, record.autonomy,
      record.pilotElevator * 10, -record.pilotRudder * 10,
      observation.east_m, observation.roll_rad * 180 / Math.PI, observation.yaw_rad * 180 / Math.PI,
    ].join(',') + '\n');
    runningMinimumAltitude = Math.min(runningMinimumAltitude, observation.altitude_m);
    maximumReascent = Math.max(maximumReascent, observation.altitude_m - runningMinimumAltitude);
    maximumFlightPathDeg = Math.max(maximumFlightPathDeg, flightPathDeg);
    if (flightPathDeg > 0) positiveFlightPathSamples += 1;
    if (safetyFailsafe) failsafeDurationS += args.dtS;
    if (sensorSampleInvalid) invalidSampleDurationS += args.dtS;
    if (deadlineMissed) deadlineMissDurationS += args.dtS;

  }
  output.end();
  await once(output, 'finish');
  bridge.stdin.end();
  await once(bridge, 'exit');
  const summary = {
    evidence_version: 1,
    release_mcu_time_us: releaseMcuTimeUs,
    generated_at: new Date().toISOString(),
    inputs: inputIdentity,
    simulated_s: observation.time_s,
    final_altitude_m: observation.altitude_m,
    maximum_flight_path_deg: maximumFlightPathDeg,
    maximum_reascent_m: maximumReascent,
    positive_flight_path_samples: positiveFlightPathSamples,
    instructions,
    timing_acceleration: args.timingAcceleration,
    timing_validated: false,
    gust_wind_ned_mps: [args.gustNorthMps, args.gustEastMps, args.gustDownMps],
    sensor_fault: {
      kind: args.sensorFault,
      start_s: args.faultStartS,
      duration_s: args.faultDurationS,
      update_count: args.faultUpdateCount,
      updates_injected: faultUpdatesInjected,
    },
    sensor_reinitializations_observed:
      bno.configurationWriteCount - preflightBnoConfigurationWrites,
    failsafe_activation_count: failsafeActivationCount,
    first_failsafe_time_s: firstFailsafeTimeS,
    failsafe_entry_latency_s: firstFailsafeTimeS === null || args.sensorFault === 'none'
      ? null
      : firstFailsafeTimeS - args.faultStartS,
    failsafe_or_arming_duration_s: failsafeDurationS,
    invalid_sample_duration_s: invalidSampleDurationS,
    safety_failsafe_at_end: safetyFailsafe,
    control_updates_observed: controlUpdateCount,
    invalid_to_failsafe_control_updates: firstInvalidControlUpdate === null || firstFailsafeControlUpdate === null
      ? null
      : firstFailsafeControlUpdate - firstInvalidControlUpdate,
    recovery_valid_to_rearmed_control_updates:
      firstRecoveryValidControlUpdate === null || firstRearmedControlUpdate === null
        ? null
        : firstRearmedControlUpdate - firstRecoveryValidControlUpdate,
    deadline_miss_activation_count: deadlineMissActivationCount,
    deadline_miss_observed_duration_s: deadlineMissDurationS,
  };
  if (args.summary) {
    const summaryPath = resolve(repositoryRoot, args.summary);
    mkdirSync(dirname(summaryPath), { recursive: true });
    writeFileSync(summaryPath, JSON.stringify(summary, null, 2) + '\n', 'utf8');
  }
  console.log(JSON.stringify(summary, null, 2));
}

function parseArgs(values: string[]) {
  const result = {
    uf2: 'target/virtual-platform/fbw-rp2040.uf2',
    bridge: 'target/debug/plant-bridge.exe',
    model: 'models/qx18-br-training-envelope.json',
    output: 'reports/virtual-platform.csv',
    summary: '',
    dtS: 0.01,
    steps: 2000,
    timingAcceleration: 1,
    gustNorthMps: 0,
    gustEastMps: 0,
    gustDownMps: 0,
    sensorFault: 'none' as SensorFaultKind,
    faultStartS: 5,
    faultDurationS: 0.2,
    faultUpdateCount: 0,
  };
  for (let index = 0; index < values.length; index += 2) {
    const name = values[index];
    const value = values[index + 1];
    if (!value) throw new Error(`${name} requires a value`);
    if (name === '--uf2') result.uf2 = value;
    else if (name === '--bridge') result.bridge = value;
    else if (name === '--model') result.model = value;
    else if (name === '--output') result.output = value;
    else if (name === '--summary') result.summary = value;
    else if (name === '--dt') result.dtS = Number(value);
    else if (name === '--steps') result.steps = Number(value);
    else if (name === '--timing-acceleration') result.timingAcceleration = Number(value);
    else if (name === '--gust-north-mps') result.gustNorthMps = Number(value);
    else if (name === '--gust-east-mps') result.gustEastMps = Number(value);
    else if (name === '--gust-down-mps') result.gustDownMps = Number(value);
    else if (name === '--sensor-fault') result.sensorFault = value as SensorFaultKind;
    else if (name === '--fault-start-s') result.faultStartS = Number(value);
    else if (name === '--fault-duration-s') result.faultDurationS = Number(value);
    else if (name === '--fault-update-count') result.faultUpdateCount = Number(value);
    else throw new Error(`unknown argument: ${name}`);
  }
  if (!Number.isFinite(result.dtS) || result.dtS <= 0) throw new Error('--dt must be positive');
  if (!Number.isInteger(result.steps) || result.steps <= 0) throw new Error('--steps must be a positive integer');
  if (!Number.isFinite(result.timingAcceleration) || result.timingAcceleration <= 0) {
    throw new Error('--timing-acceleration must be positive');
  }
  if (![result.gustNorthMps, result.gustEastMps, result.gustDownMps].every(Number.isFinite)) {
    throw new Error('gust components must be finite');
  }
  const sensorFaults: SensorFaultKind[] = [
    'none', 'bno-status', 'bno-reset', 'as5600-magnet', 'sdp-crc', 'sdp-nack', 'dps-stale', 'dps-not-ready', 'i2c-stall',
  ];
  if (!sensorFaults.includes(result.sensorFault)) throw new Error(`unknown --sensor-fault: ${result.sensorFault}`);
  if (!Number.isFinite(result.faultStartS) || result.faultStartS < 0) {
    throw new Error('--fault-start-s must be finite and non-negative');
  }
  if (!Number.isFinite(result.faultDurationS) || result.faultDurationS < 0) {
    throw new Error('--fault-duration-s must be finite and non-negative');
  }
  if (!Number.isInteger(result.faultUpdateCount) || result.faultUpdateCount < 0) {
    throw new Error('--fault-update-count must be a non-negative integer');
  }
  return result;
}

main().catch(error => {
  bridge.kill();
  console.error(error);
  process.exitCode = 1;
});
