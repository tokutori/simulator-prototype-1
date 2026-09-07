export interface PlantObservation {
  time_s: number;
  north_m: number;
  east_m: number;
  roll_rad: number;
  yaw_rad: number;
  altitude_m: number;
  pitch_rad: number;
  flight_path_rad: number;
  elevator_rad: number;
  rudder_rad: number;
  sensor_pitch_rad: number;
  sensor_roll_rad: number;
  sensor_yaw_rad: number;
  sensor_roll_rate_rad_s: number;
  sensor_pitch_rate_rad_s: number;
  sensor_yaw_rate_rad_s: number;
  sensor_airspeed_mps: number;
  sensor_differential_pressure_pa: number;
  sensor_barometric_altitude_m: number;
  /** Continuous pressure-derived input; DPS310 alone owns MCU acquisition timing. */
  sensor_barometric_input_altitude_m: number;
  sensor_alpha_rad: number;
  aero_in_range: boolean;
  surface_contact: boolean;
}

export interface I2cDevice {
  readonly address: number;
  startWrite(): boolean;
  startRead(): boolean;
  writeByte(value: number): boolean;
  readByte(): number;
}

export type SensorFaultKind =
  | 'none'
  | 'bno-status'
  | 'bno-reset'
  | 'as5600-magnet'
  | 'sdp-crc'
  | 'sdp-nack'
  | 'dps-stale'
  | 'dps-not-ready'
  | 'i2c-stall';

abstract class RegisterDevice implements I2cDevice {
  private pointer = 0;
  private expectingPointer = true;

  constructor(readonly address: number) {}

  startWrite(): boolean {
    this.expectingPointer = true;
    return true;
  }

  startRead(): boolean { return true; }

  writeByte(value: number): boolean {
    if (this.expectingPointer) {
      this.pointer = value & 0xff;
      this.expectingPointer = false;
    } else {
      this.writeRegister(this.pointer, value & 0xff);
      this.pointer = (this.pointer + 1) & 0xff;
    }
    return true;
  }

  readByte(): number {
    const value = this.readRegister(this.pointer);
    this.pointer = (this.pointer + 1) & 0xff;
    return value;
  }

  protected abstract readRegister(register: number): number;
  protected writeRegister(_register: number, _value: number): void {}
}

export class Bno055Device extends RegisterDevice {
  private readonly registers = new Uint8Array(256);
  private operationMode = 0;
  private configurationWrites = 0;

  constructor() {
    super(0x28);
    this.registers[0x00] = 0xa0;
    this.registers[0x39] = 0x05;
    this.registers[0x3a] = 0x00;
    this.registers[0x3b] = 0x80;
    this.registers[0x41] = 0x24;
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    // Physical sensor axes X=right, Y=forward, Z=up. Gyro is a vector in
    // sensor axes, not a vector named after the device's Euler fields.
    const gyroXRaw = clampI16(Math.round(observation.sensor_pitch_rate_rad_s * 180 / Math.PI * 16));
    const gyroRaw = clampI16(Math.round(observation.sensor_roll_rate_rad_s * 180 / Math.PI * 16));
    const gyroZRaw = clampI16(Math.round(-observation.sensor_yaw_rate_rad_s * 180 / Math.PI * 16));
    const r = observation.sensor_roll_rad / 2, p = observation.sensor_pitch_rad / 2, h = observation.sensor_yaw_rad / 2;
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), ch = Math.cos(h), sh = Math.sin(h);
    const w = cr * cp * ch + sr * sp * sh;
    const x = cr * sp * ch + sr * cp * sh;
    const y = sr * cp * ch - cr * sp * sh;
    const z = -(cr * cp * sh - sr * sp * ch);
    [w, x, y, z].forEach((value, index) => putI16Le(this.registers, 0x20 + index * 2, Math.round(value * 16384)));
    putI16Le(this.registers, 0x14, gyroXRaw);
    putI16Le(this.registers, 0x16, gyroRaw);
    putI16Le(this.registers, 0x18, gyroZRaw);
    if (fault === 'bno-reset') {
      this.operationMode = 0;
      this.registers[0x3d] = 0;
    }
    const fusionRunning = this.operationMode === 0x0c
      && fault !== 'bno-status'
      && fault !== 'bno-reset';
    this.registers[0x39] = fusionRunning ? 0x05 : 0x01;
    this.registers[0x3a] = fusionRunning ? 0x00 : 0x09;
  }

  get configurationWriteCount(): number {
    return this.configurationWrites;
  }

  protected readRegister(register: number): number {
    if (register >= 0x1a && register <= 0x1f) {
      throw new Error('BNO055 Euler registers are outside the quaternion installation contract');
    }
    return this.registers[register] ?? 0xff;
  }

  protected writeRegister(register: number, value: number): void {
    // Configuration subset used by this installation. Unsupported remap/units
    // cannot silently produce plausible flight data under another convention.
    if ((register === 0x3b && value !== 0x80)
      || (register === 0x41 && value !== 0x24) || (register === 0x42 && value !== 0)) {
      throw new Error('unsupported BNO055 installation configuration');
    }
    if ([0x3b, 0x41, 0x42].includes(register) && this.operationMode !== 0) {
      throw new Error('BNO055 configuration requires CONFIGMODE');
    }
    this.registers[register] = value;
    if (register === 0x3d) {
      this.operationMode = value;
      if (value === 0x0c) this.configurationWrites += 1;
      this.registers[0x39] = value === 0x0c ? 0x05 : 0x01;
      this.registers[0x3a] = value === 0x0c ? 0x00 : 0x09;
    }
  }
}

export class As5600Device extends RegisterDevice {
  private rawAngle = 2048;
  private magnetStatus = 0x20;

  constructor() {
    super(0x36);
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    this.rawAngle = modulo(Math.round(2048 + observation.sensor_alpha_rad / (2 * Math.PI) * 4096), 4096);
    this.magnetStatus = fault === 'as5600-magnet' ? 0x10 : 0x20;
  }

  protected readRegister(register: number): number {
    if (register === 0x0b) return this.magnetStatus;
    if (register === 0x0c) return (this.rawAngle >>> 8) & 0x0f;
    if (register === 0x0d) return this.rawAngle & 0xff;
    return 0;
  }
}

export class Sdp810Device implements I2cDevice {
  readonly address = 0x25;
  private command: number[] = [];
  private bytes: Uint8Array<ArrayBufferLike> = new Uint8Array(9);
  private readIndex = 0;
  private state: { kind: 'idle'; readyAtUs: number } | { kind: 'continuous'; readyAtUs: number } = { kind: 'idle', readyAtUs: 0 };

  constructor(private readonly nowUs: () => number) {}

  startWrite(): boolean {
    this.command = [];
    return this.nowUs() >= this.state.readyAtUs || this.state.kind === 'continuous';
  }

  startRead(): boolean {
    this.readIndex = 0;
    return this.state.kind === 'continuous' && this.nowUs() >= this.state.readyAtUs;
  }

  writeByte(value: number): boolean {
    this.command.push(value & 0xff);
    if (this.command.length === 1) return true;
    if (this.command.length !== 2) return false;
    const command = (this.command[0]! << 8) | this.command[1]!;
    if (command === 0x3ff9) {
      this.state = { kind: 'idle', readyAtUs: this.nowUs() + 500 };
      return true;
    }
    if (command !== 0x3615 || this.state.kind !== 'idle' || this.nowUs() < this.state.readyAtUs) return false;
    this.state = { kind: 'continuous', readyAtUs: this.nowUs() + 8000 };
    return true;
  }

  readByte(): number {
    return this.bytes[this.readIndex++] ?? 0xff;
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    const dpRaw = clampI16(Math.round(observation.sensor_differential_pressure_pa * 60));
    const temperatureRaw = 4000;
    const scale = 60;
    this.bytes = wordsWithCrc([dpRaw & 0xffff, temperatureRaw, scale]);
    if (fault === 'sdp-crc') this.bytes[2] = (this.bytes[2] ?? 0) ^ 0x01;
  }
}

export class Dps310Device extends RegisterDevice {
  private readonly registers = new Uint8Array(256);
  private nextPressureUs = 0;
  private observation: { kind: 'none' } | { kind: 'available'; value: PlantObservation } = { kind: 'none' };
  private fault: SensorFaultKind = 'none';

  constructor(private readonly nowUs: () => number) {
    super(0x77);
    this.registers[0x08] = 0xc0;
    this.registers[0x28] = 0;
    // Virtual calibration: Pcomp = 100000 + 10000 * Praw_sc.
    putDpsCoefficients(this.registers, 100_000, 10_000);
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    this.observation = { kind: 'available', value: observation };
    this.fault = fault;
    this.convert();
  }

  private convert(): void {
    if (this.observation.kind === 'none') return;
    const observation = this.observation.value;
    const fault = this.fault;
    const density = 1.164;
    const gravity = 9.80665;
    const pressurePa = 100_000 + density * gravity * (10.5 - observation.sensor_barometric_input_altitude_m);
    const scaled = (pressurePa - 100_000) / 10_000;
    const raw = clampI24(Math.round(scaled * 524_288));
    const mode = (this.registers[0x08] ?? 0) & 0x07;
    if (fault === 'dps-not-ready') {
      this.registers[0x08] = mode;
      return;
    }
    if (fault === 'dps-stale') {
      this.registers[0x08] = 0xc0 | mode;
      return;
    }
    let pressureReady = (this.registers[0x08] ?? 0) & 0x10;
    if ((mode === 5 || mode === 7) && this.nowUs() >= this.nextPressureUs) {
      const rate = 2 ** (((this.registers[0x06] ?? 0) >>> 4) & 7);
      const periodUs = 1e6 / rate;
      this.nextPressureUs += (Math.floor((this.nowUs() - this.nextPressureUs) / periodUs) + 1) * periodUs;
      putI24Be(this.registers, 0x00, raw);
      putI24Be(this.registers, 0x03, 0);
      pressureReady = 0x10;
    }
    this.registers[0x08] = 0xc0 | pressureReady | mode;
  }

  protected readRegister(register: number): number {
    if (register === 0x08 || register === 0x00) this.convert();
    const value = this.registers[register] ?? 0xff;
    if (register === 0x02) this.registers[0x08] = (this.registers[0x08] ?? 0) & ~0x10;
    return value;
  }

  protected writeRegister(register: number, value: number): void {
    if ((register === 0x06 || register === 0x07) && (value & 0x0f) !== 0) {
      throw new Error('DPS310 virtual transducer supports OSR1 only');
    }
    if (register === 0x09 && value !== 0) {
      throw new Error('DPS310 FIFO/shift/interrupt configuration is outside the virtual transducer contract');
    }
    if (register === 0x08) {
      this.registers[register] = ((this.registers[register] ?? 0) & 0xc0) | (value & 0x07);
      const rate = 2 ** (((this.registers[0x06] ?? 0) >>> 4) & 7);
      this.nextPressureUs = this.nowUs() + 1e6 / rate;
    } else {
      this.registers[register] = value;
    }
  }
}

function wordsWithCrc(words: number[]): Uint8Array {
  const result = new Uint8Array(words.length * 3);
  words.forEach((word, index) => {
    const offset = index * 3;
    result[offset] = (word >>> 8) & 0xff;
    result[offset + 1] = word & 0xff;
    result[offset + 2] = crc8(result.subarray(offset, offset + 2));
  });
  return result;
}

export function crc8(bytes: Uint8Array): number {
  let crc = 0xff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = crc & 0x80 ? ((crc << 1) ^ 0x31) & 0xff : (crc << 1) & 0xff;
    }
  }
  return crc;
}

function putDpsCoefficients(registers: Uint8Array, c00: number, c10: number): void {
  const offset = 0x10;
  registers[offset + 3] = (c00 >>> 12) & 0xff;
  registers[offset + 4] = (c00 >>> 4) & 0xff;
  registers[offset + 5] = ((c00 & 0x0f) << 4) | ((c10 >>> 16) & 0x0f);
  registers[offset + 6] = (c10 >>> 8) & 0xff;
  registers[offset + 7] = c10 & 0xff;
}

function putI16Le(registers: Uint8Array, offset: number, value: number): void {
  registers[offset] = value & 0xff;
  registers[offset + 1] = (value >>> 8) & 0xff;
}

function putI24Be(registers: Uint8Array, offset: number, value: number): void {
  registers[offset] = (value >>> 16) & 0xff;
  registers[offset + 1] = (value >>> 8) & 0xff;
  registers[offset + 2] = value & 0xff;
}

function clampI16(value: number): number {
  return Math.min(32767, Math.max(-32768, value));
}

function clampI24(value: number): number {
  return Math.min(8_388_607, Math.max(-8_388_608, value));
}

function modulo(value: number, modulus: number): number {
  return ((value % modulus) + modulus) % modulus;
}
