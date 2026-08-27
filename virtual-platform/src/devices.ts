export interface PlantObservation {
  time_s: number;
  north_m: number;
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
  sensor_alpha_rad: number;
  aero_in_range: boolean;
  surface_contact: boolean;
}

export interface I2cDevice {
  readonly address: number;
  startWrite(): void;
  startRead(): void;
  writeByte(value: number): void;
  readByte(): number;
}

export type SensorFaultKind =
  | 'none'
  | 'bno-status'
  | 'as5600-magnet'
  | 'sdp-crc'
  | 'sdp-nack'
  | 'dps-stale'
  | 'dps-not-ready';

abstract class RegisterDevice implements I2cDevice {
  private pointer = 0;
  private expectingPointer = true;

  constructor(readonly address: number) {}

  startWrite(): void {
    this.expectingPointer = true;
  }

  startRead(): void {}

  writeByte(value: number): void {
    if (this.expectingPointer) {
      this.pointer = value & 0xff;
      this.expectingPointer = false;
    } else {
      this.writeRegister(this.pointer, value & 0xff);
      this.pointer = (this.pointer + 1) & 0xff;
    }
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

  constructor() {
    super(0x28);
    this.registers[0x00] = 0xa0;
    this.registers[0x39] = 0x05;
    this.registers[0x3a] = 0x00;
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    const gyroXRaw = clampI16(Math.round(observation.sensor_roll_rate_rad_s * 180 / Math.PI * 16));
    const gyroRaw = clampI16(Math.round(observation.sensor_pitch_rate_rad_s * 180 / Math.PI * 16));
    const gyroZRaw = clampI16(Math.round(observation.sensor_yaw_rate_rad_s * 180 / Math.PI * 16));
    const headingRaw = clampI16(Math.round(observation.sensor_yaw_rad * 180 / Math.PI * 16));
    const rollRaw = clampI16(Math.round(observation.sensor_roll_rad * 180 / Math.PI * 16));
    const pitchRaw = clampI16(Math.round(observation.sensor_pitch_rad * 180 / Math.PI * 16));
    putI16Le(this.registers, 0x14, gyroXRaw);
    putI16Le(this.registers, 0x16, gyroRaw);
    putI16Le(this.registers, 0x18, gyroZRaw);
    putI16Le(this.registers, 0x1a, headingRaw);
    putI16Le(this.registers, 0x1c, rollRaw);
    putI16Le(this.registers, 0x1e, pitchRaw);
    this.registers[0x39] = fault === 'bno-status' ? 0x01 : 0x05;
    this.registers[0x3a] = fault === 'bno-status' ? 0x09 : 0x00;
  }

  protected readRegister(register: number): number {
    return this.registers[register] ?? 0xff;
  }

  protected writeRegister(register: number, value: number): void {
    this.registers[register] = value;
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

  startWrite(): void {
    this.command = [];
  }

  startRead(): void {
    this.readIndex = 0;
  }

  writeByte(value: number): void {
    this.command.push(value & 0xff);
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
  private pressureSampleBucket = -1;
  private updateCount = 0;

  constructor() {
    super(0x77);
    this.registers[0x08] = 0xc0;
    this.registers[0x28] = 0;
    // Virtual calibration: Pcomp = 100000 + 10000 * Praw_sc.
    putDpsCoefficients(this.registers, 100_000, 10_000);
  }

  update(observation: PlantObservation, fault: SensorFaultKind = 'none'): void {
    const density = 1.164;
    const gravity = 9.80665;
    const pressurePa = 100_000 + density * gravity * (10.5 - observation.sensor_barometric_altitude_m);
    const scaled = (pressurePa - 100_000) / 10_000;
    const raw = clampI24(Math.round(scaled * 524_288));
    const mode = (this.registers[0x08] ?? 0) & 0x07;
    const sampleBucket = Math.floor(this.updateCount * 32 / 100);
    this.updateCount += 1;
    if (fault === 'dps-not-ready') {
      this.registers[0x08] = mode;
      return;
    }
    if (fault === 'dps-stale') {
      this.registers[0x08] = 0xc0 | mode;
      return;
    }
    putI24Be(this.registers, 0x00, raw);
    putI24Be(this.registers, 0x03, 0);
    const pressureReady = sampleBucket !== this.pressureSampleBucket
      ? 0x10
      : (this.registers[0x08] ?? 0) & 0x10;
    this.pressureSampleBucket = sampleBucket;
    this.registers[0x08] = 0xc0 | pressureReady | mode;
  }

  protected readRegister(register: number): number {
    const value = this.registers[register] ?? 0xff;
    if (register === 0x02) this.registers[0x08] = (this.registers[0x08] ?? 0) & ~0x10;
    return value;
  }

  protected writeRegister(register: number, value: number): void {
    if (register === 0x08) {
      this.registers[register] = ((this.registers[register] ?? 0) & 0xf0) | (value & 0x07);
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
