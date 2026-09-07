/** Actual UART1 records. No controller-output reconstruction from blended PWM. */
export interface FirmwareRecord {
  sequence: number;
  timeUs: number;
  automaticValid: boolean;
  pilotElevator: number;
  pilotRudder: number;
  autonomy: number;
  automaticElevator: number;
  automaticRudder: number;
  safeElevator: number;
  safeRudder: number;
  mixedElevator: number;
  mixedRudder: number;
}

export type RecordState = { tag: 'awaiting' } | { tag: 'received'; record: FirmwareRecord };

export function crc32(bytes: Uint8Array): number {
  let crc = 0xffff_ffff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb8_8320 : 0);
  }
  return (crc ^ 0xffff_ffff) >>> 0;
}

/** Bounded resynchronizing parser: bad/truncated frames cannot become measurements. */
export class FirmwareTelemetry {
  private bytes: number[] = [];
  state: RecordState = { tag: 'awaiting' };
  rejectedFrames = 0;

  receive(byte: number): void {
    this.bytes.push(byte & 255);
    const magic = [70, 66, 87, 50];
    while (this.bytes.length > 0 && !this.bytes.slice(0, 4).every((value, index) => value === magic[index])) this.bytes.shift();
    if (this.bytes.length < 56) return;
    const data = Uint8Array.from(this.bytes);
    const view = new DataView(data.buffer);
    const values = Array.from({ length: 9 }, (_, index) => view.getFloat32(16 + index * 4, true));
    if (view.getUint32(52, true) !== crc32(data.subarray(0, 52)) || !values.every(Number.isFinite) || view.getUint32(12, true) > 1) {
      this.rejectedFrames++;
      this.bytes.shift();
      return;
    }
    this.state = { tag: 'received', record: {
      sequence: view.getUint32(4, true), timeUs: view.getUint32(8, true),
      automaticValid: view.getUint32(12, true) === 1,
      pilotElevator: values[0]!, pilotRudder: values[1]!, autonomy: values[2]!,
      automaticElevator: values[3]!, automaticRudder: values[4]!, safeElevator: values[5]!,
      mixedElevator: values[6]!, mixedRudder: values[7]!,
      safeRudder: values[8]!,
    } };
    this.bytes = [];
  }

  requireFresh(nowUs: number, maximumAgeUs = 50_000): FirmwareRecord {
    if (this.state.tag !== 'received') throw new Error('firmware UART telemetry unavailable');
    const record = this.state.record;
    const ageUs = ((Math.floor(nowUs) >>> 0) - record.timeUs) >>> 0;
    if (ageUs > maximumAgeUs) throw new Error(`firmware UART telemetry stale: ${ageUs} us`);
    return record;
  }
}
