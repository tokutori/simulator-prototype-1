import type { Simulator } from 'rp2040js';

/** Scoped UART1 TX completion model layered over rp2040js 1.3.3, whose UARTDR
 * callback is otherwise immediate and whose TX FIFO flags are constant.
 * Validates configuration and supplies FIFO backpressure to the actual HAL.
 * This is a byte/serial-time model, NOT physical GPIO voltage waveforms.
 */
export function attachUartRecorder(simulator: Simulator, receive: (byte: number) => void): void {
  const mcu = simulator.rp2040;
  const uart = mcu.uart[1];
  if (!uart) throw new Error('required UART1 recorder is missing');
  const originalRead = uart.readUint32.bind(uart);
  const originalWrite = uart.writeUint32.bind(uart);
  const pending: number[] = [];
  let shifting = false;
  const capacity = (): number => uart.fifosEnabled ? 32 : 1;
  const validate = (): number => {
    const cr = originalRead(0x30), lcr = originalRead(0x2c);
    const gpio = mcu.readUint32(0x40014044);
    if ((cr & 0x101) !== 0x101 || (cr & 0xc086) !== 0) throw new Error('UART1 recorder requires UARTEN/TXE and no IrDA/flow control/loopback');
    if ((originalRead(0x38) & 0x20) !== 0) throw new Error('UART1 TX interrupts are outside the recorder contract');
    // HAL enables UART DMA request gates during ordinary initialization. Only
    // actual active DMA transfers, not those harmless gates, are unsupported.
    for (let channel = 0; channel < 12; channel++) {
      const base = 0x50000000 + channel * 0x40;
      if ((mcu.readUint32(base + 0x0c) & 1) !== 0 && mcu.readUint32(base + 0x08) !== 0
        && mcu.readUint32(base + 0x04) === 0x40038000) {
        throw new Error('UART1 active TX DMA is outside the recorder contract');
      }
    }
    if ((gpio & 0x331f) !== 2 || (mcu.readUint32(0x4001c024) & 0x80) !== 0) throw new Error('UART1 recorder requires enabled GPIO8 UART TX mux without overrides');
    if ((lcr & 0xef) !== 0x60) throw new Error('UART1 recorder requires 8N1 without break');
    const baud = uart.baudRate;
    if (!Number.isFinite(baud) || Math.abs(baud - 1_000_000) > 10_000) throw new Error(`UART1 recorder baud incompatible: ${baud}`);
    return 10 * 1e9 / baud;
  };
  const startByte = (byte: number): void => {
    shifting = true;
    const durationNs = validate();
    const fifoEnabled = uart.fifosEnabled;
    simulator.clock.createAlarm(() => {
      if (validate() !== durationNs || fifoEnabled !== uart.fifosEnabled) {
        throw new Error('UART1 baud/FIFO configuration changed during transmission');
      }
      receive(byte);
      const next = pending.shift();
      if (next === undefined) shifting = false;
      else startByte(next);
    }).schedule(durationNs);
  };
  // Preserve RX flags and register behaviour; override only modeled TX flags.
  uart.readUint32 = offset => {
    const value = originalRead(offset);
    if (offset !== 0x18) return value;
    return (value & ~0xa8) | (shifting ? 0x08 : 0)
      | (pending.length >= capacity() ? 0x20 : 0) | (pending.length === 0 ? 0x80 : 0);
  };
  // Reject unsupported serial/interrupt modes on their MMIO configuration.
  // DMACR is not stored upstream and HAL sets it even without using DMA.
  uart.writeUint32 = (offset, value) => {
    if ((offset === 0x30 && (value & 0x06) !== 0)
      || (offset === 0x38 && (value & 0x20) !== 0)) {
      throw new Error('UART1 IrDA/TX interrupt is outside the recorder contract');
    }
    originalWrite(offset, value);
  };
  uart.onByte = byte => {
    validate();
    if (!shifting) startByte(byte);
    else {
      if (pending.length >= capacity()) throw new Error('UART1 TX FIFO overrun: firmware ignored TXFF');
      pending.push(byte);
    }
  };
}
