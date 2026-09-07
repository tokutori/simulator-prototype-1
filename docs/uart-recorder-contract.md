# UART flight recorder boundary

The recorder consumes the production firmware's actual UARTDR bytes. It no longer
treats rp2040js's immediate `onByte` callback as successful serial transmission.
Both live and batch adapters attach the same UART1 transmit model.

The accepted installation is GPIO8, UART1 enabled with TX enabled, no GPIO output
overrides or pad output-disable, no loopback/hardware flow control, 8N1/no break,
and a baud rate within 1% of 1 Mbaud. The baud calculation uses rp2040js's current
peripheral clock and actual divisor registers. Wrong configurations fail the
experiment rather than fabricating valid recorder data.

The scoped adapter fills the missing transmit FIFO behaviour in rp2040js 1.3.3:
32 FIFO slots (one holding slot with FEN clear), a separate shift register,
UARTFR TXFF/TXFE/BUSY polling, and ordered byte-completion alarms at ten bit
times per byte. This makes the production blocking HAL wait when TXFF is set.
Writes past capacity are explicit overrun errors. RX register behaviour is
delegated unchanged to rp2040js. A configuration invalidated before byte
completion cannot deliver that byte. At exact 1 Mbaud a contiguous 56-byte frame
requires 560 microseconds, and the CRC parser sees no complete record earlier.

Negative tests use actual RP2040 MMIO, including the formerly successful
unconfigured UARTDR write. Ordered delivery, FIFO backpressure/overflow, disabled
FIFO and mid-byte invalidation have regression tests. An actual-UF2 30-step
smoke at CPU time multiplier 1 also passed with zero deadline violations after
the adapter was attached (UF2 `3f960cef095543be91b73f766077d6e5d9df467cc14130226df163ca8118035b`).
Rebuild/re-run this smoke when firmware changes; this hash is not a claim about
future firmware revisions.

This is **not** a physical voltage waveform or full PL011 emulator. TX DMA,
transmit interrupts, CTS/RTS, receiver oscillator error, analog edge timing,
wire faults, and resetting/reconfiguring an active UART are not validated here.
IrDA, TX interrupt enable and active DMA descriptors targeting UART1DR are
explicitly rejected. Merely enabling UART DMA request gates is allowed because
the production HAL does this even for blocking CPU writes. The upstream emulator
does not store DMACR, so checking that register afterward is insufficient.
The scoped implementation rejects incompatible use; new production usage needs
a contract extension. GPIO configuration is checked at enqueue/completion, not
continuously between these events. Whole-system cycle timing remains unvalidated.

Sources: [RP2040 datasheet, UART functional overview and UARTFR/UARTLCR_H/UARTCR registers](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf),
[rp2040js 1.3.3 UART source](https://github.com/wokwi/rp2040js/blob/v1.3.3/src/peripherals/uart.ts).
