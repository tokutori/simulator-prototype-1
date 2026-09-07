# Electrical boundary corrections

The virtual platform executes the production UF2. This does not by itself prove
that every electrical/peripheral configuration is correct. The following
contracts are now checked at the external-device boundary.

## Differential-pressure recovery

Firmware sends SDP810 stop `0x3ff9`, waits at least 500 us using its hardware timer,
then sends `0x3615`. This also works when another critical sensor failed while
SDP810 remained in continuous measurement. The device emulator NACKs reads before
start and during the first 8 ms, repeated starts while measuring, unsupported
commands, and writes during the stop delay. ACK results propagate through I2C0.

Source: [Sensirion SDP8xx digital datasheet, sections 6.3.1–6.3.2](https://sensirion.com/media/documents/90500156/6167E43B/Sensirion_Differential_Pressure_Datasheet_SDP8xx_Digital.pdf).

Only the command subset used by this firmware is modeled. Soft reset, product ID,
sleep and triggered modes are not implemented. The pressure input remains the
sampled plant output; sensor-internal averaging/transient thermal accuracy is not
claimed to be validated.

## Barometer conversion clock

DPS310 conversion cadence uses the virtual clock and the configured pressure
rate, not the number of plant updates or fault-injection calls. Conversion occurs
only in supported continuous pressure modes. Repeated reads do not create new
conversions; data-ready clears after the pressure result is consumed. Measurement
frames explicitly distinguish a new conversion from a held pressure value.
The MCU device consumes the continuous biased/quantized pressure input, not the
host simulator's already sampled channel: there is exactly one acquisition clock
on each execution path, avoiding double sample-and-hold latency.

This remains a scoped device model, not a complete DPS310 implementation (single
measurement, FIFO, temperature conversion dynamics and all oversampling modes
are not modeled).

## Servo signal contract

Both live and batch execution observe GPIO16/17 transitions produced by rp2040js.
A target is accepted only after a complete pulse with a measured 19–21 ms period
and 990–2010 us high time (tolerance around the configured 50 Hz / 1000–2000 us
interface). The angle mapping remains the installation-specific ±10 degree
calibration, not the servo's maximum mechanical travel.

After 60 ms without a valid pulse the decoder reports signal loss and holds the
last target. Before any valid pulse the target is neutral. Holding is an explicit
actuator-model assumption requiring physical characterization; it is not a
universal claim about servo loss-of-signal behavior. Missing PWM prevents
preflight release. Tests write actual rp2040js MMIO and verify that wrong GPIO
mux, disabled PWM, wrong divider and wrong TOP do not satisfy this contract.

Analog thresholds, wiring faults, servo power/brownout, cycle-accurate scheduling,
and physical timing remain outside this virtual-platform validation. Keep
`timing_validated=false` until physical HIL establishes the relevant timing.

## Blocked I2C transactions and watchdog boundary

The blocking HAL I2C driver can wait indefinitely for hardware completion; a NACK
fault does not test a stuck transaction. Firmware therefore starts a 40 ms hardware
watchdog after the intentional 650 ms sensor power-on wait and before sensor
initialization. Only a completed control iteration feeds it. The selected 40 ms
prototype budget exceeds the nominal 10 ms control period and initial 20 ms
settling wait; actual worst-case latency still requires hardware measurement.

On physical watchdog reboot the firmware inhibits automatic rearming before GPIO
initialization and emits no servo pulses; power-cycle/manual recovery is required.
This does not command a failsafe angle or establish a safe aircraft state. Servo
position during reset and after signal loss remains a physical validation requirement.

rp2040js implements the actual watchdog timer but its default reset callback only
logs a warning. Both adapters install a callback that explicitly aborts the
experiment when that hardware timer requests reset. They do not invent a reset,
calibration recovery or successful rearm. `i2c-stall` injection deliberately omits
transaction completion to exercise this path. A real GPIO/I2C/peripheral reset
sequence and electrical bus release remain outside this simulator's implemented
reset model and require HIL.

Sources: [RP2040 HAL Watchdog API](https://docs.rs/rp2040-hal/0.12.0/rp2040_hal/watchdog/struct.Watchdog.html),
[rp2040js 1.3.3 watchdog implementation](https://github.com/wokwi/rp2040js/blob/v1.3.3/src/peripherals/watchdog.ts).
