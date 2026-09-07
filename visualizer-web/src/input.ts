export type AxisSource = "keyboard-buttons" | "gamepad-axis" | "gamepad-buttons";

export interface AxisBinding {
  source: AxisSource;
  negativeKey: string;
  positiveKey: string;
  gamepadAxis: number;
  negativeButton: number;
  positiveButton: number;
  invert: boolean;
}

export interface InputSettings {
  elevator: AxisBinding;
  rudder: AxisBinding;
  gamepadIndex: number;
  deadZone: number;
  responseExponent: number;
}

export const defaultInputSettings: InputSettings = {
  elevator: {
    source: "keyboard-buttons",
    negativeKey: "KeyW",
    positiveKey: "KeyS",
    gamepadAxis: 1,
    negativeButton: 12,
    positiveButton: 13,
    invert: false,
  },
  rudder: {
    source: "keyboard-buttons",
    negativeKey: "KeyA",
    positiveKey: "KeyD",
    gamepadAxis: 0,
    negativeButton: 14,
    positiveButton: 15,
    invert: false,
  },
  gamepadIndex: 0,
  deadZone: 0.08,
  responseExponent: 1.35,
};

export class PilotInput {
  readonly pressedCodes = new Set<string>();
  settings: InputSettings;
  elevator = 0;
  rudder = 0;

  constructor(settings: InputSettings) {
    this.settings = settings;
  }

  update(_deltaS: number, gamepad: Gamepad | null): void {
    this.elevator = this.updateAxis(
      this.settings.elevator,
      gamepad,
    );
    this.rudder = this.updateAxis(this.settings.rudder, gamepad);
  }

  clear(): void {
    this.pressedCodes.clear();
    this.elevator = 0;
    this.rudder = 0;
  }

  private updateAxis(
    binding: AxisBinding,
    gamepad: Gamepad | null,
  ): number {
    if (binding.source === "gamepad-axis") {
      const raw = gamepad?.axes[binding.gamepadAxis];
      if (raw === undefined || !Number.isFinite(raw)) {
        return 0;
      }
      const shaped = shapeAnalog(raw, this.settings.deadZone, this.settings.responseExponent);
      return binding.invert ? -shaped : shaped;
    }

    const negative =
      binding.source === "keyboard-buttons"
        ? this.pressedCodes.has(binding.negativeKey)
        : Boolean(gamepad?.buttons[binding.negativeButton]?.pressed);
    const positive =
      binding.source === "keyboard-buttons"
        ? this.pressedCodes.has(binding.positiveKey)
        : Boolean(gamepad?.buttons[binding.positiveButton]?.pressed);
    const rawTarget = negative === positive ? 0 : negative ? -1 : 1;
    const target = binding.invert ? -rawTarget : rawTarget;
    // Physical GPIO buttons are binary. Input shaping belongs in production firmware.
    return target;
  }
}

export function shapeAnalog(value: number, deadZone: number, exponent: number): number {
  if (!Number.isFinite(value) || !Number.isFinite(deadZone) || !Number.isFinite(exponent)) {
    return 0;
  }
  const clamped = Math.max(-1, Math.min(1, value));
  const safeDeadZone = Math.max(0, Math.min(0.95, deadZone));
  const magnitude = Math.abs(clamped);
  if (magnitude <= safeDeadZone) {
    return 0;
  }
  const normalized = (magnitude - safeDeadZone) / (1 - safeDeadZone);
  return Math.sign(clamped) * normalized ** Math.max(0.1, exponent);
}

export function approach(current: number, target: number, maximumDelta: number): number {
  if (current < target) {
    return Math.min(target, current + maximumDelta);
  }
  return Math.max(target, current - maximumDelta);
}

export function sanitizeSettings(value: unknown): InputSettings {
  const candidate = value as Partial<InputSettings> | null;
  if (!candidate || typeof candidate !== "object") {
    return structuredClone(defaultInputSettings);
  }
  return {
    elevator: sanitizeBinding(candidate.elevator, defaultInputSettings.elevator),
    rudder: sanitizeBinding(candidate.rudder, defaultInputSettings.rudder),
    gamepadIndex: integerWithin(candidate.gamepadIndex, 0, 3, 0),
    deadZone: numberWithin(candidate.deadZone, 0, 0.5, defaultInputSettings.deadZone),
    responseExponent: numberWithin(
      candidate.responseExponent,
      0.5,
      3,
      defaultInputSettings.responseExponent,
    ),
  };
}

function sanitizeBinding(value: unknown, fallback: AxisBinding): AxisBinding {
  const candidate = value as Partial<AxisBinding> | null;
  const sources: AxisSource[] = ["keyboard-buttons", "gamepad-axis", "gamepad-buttons"];
  return {
    source: sources.includes(candidate?.source as AxisSource)
      ? (candidate?.source as AxisSource)
      : fallback.source,
    negativeKey: validCode(candidate?.negativeKey, fallback.negativeKey),
    positiveKey: validCode(candidate?.positiveKey, fallback.positiveKey),
    gamepadAxis: integerWithin(candidate?.gamepadAxis, 0, 15, fallback.gamepadAxis),
    negativeButton: integerWithin(candidate?.negativeButton, 0, 31, fallback.negativeButton),
    positiveButton: integerWithin(candidate?.positiveButton, 0, 31, fallback.positiveButton),
    invert: typeof candidate?.invert === "boolean" ? candidate.invert : fallback.invert,
  };
}

function validCode(value: unknown, fallback: string): string {
  return typeof value === "string" && /^[A-Za-z][A-Za-z0-9]{1,31}$/.test(value)
    ? value
    : fallback;
}

function numberWithin(value: unknown, minimum: number, maximum: number, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) && value >= minimum && value <= maximum
    ? value
    : fallback;
}

function integerWithin(value: unknown, minimum: number, maximum: number, fallback: number): number {
  return typeof value === "number" && Number.isInteger(value) && value >= minimum && value <= maximum
    ? value
    : fallback;
}
