export interface RunIdentity {
  uf2_sha256: string;
  model_sha256: string;
  plant_sha256: string;
}

export function isRunIdentity(value: unknown): value is RunIdentity {
  if (!value || typeof value !== "object") return false;
  const candidate = value as Partial<RunIdentity>;
  return [candidate.uf2_sha256, candidate.model_sha256, candidate.plant_sha256]
    .every(hash => typeof hash === "string" && /^[a-f0-9]{64}$/.test(hash));
}

export type ControlTelemetry = { tag: "unavailable" } | {
  tag: "firmware";
  runIdentity: RunIdentity;
  sequence: number;
  timeUs: number;
  releaseMcuTimeUs: number;
  plantIntervalStartS: number;
  automaticValid: boolean;
  safeElevatorCommandRad: number;
  safeRudderCommandRad: number;
  observedElevatorCommandRad: number;
  observedRudderCommandRad: number;
  elevatorPwmSampleTimeUs: number;
  rudderPwmSampleTimeUs: number;
};

export interface FlightFrame {
  controlTelemetry: ControlTelemetry;
  timeS: number;
  northM: number;
  eastM: number;
  altitudeM: number;
  rollRad: number;
  pitchRad: number;
  yawRad: number;
  flightPathRad: number;
  airspeedMps: number;
  alphaRad: number;
  elevatorRad: number;
  rudderRad: number;
  pilotElevator: number;
  pilotRudder: number;
  autonomy: number;
  manualElevatorCommandRad: number;
  manualRudderCommandRad: number;
  automaticElevatorCommandRad: number;
  automaticRudderCommandRad: number;
  mixedElevatorCommandRad: number;
  mixedRudderCommandRad: number;
  surfaceContact: boolean;
}

export interface InteractiveObservation {
  run_identity: RunIdentity;
  firmware_sequence: number;
  firmware_time_us: number;
  release_mcu_time_us: number;
  plant_interval_start_s: number;
  automatic_valid: boolean;
  safe_elevator_command_rad: number;
  safe_rudder_command_rad: number;
  observed_elevator_command_rad: number;
  observed_rudder_command_rad: number;
  elevator_pwm_sample_time_us: number;
  rudder_pwm_sample_time_us: number;
  time_s: number;
  north_m: number;
  east_m: number;
  altitude_m: number;
  roll_rad: number;
  pitch_rad: number;
  yaw_rad: number;
  flight_path_rad: number;
  elevator_rad: number;
  rudder_rad: number;
  sensor_airspeed_mps: number;
  sensor_alpha_rad: number;
  pilot_elevator: number;
  pilot_rudder: number;
  autonomy: number;
  manual_elevator_command_rad: number;
  manual_rudder_command_rad: number;
  automatic_elevator_command_rad: number;
  automatic_rudder_command_rad: number;
  mixed_elevator_command_rad: number;
  mixed_rudder_command_rad: number;
  surface_contact: boolean;
  backend: "rp2040js-actual-uf2";
  emulation: {
    timing_acceleration: 1;
    processing_ms: number;
    processing_average_ms: number;
    real_time_ratio: number;
    lag_ms: number;
    deadline_missed: boolean;
    real_time: boolean;
    timing_validated: false;
  };
}

export interface PilotCommandMessage {
  pilot_elevator: number;
  pilot_rudder: number;
  autonomy: number;
  elevator_input_kind: "analog" | "buttons";
  rudder_input_kind: "analog" | "buttons";
}

export type CameraMode = "cockpit" | "chase";
