export interface FlightFrame {
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
}

export interface PilotCommandMessage {
  pilot_elevator: number;
  pilot_rudder: number;
  autonomy: number;
}

export type CameraMode = "cockpit" | "chase";
export type AppMode = "replay" | "live";
