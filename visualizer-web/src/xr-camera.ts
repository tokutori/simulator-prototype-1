import type { Object3D, Quaternion, Vector3 } from "three";

const cockpitEyeRightM = 0;
const cockpitEyeUpM = 0.11;
const cockpitEyeForwardM = 1.18;

export function setXrCockpitRig(
  rig: Object3D,
  aircraftPosition: Vector3,
  aircraftAttitude: Quaternion,
): void {
  rig.position
    .set(cockpitEyeRightM, cockpitEyeUpM, -cockpitEyeForwardM)
    .applyQuaternion(aircraftAttitude)
    .add(aircraftPosition);
  rig.quaternion.copy(aircraftAttitude);
  rig.scale.set(1, 1, 1);
  rig.updateMatrixWorld(true);
}

export function resetXrRig(rig: Object3D): void {
  rig.position.set(0, 0, 0);
  rig.quaternion.identity();
  rig.scale.set(1, 1, 1);
  rig.updateMatrixWorld(true);
}
