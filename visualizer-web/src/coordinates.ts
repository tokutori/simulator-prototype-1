import { Matrix4, Quaternion, Vector3 } from "three";

const nedToThree = new Matrix4().set(
  0, 1, 0, 0,
  0, 0, -1, 0,
  -1, 0, 0, 0,
  0, 0, 0, 1,
);
const threeToNed = nedToThree.clone().transpose();

export function nedPositionToThree(northM: number, eastM: number, altitudeM: number): Vector3 {
  return new Vector3(eastM, altitudeM, -northM);
}

export function nedEulerToThreeQuaternion(
  rollRad: number,
  pitchRad: number,
  yawRad: number,
): Quaternion {
  const bodyToNed = aerospaceRotation(rollRad, pitchRad, yawRad);
  const bodyThreeToWorldThree = nedToThree.clone().multiply(bodyToNed).multiply(threeToNed);
  return new Quaternion().setFromRotationMatrix(bodyThreeToWorldThree).normalize();
}

function aerospaceRotation(rollRad: number, pitchRad: number, yawRad: number): Matrix4 {
  const [sr, cr] = [Math.sin(rollRad), Math.cos(rollRad)];
  const [sp, cp] = [Math.sin(pitchRad), Math.cos(pitchRad)];
  const [sy, cy] = [Math.sin(yawRad), Math.cos(yawRad)];
  return new Matrix4().set(
    cp * cy,
    sr * sp * cy - cr * sy,
    cr * sp * cy + sr * sy,
    0,
    cp * sy,
    sr * sp * sy + cr * cy,
    cr * sp * sy - sr * cy,
    0,
    -sp,
    sr * cp,
    cr * cp,
    0,
    0,
    0,
    0,
    1,
  );
}
