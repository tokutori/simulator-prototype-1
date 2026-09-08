import { Box3, Quaternion, Ray, Vector3 } from "three";
import type { CameraMode } from "./types.ts";

// Representative points cover the cockpit, wing and tail of the visual silhouette.
// Only presentation occluders are affected; aircraft and flight dynamics are untouched.
const aircraftPoints = [[0, 0, 0], [0, -0.3, -1.2], [0, 0.5, 6.3], [-12.5, 0, 0], [12.5, 0, 0]] as const;

export function launchPlatformOpacity(mode: CameraMode, eye: Vector3, position: Vector3,
  attitude: Quaternion, platformBounds: Box3): number {
  if (mode !== "chase") return 1;
  // Includes the near clipping clearance; avoids popping when the eye grazes an edge.
  const bounds = platformBounds.clone().expandByScalar(0.15);
  if (bounds.containsPoint(eye)) return 0.08;
  for (const point of aircraftPoints) {
    const target = new Vector3(...point).applyQuaternion(attitude).add(position);
    if (bounds.containsPoint(target)) return 0.08;
    const direction = target.clone().sub(eye);
    const distance = direction.length();
    if (distance === 0) continue;
    const intersection = new Ray(eye, direction.divideScalar(distance)).intersectBox(bounds, new Vector3());
    if (intersection && intersection.distanceTo(eye) <= distance) return 0.08;
  }
  return 1;
}
