import { Quaternion, Vector3 } from "three";

/** Both eye and target derive from one smoothed anchor; reset on timeline discontinuities. */
export class ChaseCamera {
  private anchor: Vector3 | undefined;
  private heading = new Vector3(0, 0, -1);

  reset(): void { this.anchor = undefined; }

  update(position: Vector3, attitude: Quaternion, deltaS: number): { eye: Vector3; target: Vector3 } {
    const forward = new Vector3(0, 0, -1).applyQuaternion(attitude);
    forward.y = 0;
    if (forward.lengthSq() < 1e-8) forward.copy(this.heading);
    else forward.normalize();
    if (!this.anchor) {
      this.anchor = position.clone();
      this.heading.copy(forward);
    } else {
      const fraction = 1 - Math.exp(-Math.max(0, deltaS) * 4.5);
      this.anchor.lerp(position, fraction);
      const difference = Math.atan2(this.heading.z * forward.x - this.heading.x * forward.z, this.heading.dot(forward));
      this.heading.applyAxisAngle(new Vector3(0, 1, 0), difference * fraction);
    }
    return {
      eye: this.anchor.clone().addScaledVector(this.heading, -19).add(new Vector3(0, 6, 0)),
      target: this.anchor.clone().addScaledVector(this.heading, 4),
    };
  }
}
