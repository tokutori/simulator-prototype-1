import {
  BufferGeometry,
  CanvasTexture,
  Group,
  LineBasicMaterial,
  LineLoop,
  LinearFilter,
  Sprite,
  SpriteMaterial,
  SRGBColorSpace,
  Vector3,
} from "three";

export interface DistanceRingSpec {
  radiusM: number;
  major: boolean;
  label?: string;
}

export function distanceRingSpecs(maximumM = 900, intervalM = 25): DistanceRingSpec[] {
  const specs: DistanceRingSpec[] = [];
  for (let radiusM = intervalM; radiusM <= maximumM; radiusM += intervalM) {
    const major = radiusM % 50 === 0;
    specs.push({ radiusM, major, label: major ? `${radiusM} m` : undefined });
  }
  return specs;
}

export function createDistanceRings(): Group {
  const root = new Group();
  root.name = "start-distance-rings";
  const minorMaterial = new LineBasicMaterial({
    color: 0x83d8df,
    transparent: true,
    opacity: 0.3,
    depthWrite: false,
  });
  const majorMaterial = new LineBasicMaterial({
    color: 0xa9f2ec,
    transparent: true,
    opacity: 0.68,
    depthWrite: false,
  });

  for (const spec of distanceRingSpecs()) {
    const points: Vector3[] = [];
    const segments = spec.major ? 256 : 160;
    for (let index = 0; index < segments; index += 1) {
      const angle = (index / segments) * Math.PI * 2;
      points.push(new Vector3(Math.sin(angle) * spec.radiusM, 0.42, -Math.cos(angle) * spec.radiusM));
    }
    const ring = new LineLoop(
      new BufferGeometry().setFromPoints(points),
      spec.major ? majorMaterial : minorMaterial,
    );
    ring.renderOrder = 2;
    root.add(ring);

    if (spec.label) {
      const label = createDistanceLabel(spec.label);
      label.position.set(-7.5, 1.3, -spec.radiusM);
      label.scale.set(10, 2.5, 1);
      label.renderOrder = 3;
      root.add(label);
    }
  }
  return root;
}

function createDistanceLabel(text: string): Sprite {
  const canvas = document.createElement("canvas");
  canvas.width = 512;
  canvas.height = 128;
  const context = canvas.getContext("2d");
  if (!context) throw new Error("2D canvas is unavailable");
  context.fillStyle = "rgba(3, 22, 29, 0.78)";
  context.fillRect(4, 4, 504, 120);
  context.strokeStyle = "rgba(169, 242, 236, 0.92)";
  context.lineWidth = 6;
  context.strokeRect(7, 7, 498, 114);
  context.fillStyle = "#dffbf4";
  context.font = "500 58px ui-monospace, monospace";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(text, 256, 66);

  const texture = new CanvasTexture(canvas);
  texture.colorSpace = SRGBColorSpace;
  texture.minFilter = LinearFilter;
  const material = new SpriteMaterial({
    map: texture,
    transparent: true,
    depthWrite: false,
    fog: true,
  });
  return new Sprite(material);
}
