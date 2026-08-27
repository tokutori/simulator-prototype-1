import {
  BoxGeometry,
  CylinderGeometry,
  DoubleSide,
  Group,
  Mesh,
  MeshStandardMaterial,
  Object3D,
} from "three";

export interface AircraftVisual {
  root: Group;
  elevator: Object3D;
  rudder: Object3D;
}

export function createAircraft(): AircraftVisual {
  const root = new Group();
  root.name = "QX-18 public-data silhouette";

  const wingMaterial = new MeshStandardMaterial({
    color: 0xf2f4ee,
    roughness: 0.72,
    metalness: 0.02,
  });
  const frameMaterial = new MeshStandardMaterial({
    color: 0x273039,
    roughness: 0.65,
    metalness: 0.2,
  });
  const controlMaterial = new MeshStandardMaterial({
    color: 0xe49332,
    roughness: 0.65,
    side: DoubleSide,
  });

  const mainWing = new Mesh(new BoxGeometry(25.1, 0.09, 0.76), wingMaterial);
  mainWing.position.z = 0.1;
  mainWing.castShadow = true;
  root.add(mainWing);

  const wingTips = [-1, 1].map((side) => {
    const tip = new Mesh(new BoxGeometry(0.12, 0.24, 0.72), controlMaterial);
    tip.position.set(side * 12.45, 0.08, 0.1);
    root.add(tip);
    return tip;
  });
  void wingTips;

  const boom = new Mesh(new CylinderGeometry(0.055, 0.075, 6.7, 12), frameMaterial);
  boom.rotation.x = Math.PI / 2;
  boom.position.set(0, -0.06, 2.65);
  boom.castShadow = true;
  root.add(boom);

  const pod = new Mesh(new BoxGeometry(0.58, 0.68, 1.75), frameMaterial);
  pod.position.set(0, -0.32, -0.65);
  pod.castShadow = true;
  root.add(pod);

  const nose = new Mesh(new CylinderGeometry(0.04, 0.25, 1.15, 16), frameMaterial);
  nose.rotation.x = Math.PI / 2;
  nose.position.set(0, -0.22, -1.95);
  root.add(nose);

  const tailplane = new Mesh(new BoxGeometry(3.2, 0.06, 0.58), wingMaterial);
  tailplane.position.set(0, 0, 5.9);
  tailplane.castShadow = true;
  root.add(tailplane);

  const elevator = new Group();
  elevator.position.set(0, 0, 6.22);
  const elevatorMesh = new Mesh(new BoxGeometry(3.15, 0.045, 0.28), controlMaterial);
  elevatorMesh.position.z = 0.14;
  elevator.add(elevatorMesh);
  root.add(elevator);

  const fin = new Mesh(new BoxGeometry(0.06, 1.15, 0.72), wingMaterial);
  fin.position.set(0, 0.53, 5.92);
  fin.castShadow = true;
  root.add(fin);

  const rudder = new Group();
  rudder.position.set(0, 0.57, 6.3);
  const rudderMesh = new Mesh(new BoxGeometry(0.045, 1.05, 0.3), controlMaterial);
  rudderMesh.position.z = 0.15;
  rudder.add(rudderMesh);
  root.add(rudder);

  root.traverse((object) => {
    if (object instanceof Mesh) {
      object.receiveShadow = true;
    }
  });
  return { root, elevator, rudder };
}

export function setControlSurfaces(
  aircraft: AircraftVisual,
  elevatorRad: number,
  rudderRad: number,
): void {
  aircraft.elevator.rotation.x = elevatorRad;
  aircraft.rudder.rotation.y = -rudderRad;
}
