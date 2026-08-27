import {
  Mesh,
  MeshStandardMaterial,
  PlaneGeometry,
  type IUniform,
} from "three";

export interface AnimatedWater {
  readonly mesh: Mesh<PlaneGeometry, MeshStandardMaterial>;
  update(timeS: number): void;
}

interface WaterShaderUniforms {
  uWaveTime: IUniform<number>;
}

interface ShaderSource {
  uniforms: Record<string, IUniform>;
  vertexShader: string;
  fragmentShader: string;
}

const waveUniforms: WaterShaderUniforms = {
  uWaveTime: { value: 0 },
};

const waveFunctions = /* glsl */ `
  float birdmanLongWave(vec2 point, float timeS) {
    vec2 directionA = normalize(vec2(0.82, 0.57));
    vec2 directionB = normalize(vec2(-0.31, 0.95));
    return
      0.15 * sin(dot(point, directionA) * 0.19 + timeS * 0.72) +
      0.07 * sin(dot(point, directionB) * 0.31 + timeS * 0.96);
  }

  vec2 birdmanWaveSlope(vec2 point, float timeS) {
    vec2 directionA = normalize(vec2(0.82, 0.57));
    vec2 directionB = normalize(vec2(-0.31, 0.95));
    vec2 directionC = normalize(vec2(0.94, -0.34));
    vec2 directionD = normalize(vec2(0.18, 0.98));
    return
      directionA * (0.15 * 0.19 * cos(dot(point, directionA) * 0.19 + timeS * 0.72)) +
      directionB * (0.07 * 0.31 * cos(dot(point, directionB) * 0.31 + timeS * 0.96)) +
      directionC * (0.026 * 0.89 * cos(dot(point, directionC) * 0.89 + timeS * 1.34)) +
      directionD * (0.012 * 1.74 * cos(dot(point, directionD) * 1.74 + timeS * 1.91));
  }
`;

export function createAnimatedWater(): AnimatedWater {
  // Long waves displace the mesh, while shorter ripples modify fragment normals.
  // The water remains a visualization surface and does not feed the FDM.
  const geometry = new PlaneGeometry(2000, 2000, 160, 160);
  const material = new MeshStandardMaterial({
    color: 0x24768d,
    roughness: 0.3,
    metalness: 0.08,
  });

  material.onBeforeCompile = (shader) => patchWaterShader(shader);
  material.customProgramCacheKey = () => "birdman-animated-water-v1";

  const mesh = new Mesh(geometry, material);
  mesh.name = "animated-water";
  mesh.rotation.x = -Math.PI / 2;
  mesh.receiveShadow = true;

  return {
    mesh,
    update(timeS: number): void {
      waveUniforms.uWaveTime.value = timeS;
    },
  };
}

export function patchWaterShader(shader: ShaderSource): void {
  shader.uniforms.uWaveTime = waveUniforms.uWaveTime;
  shader.vertexShader = `
      uniform float uWaveTime;
      varying vec3 vBirdmanWaterWorld;
      varying float vBirdmanWaveHeight;
      ${waveFunctions}
      ${shader.vertexShader}
    `.replace(
      "#include <begin_vertex>",
      `#include <begin_vertex>
       vec4 birdmanWaterBase = modelMatrix * vec4(position, 1.0);
       float birdmanHeight = birdmanLongWave(birdmanWaterBase.xz, uWaveTime);
       transformed.z += birdmanHeight;
       vec4 birdmanWaterDisplaced = modelMatrix * vec4(transformed, 1.0);
       vBirdmanWaterWorld = birdmanWaterDisplaced.xyz;
       vBirdmanWaveHeight = birdmanHeight;`,
    );
  shader.fragmentShader = `
      uniform float uWaveTime;
      varying vec3 vBirdmanWaterWorld;
      varying float vBirdmanWaveHeight;
      ${waveFunctions}
      ${shader.fragmentShader}
    `
      .replace(
        "#include <normal_fragment_maps>",
        `#include <normal_fragment_maps>
         vec2 birdmanSlope = birdmanWaveSlope(vBirdmanWaterWorld.xz, uWaveTime);
         vec3 birdmanWorldNormal = normalize(vec3(-birdmanSlope.x, 1.0, -birdmanSlope.y));
         normal = normalize(mat3(viewMatrix) * birdmanWorldNormal);`,
      )
      .replace(
        "#include <color_fragment>",
        `#include <color_fragment>
         float birdmanRipple = 0.5 + 0.5 * sin(
           dot(vBirdmanWaterWorld.xz, normalize(vec2(0.94, -0.34))) * 0.89 + uWaveTime * 1.34
         );
         float birdmanCrest = smoothstep(0.12, 0.22, vBirdmanWaveHeight);
         diffuseColor.rgb *= mix(0.86, 1.13, birdmanRipple);
         diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.58, 0.82, 0.86), birdmanCrest * 0.2);`,
      );
}
