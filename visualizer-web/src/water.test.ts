import assert from "node:assert/strict";
import { test } from "node:test";
import type { IUniform } from "three";

import { patchWaterShader } from "./water.ts";

test("water shader preserves built-in material stages and injects animated waves", () => {
  const shader = {
    uniforms: {} as Record<string, IUniform>,
    vertexShader: "void main() {\n#include <begin_vertex>\n#include <shadowmap_vertex>\n}",
    fragmentShader: [
      "void main() {",
      "#include <normal_fragment_maps>",
      "#include <color_fragment>",
      "#include <lights_fragment_begin>",
      "}",
    ].join("\n"),
  };
  patchWaterShader(shader);
  assert.ok(shader.uniforms.uWaveTime);
  assert.match(shader.vertexShader, /birdmanLongWave/);
  assert.match(shader.vertexShader, /shadowmap_vertex/);
  assert.match(shader.fragmentShader, /birdmanWaveSlope/);
  assert.match(shader.fragmentShader, /lights_fragment_begin/);
});
