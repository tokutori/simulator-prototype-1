import { resolve } from "node:path";

import { defineConfig } from "vite";

export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        simulator: resolve(import.meta.dirname, "index.html"),
        analysis: resolve(import.meta.dirname, "analysis.html"),
      },
    },
  },
});
