import { mkdirSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "../..");
run("cargo", ["build", "-p", "sim-cli", "--bin", "plant-bridge"]);
run("cargo", [
  "build", "--manifest-path", "firmware/fbw-rp2040/Cargo.toml",
  "--target", "thumbv6m-none-eabi", "--release",
]);
const output = resolve(root, "target/virtual-platform/fbw-rp2040.uf2");
mkdirSync(dirname(output), { recursive: true });
run("elf2uf2-rs", [
  "firmware/fbw-rp2040/target/thumbv6m-none-eabi/release/fbw-rp2040",
  output,
]);

function run(command: string, arguments_: string[]): void {
  const executable = process.platform === "win32" && command === "cargo" ? "cargo.exe" : command;
  const result = spawnSync(executable, arguments_, { cwd: root, stdio: "inherit", windowsHide: true });
  if (result.error) throw result.error;
  if (result.status !== 0) throw new Error(`${command} exited with ${result.status}`);
}
