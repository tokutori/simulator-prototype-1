import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const visualizerDirectory = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const inputPath = resolve(visualizerDirectory, "..", "reports", "run.csv");
const outputPath = resolve(visualizerDirectory, "public", "sample-flight.csv");
const selectedColumns = [
  "time_s",
  "north_m",
  "east_m",
  "altitude_m",
  "roll_deg",
  "pitch_deg",
  "yaw_deg",
  "flight_path_deg",
  "airspeed_mps",
  "alpha_deg",
  "elevator_deg",
  "rudder_deg",
  "elevator_command_deg",
];

const lines = readFileSync(inputPath, "utf8").replace(/^\uFEFF/, "").trim().split(/\r?\n/);
const header = lines.shift()?.split(",") ?? [];
const indices = selectedColumns.map((name) => {
  const index = header.indexOf(name);
  if (index < 0) throw new Error(`run.csv is missing ${name}`);
  return index;
});
const data = lines.filter((_, index) => index % 5 === 0 || index === lines.length - 1);
const output = [
  selectedColumns.join(","),
  ...data.map((line) => {
    const values = line.split(",");
    return indices.map((index) => values[index] ?? "").join(",");
  }),
].join("\n") + "\n";

mkdirSync(dirname(outputPath), { recursive: true });
writeFileSync(outputPath, output, "utf8");
process.stdout.write(`Wrote ${data.length} samples to ${outputPath}\n`);
