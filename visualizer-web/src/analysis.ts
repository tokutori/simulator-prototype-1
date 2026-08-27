import "./analysis.css";

import {
  analysisStorageKey,
  parseAnalysisDataset,
  summarizeFlight,
  type FlightAnalysisDataset,
} from "./analysis-data.ts";
import type { FlightFrame } from "./types.ts";
import { applyUiScale } from "./ui-scale.ts";

const svgNamespace = "http://www.w3.org/2000/svg";
const degree = 180 / Math.PI;
const seriesColors = ["series-0", "series-1", "series-2", "series-3", "series-4"];
let dataset: FlightAnalysisDataset | undefined;

applyUiScale(window.innerWidth, window.innerHeight);
try {
  const raw = localStorage.getItem(analysisStorageKey);
  if (!raw) throw new Error("No flight has been sent from the simulator. Open Flight analysis after a replay or landing.");
  dataset = parseAnalysisDataset(raw);
  render(dataset);
} catch (error) {
  element("analysis-status").textContent = "NO DATA";
  const message = element("analysis-error");
  message.textContent = String(error);
  message.hidden = false;
}

window.addEventListener("resize", () => {
  applyUiScale(window.innerWidth, window.innerHeight);
  if (dataset) renderCharts(dataset.frames);
});

function render(value: FlightAnalysisDataset): void {
  const summary = summarizeFlight(value.frames);
  element("analysis-status").textContent = summary.surfaceContact ? "WATER CONTACT" : "RECORD ENDED ALOFT";
  element("analysis-name").textContent = value.name;
  element("summary-time").textContent = `${summary.durationS.toFixed(2)} s`;
  element("summary-range").textContent = `${summary.rangeM.toFixed(1)} m`;
  element("summary-track").textContent = `${summary.trackM.toFixed(1)} m`;
  element("summary-altitude").textContent = `${summary.maximumAltitudeM.toFixed(1)} m`;
  element("summary-speed").textContent = `${summary.finalAirspeedMps.toFixed(1)} m/s`;
  element("summary-roll").textContent = `${summary.maximumAbsRollDeg.toFixed(1)}°`;
  element("analysis-content").hidden = false;
  renderCharts(value.frames);
}

function renderCharts(frames: readonly FlightFrame[]): void {
  renderTrajectory("trajectory-chart", frames);
  renderTimeChart("altitude-chart", frames, "Altitude", "m", [series("Altitude", frame => frame.altitudeM)]);
  renderTimeChart("airspeed-chart", frames, "Airspeed", "m/s", [series("IAS", frame => frame.airspeedMps)]);
  renderTimeChart("pilot-chart", frames, "Pilot input", "%", [
    series("Elevator", frame => frame.pilotElevator * 100),
    series("Rudder", frame => frame.pilotRudder * 100),
  ]);
  renderTimeChart("elevator-chart", frames, "Elevator", "deg", [
    series("Manual", frame => frame.manualElevatorCommandRad * degree),
    series("Automatic", frame => frame.automaticElevatorCommandRad * degree),
    series("Mixed command", frame => frame.mixedElevatorCommandRad * degree),
    series("Actual", frame => frame.elevatorRad * degree),
  ]);
  renderTimeChart("rudder-chart", frames, "Rudder", "deg", [
    series("Manual", frame => frame.manualRudderCommandRad * degree),
    series("Automatic", frame => frame.automaticRudderCommandRad * degree),
    series("Mixed command", frame => frame.mixedRudderCommandRad * degree),
    series("Actual", frame => frame.rudderRad * degree),
  ]);
  renderTimeChart("attitude-chart", frames, "Attitude", "deg", [
    series("Roll", frame => frame.rollRad * degree),
    series("Pitch", frame => frame.pitchRad * degree),
    series("Flight path", frame => frame.flightPathRad * degree),
  ]);
}

interface SeriesDefinition {
  name: string;
  value(frame: FlightFrame): number;
}

function series(name: string, value: (frame: FlightFrame) => number): SeriesDefinition {
  return { name, value };
}

function renderTimeChart(
  id: string,
  frames: readonly FlightFrame[],
  accessibleName: string,
  unit: string,
  definitions: readonly SeriesDefinition[],
): void {
  const host = element(id);
  host.replaceChildren();
  const svg = svgElement("svg", { viewBox: "0 0 900 270", role: "img", "aria-label": `${accessibleName} time series` });
  const left = 64;
  const right = 22;
  const top = 38;
  const bottom = 44;
  const width = 900 - left - right;
  const height = 270 - top - bottom;
  const firstTime = frames[0]?.timeS ?? 0;
  const duration = Math.max(1e-6, (frames.at(-1)?.timeS ?? firstTime) - firstTime);
  const values = definitions.flatMap(definition => frames.map(frame => definition.value(frame))).filter(Number.isFinite);
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  if (minimum === maximum) { minimum -= 1; maximum += 1; }
  const padding = (maximum - minimum) * 0.08;
  minimum -= padding;
  maximum += padding;
  const x = (timeS: number): number => left + ((timeS - firstTime) / duration) * width;
  const y = (value: number): number => top + (1 - (value - minimum) / (maximum - minimum)) * height;

  svg.appendChild(svgElement("rect", { class: "frame", x: left, y: top, width, height }));
  for (let tick = 0; tick <= 4; tick += 1) {
    const fraction = tick / 4;
    const yPosition = top + fraction * height;
    const value = maximum - fraction * (maximum - minimum);
    svg.appendChild(svgElement("line", { class: "grid", x1: left, y1: yPosition, x2: left + width, y2: yPosition }));
    svg.appendChild(svgText(left - 8, yPosition + 4, value.toFixed(1), "end"));
  }
  for (let tick = 0; tick <= 5; tick += 1) {
    const fraction = tick / 5;
    const xPosition = left + fraction * width;
    svg.appendChild(svgText(xPosition, top + height + 20, (duration * fraction).toFixed(1), "middle"));
  }

  definitions.forEach((definition, index) => {
    const color = seriesColor(index);
    const path = frames.map((frame, frameIndex) =>
      `${frameIndex === 0 ? "M" : "L"}${x(frame.timeS).toFixed(2)},${y(definition.value(frame)).toFixed(2)}`).join(" ");
    svg.appendChild(svgElement("path", { class: `series ${color}`, d: path }));
  });
  const legend = svgElement("g", { class: "legend" });
  definitions.forEach((definition, index) => {
    const offset = left + index * 150;
    legend.appendChild(svgElement("line", { class: seriesColor(index), x1: offset, y1: 17, x2: offset + 24, y2: 17, "stroke-width": 3 }));
    legend.appendChild(svgText(offset + 31, 21, definition.name, "start"));
  });
  svg.appendChild(legend);
  svg.appendChild(svgText(left + width / 2, 263, "Time (s)", "middle", "axis-title"));
  const yTitle = svgText(15, top + height / 2, `${accessibleName} (${unit})`, "middle", "axis-title");
  yTitle.setAttribute("transform", `rotate(-90 15 ${top + height / 2})`);
  svg.appendChild(yTitle);
  host.appendChild(svg);
}

function seriesColor(index: number): string {
  return seriesColors[index % seriesColors.length] ?? "series-0";
}

function renderTrajectory(id: string, frames: readonly FlightFrame[]): void {
  const host = element(id);
  host.replaceChildren();
  const svg = svgElement("svg", { viewBox: "0 0 900 430", role: "img", "aria-label": "Top-down flight trajectory from launch" });
  const first = frames[0];
  if (!first) return;
  const relative = frames.map(frame => ({ east: frame.eastM - first.eastM, north: frame.northM - first.northM }));
  const maximumRadius = Math.max(50, ...relative.map(point => Math.hypot(point.east, point.north)));
  const ringMaximum = Math.ceil(maximumRadius / 50) * 50;
  const centerX = 450;
  const centerY = 395;
  const scale = Math.min(360 / ringMaximum, 390 / ringMaximum);
  for (let radius = 50; radius <= ringMaximum; radius += 50) {
    svg.appendChild(svgElement("circle", { class: "ring", cx: centerX, cy: centerY, r: radius * scale }));
    svg.appendChild(svgText(centerX + 5, centerY - radius * scale + 14, `${radius} m`, "start"));
  }
  const path = relative.map((point, index) =>
    `${index === 0 ? "M" : "L"}${(centerX + point.east * scale).toFixed(2)},${(centerY - point.north * scale).toFixed(2)}`).join(" ");
  svg.appendChild(svgElement("path", { class: "trajectory", d: path }));
  svg.appendChild(svgElement("circle", { class: "start", cx: centerX, cy: centerY, r: 6 }));
  const finish = relative.at(-1) ?? { east: 0, north: 0 };
  svg.appendChild(svgElement("circle", { class: "finish", cx: centerX + finish.east * scale, cy: centerY - finish.north * scale, r: 6 }));
  svg.appendChild(svgText(450, 424, "East / west offset and north distance (m)", "middle", "axis-title"));
  host.appendChild(svg);
}

function svgElement<K extends keyof SVGElementTagNameMap>(
  name: K,
  attributes: Record<string, string | number>,
): SVGElementTagNameMap[K] {
  const node = document.createElementNS(svgNamespace, name);
  for (const [attribute, value] of Object.entries(attributes)) node.setAttribute(attribute, String(value));
  return node;
}

function svgText(x: number, y: number, text: string, anchor: string, className?: string): SVGTextElement {
  const node = svgElement("text", { x, y, "text-anchor": anchor });
  if (className) node.setAttribute("class", className);
  node.textContent = text;
  return node;
}

function element<T extends HTMLElement = HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (!found) throw new Error(`missing element #${id}`);
  return found as T;
}
