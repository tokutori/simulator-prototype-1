"""Sweep a proactive ground-referenced sink-rate barrier for vertical gusts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt

GROUND_CLIMB_LIMITS_MPS = [0.0, -0.15, -0.20, -0.25, -0.30]
GROUND_GAINS_RAD_PER_MPS = [0.6, 0.9, 1.2]
FILTER_CONSTANTS_S = [0.10, 0.15, 0.25, 0.40]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def simulate(binary: Path, model: dict, gust_down_mps: float) -> list[dict[str, str]]:
    case = json.loads(json.dumps(model))
    gust = case["environment"]["one_minus_cosine_gust"]
    gust["enabled"] = gust_down_mps != 0.0
    gust["start_north_m"] = 60.0
    gust["length_m"] = 40.0
    gust["peak_wind_ned_mps"] = [0.0, 0.0, gust_down_mps]
    with tempfile.TemporaryDirectory(prefix="tokutori-ground-path-") as temporary:
        root = Path(temporary)
        model_path = root / "model.json"
        output_path = root / "run.csv"
        model_path.write_text(json.dumps(case, ensure_ascii=False), encoding="utf-8")
        completed = subprocess.run(
            [
                str(binary),
                "--model",
                str(model_path),
                "--duration",
                "14",
                "--dt",
                "0.01",
                "--output",
                str(output_path),
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr)
        with output_path.open(encoding="utf-8", newline="") as handle:
            return list(csv.DictReader(handle))


def metrics(rows: list[dict[str, str]]) -> dict[str, float | int | bool]:
    altitude = [float(row["altitude_m"]) for row in rows]
    gamma = [float(row["flight_path_deg"]) for row in rows]
    elevator = [float(row["elevator_deg"]) for row in rows]
    running_minimum = math.inf
    maximum_reascent = 0.0
    for value in altitude:
        running_minimum = min(running_minimum, value)
        maximum_reascent = max(maximum_reascent, value - running_minimum)
    return {
        "maximum_reascent_m": maximum_reascent,
        "maximum_flight_path_deg": max(gamma),
        "positive_flight_path_samples": sum(value > 0.0 for value in gamma),
        "final_altitude_m": altitude[-1],
        "surface_contact": float(rows[-1]["time_s"]) < 13.99 and altitude[-1] <= 0.01,
        "elevator_saturation_s": 0.01 * sum(abs(value) >= 9.9 for value in elevator),
        "elevator_total_variation_deg": sum(
            abs(right - left) for left, right in zip(elevator, elevator[1:], strict=False)
        ),
    }


def run_case(binary: Path, base_model: dict, parameters: tuple[float, float, float]) -> dict:
    climb_limit_mps, gain, filter_constant_s = parameters
    model = json.loads(json.dumps(base_model))
    controller = model["reference_controller"]
    controller["ground_climb_limit_mps"] = climb_limit_mps
    controller["ground_climb_suppression_gain_rad_per_mps"] = gain
    controller["vertical_speed_filter_time_constant_s"] = filter_constant_s
    nominal = metrics(simulate(binary, model, 0.0))
    gust = metrics(simulate(binary, model, -2.0))
    result = {
        "ground_climb_limit_mps": climb_limit_mps,
        "ground_gain_rad_per_mps": gain,
        "filter_time_constant_s": filter_constant_s,
    }
    result.update({f"nominal_{key}": value for key, value in nominal.items()})
    result.update({f"gust_{key}": value for key, value in gust.items()})
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict]) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(14, 6), constrained_layout=True)
    color_map = plt.get_cmap("viridis")
    normalization = plt.Normalize(
        min(GROUND_GAINS_RAD_PER_MPS),
        max(GROUND_GAINS_RAD_PER_MPS),
    )
    for row in rows:
        color = color_map(normalization(float(row["ground_gain_rad_per_mps"])))
        axes[0].scatter(
            float(row["gust_maximum_reascent_m"]),
            float(row["gust_elevator_total_variation_deg"]),
            color=color,
            alpha=0.75,
        )
        axes[1].scatter(
            float(row["gust_maximum_reascent_m"]),
            float(row["nominal_final_altitude_m"]),
            color=color,
            alpha=0.75,
        )
    highlighted = (
        ("current: -0.25 m/s, gain 0.9, tau 0.25 s", -0.25, 0.9, 0.25, "X"),
        ("no ground barrier", 0.0, 0.9, 0.25, "s"),
    )
    for label, limit, gain, filter_constant, marker in highlighted:
        row = next(
            item
            for item in rows
            if math.isclose(float(item["ground_climb_limit_mps"]), limit)
            and math.isclose(float(item["ground_gain_rad_per_mps"]), gain)
            and math.isclose(float(item["filter_time_constant_s"]), filter_constant)
        )
        for axis, y_key in zip(
            axes,
            ("gust_elevator_total_variation_deg", "nominal_final_altitude_m"),
            strict=True,
        ):
            axis.scatter(
                float(row["gust_maximum_reascent_m"]),
                float(row[y_key]),
                marker=marker,
                s=100,
                facecolors="none",
                edgecolors="black",
                linewidths=1.6,
                label=label,
                zorder=4,
            )
    axes[0].set(xlabel="2 m/s upgust maximum re-ascent (m)", ylabel="Elevator total variation (deg)")
    axes[1].set(xlabel="2 m/s upgust maximum re-ascent (m)", ylabel="Nominal altitude at 14 s (m)")
    for axis in axes:
        axis.grid(alpha=0.3)
        axis.legend(loc="best")
    color_bar = figure.colorbar(
        plt.cm.ScalarMappable(norm=normalization, cmap=color_map),
        ax=axes,
        shrink=0.82,
    )
    color_bar.set_label("Ground-climb suppression gain (rad per m/s)")
    figure.suptitle(
        "Ground-referenced sink-rate barrier sweep\n"
        "Color = suppression gain; limits are deterministic engineering probes"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    parameters = [
        (limit, gain, filter_constant_s)
        for limit in GROUND_CLIMB_LIMITS_MPS
        for gain in GROUND_GAINS_RAD_PER_MPS
        for filter_constant_s in FILTER_CONSTANTS_S
    ]
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(executor.map(lambda values: run_case(args.binary, base_model, values), parameters))
    rows.sort(key=lambda row: (float(row["gust_maximum_reascent_m"]), -float(row["nominal_final_altitude_m"])))
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(rows[:20], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
