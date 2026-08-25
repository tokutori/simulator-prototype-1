#!/usr/bin/env python3
"""Sweep gust feedback without collapsing re-ascent and over-control into one score."""

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

GROUND_GAINS = [0.3, 0.5, 0.7, 0.9, 1.1]
FILTER_CONSTANTS_S = [0.10, 0.15, 0.25, 0.40]
PITCH_RATE_GAINS_S = [0.05, 0.10, 0.20, 0.40, 0.60]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def run_simulation(binary: Path, model: dict, gust_down_mps: float) -> list[dict[str, str]]:
    case = json.loads(json.dumps(model))
    gust = case["environment"]["one_minus_cosine_gust"]
    gust["enabled"] = gust_down_mps != 0.0
    gust["start_north_m"] = 60.0
    gust["length_m"] = 40.0
    gust["peak_wind_ned_mps"] = [0.0, 0.0, gust_down_mps]
    with tempfile.TemporaryDirectory(prefix="tokutori-gust-control-") as temporary:
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


def metrics(rows: list[dict[str, str]]) -> dict[str, float | int]:
    altitude = [float(row["altitude_m"]) for row in rows]
    gamma = [float(row["flight_path_deg"]) for row in rows]
    north = [float(row["north_m"]) for row in rows]
    elevator = [float(row["elevator_deg"]) for row in rows]
    running_minimum = math.inf
    maximum_reascent = 0.0
    for value in altitude:
        running_minimum = min(running_minimum, value)
        maximum_reascent = max(maximum_reascent, value - running_minimum)
    gust_elevator = [
        value
        for position, value in zip(north, elevator, strict=True)
        if 55.0 <= position <= 110.0
    ]
    return {
        "maximum_reascent_m": maximum_reascent,
        "maximum_flight_path_deg": max(gamma),
        "positive_flight_path_samples": sum(value > 0.0 for value in gamma),
        "maximum_abs_elevator_deg": max(map(abs, gust_elevator)),
        "elevator_saturation_s": 0.01
        * sum(abs(value) >= 9.9 for value in gust_elevator),
        "elevator_total_variation_deg": sum(
            abs(right - left)
            for left, right in zip(gust_elevator, gust_elevator[1:], strict=False)
        ),
    }


def run_case(
    binary: Path,
    base_model: dict,
    parameters: tuple[float, float, float],
) -> dict[str, float | int]:
    ground_gain, filter_constant, pitch_rate_gain = parameters
    model = json.loads(json.dumps(base_model))
    controller = model["reference_controller"]
    controller["ground_climb_suppression_gain_rad_per_mps"] = ground_gain
    controller["vertical_speed_filter_time_constant_s"] = filter_constant
    controller["glide_pitch_rate_gain_s"] = pitch_rate_gain
    nominal = metrics(run_simulation(binary, model, 0.0))
    gust = metrics(run_simulation(binary, model, -2.0))
    return {
        "ground_gain_rad_per_mps": ground_gain,
        "filter_time_constant_s": filter_constant,
        "glide_pitch_rate_gain_s": pitch_rate_gain,
        "nominal_maximum_reascent_m": nominal["maximum_reascent_m"],
        "nominal_maximum_flight_path_deg": nominal["maximum_flight_path_deg"],
        "gust_maximum_reascent_m": gust["maximum_reascent_m"],
        "gust_maximum_flight_path_deg": gust["maximum_flight_path_deg"],
        "gust_positive_flight_path_samples": gust["positive_flight_path_samples"],
        "gust_maximum_abs_elevator_deg": gust["maximum_abs_elevator_deg"],
        "gust_elevator_saturation_s": gust["elevator_saturation_s"],
        "gust_elevator_total_variation_deg": gust["elevator_total_variation_deg"],
    }


def pareto(rows: list[dict[str, float | int]]) -> list[dict[str, float | int]]:
    fields = [
        "gust_maximum_reascent_m",
        "gust_elevator_saturation_s",
        "gust_elevator_total_variation_deg",
    ]
    return [
        row
        for row in rows
        if not any(
            other is not row
            and all(float(other[field]) <= float(row[field]) for field in fields)
            and any(float(other[field]) < float(row[field]) for field in fields)
            for other in rows
        )
    ]


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
    for row in rows:
        baseline = (
            float(row["ground_gain_rad_per_mps"]) == 0.9
            and float(row["filter_time_constant_s"]) == 0.10
            and float(row["glide_pitch_rate_gain_s"]) == 0.6
        )
        marker = "*" if baseline else "o"
        size = 180 if baseline else 50
        color = float(row["glide_pitch_rate_gain_s"])
        axes[0].scatter(
            float(row["gust_maximum_reascent_m"]),
            float(row["gust_elevator_total_variation_deg"]),
            c=[color], vmin=min(PITCH_RATE_GAINS_S), vmax=max(PITCH_RATE_GAINS_S),
            cmap="viridis", marker=marker, s=size,
        )
        axes[1].scatter(
            float(row["gust_maximum_reascent_m"]),
            float(row["gust_elevator_saturation_s"]),
            c=[color], vmin=min(PITCH_RATE_GAINS_S), vmax=max(PITCH_RATE_GAINS_S),
            cmap="viridis", marker=marker, s=size,
        )
    axes[0].set(xlabel="maximum re-ascent (m)", ylabel="elevator total variation (deg)")
    axes[1].set(xlabel="maximum re-ascent (m)", ylabel="elevator saturation (s)")
    axes[0].set_title("Re-ascent vs command activity")
    axes[1].set_title("Re-ascent vs saturation")
    for axis in axes:
        axis.grid(alpha=0.3)
    figure.suptitle(
        "2 m/s upgust controller sweep; star = current profile; color = pitch-rate gain\n"
        "No scalar score: lower-left trade-offs must be checked against real-aircraft constraints"
    )
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    parameters = [
        (ground, filter_constant, pitch_rate)
        for ground in GROUND_GAINS
        for filter_constant in FILTER_CONSTANTS_S
        for pitch_rate in PITCH_RATE_GAINS_S
    ]
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(
            executor.map(
                lambda values: run_case(args.binary, base_model, values),
                parameters,
            )
        )
    rows.sort(
        key=lambda row: (
            float(row["gust_maximum_reascent_m"]),
            float(row["gust_elevator_total_variation_deg"]),
        )
    )
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(pareto(rows), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
