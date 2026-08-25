"""Evaluate launch pitch-rate damping versus AoA-envelope gain."""

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

PITCH_RATE_GAINS = [0.05, 0.1, 0.2, 0.4, 0.8, 1.2]
ALPHA_LIMIT_GAINS = [0.5, 1.0, 2.0, 3.0, 4.0, 6.0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def metrics(rows: list[dict[str, str]]) -> dict[str, float | int]:
    altitude = [float(row["altitude_m"]) for row in rows]
    gamma = [float(row["flight_path_deg"]) for row in rows]
    alpha = [float(row["alpha_deg"]) for row in rows]
    elevator = [float(row["elevator_deg"]) for row in rows]
    time = [float(row["time_s"]) for row in rows]
    dt_s = time[1] - time[0]
    minimum_gamma_index = min(range(len(gamma)), key=gamma.__getitem__)
    recovery_index = next(
        (index for index in range(minimum_gamma_index, len(rows)) if gamma[index] >= -3.0),
        len(rows) - 1,
    )
    running_minimum = math.inf
    max_reascent = 0.0
    for value in altitude:
        running_minimum = min(running_minimum, value)
        max_reascent = max(max_reascent, value - running_minimum)
    out_of_range_samples = sum(row["aero_in_range"] == "false" for row in rows)
    positive_gamma_samples = sum(value > 0.0 for value in gamma)
    saturation_samples = sum(abs(value) >= 9.9 for value in elevator)
    pullout_loss_m = altitude[0] - altitude[recovery_index]
    score = (
        pullout_loss_m
        + 0.25 * out_of_range_samples * dt_s
        + 0.05 * saturation_samples * dt_s
        + 100.0 * max_reascent
        + 10.0 * positive_gamma_samples * dt_s
    )
    return {
        "pullout_loss_m": pullout_loss_m,
        "pullout_time_s": time[recovery_index],
        "max_reascent_m": max_reascent,
        "positive_gamma_samples": positive_gamma_samples,
        "out_of_range_s": out_of_range_samples * dt_s,
        "elevator_saturation_s": saturation_samples * dt_s,
        "max_alpha_deg": max(alpha),
        "maximum_flight_path_deg": max(gamma),
        "minimum_flight_path_deg": min(gamma),
        "score": score,
    }


def run_case(
    binary: Path,
    base_model: dict,
    pitch_rate_gain: float,
    alpha_limit_gain: float,
) -> dict[str, float | int]:
    model = json.loads(json.dumps(base_model))
    controller = model["reference_controller"]
    controller["launch_pitch_rate_gain_s"] = pitch_rate_gain
    controller["alpha_limit_gain"] = alpha_limit_gain
    with tempfile.TemporaryDirectory(prefix="tokutori-control-sweep-") as temporary:
        root = Path(temporary)
        model_path = root / "model.json"
        csv_path = root / "run.csv"
        model_path.write_text(json.dumps(model, ensure_ascii=False), encoding="utf-8")
        completed = subprocess.run(
            [
                str(binary),
                "--model",
                str(model_path),
                "--duration",
                "8",
                "--dt",
                "0.01",
                "--output",
                str(csv_path),
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr)
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    result = metrics(rows)
    result["launch_pitch_rate_gain_s"] = pitch_rate_gain
    result["alpha_limit_gain"] = alpha_limit_gain
    return result


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lookup = {
        (float(row["launch_pitch_rate_gain_s"]), float(row["alpha_limit_gain"])): row
        for row in rows
    }
    figure, axes = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
    fields = [("pullout_loss_m", "Altitude loss at -3 deg recovery [m]"), ("score", "Composite score")]
    for axis, (field, title) in zip(axes, fields, strict=True):
        values = [
            [float(lookup[(pitch_gain, alpha_gain)][field]) for pitch_gain in PITCH_RATE_GAINS]
            for alpha_gain in ALPHA_LIMIT_GAINS
        ]
        image = axis.imshow(values, origin="lower", aspect="auto", cmap="viridis")
        axis.set_xticks(range(len(PITCH_RATE_GAINS)), PITCH_RATE_GAINS)
        axis.set_yticks(range(len(ALPHA_LIMIT_GAINS)), ALPHA_LIMIT_GAINS)
        axis.set_xlabel("Pitch-rate damping gain [s]")
        axis.set_ylabel("AoA limit gain [-]")
        axis.set_title(title)
        for row_index, row_values in enumerate(values):
            for column_index, value in enumerate(row_values):
                axis.text(column_index, row_index, f"{value:.2f}", ha="center", va="center", color="white" if value > (min(map(min, values)) + max(map(max, values))) / 2 else "black")
        figure.colorbar(image, ax=axis)
    figure.suptitle("QX-18 launch-controller parameter sweep (36 deterministic cases)")
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    cases = [
        (pitch_gain, alpha_gain)
        for alpha_gain in ALPHA_LIMIT_GAINS
        for pitch_gain in PITCH_RATE_GAINS
    ]
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(
            executor.map(
                lambda case: run_case(args.binary, base_model, case[0], case[1]),
                cases,
            )
        )
    rows.sort(key=lambda row: float(row["score"]))
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(rows[:5], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
