"""Compare ground-effect and deterministic one-minus-cosine gust stress cases."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def cases() -> list[tuple[str, bool, tuple[float, float, float]]]:
    return [
        ("free-air", False, (0.0, 0.0, 0.0)),
        ("ground-effect", True, (0.0, 0.0, 0.0)),
        ("upgust-0.5", True, (0.0, 0.0, -0.5)),
        ("upgust-1.0", True, (0.0, 0.0, -1.0)),
        ("upgust-2.0", True, (0.0, 0.0, -2.0)),
        ("downgust-0.5", True, (0.0, 0.0, 0.5)),
        ("downgust-1.0", True, (0.0, 0.0, 1.0)),
        ("downgust-2.0", True, (0.0, 0.0, 2.0)),
        ("headwind-2.0", True, (-2.0, 0.0, 0.0)),
        ("tailwind-2.0", True, (2.0, 0.0, 0.0)),
        ("crosswind-2.0", True, (0.0, 2.0, 0.0)),
    ]


def run_case(
    binary: Path,
    base_model: dict,
    definition: tuple[str, bool, tuple[float, float, float]],
) -> dict[str, float | int | str]:
    case_name, ground_effect_enabled, gust_peak = definition
    model = json.loads(json.dumps(base_model))
    model["aerodynamics"]["ground_effect"]["enabled"] = ground_effect_enabled
    gust = model["environment"]["one_minus_cosine_gust"]
    gust["enabled"] = any(component != 0.0 for component in gust_peak)
    gust["start_north_m"] = 60.0
    gust["length_m"] = 40.0
    gust["peak_wind_ned_mps"] = list(gust_peak)
    with tempfile.TemporaryDirectory(prefix="tokutori-environment-") as temporary:
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
                "20",
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
            raise RuntimeError(f"{case_name}: {completed.stderr}")
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    altitude = [float(row["altitude_m"]) for row in rows]
    gamma = [float(row["flight_path_deg"]) for row in rows]
    alpha = [float(row["alpha_deg"]) for row in rows]
    roll = [float(row["roll_deg"]) for row in rows]
    east = [float(row["east_m"]) for row in rows]
    north = [float(row["north_m"]) for row in rows]
    elevator = [float(row["elevator_deg"]) for row in rows]
    vertical_speed_estimate = [
        float(row["estimated_vertical_speed_mps"]) for row in rows
    ]
    running_minimum = math.inf
    max_reascent = 0.0
    for value in altitude:
        running_minimum = min(running_minimum, value)
        max_reascent = max(max_reascent, value - running_minimum)
    termination_match = re.search(r"termination=([^\s]+)", completed.stderr)
    return {
        "case": case_name,
        "ground_effect_enabled": ground_effect_enabled,
        "gust_north_mps": gust_peak[0],
        "gust_east_mps": gust_peak[1],
        "gust_down_mps": gust_peak[2],
        "simulated_s": float(rows[-1]["time_s"]),
        "termination": termination_match.group(1) if termination_match else "unknown",
        "final_altitude_m": altitude[-1],
        "max_reascent_m": max_reascent,
        "positive_gamma_samples": sum(value > 0.0 for value in gamma),
        "maximum_flight_path_deg": max(gamma),
        "minimum_flight_path_deg": min(gamma),
        "maximum_alpha_deg": max(alpha),
        "minimum_alpha_deg": min(alpha),
        "maximum_abs_roll_deg": max(map(abs, roll)),
        "maximum_abs_east_m": max(map(abs, east)),
        "maximum_abs_elevator_deg": max(map(abs, elevator)),
        "maximum_abs_elevator_during_gust_deg": max(
            abs(value)
            for position, value in zip(north, elevator, strict=True)
            if 60.0 <= position <= 100.0
        ),
        "elevator_saturation_s": 0.01 * sum(abs(value) >= 9.95 for value in elevator),
        "maximum_estimated_vertical_speed_mps": max(vertical_speed_estimate),
        "minimum_ground_effect_ratio": min(
            float(row["ground_effect_induced_drag_ratio"]) for row in rows
        ),
        "aero_out_of_range_s": 0.01
        * sum(row["aero_in_range"] == "false" for row in rows),
    }


def write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    names = [str(row["case"]) for row in rows]
    x = range(len(rows))
    figure, axes = plt.subplots(2, 2, figsize=(16, 11), constrained_layout=True)
    panels = [
        ("max_reascent_m", "Maximum re-ascent [m]"),
        ("maximum_flight_path_deg", "Maximum flight-path angle [deg]"),
        ("maximum_alpha_deg", "Maximum angle of attack [deg]"),
        ("simulated_s", "Simulation time before termination [s]"),
    ]
    colors = ["#2ca02c" if row["case"] == "ground-effect" else "#1f77b4" for row in rows]
    for axis, (field, title) in zip(axes.flat, panels, strict=True):
        values = [float(row[field]) for row in rows]
        axis.bar(x, values, color=colors)
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.3)
        axis.set_xticks(list(x), names, rotation=50, ha="right")
    axes[0, 1].axhline(0.0, color="black", linewidth=1)
    axes[1, 0].axhline(20.0, color="#ff7f0e", linestyle=":", label="training alpha limit")
    axes[1, 0].legend()
    axes[1, 1].axhline(20.0, color="black", linestyle="--", label="requested duration")
    axes[1, 1].legend()
    figure.suptitle(
        "QX-18 ground-effect and one-minus-cosine gust stress cases\n"
        "Gust pulse: north 60–100 m; magnitudes are engineering probes, not Lake Biwa statistics"
    )
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    definitions = cases()
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(
            executor.map(
                lambda definition: run_case(args.binary, base_model, definition), definitions
            )
        )
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
