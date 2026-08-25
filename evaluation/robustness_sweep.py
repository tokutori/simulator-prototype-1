"""Run deterministic model-uncertainty stress cases for the reference controller."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt

from control_sweep import metrics

Mutation = Callable[[dict], None]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def scale_mass(model: dict, scale: float) -> None:
    model["mass_properties"]["mass_kg"] *= scale


def scale_lift(model: dict, scale: float) -> None:
    for point in model["aerodynamics"]["longitudinal_table"]:
        point["cl"] *= scale
    model["aerodynamics"]["derivatives_per_rad"]["cl_elevator"] *= scale


def scale_drag(model: dict, scale: float) -> None:
    for point in model["aerodynamics"]["longitudinal_table"]:
        point["cd"] *= scale


def scale_static_pitch(model: dict, scale: float) -> None:
    for point in model["aerodynamics"]["longitudinal_table"]:
        point["cm"] *= scale


def scale_pitch_damping(model: dict, scale: float) -> None:
    model["aerodynamics"]["derivatives_per_rad"]["cm_pitch_rate"] *= scale


def scale_elevator_effectiveness(model: dict, scale: float) -> None:
    derivatives = model["aerodynamics"]["derivatives_per_rad"]
    derivatives["cl_elevator"] *= scale
    derivatives["cm_elevator"] *= scale


def set_servo_time_constant(model: dict, value_s: float) -> None:
    model["actuators"]["elevator"]["time_constant_s"] = value_s


def compose(*mutations: Mutation) -> Mutation:
    def apply(model: dict) -> None:
        for mutation in mutations:
            mutation(model)

    return apply


def cases() -> list[tuple[str, str, Mutation]]:
    return [
        ("baseline", "public reconstruction", lambda _model: None),
        ("mass-low", "mass -5%", lambda model: scale_mass(model, 0.95)),
        ("mass-high", "mass +5%", lambda model: scale_mass(model, 1.05)),
        ("lift-low", "CL and CLde -10%", lambda model: scale_lift(model, 0.90)),
        ("lift-high", "CL and CLde +10%", lambda model: scale_lift(model, 1.10)),
        ("drag-low", "CD -20%", lambda model: scale_drag(model, 0.80)),
        ("drag-high", "CD +20%", lambda model: scale_drag(model, 1.20)),
        ("static-pitch-weak", "Cm(alpha) -20%", lambda model: scale_static_pitch(model, 0.80)),
        ("static-pitch-strong", "Cm(alpha) +20%", lambda model: scale_static_pitch(model, 1.20)),
        ("pitch-damping-weak", "Cmq -30%", lambda model: scale_pitch_damping(model, 0.70)),
        ("pitch-damping-strong", "Cmq +30%", lambda model: scale_pitch_damping(model, 1.30)),
        ("elevator-weak", "CLde and Cmde -20%", lambda model: scale_elevator_effectiveness(model, 0.80)),
        ("elevator-strong", "CLde and Cmde +20%", lambda model: scale_elevator_effectiveness(model, 1.20)),
        ("servo-fast", "servo time constant 0.03 s", lambda model: set_servo_time_constant(model, 0.03)),
        ("servo-slow", "servo time constant 0.12 s", lambda model: set_servo_time_constant(model, 0.12)),
        (
            "adverse-corner",
            "mass +5%, CL -10%, CD +20%, Cmq -30%, elevator -20%, servo 0.12 s",
            compose(
                lambda model: scale_mass(model, 1.05),
                lambda model: scale_lift(model, 0.90),
                lambda model: scale_drag(model, 1.20),
                lambda model: scale_pitch_damping(model, 0.70),
                lambda model: scale_elevator_effectiveness(model, 0.80),
                lambda model: set_servo_time_constant(model, 0.12),
            ),
        ),
        (
            "favorable-corner",
            "mass -5%, CL +10%, CD -20%, Cmq +30%, elevator +20%, servo 0.03 s",
            compose(
                lambda model: scale_mass(model, 0.95),
                lambda model: scale_lift(model, 1.10),
                lambda model: scale_drag(model, 0.80),
                lambda model: scale_pitch_damping(model, 1.30),
                lambda model: scale_elevator_effectiveness(model, 1.20),
                lambda model: set_servo_time_constant(model, 0.03),
            ),
        ),
    ]


def run_case(
    binary: Path,
    base_model: dict,
    case_id: str,
    description: str,
    mutation: Mutation,
) -> dict[str, float | int | str]:
    model = json.loads(json.dumps(base_model))
    mutation(model)
    with tempfile.TemporaryDirectory(prefix="tokutori-robustness-") as temporary:
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
                "12",
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
            raise RuntimeError(f"{case_id}: {completed.stderr}")
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    result: dict[str, float | int | str] = {
        "case": case_id,
        "description": description,
    }
    result.update(metrics(rows))
    result["final_altitude_m"] = float(rows[-1]["altitude_m"])
    result["minimum_flight_path_deg"] = min(float(row["flight_path_deg"]) for row in rows)
    return result


def write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [str(row["case"]) for row in rows]
    pullout_loss = [float(row["pullout_loss_m"]) for row in rows]
    minimum_flight_path = [float(row["minimum_flight_path_deg"]) for row in rows]
    maximum_alpha = [float(row["max_alpha_deg"]) for row in rows]
    all_cases_avoid_reascent = all(float(row["max_reascent_m"]) == 0.0 for row in rows)
    colors = ["#d62728" if label == "adverse-corner" else "#1f77b4" for label in labels]
    colors[labels.index("baseline")] = "#2ca02c"
    figure, axes = plt.subplots(3, 1, figsize=(15, 12), sharex=True)
    axes[0].bar(labels, pullout_loss, color=colors)
    axes[0].set_ylabel("Altitude loss [m]")
    axes[0].set_title("Altitude loss until flight path recovers to -3 deg")
    axes[1].bar(labels, minimum_flight_path, color=colors)
    axes[1].set_ylabel("Minimum gamma [deg]")
    axes[1].set_title("Steepest downward flight-path angle")
    axes[2].bar(labels, maximum_alpha, color=colors)
    axes[2].axhline(8.0, color="black", linestyle="--", label="strict validation limit")
    axes[2].axhline(20.0, color="#ff7f0e", linestyle=":", label="BR training limit")
    axes[2].set_ylabel("Maximum alpha [deg]")
    axes[2].set_title("Maximum angle of attack (all cases avoid re-ascent: " + str(all_cases_avoid_reascent) + ")")
    axes[2].legend()
    axes[2].tick_params(axis="x", rotation=55)
    figure.suptitle(
        "QX-18 deterministic model-uncertainty stress cases\n"
        "Ranges are engineering probes, not identified probability distributions",
        y=0.985,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    definitions = cases()
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(
            executor.map(
                lambda case: run_case(args.binary, base_model, *case),
                definitions,
            )
        )
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
