"""Compare strict exit, legacy endpoint hold, and BR training envelopes."""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--strict-model", type=Path, required=True)
    parser.add_argument("--training-model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def launch_state(model: dict) -> None:
    model["initial_state"].update(
        {
            "altitude_m": 10.5,
            "airspeed_mps": 5.0,
            "alpha_deg": 1.682,
            "roll_deg": 0.0,
            "pitch_deg": -1.318,
            "heading_deg": 0.0,
        }
    )


def run_case(
    binary: Path,
    case_name: str,
    source_model: dict,
    policy: str,
) -> tuple[list[dict[str, str]], dict[str, float | str | None]]:
    model = json.loads(json.dumps(source_model))
    launch_state(model)
    model["aerodynamics"]["out_of_range_policy"] = policy
    with tempfile.TemporaryDirectory(prefix="tokutori-aero-envelope-") as temporary:
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
            raise RuntimeError(f"{case_name}: {completed.stderr}")
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    for row in rows:
        row["case"] = case_name
    altitude = [float(row["altitude_m"]) for row in rows]
    gamma = [float(row["flight_path_deg"]) for row in rows]
    alpha = [float(row["alpha_deg"]) for row in rows]
    minimum_gamma_index = min(range(len(rows)), key=gamma.__getitem__)
    recovery_index = next(
        (index for index in range(minimum_gamma_index, len(rows)) if gamma[index] >= -3.0),
        None,
    )
    termination_match = re.search(r"termination=([^\s]+)", completed.stderr)
    summary: dict[str, float | str | None] = {
        "case": case_name,
        "termination": termination_match.group(1) if termination_match else "unknown",
        "simulated_s": float(rows[-1]["time_s"]),
        "maximum_alpha_deg": max(alpha),
        "minimum_flight_path_deg": min(gamma),
        "out_of_range_s": sum(row["aero_in_range"] == "false" for row in rows) * 0.01,
        "pullout_loss_m": None,
    }
    if recovery_index is not None:
        summary["pullout_loss_m"] = altitude[0] - altitude[recovery_index]
    return rows, summary


def write_csv(path: Path, cases: list[list[dict[str, str]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [row for case_rows in cases for row in case_rows]
    fieldnames = ["case", *[field for field in rows[0] if field != "case"]]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, cases: list[list[dict[str, str]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    colors = {
        "strict-stop": "#d62728",
        "legacy-endpoint-hold": "#7f7f7f",
        "br-training-envelope": "#1f77b4",
    }
    figure, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    for rows in cases:
        case_name = rows[0]["case"]
        color = colors[case_name]
        time = [float(row["time_s"]) for row in rows]
        axes[0, 0].plot(
            [float(row["north_m"]) for row in rows],
            [float(row["altitude_m"]) for row in rows],
            label=case_name,
            color=color,
        )
        axes[0, 1].plot(time, [float(row["alpha_deg"]) for row in rows], color=color)
        axes[1, 0].plot(
            time,
            [float(row["flight_path_deg"]) for row in rows],
            color=color,
        )
        axes[1, 1].plot(time, [float(row["airspeed_mps"]) for row in rows], color=color)
        if case_name == "strict-stop":
            axes[0, 0].scatter(
                float(rows[-1]["north_m"]),
                float(rows[-1]["altitude_m"]),
                marker="x",
                s=100,
                linewidths=2,
                color=color,
                zorder=5,
            )
            for axis, field in (
                (axes[0, 1], "alpha_deg"),
                (axes[1, 0], "flight_path_deg"),
                (axes[1, 1], "airspeed_mps"),
            ):
                axis.scatter(
                    time[-1],
                    float(rows[-1][field]),
                    marker="x",
                    s=100,
                    linewidths=2,
                    color=color,
                    zorder=5,
                )
    axes[0, 0].set_title("Trajectory")
    axes[0, 0].set_xlabel("North distance [m]")
    axes[0, 0].set_ylabel("Altitude [m]")
    axes[0, 0].legend()
    axes[0, 1].axhline(8.0, color="black", linestyle="--", label="strict table upper limit")
    axes[0, 1].set_title("Angle of attack")
    axes[0, 1].set_xlabel("Time [s]")
    axes[0, 1].set_ylabel("Alpha [deg]")
    axes[0, 1].legend()
    axes[1, 0].axhline(0.0, color="black", linewidth=1)
    axes[1, 0].set_title("Flight-path angle")
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Gamma [deg]")
    axes[1, 1].set_title("Airspeed")
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Airspeed [m/s]")
    figure.suptitle(
        "QX-18 launch: aerodynamic-envelope policy comparison\n"
        "BR envelope is a training behavior reconstruction, not measured stall data"
    )
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    strict_model = json.loads(args.strict_model.read_text(encoding="utf-8"))
    training_model = json.loads(args.training_model.read_text(encoding="utf-8"))
    definitions = [
        ("strict-stop", strict_model, "terminate"),
        ("legacy-endpoint-hold", strict_model, "clamp-and-flag"),
        ("br-training-envelope", training_model, "terminate"),
    ]
    results = [run_case(args.binary, *definition) for definition in definitions]
    rows = [result[0] for result in results]
    summaries = [result[1] for result in results]
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(summaries, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
