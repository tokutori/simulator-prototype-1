"""Stress fixed failsafe commands against deterministic aircraft-model uncertainty."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from failsafe_command_sweep import load_nominal_commands, simulate
from robustness_sweep import cases


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-binary", type=Path, default=Path("target/debug/sim-cli.exe"))
    parser.add_argument("--bridge", type=Path, default=Path("target/debug/plant-bridge.exe"))
    parser.add_argument("--model", type=Path, default=Path("models/qx18-br-training-envelope.json"))
    parser.add_argument("--fault-start-s", type=float, default=2.0)
    parser.add_argument("--output-csv", type=Path, default=Path("reports/failsafe-robustness-sweep.csv"))
    parser.add_argument("--plot", type=Path, default=Path("reports/failsafe-robustness-sweep.png"))
    return parser.parse_args()


def run_model_case(
    sim_binary: Path,
    bridge: Path,
    base_model: dict,
    definition: tuple,
    fault_start_s: float,
) -> list[dict[str, float | bool | str]]:
    case_id, description, mutation = definition
    model = json.loads(json.dumps(base_model))
    mutation(model)
    with tempfile.TemporaryDirectory(prefix="tokutori-failsafe-robustness-") as temporary:
        root = Path(temporary)
        model_path = root / "model.json"
        nominal_path = root / "nominal.csv"
        model_path.write_text(json.dumps(model, ensure_ascii=False), encoding="utf-8")
        completed = subprocess.run(
            [
                str(sim_binary),
                "--model",
                str(model_path),
                "--duration",
                "12",
                "--dt",
                "0.01",
                "--output",
                str(nominal_path),
            ],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        if completed.returncode != 0:
            raise RuntimeError(f"{case_id}: {completed.stderr}")
        nominal_commands = load_nominal_commands(nominal_path)
        rows = []
        for command_deg in (0.75, 1.0, 1.5):
            result = simulate(
                bridge,
                model_path,
                nominal_commands,
                fault_start_s,
                command_deg,
                12.0,
            )
            result["model_case"] = case_id
            result["model_description"] = description
            rows.append(result)
        return rows


def write_csv(path: Path, rows: list[dict[str, float | bool | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | bool | str]]) -> None:
    model_cases = list(dict.fromkeys(str(row["model_case"]) for row in rows))
    commands = (0.75, 1.0, 1.5)
    lookup = {
        (str(row["model_case"]), float(row["fixed_command_deg"])): row
        for row in rows
    }
    reascent = np.array(
        [
            [float(lookup[(case, command)]["maximum_reascent_m"]) for command in commands]
            for case in model_cases
        ]
    )
    maximum_gamma = np.array(
        [
            [float(lookup[(case, command)]["maximum_flight_path_deg"]) for command in commands]
            for case in model_cases
        ]
    )
    contact = np.array(
        [
            [bool(lookup[(case, command)]["surface_contact"]) for command in commands]
            for case in model_cases
        ]
    )

    figure, axes = plt.subplots(1, 2, figsize=(10, 9), constrained_layout=True)
    images = (
        axes[0].imshow(reascent, aspect="auto", cmap="magma", vmin=0.0),
        axes[1].imshow(maximum_gamma, aspect="auto", cmap="coolwarm", vmin=-3.0, vmax=3.0),
    )
    axes[0].set_title("Maximum re-ascent (m)")
    axes[1].set_title("Maximum flight-path angle (deg)")
    for axis, image, values in zip(axes, images, (reascent, maximum_gamma)):
        axis.set_xticks(range(len(commands)), [f"+{value:.2f}" for value in commands])
        axis.set_yticks(range(len(model_cases)), model_cases)
        axis.set_xlabel("Fixed failsafe elevator (deg, nose down)")
        figure.colorbar(image, ax=axis, shrink=0.75)
        for row_index in range(len(model_cases)):
            for column_index in range(len(commands)):
                value = values[row_index, column_index]
                axis.text(
                    column_index,
                    row_index,
                    f"{value:.2f}",
                    ha="center",
                    va="center",
                    fontsize=7,
                    color="white" if abs(value) > 0.4 else "black",
                )
                if contact[row_index, column_index]:
                    axis.scatter(column_index, row_index, marker="x", color="#00ffff", s=45, linewidths=1.5)
    figure.suptitle(
        "Persistent sensor loss at 2.0 s under deterministic model uncertainty\n"
        "cyan x = surface contact; ranges are engineering probes, not identified distributions"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    with ThreadPoolExecutor(max_workers=4) as executor:
        grouped_rows = list(
            executor.map(
                lambda definition: run_model_case(
                    args.sim_binary,
                    args.bridge,
                    base_model,
                    definition,
                    args.fault_start_s,
                ),
                cases(),
            )
        )
    rows = [row for group in grouped_rows for row in group]
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    worst = sorted(rows, key=lambda row: float(row["maximum_reascent_m"]), reverse=True)[:5]
    print(json.dumps(worst, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
