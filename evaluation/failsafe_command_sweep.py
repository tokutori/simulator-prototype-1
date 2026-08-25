"""Sweep fixed elevator failsafe commands after loss of all sensor updates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt


def load_nominal_commands(path: Path) -> list[float]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [math.radians(float(row["elevator_command_deg"])) for row in csv.DictReader(handle)]


def simulate(
    bridge: Path,
    model: Path,
    nominal_commands: list[float],
    fault_start_s: float,
    fixed_command_deg: float | None,
    duration_s: float,
) -> dict[str, float | bool | str]:
    process = subprocess.Popen(
        [str(bridge), "--model", str(model), "--dt", "0.01"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )
    if process.stdin is None or process.stdout is None:
        raise RuntimeError("plant bridge pipes were not created")
    observation = json.loads(process.stdout.readline())
    minimum_altitude = float(observation["altitude_m"])
    maximum_reascent = 0.0
    maximum_flight_path_deg = math.degrees(float(observation["flight_path_rad"]))
    failed_command_rad: float | None = None
    step = 0
    while float(observation["time_s"]) < duration_s and not observation["surface_contact"]:
        nominal = nominal_commands[min(step, len(nominal_commands) - 1)]
        if float(observation["time_s"]) < fault_start_s + 0.03:
            command = nominal
        else:
            if failed_command_rad is None:
                failed_command_rad = nominal if fixed_command_deg is None else math.radians(fixed_command_deg)
            command = failed_command_rad
        process.stdin.write(json.dumps({"elevator_command_rad": command, "rudder_command_rad": 0.0}) + "\n")
        process.stdin.flush()
        observation = json.loads(process.stdout.readline())
        altitude = float(observation["altitude_m"])
        minimum_altitude = min(minimum_altitude, altitude)
        maximum_reascent = max(maximum_reascent, altitude - minimum_altitude)
        maximum_flight_path_deg = max(
            maximum_flight_path_deg,
            math.degrees(float(observation["flight_path_rad"])),
        )
        step += 1
    process.stdin.close()
    process.wait(timeout=5)
    return {
        "fault_start_s": fault_start_s,
        "strategy": "hold-last" if fixed_command_deg is None else f"fixed-{fixed_command_deg:.2f}-deg",
        "fixed_command_deg": math.nan if fixed_command_deg is None else fixed_command_deg,
        "maximum_reascent_m": maximum_reascent,
        "maximum_flight_path_deg": maximum_flight_path_deg,
        "end_time_s": float(observation["time_s"]),
        "final_altitude_m": float(observation["altitude_m"]),
        "surface_contact": bool(observation["surface_contact"]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, default=Path("target/debug/plant-bridge.exe"))
    parser.add_argument("--model", type=Path, default=Path("models/qx18-br-training-envelope.json"))
    parser.add_argument("--nominal", type=Path, default=Path("reports/virtual-platform.csv"))
    parser.add_argument("--output-csv", type=Path, default=Path("reports/failsafe-command-sweep.csv"))
    parser.add_argument("--plot", type=Path, default=Path("reports/failsafe-command-sweep.png"))
    args = parser.parse_args()

    starts = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0]
    commands = [0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0]
    nominal = load_nominal_commands(args.nominal)
    rows = [
        simulate(args.bridge, args.model, nominal, start, command, 12.0)
        for start in starts
        for command in [*commands, None]
    ]

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    fixed = [row for row in rows if row["strategy"] != "hold-last"]
    grid = [
        [
            float(next(row["maximum_reascent_m"] for row in fixed
                       if row["fault_start_s"] == start and row["fixed_command_deg"] == command))
            for command in commands
        ]
        for start in starts
    ]
    figure, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    image = axes[0].imshow(grid, aspect="auto", origin="lower", cmap="magma")
    axes[0].set_xticks(range(len(commands)), [str(value) for value in commands])
    axes[0].set_yticks(range(len(starts)), [str(value) for value in starts])
    axes[0].set_xlabel("Fixed failsafe elevator (deg, positive nose-down)")
    axes[0].set_ylabel("Sensor-loss time (s)")
    axes[0].set_title("Maximum re-ascent under persistent sensor loss (m)")
    figure.colorbar(image, ax=axes[0], label="Maximum re-ascent (m)")

    hold_rows = [row for row in rows if row["strategy"] == "hold-last"]
    axes[1].plot(starts, [float(row["maximum_reascent_m"]) for row in hold_rows], "o-", label="hold-last")
    for command in (0.0, 0.5, 1.0, 2.0):
        command_rows = [row for row in fixed if row["fixed_command_deg"] == command]
        axes[1].plot(
            starts,
            [float(row["maximum_reascent_m"]) for row in command_rows],
            "o-",
            label=f"fixed {command:.2f} deg",
        )
    axes[1].set_xlabel("Sensor-loss time (s)")
    axes[1].set_ylabel("Maximum re-ascent (m)")
    axes[1].set_yscale("symlog", linthresh=0.01)
    axes[1].grid(alpha=0.25)
    axes[1].legend(ncol=3, fontsize=8)
    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.plot, dpi=180)


if __name__ == "__main__":
    main()
