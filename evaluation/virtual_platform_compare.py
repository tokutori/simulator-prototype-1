#!/usr/bin/env python3
"""Compare native shared-controller and RP2040-UF2 closed-loop trajectories."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--virtual", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [
            {key: numeric(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def numeric(value: str) -> float:
    if value == "true":
        return 1.0
    if value == "false":
        return 0.0
    return float(value)


def rmse(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def main() -> None:
    args = parse_args()
    host = read_rows(args.host)
    virtual = read_rows(args.virtual)
    host_by_tick = {round(row["time_s"], 5): row for row in host}
    pairs = [
        (host_by_tick[round(row["time_s"], 5)], row)
        for row in virtual
        if round(row["time_s"], 5) in host_by_tick
    ]
    if not pairs:
        raise SystemExit("no matching timestamps")

    time = [virtual_row["time_s"] for _, virtual_row in pairs]
    altitude_difference = [
        virtual_row["altitude_m"] - host_row["altitude_m"]
        for host_row, virtual_row in pairs
    ]
    flight_path_difference = [
        virtual_row["flight_path_deg"] - host_row["flight_path_deg"]
        for host_row, virtual_row in pairs
    ]
    elevator_difference = [
        virtual_row["elevator_actual_deg"] - host_row["elevator_deg"]
        for host_row, virtual_row in pairs
    ]
    summary = {
        "matched_samples": len(pairs),
        "altitude_rmse_m": rmse(altitude_difference),
        "altitude_final_difference_m": altitude_difference[-1],
        "flight_path_rmse_deg": rmse(flight_path_difference),
        "flight_path_final_difference_deg": flight_path_difference[-1],
        "elevator_actual_rmse_deg": rmse(elevator_difference),
        "virtual_max_reascent_m": maximum_reascent(
            [virtual_row["altitude_m"] for _, virtual_row in pairs]
        ),
        "virtual_positive_flight_path_samples": sum(
            virtual_row["flight_path_deg"] > 0.0 for _, virtual_row in pairs
        ),
        "interpretation": (
            "software-path equivalence only; coefficient and real-aircraft validity remain unproven"
        ),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    axes[0, 0].plot(time, [row["altitude_m"] for row, _ in pairs], label="native host")
    axes[0, 0].plot(time, [row["altitude_m"] for _, row in pairs], "--", label="RP2040 UF2")
    axes[0, 0].set(title="Altitude", ylabel="m")
    axes[0, 0].legend()

    axes[0, 1].plot(time, [row["flight_path_deg"] for row, _ in pairs], label="native host")
    axes[0, 1].plot(time, [row["flight_path_deg"] for _, row in pairs], "--", label="RP2040 UF2")
    axes[0, 1].axhline(0.0, color="black", linewidth=0.8)
    axes[0, 1].set(title="Flight-path angle", ylabel="deg")
    axes[0, 1].legend()

    axes[1, 0].plot(time, [row["elevator_deg"] for row, _ in pairs], label="native actual")
    axes[1, 0].plot(time, [row["elevator_actual_deg"] for _, row in pairs], "--", label="UF2 actual")
    axes[1, 0].plot(time, [row["elevator_command_deg"] for _, row in pairs], ":", label="UF2 PWM command")
    axes[1, 0].set(title="Elevator (positive = nose down)", xlabel="time (s)", ylabel="deg")
    axes[1, 0].legend()

    axes[1, 1].plot(time, altitude_difference, label="altitude (m)")
    axes[1, 1].plot(time, flight_path_difference, label="flight path (deg)")
    axes[1, 1].plot(time, elevator_difference, label="actual elevator (deg)")
    axes[1, 1].axhline(0.0, color="black", linewidth=0.8)
    axes[1, 1].set(title="UF2 minus native", xlabel="time (s)")
    axes[1, 1].legend()
    figure.suptitle("Shared FBW core: native adapter vs actual RP2040 UF2")
    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.plot, dpi=160)
    plt.close(figure)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def maximum_reascent(altitudes: list[float]) -> float:
    running_minimum = math.inf
    maximum = 0.0
    for altitude in altitudes:
        running_minimum = min(running_minimum, altitude)
        maximum = max(maximum, altitude - running_minimum)
    return maximum


if __name__ == "__main__":
    main()
