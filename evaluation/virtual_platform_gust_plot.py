#!/usr/bin/env python3
"""Plot actual-UF2 nominal and vertical-gust closed-loop runs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nominal", type=Path, required=True)
    parser.add_argument("--upgust-0-5", type=Path, required=True)
    parser.add_argument("--upgust-1-0", type=Path, required=True)
    parser.add_argument("--upgust-2-0", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def maximum_reascent(rows: list[dict[str, float]]) -> float:
    running_minimum = float("inf")
    maximum = 0.0
    for row in rows:
        running_minimum = min(running_minimum, row["altitude_m"])
        maximum = max(maximum, row["altitude_m"] - running_minimum)
    return maximum


def metrics(rows: list[dict[str, float]]) -> dict[str, float | int]:
    elevator = [
        row["elevator_actual_deg"]
        for row in rows
        if 55.0 <= row["north_m"] <= 110.0
    ]
    flight_path = [row["flight_path_deg"] for row in rows]
    dt_s = rows[1]["time_s"] - rows[0]["time_s"]
    return {
        "maximum_reascent_m": maximum_reascent(rows),
        "maximum_flight_path_deg": max(flight_path),
        "positive_flight_path_samples": sum(value > 0.0 for value in flight_path),
        "maximum_abs_elevator_deg": max(map(abs, elevator)),
        "elevator_saturation_s_in_gust_window": dt_s
        * sum(abs(value) >= 9.9 for value in elevator),
        "elevator_total_variation_deg": sum(
            abs(right - left) for left, right in zip(elevator, elevator[1:], strict=False)
        ),
    }


def main() -> None:
    args = parse_args()
    cases = [
        ("nominal", read_rows(args.nominal)),
        ("upgust 0.5 m/s", read_rows(args.upgust_0_5)),
        ("upgust 1.0 m/s", read_rows(args.upgust_1_0)),
        ("upgust 2.0 m/s", read_rows(args.upgust_2_0)),
    ]
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    for label, rows in cases:
        reascent = maximum_reascent(rows)
        legend = f"{label} (re-ascent {reascent:.3f} m)"
        time = [row["time_s"] for row in rows]
        axes[0].plot(time, [row["altitude_m"] for row in rows], label=legend)
        axes[1].plot(time, [row["flight_path_deg"] for row in rows], label=label)
        axes[2].plot(time, [row["elevator_actual_deg"] for row in rows], label=label)
    axes[0].set(ylabel="altitude (m)", title="Actual RP2040 UF2: deterministic vertical-gust stress")
    axes[0].legend()
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set(ylabel="flight path (deg)")
    axes[1].legend()
    axes[2].set(xlabel="time (s)", ylabel="elevator (deg)\npositive = nose down")
    axes[2].legend()
    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.plot, dpi=160)
    plt.close(figure)
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(
            json.dumps({label: metrics(rows) for label, rows in cases}, indent=2) + "\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
