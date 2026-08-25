"""Compare an adapted RC/HPA flight log with a simulator replay.

Both inputs must already follow the simulator CSV coordinate and unit contract.
This tool intentionally does not auto-align time or estimate sensor bias: those
effects are validation targets and must not be silently tuned away.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

DEFAULT_COLUMNS = [
    "altitude_m",
    "airspeed_mps",
    "alpha_deg",
    "pitch_deg",
    "flight_path_deg",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
]

LABELS = {
    "altitude_m": "Altitude [m]",
    "airspeed_mps": "Airspeed [m/s]",
    "alpha_deg": "Alpha [deg]",
    "pitch_deg": "Pitch [deg]",
    "flight_path_deg": "Flight path [deg]",
    "p_rad_s": "Roll rate [rad/s]",
    "q_rad_s": "Pitch rate [rad/s]",
    "r_rad_s": "Yaw rate [rad/s]",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--measured-csv", type=Path, required=True)
    parser.add_argument("--simulation-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--metrics-json", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    parser.add_argument("--columns", nargs="+", default=DEFAULT_COLUMNS)
    parser.add_argument(
        "--relative-columns",
        nargs="*",
        default=[],
        help="subtract each trace's first value before comparison",
    )
    parser.add_argument(
        "--simulation-time-offset-s",
        type=float,
        default=0.0,
        help="query simulation at measured_time + offset; determine from a shared trigger",
    )
    return parser.parse_args()


def read_numeric_csv(path: Path, columns: list[str]) -> dict[str, np.ndarray]:
    required = ["time_s", *columns]
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(missing)}")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError(f"{path}: at least two samples are required")
    result = {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in required
    }
    if not all(np.isfinite(values).all() for values in result.values()):
        raise ValueError(f"{path}: non-finite numeric value")
    if not np.all(np.diff(result["time_s"]) > 0.0):
        raise ValueError(f"{path}: time_s must be strictly increasing")
    return result


def compare(args: argparse.Namespace) -> tuple[list[dict[str, float]], dict]:
    measured = read_numeric_csv(args.measured_csv, args.columns)
    simulation = read_numeric_csv(args.simulation_csv, args.columns)
    query_time = measured["time_s"] + args.simulation_time_offset_s
    overlap = (query_time >= simulation["time_s"][0]) & (
        query_time <= simulation["time_s"][-1]
    )
    if np.count_nonzero(overlap) < 2:
        raise ValueError("fewer than two measured samples overlap the simulation")

    measured_time = measured["time_s"][overlap]
    query_time = query_time[overlap]
    rows = [{"time_s": float(value)} for value in measured_time]
    metrics: dict[str, object] = {
        "measured_samples": int(measured_time.size),
        "simulation_time_offset_s": args.simulation_time_offset_s,
        "relative_columns": args.relative_columns,
        "states": {},
    }

    for column in args.columns:
        measured_values = measured[column][overlap].copy()
        simulated_values = np.interp(
            query_time, simulation["time_s"], simulation[column]
        )
        if column in args.relative_columns:
            measured_values -= measured_values[0]
            simulated_values -= simulated_values[0]
        residual = simulated_values - measured_values
        metrics["states"][column] = {
            "bias": float(np.mean(residual)),
            "mae": float(np.mean(np.abs(residual))),
            "rmse": float(np.sqrt(np.mean(np.square(residual)))),
            "max_abs": float(np.max(np.abs(residual))),
        }
        for row, measured_value, simulated_value, error in zip(
            rows, measured_values, simulated_values, residual, strict=True
        ):
            row[f"measured_{column}"] = float(measured_value)
            row[f"simulated_{column}"] = float(simulated_value)
            row[f"residual_{column}"] = float(error)
    return rows, metrics


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path: Path, rows: list[dict[str, float]], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    column_count = 2
    row_count = (len(columns) + column_count - 1) // column_count
    figure, axes = plt.subplots(
        row_count, column_count, figsize=(14, 3.6 * row_count), squeeze=False
    )
    time = [row["time_s"] for row in rows]
    for axis, column in zip(axes.flat, columns, strict=False):
        axis.plot(time, [row[f"measured_{column}"] for row in rows], label="Measured")
        axis.plot(
            time,
            [row[f"simulated_{column}"] for row in rows],
            label="Simulation",
            linestyle="--",
        )
        axis.set_xlabel("Measured time [s]")
        axis.set_ylabel(LABELS.get(column, column))
        axis.grid(True, alpha=0.3)
        axis.legend()
    for axis in axes.flat[len(columns) :]:
        axis.set_visible(False)
    figure.suptitle("Flight-log holdout validation")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    unknown_relative = set(args.relative_columns) - set(args.columns)
    if unknown_relative:
        raise ValueError(
            f"relative columns not selected for comparison: {sorted(unknown_relative)}"
        )
    rows, metrics = compare(args)
    write_csv(args.output_csv, rows)
    args.metrics_json.parent.mkdir(parents=True, exist_ok=True)
    args.metrics_json.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_plot(args.plot, rows, args.columns)
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
