"""Explore pull-out scheduling without permitting a positive flight path."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt

from control_sweep import metrics

PULL_OUT_STARTS = [6.0, 6.5, 7.0, 7.5]
PULL_OUT_FULLS = [7.5, 8.0, 8.5, 9.0]
LAUNCH_ALPHA_TARGETS = [4.5, 6.0, 7.5]
LOOKAHEADS = [0.10, 0.15, 0.25]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    return parser.parse_args()


def run_case(binary: Path, base_model: dict, case: tuple[float, ...]) -> dict[str, float | int]:
    start, full, alpha_target, lookahead = case
    model = json.loads(json.dumps(base_model))
    controller = model["reference_controller"]
    controller["pull_out_start_airspeed_mps"] = start
    controller["pull_out_full_airspeed_mps"] = full
    controller["launch_target_alpha_deg"] = alpha_target
    controller["flight_path_lookahead_s"] = lookahead
    with tempfile.TemporaryDirectory(prefix="tokutori-launch-strategy-") as temporary:
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
    result.update(
        {
            "pull_out_start_mps": start,
            "pull_out_full_mps": full,
            "launch_alpha_target_deg": alpha_target,
            "lookahead_s": lookahead,
        }
    )
    return result


def write_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(12, 7), constrained_layout=True)
    accepted = [row for row in rows if int(row["positive_gamma_samples"]) == 0]
    rejected = [row for row in rows if int(row["positive_gamma_samples"]) != 0]
    scatter = axis.scatter(
        [float(row["pullout_loss_m"]) for row in accepted],
        [float(row["max_alpha_deg"]) for row in accepted],
        c=[float(row["minimum_flight_path_deg"]) for row in accepted],
        cmap="viridis",
        s=55,
        label="no positive flight-path samples",
    )
    if rejected:
        axis.scatter(
            [float(row["pullout_loss_m"]) for row in rejected],
            [float(row["max_alpha_deg"]) for row in rejected],
            marker="x",
            color="#d62728",
            label="positive flight path occurred",
        )
    selected = next(
        row
        for row in rows
        if float(row["pull_out_start_mps"]) == 6.0
        and float(row["pull_out_full_mps"]) == 7.5
        and float(row["launch_alpha_target_deg"]) == 6.0
        and float(row["lookahead_s"]) == 0.10
    )
    axis.scatter(
        float(selected["pullout_loss_m"]),
        float(selected["max_alpha_deg"]),
        marker="*",
        facecolors="none",
        edgecolors="black",
        linewidths=1.8,
        s=260,
        label="selected schedule",
        zorder=5,
    )
    axis.set_xlabel("Altitude loss at -3 deg recovery [m]")
    axis.set_ylabel("Maximum alpha [deg]")
    axis.set_title(
        "QX-18 launch scheduling sweep\n"
        "135 deterministic BR-training-model cases; lower-left is preferable"
    )
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.colorbar(scatter, ax=axis, label="Minimum flight-path angle [deg]")
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    cases = [
        (start, full, alpha_target, lookahead)
        for start in PULL_OUT_STARTS
        for full in PULL_OUT_FULLS
        if full > start
        for alpha_target in LAUNCH_ALPHA_TARGETS
        for lookahead in LOOKAHEADS
    ]
    with ThreadPoolExecutor(max_workers=6) as executor:
        rows = list(executor.map(lambda case: run_case(args.binary, base_model, case), cases))
    rows.sort(key=lambda row: float(row["score"]))
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(rows[:10], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
