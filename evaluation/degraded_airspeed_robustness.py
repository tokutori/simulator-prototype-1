"""Run the actual RP2040 UF2 with persistent SDP loss across model stress cases."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib.pyplot as plt

from robustness_sweep import cases


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=Path("models/qx18-br-training-envelope.json"))
    parser.add_argument("--sensor-fault", choices=("none", "sdp-nack"), default="sdp-nack")
    parser.add_argument("--fault-start-s", type=float, default=2.0)
    parser.add_argument("--output-csv", type=Path, default=Path("reports/degraded-airspeed-robustness.csv"))
    parser.add_argument("--plot", type=Path, default=Path("reports/degraded-airspeed-robustness.png"))
    parser.add_argument(
        "--log-directory",
        type=Path,
        help="retain one actual-UF2 CSV and JSON log per model case",
    )
    return parser.parse_args()


def run_case(
    repository: Path,
    base_model: dict,
    definition: tuple,
    sensor_fault: str,
    fault_start_s: float,
    log_directory: Path | None,
) -> dict:
    case_id, description, mutation = definition
    model = json.loads(json.dumps(base_model))
    mutation(model)
    with tempfile.TemporaryDirectory(prefix="tokutori-uf2-airspeed-") as temporary:
        root = Path(temporary)
        model_path = root / "model.json"
        log_path = root / "run.csv"
        summary_path = root / "summary.json"
        model_path.write_text(json.dumps(model, ensure_ascii=False), encoding="utf-8")
        completed = subprocess.run(
            [
                "npm.cmd",
                "run",
                "simulate",
                "--prefix",
                "virtual-platform",
                "--",
                "--steps",
                "1200",
                "--timing-acceleration",
                "50",
                "--model",
                str(model_path),
                "--sensor-fault",
                sensor_fault,
                "--fault-start-s",
                str(fault_start_s),
                "--fault-duration-s",
                "20",
                "--output",
                str(log_path),
                "--summary",
                str(summary_path),
            ],
            cwd=repository,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        if completed.returncode != 0:
            raise RuntimeError(f"{case_id}: {completed.stderr}\n{completed.stdout}")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if log_directory is not None:
            log_directory.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(log_path, log_directory / f"{case_id}.csv")
            shutil.copyfile(summary_path, log_directory / f"{case_id}.json")
    return {
        "model_case": case_id,
        "model_description": description,
        "sensor_fault": sensor_fault,
        "fault_start_s": fault_start_s,
        "maximum_reascent_m": summary["maximum_reascent_m"],
        "maximum_flight_path_deg": summary["maximum_flight_path_deg"],
        "positive_flight_path_samples": summary["positive_flight_path_samples"],
        "final_altitude_m": summary["final_altitude_m"],
        "end_time_s": summary["simulated_s"],
        "surface_contact": summary["simulated_s"] < 11.99,
        "failsafe_activation_count": summary["failsafe_activation_count"],
        "raw_invalid_duration_s": summary["invalid_sample_duration_s"],
        "deadline_miss_activation_count": summary["deadline_miss_activation_count"],
        "timing_validated": summary["timing_validated"],
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict], sensor_fault: str, fault_start_s: float) -> None:
    labels = [str(row["model_case"]) for row in rows]
    colors = ["#d62728" if row["surface_contact"] else "#1f77b4" for row in rows]
    reascent = [float(row["maximum_reascent_m"]) for row in rows]
    gamma = [float(row["maximum_flight_path_deg"]) for row in rows]
    figure, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    axes[0].bar(labels, reascent, color=colors)
    axes[0].set_ylim(0.0, max(max(reascent) * 1.1, 0.01))
    if all(value == 0.0 for value in reascent):
        axes[0].text(
            0.5,
            0.55,
            "all 17 cases: 0.000 m",
            transform=axes[0].transAxes,
            ha="center",
            va="center",
            fontsize=15,
        )
    axes[0].set_ylabel("Maximum re-ascent (m)")
    if sensor_fault == "none":
        title = "Actual RP2040 UF2: nominal sensors"
    else:
        title = (
            f"Actual RP2040 UF2: persistent SDP NACK at {fault_start_s:.1f} s "
            "with degraded airspeed control"
        )
    axes[0].set_title(title)
    axes[1].bar(labels, gamma, color=colors)
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Maximum flight-path angle (deg)")
    axes[1].set_xlabel("Deterministic model-uncertainty case (red = surface contact)")
    axes[1].tick_params(axis="x", rotation=55)
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    figure.suptitle("Ranges are engineering probes, not identified probability distributions")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)


def main() -> None:
    args = parse_args()
    repository = Path(__file__).resolve().parent.parent
    base_model = json.loads(args.model.read_text(encoding="utf-8"))
    with ThreadPoolExecutor(max_workers=4) as executor:
        rows = list(
            executor.map(
                lambda definition: run_case(
                    repository,
                    base_model,
                    definition,
                    args.sensor_fault,
                    args.fault_start_s,
                    args.log_directory,
                ),
                cases(),
            )
        )
    write_csv(args.output_csv, rows)
    plot(args.plot, rows, args.sensor_fault, args.fault_start_s)
    print(json.dumps(sorted(rows, key=lambda row: row["maximum_reascent_m"], reverse=True), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
