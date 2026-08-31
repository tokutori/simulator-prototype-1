"""Plot actual-UF2 sensor-fault injection runs."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_log(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return {
        key: [float(row[key]) for row in rows]
        for key in rows[0]
        if key is not None
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reports", type=Path, default=Path("reports"))
    parser.add_argument("--plot", type=Path, default=Path("reports/fault-injection.png"))
    args = parser.parse_args()

    logs = {
        "nominal": read_log(args.reports / "fault-update-nominal.csv"),
        "1-update SDP CRC": read_log(args.reports / "fault-update-one.csv"),
        "3-update SDP CRC": read_log(args.reports / "fault-update-sdp-three.csv"),
        "3-update SDP NACK": read_log(args.reports / "fault-update-sdp-nack-three.csv"),
        "3-update BNO status": read_log(args.reports / "fault-update-bno-three.csv"),
        "1-update BNO reset": read_log(args.reports / "fault-update-bno-reset.csv"),
        "3-update AS5600 magnet": read_log(args.reports / "fault-update-as-three.csv"),
        "3-update DPS ready": read_log(args.reports / "fault-update-dps-three.csv"),
    }
    nominal = logs["nominal"]

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    for label in ("nominal", "1-update SDP CRC", "3-update SDP CRC"):
        log = logs[label]
        axes[0].plot(log["time_s"], log["elevator_command_deg"], label=label)
    axes[0].axvline(3.0, color="black", linestyle=":", linewidth=1)
    axes[0].set_xlim(2.5, 4.5)
    axes[0].set_ylabel("Elevator command (deg)")
    axes[0].set_title("Actual production UF2: deterministic sensor-fault response")
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="best")

    for index, label in enumerate(("1-update SDP CRC", "3-update SDP CRC")):
        log = logs[label]
        offset = index * 2.4
        axes[1].step(
            log["time_s"],
            [value + offset for value in log["sensor_sample_invalid"]],
            where="post",
            label=f"{label}: raw invalid",
        )
        axes[1].step(
            log["time_s"],
            [value + 1.1 + offset for value in log["safety_failsafe"]],
            where="post",
            label=f"{label}: unarmed/failsafe",
        )
    axes[1].set_xlim(2.8, 4.2)
    axes[1].set_yticks([])
    axes[1].set_ylabel("GPIO state")
    axes[1].grid(axis="x", alpha=0.25)
    axes[1].legend(loc="upper right", fontsize=8, ncol=2)

    for label in (
        "3-update SDP CRC",
        "3-update SDP NACK",
        "3-update BNO status",
        "1-update BNO reset",
        "3-update AS5600 magnet",
        "3-update DPS ready",
    ):
        log = logs[label]
        altitude_delta_cm = [
            100.0 * (fault_altitude - nominal_altitude)
            for fault_altitude, nominal_altitude in zip(log["altitude_m"], nominal["altitude_m"])
        ]
        axes[2].plot(log["time_s"], altitude_delta_cm, label=label)
    axes[2].axvline(3.0, color="black", linestyle=":", linewidth=1)
    axes[2].set_xlabel("Plant time (s; accelerated MCU timing is not validated)")
    axes[2].set_ylabel("Altitude delta from nominal (cm)")
    axes[2].grid(alpha=0.25)
    axes[2].legend(loc="best", fontsize=8)

    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.plot, dpi=180)


if __name__ == "__main__":
    main()
