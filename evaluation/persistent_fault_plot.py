"""Plot actual-UF2 responses to persistent sensor loss at several flight phases."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_log(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"empty log: {path}")
    return {
        key: [float(row[key]) for row in rows]
        for key in rows[0]
        if key is not None
    }


def value_at_or_after(log: dict[str, list[float]], key: str, time_s: float) -> tuple[float, float]:
    for sample_time, value in zip(log["time_s"], log[key]):
        if sample_time >= time_s:
            return sample_time, value
    return log["time_s"][-1], log[key][-1]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reports", type=Path, default=Path("reports"))
    parser.add_argument("--input-prefix", default="failsafe-bno-start")
    parser.add_argument(
        "--title",
        default="Actual RP2040 UF2: persistent BNO055 status fault with +0.75 deg nose-down failsafe",
    )
    parser.add_argument("--hide-reference-command", action="store_true")
    parser.add_argument(
        "--plot",
        type=Path,
        default=Path("reports/persistent-fault-response.png"),
    )
    args = parser.parse_args()

    fault_starts = (0.5, 1.0, 2.0, 3.0, 5.0, 7.0)
    logs = {
        start: read_log(args.reports / f"{args.input_prefix}-{start:.1f}.csv")
        for start in fault_starts
    }

    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    for start, log in logs.items():
        label = f"loss at {start:.1f} s"
        line = axes[0].plot(log["time_s"], log["altitude_m"], label=label)[0]
        color = line.get_color()
        axes[1].plot(log["time_s"], log["flight_path_deg"], color=color)
        axes[2].plot(log["time_s"], log["elevator_command_deg"], color=color)
        for axis, key in zip(
            axes,
            ("altitude_m", "flight_path_deg", "elevator_command_deg"),
        ):
            marker_time, marker_value = value_at_or_after(log, key, start)
            axis.scatter(marker_time, marker_value, color=color, marker="o", s=18, zorder=3)
        if log["surface_contact"][-1] > 0.5:
            axes[0].scatter(
                log["time_s"][-1],
                log["altitude_m"][-1],
                color=color,
                marker="x",
                s=55,
                linewidths=2,
                zorder=4,
            )

    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title(
        f"{args.title}\ncircles = fault onset, crosses = surface contact"
    )
    axes[0].legend(loc="best", ncol=2, fontsize=8)

    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Flight path (deg)")
    axes[1].set_ylim(-26.0, 2.0)

    if not args.hide_reference_command:
        axes[2].axhline(
            0.75,
            color="black",
            linestyle=":",
            linewidth=1.0,
            label="failsafe +0.75 deg",
        )
    axes[2].set_ylabel("Elevator command (deg)\npositive = nose down")
    axes[2].set_xlabel("Plant time (s; accelerated MCU timing is not validated)")
    if not args.hide_reference_command:
        axes[2].legend(loc="best", fontsize=8)

    for axis in axes:
        axis.grid(alpha=0.25)
        axis.set_xlim(0.0, 12.0)

    args.plot.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.plot, dpi=180)


if __name__ == "__main__":
    main()
