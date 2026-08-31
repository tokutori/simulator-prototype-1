"""Verify actual-UF2 sensor recovery summaries."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_summary(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def require_common(summary: dict[str, Any], label: str) -> None:
    if summary["failsafe_activation_count"] != 1:
        raise ValueError(f"{label}: expected one failsafe activation")
    if summary["recovery_valid_to_rearmed_control_updates"] != 19:
        raise ValueError(f"{label}: recovery did not rearm on the twentieth valid update")
    if summary["safety_failsafe_at_end"]:
        raise ValueError(f"{label}: firmware remained in failsafe")
    if summary["maximum_reascent_m"] != 0:
        raise ValueError(f"{label}: recovery trajectory re-ascended")
    if summary["deadline_miss_activation_count"] != 0:
        raise ValueError(f"{label}: recovery caused a deadline miss")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--transient", type=Path, required=True)
    parser.add_argument("--single-reset", type=Path, required=True)
    parser.add_argument("--persistent-reset", type=Path, required=True)
    args = parser.parse_args()

    transient = load_summary(args.transient)
    single = load_summary(args.single_reset)
    persistent = load_summary(args.persistent_reset)

    require_common(transient, "transient")
    require_common(single, "single reset")
    require_common(persistent, "persistent reset")

    if transient["sensor_reinitializations_observed"] != 0:
        raise ValueError("transient: three read failures must not reinitialize all devices")
    if single["sensor_reinitializations_observed"] != 1:
        raise ValueError("single reset: expected exactly one device reinitialization")
    if persistent["sensor_reinitializations_observed"] < 2:
        raise ValueError("persistent reset: expected repeated retries before recovery")

    print(
        json.dumps(
            {
                "transient_reinitializations": transient["sensor_reinitializations_observed"],
                "single_reset_reinitializations": single["sensor_reinitializations_observed"],
                "persistent_reset_reinitializations": persistent[
                    "sensor_reinitializations_observed"
                ],
                "persistent_invalid_duration_s": persistent["invalid_sample_duration_s"],
                "interpretation": "actual-UF2 software recovery only; physical bus recovery remains unvalidated",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
