"""Replay a Rust FDM control history in an independent JSBSim plant.

This is an evaluation adapter, not a production dependency. It generates a
temporary JSBSim aircraft from the same JSON coefficient database, replays the
recorded elevator history, and compares state histories.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import tempfile
from pathlib import Path

import jsbsim
import matplotlib.pyplot as plt

M_TO_FT = 3.280839895013123
MPS_TO_FPS = M_TO_FT
KG_TO_LB = 2.2046226218487757
KG_M2_TO_SLUG_FT2 = 0.7375621492772656
SLUG_FT3_TO_KG_M3 = 515.3788183931961


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--rust-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot", type=Path, required=True)
    parser.add_argument("--compare-seconds", type=float, default=5.0)
    return parser.parse_args()


def table_xml(points: list[dict[str, float]], key: str) -> str:
    rows = "\n".join(
        f"{math.radians(point['alpha_deg']):.15g} {point[key]:.15g}"
        for point in points
    )
    return f"""<table>
      <independentVar lookup="row">aero/alpha-rad</independentVar>
      <tableData>
{rows}
      </tableData>
</table>"""


def ground_effect_drag_xml(
    model: dict, points: list[dict[str, float]], derivatives: dict
) -> str:
    ground_effect = model["aerodynamics"]["ground_effect"]
    if not ground_effect["enabled"]:
        return ""

    geometry = model["reference_geometry"]
    height_over_span = f"""<quotient>
          <sum><property>position/h-agl-ft</property><value>{ground_effect['wing_height_offset_m'] * M_TO_FT:.15g}</value></sum>
          <value>{geometry['span_m'] * M_TO_FT:.15g}</value>
        </quotient>"""
    scaled_height = f"""<product>
        <value>{ground_effect['correlation_gain']:.15g}</value>
        <pow>{height_over_span}<value>{ground_effect['height_exponent']:.15g}</value></pow>
      </product>"""
    ratio = f"""<quotient>
      <sum><value>{ground_effect['minimum_induced_drag_ratio']:.15g}</value>{scaled_height}</sum>
      <sum><value>1</value>{scaled_height}</sum>
    </quotient>"""
    lift_coefficient = f"""<sum>
      {table_xml(points, 'cl')}
      <product><property>fcs/elevator-pos-rad</property><value>{derivatives['cl_elevator']:.15g}</value></product>
    </sum>"""
    return f"""<function name="aero/qx18/CD-ground-effect-delta"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>
        <difference>{ratio}<value>1</value></difference>
        <value>{ground_effect['induced_drag_factor']:.15g}</value>
        <pow>{lift_coefficient}<value>2</value></pow>
      </product></function>"""


def aircraft_xml(model: dict) -> str:
    mass = model["mass_properties"]
    inertia = mass["inertia_body_kg_m2"]
    geometry = model["reference_geometry"]
    aero = model["aerodynamics"]
    derivatives = aero["derivatives_per_rad"]
    points = aero["longitudinal_table"]
    elevator_limit = model["actuators"]["elevator"]["max_abs_deg"]
    ground_effect_drag = ground_effect_drag_xml(model, points, derivatives)

    # JSBSim XML uses the aeronautical product-of-inertia convention, so its
    # ixz entry is the negative of this project's literal tensor matrix entry.
    jsbsim_ixz = -inertia["ixz"] * KG_M2_TO_SLUG_FT2
    return f"""<?xml version="1.0"?>
<fdm_config name="QX18_REFERENCE" version="2.0" release="BETA">
  <fileheader>
    <author>tokutori simulator1 evaluation adapter</author>
    <filecreationdate>2026-08-25</filecreationdate>
    <version>0.1</version>
    <description>Independent JSBSim transcription of the QX-18 public reconstruction.</description>
  </fileheader>
  <metrics>
    <wingarea unit="FT2">{geometry['area_m2'] * M_TO_FT * M_TO_FT:.15g}</wingarea>
    <wingspan unit="FT">{geometry['span_m'] * M_TO_FT:.15g}</wingspan>
    <chord unit="FT">{geometry['chord_m'] * M_TO_FT:.15g}</chord>
    <htailarea unit="FT2">0</htailarea><htailarm unit="FT">0</htailarm>
    <vtailarea unit="FT2">0</vtailarea><vtailarm unit="FT">0</vtailarm>
    <location name="AERORP" unit="IN"><x>0</x><y>0</y><z>0</z></location>
    <location name="EYEPOINT" unit="IN"><x>0</x><y>0</y><z>0</z></location>
    <location name="VRP" unit="IN"><x>0</x><y>0</y><z>0</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2">{inertia['ixx'] * KG_M2_TO_SLUG_FT2:.15g}</ixx>
    <iyy unit="SLUG*FT2">{inertia['iyy'] * KG_M2_TO_SLUG_FT2:.15g}</iyy>
    <izz unit="SLUG*FT2">{inertia['izz'] * KG_M2_TO_SLUG_FT2:.15g}</izz>
    <ixy unit="SLUG*FT2">0</ixy><ixz unit="SLUG*FT2">{jsbsim_ixz:.15g}</ixz><iyz unit="SLUG*FT2">0</iyz>
    <emptywt unit="LBS">{mass['mass_kg'] * KG_TO_LB:.15g}</emptywt>
    <location name="CG" unit="IN"><x>0</x><y>0</y><z>0</z></location>
  </mass_balance>
  <ground_reactions/>
  <propulsion/>
  <flight_control name="QX18 controls">
    <channel name="Pitch">
      <aerosurface_scale name="Elevator Control">
        <input>fcs/elevator-cmd-norm</input>
        <gain>{math.pi / 180.0:.15g}</gain>
        <range><min>{-elevator_limit:.15g}</min><max>{elevator_limit:.15g}</max></range>
        <output>fcs/elevator-pos-rad</output>
      </aerosurface_scale>
    </channel>
  </flight_control>
  <aerodynamics>
    <axis name="DRAG">
      <function name="aero/qx18/CD"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>
        {table_xml(points, 'cd')}
      </product></function>
      {ground_effect_drag}
    </axis>
    <axis name="LIFT">
      <function name="aero/qx18/CL-base"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>
        {table_xml(points, 'cl')}
      </product></function>
      <function name="aero/qx18/CL-elevator"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>
        <property>fcs/elevator-pos-rad</property><value>{derivatives['cl_elevator']:.15g}</value>
      </product></function>
    </axis>
    <axis name="PITCH">
      <function name="aero/qx18/Cm-base"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>
        {table_xml(points, 'cm')}
      </product></function>
      <function name="aero/qx18/Cm-q"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>
        <property>velocities/q-aero-rad_sec</property><property>aero/ci2vel</property>
        <value>{derivatives['cm_pitch_rate']:.15g}</value>
      </product></function>
      <function name="aero/qx18/Cm-elevator"><product>
        <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>
        <property>fcs/elevator-pos-rad</property><value>{derivatives['cm_elevator']:.15g}</value>
      </product></function>
    </axis>
  </aerodynamics>
</fdm_config>
"""


def read_rust_rows(path: Path, compare_seconds: float) -> list[dict[str, float]]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = [
            {
                key: value if key in {"aero_in_range", "vertical_speed_estimate_valid"} else float(value)
                for key, value in row.items()
            }
            for row in csv.DictReader(handle)
            if float(row["time_s"]) <= compare_seconds
        ]
    if len(rows) < 2:
        raise ValueError("Rust CSV must contain at least two samples")
    return rows


def matching_density_altitude(fdm: jsbsim.FGFDMExec, target_density: float, initial: dict) -> float:
    lower_m, upper_m = 0.0, 5000.0
    for _ in range(40):
        altitude_m = (lower_m + upper_m) * 0.5
        set_initial_conditions(fdm, initial, altitude_m, initial["altitude_m"])
        fdm.run_ic()
        density = fdm["atmosphere/rho-slugs_ft3"] * SLUG_FT3_TO_KG_M3
        if density > target_density:
            lower_m = altitude_m
        else:
            upper_m = altitude_m
    return (lower_m + upper_m) * 0.5


def set_initial_conditions(
    fdm: jsbsim.FGFDMExec, initial: dict, altitude_m: float, agl_m: float
) -> None:
    fdm["ic/h-sl-ft"] = altitude_m * M_TO_FT
    fdm["ic/terrain-elevation-ft"] = (altitude_m - agl_m) * M_TO_FT
    fdm["ic/vt-fps"] = initial["airspeed_mps"] * MPS_TO_FPS
    fdm["ic/alpha-deg"] = initial["alpha_deg"]
    fdm["ic/beta-deg"] = 0.0
    fdm["ic/phi-deg"] = initial["roll_deg"]
    fdm["ic/theta-deg"] = initial["pitch_deg"]
    fdm["ic/psi-true-deg"] = initial["heading_deg"]
    fdm["ic/lat-gc-deg"] = 35.0
    fdm["ic/long-gc-deg"] = 135.0


def jsbsim_sample(fdm: jsbsim.FGFDMExec, initial_altitude_m: float) -> dict[str, float]:
    pitch_deg = fdm["attitude/theta-deg"]
    alpha_deg = math.degrees(fdm["aero/alpha-rad"])
    return {
        "jsbsim_altitude_delta_m": fdm["position/h-sl-ft"] / M_TO_FT - initial_altitude_m,
        "jsbsim_airspeed_mps": fdm["velocities/vtrue-fps"] / MPS_TO_FPS,
        "jsbsim_alpha_deg": alpha_deg,
        "jsbsim_pitch_deg": pitch_deg,
        "jsbsim_flight_path_deg": pitch_deg - alpha_deg,
        "jsbsim_q_rad_s": fdm["velocities/q-rad_sec"],
    }


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def compare(model: dict, rust_rows: list[dict[str, float]]) -> tuple[list[dict[str, float]], dict[str, float]]:
    dt_s = rust_rows[1]["time_s"] - rust_rows[0]["time_s"]
    with tempfile.TemporaryDirectory(prefix="tokutori-jsbsim-") as temporary:
        root = Path(temporary)
        aircraft_dir = root / "aircraft" / "qx18"
        aircraft_dir.mkdir(parents=True)
        (aircraft_dir / "qx18.xml").write_text(aircraft_xml(model), encoding="utf-8")

        fdm = jsbsim.FGFDMExec(str(root))
        fdm.set_debug_level(0)
        fdm.set_dt(dt_s)
        if not fdm.load_model("qx18"):
            raise RuntimeError("JSBSim failed to load generated QX-18 model")
        initial = model["initial_state"]
        altitude_m = matching_density_altitude(fdm, model["environment"]["density_kg_m3"], initial)
        set_initial_conditions(fdm, initial, altitude_m, initial["altitude_m"])
        fdm.run_ic()
        density = fdm["atmosphere/rho-slugs_ft3"] * SLUG_FT3_TO_KG_M3

        combined: list[dict[str, float]] = []
        initial_rust_altitude = rust_rows[0]["altitude_m"]
        for index, rust in enumerate(rust_rows):
            if index > 0:
                limit = model["actuators"]["elevator"]["max_abs_deg"]
                fdm["fcs/elevator-cmd-norm"] = rust["elevator_deg"] / limit
                if not fdm.run():
                    raise RuntimeError(f"JSBSim stopped at sample {index}")
            row = {
                "time_s": rust["time_s"],
                "elevator_deg": rust["elevator_deg"],
                "rust_altitude_delta_m": rust["altitude_m"] - initial_rust_altitude,
                "rust_airspeed_mps": rust["airspeed_mps"],
                "rust_alpha_deg": rust["alpha_deg"],
                "rust_pitch_deg": rust["pitch_deg"],
                "rust_flight_path_deg": rust["flight_path_deg"],
                "rust_q_rad_s": rust["q_rad_s"],
            }
            row.update(jsbsim_sample(fdm, altitude_m))
            combined.append(row)

    pairs = [
        ("altitude_delta_m", "m"),
        ("airspeed_mps", "m/s"),
        ("alpha_deg", "deg"),
        ("pitch_deg", "deg"),
        ("flight_path_deg", "deg"),
        ("q_rad_s", "rad/s"),
    ]
    metrics = {"matched_density_kg_m3": density, "matched_altitude_m": altitude_m, "dt_s": dt_s}
    for name, _unit in pairs:
        metrics[f"rms_{name}"] = rms(
            [row[f"rust_{name}"] - row[f"jsbsim_{name}"] for row in combined]
        )
        metrics[f"max_abs_{name}"] = max(
            abs(row[f"rust_{name}"] - row[f"jsbsim_{name}"]) for row in combined
        )
    return combined, metrics


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    time = [row["time_s"] for row in rows]
    series = [
        ("altitude_delta_m", "Altitude change [m]"),
        ("airspeed_mps", "Airspeed [m/s]"),
        ("alpha_deg", "Alpha [deg]"),
        ("pitch_deg", "Pitch [deg]"),
        ("flight_path_deg", "Flight path [deg]"),
        ("q_rad_s", "Pitch rate [rad/s]"),
    ]
    figure, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    for axis, (name, label) in zip(axes.flat, series, strict=True):
        axis.plot(time, [row[f"rust_{name}"] for row in rows], label="Rust RK4", linewidth=2)
        axis.plot(time, [row[f"jsbsim_{name}"] for row in rows], label="JSBSim 1.3.1", linestyle="--")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.suptitle("QX-18 FDM independent replay comparison")
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    model = json.loads(args.model.read_text(encoding="utf-8"))
    rust_rows = read_rust_rows(args.rust_csv, args.compare_seconds)
    rows, metrics = compare(model, rust_rows)
    write_csv(args.output_csv, rows)
    plot(args.plot, rows)
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
