//! Host adapter for loading a model, running a deterministic scenario, and writing CSV.

use std::{
    env,
    fs::{self, File},
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

use flight_dynamics_core::{
    Actuator, ActuatorError, AeroLoads, ControlSurfaceDeflection, LongitudinalLinearization,
    RigidBodyState, SensorError, SensorSample, SensorSuite, StepError, aerodynamic_loads,
    linearize_steady_glide, steady_glide_trim, step_rk4,
};
use nalgebra::{Complex, SMatrix};
use plotters::coord::Shift;
use plotters::prelude::*;
use sim_cli::config::{ConfigError, LoadedSimulation, deg_to_rad, rad_to_deg};
use sim_cli::controller::{ControlDecision, ReferenceControllerState};
use sim_cli::session::{PlantTermination, terminal_condition};

const DEFAULT_MODEL: &str = "models/qx18-br-training-envelope.json";
// A 120 s horizon covers about 1.2 km at the nominal 10 m/s glide speed.
// Simulation still terminates earlier on water contact or an aerodynamic
// envelope exit; this is a recording capacity, not a forced flight length.
const DEFAULT_DURATION_S: f64 = 120.0;
const DEFAULT_DT_S: f64 = 0.01;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), AppError> {
    let args = Args::parse(env::args().skip(1))?;
    if args.help {
        print_help();
        return Ok(());
    }
    let loaded = LoadedSimulation::load(&args.model_path).map_err(AppError::Config)?;
    let model = loaded.model().map_err(AppError::Config)?;
    let environment = loaded.environment();
    if let Ok(trim) = steady_glide_trim(model, environment, 0.0) {
        eprintln!(
            "nominal_unpowered_trim airspeed={:.3}m/s alpha={:.3}deg flight_path={:.3}deg pitch={:.3}deg L/D={:.1}",
            trim.airspeed_mps,
            rad_to_deg(trim.alpha_rad),
            rad_to_deg(trim.flight_path_rad),
            rad_to_deg(trim.pitch_rad),
            trim.coefficients.lift / trim.coefficients.drag,
        );
    }
    if args.linearization_output_path.is_some() || args.modes_plot_path.is_some() {
        let analysis = analyze_longitudinal_modes(model, environment)?;
        report_modes(&analysis);
        if let Some(path) = &args.linearization_output_path {
            write_linearization(path, &analysis)?;
        }
        if let Some(path) = &args.modes_plot_path {
            render_modes_plot(path, &analysis, &loaded.file.metadata.name)?;
        }
    }
    let summary = if let Some(path) = &args.output_path {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(AppError::Io)?;
        }
        let file = File::create(path).map_err(AppError::Io)?;
        let mut writer = BufWriter::new(file);
        simulate(&loaded, args.duration_s, args.dt_s, &mut writer)?
    } else {
        let stdout = io::stdout();
        let mut writer = BufWriter::new(stdout.lock());
        simulate(&loaded, args.duration_s, args.dt_s, &mut writer)?
    };
    if let Some(path) = &args.plot_path {
        render_plot(path, &summary.samples, &loaded.file.metadata.name)?;
    }
    eprintln!(
        "model={} status={} simulated={:.2}s steps={} final_altitude={:.2}m final_flight_path={:.2}deg max_flight_path={:.2}deg max_reascent={:.3}m positive_flight_path_samples={} glide_ratio_last_quarter={:.1} min_ground_effect_ratio={:.3} max_wind={:.2}m/s aero_out_of_range_samples={} termination={}",
        loaded.file.metadata.name,
        loaded.file.metadata.validation_status,
        summary.time_s,
        summary.steps,
        summary.final_altitude_m,
        summary.final_flight_path_deg,
        summary.max_flight_path_deg,
        summary.max_reascent_m,
        summary.positive_flight_path_samples,
        summary.glide_ratio_last_quarter,
        summary.min_ground_effect_ratio,
        summary.max_wind_mps,
        summary.aero_out_of_range_samples,
        summary.termination.as_str(),
    );
    Ok(())
}

fn simulate(
    loaded: &LoadedSimulation,
    duration_s: f64,
    dt_s: f64,
    writer: &mut dyn Write,
) -> Result<Summary, AppError> {
    let max_steps = calculate_max_steps(duration_s, dt_s)?;
    let model = loaded.model().map_err(AppError::Config)?;
    let mut state = loaded.initial_rigid_body_state();
    let elevator_config = loaded.elevator_config();
    let rudder_config = loaded.rudder_config();
    let controller = &loaded.file.reference_controller;
    // The launch preset is applied before release, so the virtual servo begins
    // at its commanded position rather than unrealistically slewing from zero.
    let launch_elevator_rad = deg_to_rad(controller.launch_elevator_feedforward_deg);
    let mut elevator =
        Actuator::new(launch_elevator_rad, elevator_config).map_err(AppError::Actuator)?;
    let mut rudder = Actuator::new(0.0, rudder_config).map_err(AppError::Actuator)?;
    let mut sensors = SensorSuite::new(loaded.sensor_model()).map_err(AppError::Sensor)?;
    let mut controls = ControlSurfaceDeflection {
        elevator_rad: launch_elevator_rad,
        ..ControlSurfaceDeflection::default()
    };
    let mut reference_controller_state = ReferenceControllerState::default();
    let mut steps = 0_u32;
    let mut aero_out_of_range_samples = 0_u32;
    let mut samples = Vec::with_capacity(
        usize::try_from(max_steps)
            .unwrap_or(usize::MAX)
            .saturating_add(1),
    );

    writeln!(
        writer,
        "time_s,north_m,east_m,altitude_m,u_mps,v_mps,w_mps,roll_deg,pitch_deg,yaw_deg,flight_path_deg,p_rad_s,q_rad_s,r_rad_s,airspeed_mps,alpha_deg,beta_deg,elevator_deg,elevator_command_deg,rudder_deg,sensor_pitch_deg,sensor_q_rad_s,sensor_airspeed_mps,sensor_dp_pa,sensor_baro_altitude_m,sensor_alpha_deg,estimated_vertical_speed_mps,vertical_speed_estimate_valid,wind_north_mps,wind_east_mps,wind_down_mps,ground_effect_induced_drag_ratio,aero_in_range,surface_contact"
    )
    .map_err(AppError::Io)?;

    let termination = loop {
        let time_s = f64::from(steps) * dt_s;
        let environment = loaded.environment_at_north(state.position_ned_m.x);
        let loads = aerodynamic_loads(model, state, controls, environment);
        let sensor_sample = sensors
            .step(state, loads, environment, model.mass_kg, dt_s)
            .map_err(AppError::Sensor)?;
        let aero_in_range = model.longitudinal.contains(loads.condition.alpha_rad);
        let control_decision = reference_controller_state.step(controller, sensor_sample, dt_s);
        if !aero_in_range {
            aero_out_of_range_samples = aero_out_of_range_samples.saturating_add(1);
        }
        let sample = TelemetrySample {
            time_s,
            state,
            loads,
            controls,
            sensors: sensor_sample,
            control_decision,
            environment,
            aero_in_range,
        };
        write_sample(writer, &sample)?;
        samples.push(sample);
        if let Some(reason) = terminal_condition(
            state.position_ned_m.z >= 0.0,
            aero_in_range,
            loaded.file.aerodynamics.out_of_range_policy,
        ) {
            break match reason {
                PlantTermination::SurfaceContact => Termination::SurfaceContact,
                PlantTermination::AeroEnvelopeExit => Termination::AeroEnvelopeExit,
            };
        }
        if steps >= max_steps {
            break Termination::Duration;
        }
        let elevator_command = f64::from(control_decision.elevator_command_rad);
        controls.elevator_rad = elevator
            .step(elevator_command, elevator_config, dt_s)
            .map_err(AppError::Actuator)?;
        controls.rudder_rad = rudder
            .step(0.0, rudder_config, dt_s)
            .map_err(AppError::Actuator)?;
        state = step_rk4(model, state, controls, environment, dt_s).map_err(AppError::Step)?;
        steps += 1;
    };
    writer.flush().map_err(AppError::Io)?;
    Ok(build_summary(
        steps,
        dt_s,
        state,
        termination,
        aero_out_of_range_samples,
        samples,
    ))
}

fn build_summary(
    steps: u32,
    dt_s: f64,
    state: RigidBodyState,
    termination: Termination,
    aero_out_of_range_samples: u32,
    samples: Vec<TelemetrySample>,
) -> Summary {
    let glide_ratio_last_quarter = glide_ratio_over_last_quarter(&samples);
    let max_reascent_m = max_reascent_m(&samples);
    let positive_flight_path_samples = samples
        .iter()
        .filter(|sample| flight_path_angle_rad(sample.state) > 0.0)
        .count();
    let max_flight_path_deg = samples
        .iter()
        .map(|sample| rad_to_deg(flight_path_angle_rad(sample.state)))
        .reduce(f64::max)
        .unwrap_or(f64::NAN);
    let min_ground_effect_ratio = samples
        .iter()
        .map(|sample| sample.loads.induced_drag_ground_effect_ratio)
        .reduce(f64::min)
        .unwrap_or(f64::NAN);
    let max_wind_mps = samples
        .iter()
        .map(|sample| sample.environment.wind_ned_mps.norm())
        .reduce(f64::max)
        .unwrap_or(f64::NAN);
    Summary {
        time_s: f64::from(steps) * dt_s,
        steps,
        final_altitude_m: -state.position_ned_m.z,
        final_flight_path_deg: rad_to_deg(flight_path_angle_rad(state)),
        max_flight_path_deg,
        max_reascent_m,
        positive_flight_path_samples,
        glide_ratio_last_quarter,
        min_ground_effect_ratio,
        max_wind_mps,
        termination,
        aero_out_of_range_samples,
        samples,
    }
}

fn max_reascent_m(samples: &[TelemetrySample]) -> f64 {
    let mut running_minimum_altitude_m = f64::INFINITY;
    let mut maximum_reascent_m: f64 = 0.0;
    for sample in samples {
        let altitude_m = -sample.state.position_ned_m.z;
        running_minimum_altitude_m = running_minimum_altitude_m.min(altitude_m);
        maximum_reascent_m = maximum_reascent_m.max(altitude_m - running_minimum_altitude_m);
    }
    maximum_reascent_m
}

fn glide_ratio_over_last_quarter(samples: &[TelemetrySample]) -> f64 {
    let start_index = samples.len().saturating_mul(3) / 4;
    let Some(start) = samples.get(start_index) else {
        return f64::NAN;
    };
    let Some(end) = samples.last() else {
        return f64::NAN;
    };
    let north = end.state.position_ned_m.x - start.state.position_ned_m.x;
    let east = end.state.position_ned_m.y - start.state.position_ned_m.y;
    let horizontal = (north * north + east * east).sqrt();
    let altitude_loss = end.state.position_ned_m.z - start.state.position_ned_m.z;
    if altitude_loss > 0.0 {
        horizontal / altitude_loss
    } else {
        f64::INFINITY
    }
}

fn write_sample(writer: &mut dyn Write, sample: &TelemetrySample) -> Result<(), AppError> {
    let euler = sample.state.attitude_body_to_ned.to_euler();
    writeln!(
        writer,
        "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{:.6},{},{}",
        sample.time_s,
        sample.state.position_ned_m.x,
        sample.state.position_ned_m.y,
        -sample.state.position_ned_m.z,
        sample.state.velocity_body_mps.x,
        sample.state.velocity_body_mps.y,
        sample.state.velocity_body_mps.z,
        rad_to_deg(euler.x),
        rad_to_deg(euler.y),
        rad_to_deg(euler.z),
        rad_to_deg(flight_path_angle_rad(sample.state)),
        sample.state.rates_body_rad_s.x,
        sample.state.rates_body_rad_s.y,
        sample.state.rates_body_rad_s.z,
        sample.loads.condition.airspeed_mps,
        rad_to_deg(sample.loads.condition.alpha_rad),
        rad_to_deg(sample.loads.condition.beta_rad),
        rad_to_deg(sample.controls.elevator_rad),
        rad_to_deg(f64::from(sample.control_decision.elevator_command_rad)),
        rad_to_deg(sample.controls.rudder_rad),
        rad_to_deg(sample.sensors.pitch_rad),
        sample.sensors.gyro_rad_s.y,
        sample.sensors.airspeed_mps,
        sample.sensors.differential_pressure_pa,
        sample.sensors.barometric_altitude_m,
        rad_to_deg(sample.sensors.alpha_rad),
        f64::from(sample.control_decision.estimated_vertical_speed_mps),
        sample.control_decision.vertical_speed_estimate_valid,
        sample.environment.wind_ned_mps.x,
        sample.environment.wind_ned_mps.y,
        sample.environment.wind_ned_mps.z,
        sample.loads.induced_drag_ground_effect_ratio,
        sample.aero_in_range,
        sample.state.position_ned_m.z >= 0.0,
    )
    .map_err(AppError::Io)
}

fn render_plot(path: &Path, samples: &[TelemetrySample], title: &str) -> Result<(), AppError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(AppError::Io)?;
    }
    let root = BitMapBackend::new(path, (1_400, 1_000)).into_drawing_area();
    root.fill(&WHITE).map_err(plot_error)?;
    let areas = root.split_evenly((2, 2));
    plot_trajectory(&areas[0], samples, title)?;
    plot_attitude(&areas[1], samples)?;
    plot_airspeed(&areas[2], samples)?;
    plot_controls(&areas[3], samples)?;
    root.present().map_err(plot_error)
}

fn plot_trajectory(
    area: &DrawingArea<BitMapBackend<'_>, Shift>,
    samples: &[TelemetrySample],
    title: &str,
) -> Result<(), AppError> {
    let north_range = padded_range(samples.iter().map(|sample| sample.state.position_ned_m.x));
    let altitude_range = padded_range(samples.iter().map(|sample| -sample.state.position_ned_m.z));
    let mut chart = ChartBuilder::on(area)
        .caption(
            format!("{title}: trajectory (vertical scale exaggerated; invalid aero red)"),
            ("sans-serif", 18),
        )
        .margin(15)
        .x_label_area_size(45)
        .y_label_area_size(55)
        .build_cartesian_2d(north_range, altitude_range)
        .map_err(plot_error)?;
    chart
        .configure_mesh()
        .x_desc("North distance [m]")
        .y_desc("Altitude [m]")
        .draw()
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples.iter().map(|sample| {
                (
                    sample.state.position_ned_m.x,
                    -sample.state.position_ned_m.z,
                )
            }),
            &BLUE,
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(
            samples
                .iter()
                .filter(|sample| !sample.aero_in_range)
                .map(|sample| {
                    Circle::new(
                        (
                            sample.state.position_ned_m.x,
                            -sample.state.position_ned_m.z,
                        ),
                        2,
                        RED.filled(),
                    )
                }),
        )
        .map_err(plot_error)?;
    Ok(())
}

fn plot_attitude(
    area: &DrawingArea<BitMapBackend<'_>, Shift>,
    samples: &[TelemetrySample],
) -> Result<(), AppError> {
    let time_range = padded_range(samples.iter().map(|sample| sample.time_s));
    let zero_reference_time_range = time_range.clone();
    let angle_range = padded_range(samples.iter().flat_map(|sample| {
        [
            rad_to_deg(sample.state.attitude_body_to_ned.to_euler().y),
            rad_to_deg(sample.loads.condition.alpha_rad),
            rad_to_deg(flight_path_angle_rad(sample.state)),
        ]
    }));
    let mut chart = ChartBuilder::on(area)
        .caption(
            "Pitch (blue) / alpha (red) / flight path (green) / zero path (gray)",
            ("sans-serif", 20),
        )
        .margin(15)
        .x_label_area_size(45)
        .y_label_area_size(55)
        .build_cartesian_2d(time_range, angle_range)
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            [
                (zero_reference_time_range.start, 0.0),
                (zero_reference_time_range.end, 0.0),
            ],
            BLACK.mix(0.35).stroke_width(1),
        ))
        .map_err(plot_error)?;
    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Angle [deg]")
        .draw()
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples.iter().map(|sample| {
                (
                    sample.time_s,
                    rad_to_deg(sample.state.attitude_body_to_ned.to_euler().y),
                )
            }),
            &BLUE,
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_s, rad_to_deg(sample.loads.condition.alpha_rad))),
            &RED,
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples.iter().map(|sample| {
                (
                    sample.time_s,
                    rad_to_deg(flight_path_angle_rad(sample.state)),
                )
            }),
            &GREEN,
        ))
        .map_err(plot_error)?;
    Ok(())
}

fn flight_path_angle_rad(state: RigidBodyState) -> f64 {
    let velocity_ned = state
        .attitude_body_to_ned
        .rotate_body_to_ned(state.velocity_body_mps);
    let horizontal_mps = (velocity_ned.x * velocity_ned.x + velocity_ned.y * velocity_ned.y).sqrt();
    (-velocity_ned.z).atan2(horizontal_mps)
}

fn plot_airspeed(
    area: &DrawingArea<BitMapBackend<'_>, Shift>,
    samples: &[TelemetrySample],
) -> Result<(), AppError> {
    let time_range = padded_range(samples.iter().map(|sample| sample.time_s));
    let airspeed_range = padded_range(samples.iter().flat_map(|sample| {
        [
            sample.loads.condition.airspeed_mps,
            sample.sensors.airspeed_mps,
        ]
    }));
    let mut chart = ChartBuilder::on(area)
        .caption(
            "Airspeed: truth (green) / SDP810 model (blue)",
            ("sans-serif", 22),
        )
        .margin(15)
        .x_label_area_size(45)
        .y_label_area_size(55)
        .build_cartesian_2d(time_range, airspeed_range)
        .map_err(plot_error)?;
    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Airspeed [m/s]")
        .draw()
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_s, sample.loads.condition.airspeed_mps)),
            &GREEN,
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_s, sample.sensors.airspeed_mps)),
            &BLUE,
        ))
        .map_err(plot_error)?;
    Ok(())
}

fn plot_controls(
    area: &DrawingArea<BitMapBackend<'_>, Shift>,
    samples: &[TelemetrySample],
) -> Result<(), AppError> {
    let time_range = padded_range(samples.iter().map(|sample| sample.time_s));
    let control_range = padded_range(samples.iter().flat_map(|sample| {
        [
            rad_to_deg(sample.controls.elevator_rad),
            rad_to_deg(sample.controls.rudder_rad),
        ]
    }));
    let mut chart = ChartBuilder::on(area)
        .caption("Elevator (magenta) / rudder (black)", ("sans-serif", 22))
        .margin(15)
        .x_label_area_size(45)
        .y_label_area_size(55)
        .build_cartesian_2d(time_range, control_range)
        .map_err(plot_error)?;
    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Deflection [deg]")
        .draw()
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_s, rad_to_deg(sample.controls.elevator_rad))),
            &MAGENTA,
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            samples
                .iter()
                .map(|sample| (sample.time_s, rad_to_deg(sample.controls.rudder_rad))),
            &BLACK,
        ))
        .map_err(plot_error)?;
    Ok(())
}

struct LongitudinalModeAnalysis {
    linearization: LongitudinalLinearization,
    eigenvalues: [Complex<f64>; 4],
}

fn analyze_longitudinal_modes(
    model: flight_dynamics_core::AircraftModel<'_>,
    environment: flight_dynamics_core::Environment,
) -> Result<LongitudinalModeAnalysis, AppError> {
    let linearization = linearize_steady_glide(model, environment, 0.0)
        .map_err(|error| AppError::Linearization(format!("{error:?}")))?;
    let flat_matrix = linearization.state_matrix.concat();
    let matrix = SMatrix::<f64, 4, 4>::from_row_slice(&flat_matrix);
    let raw_eigenvalues = matrix.complex_eigenvalues();
    let mut eigenvalues = [
        raw_eigenvalues[0],
        raw_eigenvalues[1],
        raw_eigenvalues[2],
        raw_eigenvalues[3],
    ];
    eigenvalues.sort_by(|left, right| {
        left.re
            .total_cmp(&right.re)
            .then(left.im.total_cmp(&right.im))
    });
    Ok(LongitudinalModeAnalysis {
        linearization,
        eigenvalues,
    })
}

fn report_modes(analysis: &LongitudinalModeAnalysis) {
    let residual = analysis.linearization.trim_residual;
    eprintln!(
        "longitudinal_trim_residual udot={:.3e}m/s2 wdot={:.3e}m/s2 qdot={:.3e}rad/s2",
        residual.forward_acceleration_mps2,
        residual.down_acceleration_mps2,
        residual.pitch_acceleration_rad_s2,
    );
    for (index, eigenvalue) in analysis.eigenvalues.iter().enumerate() {
        let (natural_frequency, damping_ratio, period_s, time_constant_s) =
            mode_metrics(*eigenvalue);
        eprintln!(
            "longitudinal_mode index={} eigenvalue={:+.6}{:+.6}j 1/s wn={:.6}rad/s zeta={:.4} period={:.3}s time_constant={:.3}s",
            index + 1,
            eigenvalue.re,
            eigenvalue.im,
            natural_frequency,
            damping_ratio,
            period_s,
            time_constant_s,
        );
    }
}

fn write_linearization(path: &Path, analysis: &LongitudinalModeAnalysis) -> Result<(), AppError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(AppError::Io)?;
    }
    let file = File::create(path).map_err(AppError::Io)?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "kind,index,value_1,value_2,value_3,value_4").map_err(AppError::Io)?;
    for (row, values) in analysis.linearization.state_matrix.iter().enumerate() {
        writeln!(
            writer,
            "A,{row},{:.12e},{:.12e},{:.12e},{:.12e}",
            values[0], values[1], values[2], values[3]
        )
        .map_err(AppError::Io)?;
    }
    let input = analysis.linearization.elevator_input;
    writeln!(
        writer,
        "B,0,{:.12e},{:.12e},{:.12e},{:.12e}",
        input[0], input[1], input[2], input[3]
    )
    .map_err(AppError::Io)?;
    let residual = analysis.linearization.trim_residual;
    writeln!(
        writer,
        "residual,0,{:.12e},{:.12e},{:.12e},{:.12e}",
        residual.forward_acceleration_mps2,
        residual.down_acceleration_mps2,
        residual.pitch_acceleration_rad_s2,
        residual.pitch_rate_rad_s,
    )
    .map_err(AppError::Io)?;
    for (index, eigenvalue) in analysis.eigenvalues.iter().enumerate() {
        let (natural_frequency, damping_ratio, _, _) = mode_metrics(*eigenvalue);
        writeln!(
            writer,
            "mode,{index},{:.12e},{:.12e},{:.12e},{:.12e}",
            eigenvalue.re, eigenvalue.im, natural_frequency, damping_ratio,
        )
        .map_err(AppError::Io)?;
    }
    writer.flush().map_err(AppError::Io)
}

fn render_modes_plot(
    path: &Path,
    analysis: &LongitudinalModeAnalysis,
    model_name: &str,
) -> Result<(), AppError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(AppError::Io)?;
    }
    let root = BitMapBackend::new(path, (1_100, 760)).into_drawing_area();
    root.fill(&WHITE).map_err(plot_error)?;
    let (chart_area, text_area) = root.split_vertically(540);
    let minimum_real = analysis
        .eigenvalues
        .iter()
        .map(|value| value.re)
        .fold(0.0_f64, f64::min);
    let maximum_real = analysis
        .eigenvalues
        .iter()
        .map(|value| value.re)
        .fold(0.0_f64, f64::max);
    let maximum_imaginary = analysis
        .eigenvalues
        .iter()
        .map(|value| value.im.abs())
        .fold(0.0_f64, f64::max);
    let real_padding = (maximum_real - minimum_real).abs().max(0.1) * 0.15;
    let imaginary_limit = maximum_imaginary.max(0.1) * 1.2;
    let mut chart = ChartBuilder::on(&chart_area)
        .caption(
            format!("{model_name}: open-loop longitudinal modes at steady glide"),
            ("sans-serif", 24),
        )
        .margin(20)
        .x_label_area_size(55)
        .y_label_area_size(65)
        .build_cartesian_2d(
            (minimum_real - real_padding)..(maximum_real + real_padding),
            -imaginary_limit..imaginary_limit,
        )
        .map_err(plot_error)?;
    chart
        .configure_mesh()
        .x_desc("Real part [1/s] (left is stable)")
        .y_desc("Imaginary part [rad/s]")
        .draw()
        .map_err(plot_error)?;
    chart
        .draw_series(LineSeries::new(
            [(0.0, -imaginary_limit), (0.0, imaginary_limit)],
            BLACK.mix(0.35).stroke_width(2),
        ))
        .map_err(plot_error)?;
    chart
        .draw_series(analysis.eigenvalues.iter().map(|value| {
            let color = if value.re <= 0.0 { &GREEN } else { &RED };
            Circle::new((value.re, value.im), 7, color.filled())
        }))
        .map_err(plot_error)?;
    text_area
        .draw(&Text::new(
            "Eigenvalues and derived metrics (green: stable, red: unstable)",
            (20, 25),
            ("sans-serif", 20).into_font(),
        ))
        .map_err(plot_error)?;
    for (index, eigenvalue) in analysis.eigenvalues.iter().enumerate() {
        let (natural_frequency, damping_ratio, period_s, time_constant_s) =
            mode_metrics(*eigenvalue);
        text_area
            .draw(&Text::new(
                format!(
                    "{}: {:+.5} {:+.5}j 1/s    wn={:.4} rad/s    zeta={:.3}    period={:.2} s    tau={:.2} s",
                    index + 1,
                    eigenvalue.re,
                    eigenvalue.im,
                    natural_frequency,
                    damping_ratio,
                    period_s,
                    time_constant_s,
                ),
                (35, 65 + i32::try_from(index).unwrap_or(0) * 34),
                ("sans-serif", 18).into_font(),
            ))
            .map_err(plot_error)?;
    }
    root.present().map_err(plot_error)
}

fn mode_metrics(eigenvalue: Complex<f64>) -> (f64, f64, f64, f64) {
    let natural_frequency = eigenvalue.norm();
    let damping_ratio = if natural_frequency > 0.0 {
        -eigenvalue.re / natural_frequency
    } else {
        f64::NAN
    };
    let period_s = if eigenvalue.im.abs() > 1.0e-12 {
        std::f64::consts::TAU / eigenvalue.im.abs()
    } else {
        f64::INFINITY
    };
    let time_constant_s = if eigenvalue.re < 0.0 {
        -1.0 / eigenvalue.re
    } else {
        f64::INFINITY
    };
    (natural_frequency, damping_ratio, period_s, time_constant_s)
}

fn padded_range(values: impl Iterator<Item = f64>) -> std::ops::Range<f64> {
    let (minimum, maximum) = values.fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(minimum, maximum), value| (minimum.min(value), maximum.max(value)),
    );
    if !minimum.is_finite() || !maximum.is_finite() {
        return 0.0..1.0;
    }
    let span = maximum - minimum;
    let padding = if span > 0.0 {
        span * 0.08
    } else {
        minimum.abs().max(1.0) * 0.08
    };
    (minimum - padding)..(maximum + padding)
}

fn plot_error(error: impl std::fmt::Debug) -> AppError {
    AppError::Plot(format!("{error:?}"))
}

fn calculate_max_steps(duration_s: f64, dt_s: f64) -> Result<u32, AppError> {
    if !duration_s.is_finite() || duration_s <= 0.0 || !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(AppError::InvalidNumericArgument);
    }
    let exact_steps = duration_s / dt_s;
    if exact_steps > f64::from(u32::MAX) {
        return Err(AppError::InvalidNumericArgument);
    }
    // The finite, positive, u32-bounded domain has been established above.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let whole_steps = exact_steps.floor() as u32;
    Ok(whole_steps)
}

#[derive(Debug)]
struct Args {
    model_path: PathBuf,
    output_path: Option<PathBuf>,
    plot_path: Option<PathBuf>,
    linearization_output_path: Option<PathBuf>,
    modes_plot_path: Option<PathBuf>,
    duration_s: f64,
    dt_s: f64,
    help: bool,
}

impl Args {
    fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, AppError> {
        let mut parsed = Self {
            model_path: PathBuf::from(DEFAULT_MODEL),
            output_path: None,
            plot_path: None,
            linearization_output_path: None,
            modes_plot_path: None,
            duration_s: DEFAULT_DURATION_S,
            dt_s: DEFAULT_DT_S,
            help: false,
        };
        let mut arguments = arguments;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => {
                    parsed.model_path = PathBuf::from(next_value(&mut arguments, "--model")?);
                }
                "--output" => {
                    parsed.output_path =
                        Some(PathBuf::from(next_value(&mut arguments, "--output")?));
                }
                "--plot" => {
                    parsed.plot_path = Some(PathBuf::from(next_value(&mut arguments, "--plot")?));
                }
                "--linearization-output" => {
                    parsed.linearization_output_path = Some(PathBuf::from(next_value(
                        &mut arguments,
                        "--linearization-output",
                    )?));
                }
                "--modes-plot" => {
                    parsed.modes_plot_path =
                        Some(PathBuf::from(next_value(&mut arguments, "--modes-plot")?));
                }
                "--duration" => {
                    parsed.duration_s = parse_number(next_value(&mut arguments, "--duration")?)?;
                }
                "--dt" => parsed.dt_s = parse_number(next_value(&mut arguments, "--dt")?)?,
                "--help" | "-h" => parsed.help = true,
                _ => return Err(AppError::UnknownArgument(argument)),
            }
        }
        Ok(parsed)
    }
}

fn next_value(
    arguments: &mut impl Iterator<Item = String>,
    option: &'static str,
) -> Result<String, AppError> {
    arguments.next().ok_or(AppError::MissingValue(option))
}

fn parse_number(value: String) -> Result<f64, AppError> {
    value
        .parse::<f64>()
        .map_err(|_| AppError::InvalidNumber(value))
}

fn print_help() {
    println!(
        "sim-cli - deterministic Birdman FDM prototype\n\
         \nUSAGE:\n  cargo run -p sim-cli -- [OPTIONS]\n\
         \nOPTIONS:\n  --model <PATH>       model JSON (default: {DEFAULT_MODEL})\n  \
         --output <PATH>      write CSV; otherwise stdout\n  --plot <PATH>        write a four-panel PNG chart\n  --linearization-output <PATH> write trim Jacobian and modes CSV\n  --modes-plot <PATH>  write a longitudinal eigenvalue PNG\n  --duration <SECONDS> simulation duration (default: {DEFAULT_DURATION_S})\n  \
         --dt <SECONDS>       fixed RK4 step (default: {DEFAULT_DT_S})\n  -h, --help           show help"
    );
}

#[derive(Debug)]
enum AppError {
    Config(ConfigError),
    Io(io::Error),
    Actuator(ActuatorError),
    Sensor(SensorError),
    Step(StepError),
    Plot(String),
    Linearization(String),
    MissingValue(&'static str),
    InvalidNumber(String),
    InvalidNumericArgument,
    UnknownArgument(String),
}

impl std::fmt::Display for AppError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Config(error) => error.fmt(formatter),
            Self::Io(error) => write!(formatter, "I/O error: {error}"),
            Self::Actuator(error) => write!(formatter, "actuator error: {error:?}"),
            Self::Sensor(error) => write!(formatter, "sensor error: {error:?}"),
            Self::Step(error) => write!(formatter, "integration error: {error:?}"),
            Self::Plot(error) => write!(formatter, "plot error: {error}"),
            Self::Linearization(error) => write!(formatter, "linearization error: {error}"),
            Self::MissingValue(option) => write!(formatter, "{option} の値がありません"),
            Self::InvalidNumber(value) => write!(formatter, "数値として解釈できません: {value}"),
            Self::InvalidNumericArgument => {
                write!(formatter, "durationとdtは有限の正数でなければなりません")
            }
            Self::UnknownArgument(argument) => write!(formatter, "未知の引数です: {argument}"),
        }
    }
}

impl std::error::Error for AppError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Termination {
    Duration,
    SurfaceContact,
    AeroEnvelopeExit,
}

impl Termination {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Duration => "duration",
            Self::SurfaceContact => "surface-contact",
            Self::AeroEnvelopeExit => "aero-envelope-exit",
        }
    }
}

struct Summary {
    time_s: f64,
    steps: u32,
    final_altitude_m: f64,
    final_flight_path_deg: f64,
    max_flight_path_deg: f64,
    max_reascent_m: f64,
    positive_flight_path_samples: usize,
    glide_ratio_last_quarter: f64,
    min_ground_effect_ratio: f64,
    max_wind_mps: f64,
    termination: Termination,
    aero_out_of_range_samples: u32,
    samples: Vec<TelemetrySample>,
}

#[cfg(test)]
mod scenario_tests {
    use super::*;

    #[test]
    fn default_recording_horizon_covers_one_kilometre_at_nominal_speed() {
        let args = Args::parse(std::iter::empty()).expect("default arguments must parse");

        assert_eq!(args.duration_s, 120.0);
        assert!(args.duration_s * 10.0 >= 1_000.0);
    }

    #[test]
    fn long_horizon_records_surface_contact_explicitly() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        let mut csv = Vec::new();
        let summary = simulate(&loaded, DEFAULT_DURATION_S, DEFAULT_DT_S, &mut csv)
            .expect("reference scenario must reach the surface");
        let csv = String::from_utf8(csv).expect("CSV must be UTF-8");

        assert_eq!(summary.termination, Termination::SurfaceContact);
        assert!(summary.time_s < DEFAULT_DURATION_S);
        assert!(
            csv.lines()
                .next()
                .is_some_and(|header| header.ends_with("surface_contact"))
        );
        assert!(
            csv.lines()
                .next_back()
                .is_some_and(|row| row.ends_with("true"))
        );
    }

    #[test]
    fn qx18_reference_control_does_not_reascend() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        let mut csv = Vec::new();
        let summary =
            simulate(&loaded, 12.0, DEFAULT_DT_S, &mut csv).expect("reference scenario must run");

        assert_eq!(summary.positive_flight_path_samples, 0);
        assert!(summary.max_flight_path_deg <= 0.0);
        assert!(summary.max_reascent_m <= 1.0e-9);
    }

    #[test]
    fn barometric_climb_feedback_limits_one_mps_upgust_reascent() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-br-training-envelope.json");
        let mut loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        let gust = &mut loaded.file.environment.one_minus_cosine_gust;
        gust.enabled = true;
        gust.start_north_m = 60.0;
        gust.length_m = 40.0;
        gust.peak_wind_ned_mps = [0.0, 0.0, -1.0];
        let mut csv = Vec::new();
        let summary = simulate(&loaded, 20.0, DEFAULT_DT_S, &mut csv).expect("gust case must run");

        assert!(summary.max_reascent_m > 0.0);
        assert!(summary.max_reascent_m < 0.35);
        assert!(summary.max_flight_path_deg > 0.0);
    }

    #[test]
    fn strict_qx18_model_stops_when_launch_leaves_aero_table() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-public-reconstruction.json");
        let mut loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        loaded.file.initial_state.velocity = sim_cli::config::InitialVelocityFile::GroundRelative {
            speed_mps: 5.0,
            flight_path_deg: -3.0,
            track_deg: 0.0,
        };
        loaded.file.initial_state.pitch_deg = -1.318;
        let mut csv = Vec::new();
        let summary = simulate(&loaded, 8.0, DEFAULT_DT_S, &mut csv)
            .expect("strict launch scenario must run until envelope exit");

        assert_eq!(summary.termination, Termination::AeroEnvelopeExit);
        assert_eq!(summary.aero_out_of_range_samples, 1);
        assert!(summary.time_s < 2.0);
    }

    #[test]
    fn qx18_public_model_has_expected_nominal_glide_trim() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-public-reconstruction.json");
        let loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        let trim = steady_glide_trim(
            loaded.model().expect("valid model"),
            loaded.environment(),
            0.0,
        )
        .expect("nominal trim must exist");

        assert!((trim.airspeed_mps - 9.698).abs() < 0.001);
        assert!((rad_to_deg(trim.alpha_rad) - 1.682).abs() < 0.001);
        assert!((rad_to_deg(trim.flight_path_rad) + 1.625).abs() < 0.001);
        assert!((trim.coefficients.lift / trim.coefficients.drag - 35.25).abs() < 0.1);
    }

    #[test]
    fn qx18_public_model_has_stable_longitudinal_modes_at_trim() {
        let model_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/qx18-public-reconstruction.json");
        let loaded = LoadedSimulation::load(&model_path).expect("sample model must load");
        let analysis =
            analyze_longitudinal_modes(loaded.model().expect("valid model"), loaded.environment())
                .expect("linearization must exist");

        assert!(analysis.eigenvalues.iter().all(|value| value.re < 0.0));
        let frequencies = analysis
            .eigenvalues
            .map(Complex::norm)
            .map(|value| (value * 1_000.0).round() / 1_000.0);
        assert_eq!(frequencies, [8.227, 8.227, 0.617, 0.617]);
    }
}

#[derive(Clone, Copy)]
struct TelemetrySample {
    time_s: f64,
    state: RigidBodyState,
    loads: AeroLoads,
    controls: ControlSurfaceDeflection,
    sensors: SensorSample,
    control_decision: ControlDecision,
    environment: flight_dynamics_core::Environment,
    aero_in_range: bool,
}
