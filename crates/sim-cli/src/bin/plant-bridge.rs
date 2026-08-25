//! Line-oriented bridge between an external controller and the Rust aircraft plant.

use std::{
    env,
    io::{self, BufRead, BufWriter, Write},
    path::PathBuf,
    process::ExitCode,
};

use serde::Deserialize;
use sim_cli::config::LoadedSimulation;
use sim_cli::session::{PlantObservation, PlantSession};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ControlCommand {
    elevator_command_rad: f64,
    #[serde(default)]
    rudder_command_rad: f64,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("plant-bridge: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let mut loaded = LoadedSimulation::load(&args.model_path)?;
    if args.gust_wind_ned_mps.iter().any(|value| *value != 0.0) {
        loaded.file.environment.one_minus_cosine_gust.enabled = true;
        loaded
            .file
            .environment
            .one_minus_cosine_gust
            .peak_wind_ned_mps = args.gust_wind_ned_mps;
    }
    let mut plant = PlantSession::new(loaded)?;
    let stdout = io::stdout();
    let mut writer = BufWriter::new(stdout.lock());
    write_observation(&mut writer, plant.observe(args.dt_s)?)?;

    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let command: ControlCommand = serde_json::from_str(&line)?;
        let observation = plant.step(
            command.elevator_command_rad,
            command.rudder_command_rad,
            args.dt_s,
        )?;
        write_observation(&mut writer, observation)?;
    }
    Ok(())
}

fn write_observation(
    writer: &mut impl Write,
    observation: PlantObservation,
) -> Result<(), io::Error> {
    serde_json::to_writer(&mut *writer, &observation)?;
    writer.write_all(b"\n")?;
    writer.flush()
}

struct BridgeArgs {
    model_path: PathBuf,
    dt_s: f64,
    gust_wind_ned_mps: [f64; 3],
}

fn parse_args() -> Result<BridgeArgs, Box<dyn std::error::Error>> {
    let mut model_path = PathBuf::from("models/qx18-br-training-envelope.json");
    let mut dt_s: f64 = 0.01;
    let mut gust_wind_ned_mps: [f64; 3] = [0.0; 3];
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--model" => {
                model_path = PathBuf::from(args.next().ok_or("--model requires a path")?);
            }
            "--dt" => dt_s = args.next().ok_or("--dt requires seconds")?.parse()?,
            "--gust-north-mps" => {
                gust_wind_ned_mps[0] = args.next().ok_or("gust value required")?.parse()?;
            }
            "--gust-east-mps" => {
                gust_wind_ned_mps[1] = args.next().ok_or("gust value required")?.parse()?;
            }
            "--gust-down-mps" => {
                gust_wind_ned_mps[2] = args.next().ok_or("gust value required")?.parse()?;
            }
            "--help" | "-h" => {
                println!(
                    "plant-bridge [--model MODEL.json] [--dt SECONDS] \
                     [--gust-north-mps V] [--gust-east-mps V] [--gust-down-mps V]\n\
                     emits one initial observation, then accepts one JSON control command per line"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err("--dt must be finite and positive".into());
    }
    if gust_wind_ned_mps.iter().any(|value| !value.is_finite()) {
        return Err("gust components must be finite".into());
    }
    Ok(BridgeArgs {
        model_path,
        dt_s,
        gust_wind_ned_mps,
    })
}
