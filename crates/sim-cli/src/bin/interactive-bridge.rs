//! Line-oriented pilot-in-the-loop bridge around the Rust FBW and aircraft plant.

use std::{
    env,
    io::{self, BufRead, BufWriter, Write},
    path::PathBuf,
    process::ExitCode,
};

use serde::{Deserialize, Serialize};
use sim_cli::{
    config::LoadedSimulation,
    interactive::{InteractiveController, MixedControl, PilotCommand},
    session::{PlantObservation, PlantSession},
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PilotCommandMessage {
    pilot_elevator: f64,
    pilot_rudder: f64,
    autonomy: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct InteractiveObservation {
    #[serde(flatten)]
    plant: PlantObservation,
    #[serde(flatten)]
    control: MixedControl,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("interactive-bridge: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let loaded = LoadedSimulation::load(&args.model_path)?;
    let mut controller = InteractiveController::from_model(&loaded.file);
    let mut plant = PlantSession::new(loaded)?;
    let mut observation = plant.observe(args.dt_s)?;
    let stdout = io::stdout();
    let mut writer = BufWriter::new(stdout.lock());
    let initial_control = MixedControl {
        pilot_elevator: 0.0,
        pilot_rudder: 0.0,
        autonomy: args.initial_autonomy,
        manual_elevator_command_rad: 0.0,
        manual_rudder_command_rad: 0.0,
        automatic_elevator_command_rad: 0.0,
        automatic_rudder_command_rad: 0.0,
        mixed_elevator_command_rad: 0.0,
        mixed_rudder_command_rad: 0.0,
    };
    write_observation(&mut writer, observation, initial_control)?;

    for line in io::stdin().lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let message: PilotCommandMessage = serde_json::from_str(&line)?;
        let control = controller.step(
            observation,
            PilotCommand {
                elevator: message.pilot_elevator,
                rudder: message.pilot_rudder,
                autonomy: message.autonomy,
            },
            args.dt_s,
        )?;
        observation = plant.step(
            control.mixed_elevator_command_rad,
            control.mixed_rudder_command_rad,
            args.dt_s,
        )?;
        write_observation(&mut writer, observation, control)?;
        if observation.surface_contact {
            break;
        }
    }
    Ok(())
}

fn write_observation(
    writer: &mut impl Write,
    plant: PlantObservation,
    control: MixedControl,
) -> Result<(), io::Error> {
    serde_json::to_writer(&mut *writer, &InteractiveObservation { plant, control })?;
    writer.write_all(b"\n")?;
    writer.flush()
}

struct BridgeArgs {
    model_path: PathBuf,
    dt_s: f64,
    initial_autonomy: f64,
}

fn parse_args() -> Result<BridgeArgs, Box<dyn std::error::Error>> {
    let mut model_path = PathBuf::from("models/qx18-br-training-envelope.json");
    let mut dt_s = 0.01_f64;
    let mut initial_autonomy = 1.0_f64;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--model" => model_path = PathBuf::from(args.next().ok_or("--model requires a path")?),
            "--dt" => dt_s = args.next().ok_or("--dt requires seconds")?.parse()?,
            "--autonomy" => {
                initial_autonomy = args.next().ok_or("--autonomy requires a value")?.parse()?;
            }
            "--help" | "-h" => {
                println!(
                    "interactive-bridge [--model MODEL.json] [--dt SECONDS] [--autonomy 0..1]\n\
                     emits an initial observation, accepts pilot_elevator/pilot_rudder/autonomy JSON"
                );
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err("--dt must be finite and positive".into());
    }
    if !initial_autonomy.is_finite() || !(0.0..=1.0).contains(&initial_autonomy) {
        return Err("--autonomy must be within 0..1".into());
    }
    Ok(BridgeArgs {
        model_path,
        dt_s,
        initial_autonomy,
    })
}
