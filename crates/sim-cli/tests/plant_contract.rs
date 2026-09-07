//! Exercise the actual line protocol, not only an in-process plant helper.

use std::io::{BufRead, BufReader, Read, Write};
use std::process::{Command, Stdio};

#[test]
fn strict_plant_bridge_stops_on_envelope_exit() {
    let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../models/qx18-public-reconstruction.json");
    let mut child = Command::new(env!("CARGO_BIN_EXE_plant-bridge"))
        .arg("--model")
        .arg(model)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("bridge starts");
    let mut input = child.stdin.take().expect("stdin");
    let mut output = BufReader::new(child.stdout.take().expect("stdout"));
    let mut line = String::new();
    assert!(output.read_line(&mut line).expect("initial response") > 0);
    let mut stopped = false;
    for _ in 0..1000 {
        writeln!(input, "{{\"elevator_command_rad\":0.174532925}}").expect("write command");
        input.flush().expect("flush command");
        line.clear();
        if output.read_line(&mut line).expect("step response") == 0 {
            stopped = true;
            break;
        }
        let observation: serde_json::Value = serde_json::from_str(&line).expect("JSON");
        assert_eq!(observation["aero_in_range"], true);
    }
    drop(input);
    if !stopped {
        child.kill().expect("stop unexpected continuing bridge");
    }
    let status = child.wait().expect("bridge exits");
    let mut error = String::new();
    child
        .stderr
        .take()
        .expect("stderr")
        .read_to_string(&mut error)
        .expect("error message");
    assert!(stopped, "strict model must terminate");
    assert!(!status.success());
    assert!(error.contains("AeroEnvelopeExit"), "{error}");
}
