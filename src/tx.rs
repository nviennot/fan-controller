use anyhow::{bail, Context, Result};
use clap::Parser;
use num::complex::Complex32 as c32;
use serde::{Deserialize, Serialize};
use soapysdr::{Device, Direction};
use std::f32::consts::TAU;
use std::fs;
use std::path::PathBuf;

// Fan remote OOK protocol at 433.816 MHz
// Code format: [device_id:20][button_hi:4][rolling:4][check:4]
//   button:  5 bits (hi:4 | lo:1)
//   counter: 0-7, increments each press (mod 8)
//   Vendor A: rolling = (lo << 3) | counter, check = rolling ^ hi ^ key
//   Vendor B: rolling = counter, lo XORed into bit 12, check = hi ^ key
const SAMPLE_RATE: f64 = 6_000_000.0;
const CENTER_FREQUENCY: f64 = 433_900_000.0;
const FREQUENCY_VENDOR_A: f64 = 433_899_000.0;
const FREQUENCY_VENDOR_B: f64 = 433_935_500.0;
const BANDWIDTH: f64 = 1_500_000.0;

const LEAD_SILENCE: f32 = 5e-3;
const REPEATS: usize = 6;

// Vendor A: gap-width encoding (fixed pulse, variable gap)
const VA_PULSE: f32 = 260e-6;
const VA_SHORT_GAP: f32 = 500e-6;
const VA_LONG_GAP: f32 = 1500e-6;
const VA_FRAME_GAP: f32 = 4300e-6;

// Vendor B: pulse-width encoding (variable pulse, variable gap, inverted)
const VB_SHORT_PULSE: f32 = 390e-6;
const VB_LONG_PULSE: f32 = 1170e-6;
const VB_SHORT_GAP: f32 = 400e-6;
const VB_LONG_GAP: f32 = 1180e-6;
const VB_FRAME_GAP: f32 = 12000e-6;

const BUTTONS_VENDOR_A: &[(&str, u8)] = &[
    ("toggle_light", 0x08),
    ("off", 0x06),
    ("speed1", 0x10),
    ("speed2", 0x12),
    ("speed3", 0x1C),
    ("speed4", 0x0A),
    ("speed5", 0x0E),
    ("speed6", 0x0C),
    ("fan_off", 0x16),
    ("forward", 0x04),
    ("reverse", 0x11),
    ("waves", 0x14),
    ("1h", 0x02),
    ("4h", 0x18),
    ("8h", 0x03),
];

const BUTTONS_VENDOR_B: &[(&str, u8)] = &[
    ("toggle_light", 0x18),
    ("off", 0x1C),
    ("speed1", 0x16),
    ("speed2", 0x0A),
    ("speed3", 0x0E),
    ("speed4", 0x06),
    ("speed5", 0x12),
    ("speed6", 0x14),
    ("fan_off", 0x08),
    ("direction", 0x1D),
    ("waves", 0x1F),
    ("home", 0x1B),
    ("1h", 0x04),
    ("4h", 0x02),
    ("8h", 0x00),
];

#[derive(Deserialize)]
struct ConfigFan {
    name: String,
    vendor: String,
    device_id: u32,
}

fn xor_nibbles(mut v: u32) -> u8 {
    let mut x = 0u8;
    while v > 0 {
        x ^= (v & 0xF) as u8;
        v >>= 4;
    }
    x
}

use std::collections::HashMap;

#[derive(Deserialize)]
struct Config {
    fans: Vec<ConfigFan>,
    #[serde(default)]
    rooms: HashMap<String, String>,
}

struct Fan {
    name: String,
    device_id: u32,
    key: u8,
    frequency: f64,
    buttons: &'static [(&'static str, u8)],
    rolling_in_check: bool,
}

struct LoadedConfig {
    fans: Vec<Fan>,
    rooms: HashMap<String, String>,
}

fn load_config(path: &str) -> Result<LoadedConfig> {
    let yaml = fs::read_to_string(path).with_context(|| format!("Failed to read {path}"))?;
    let config: Config =
        serde_yaml::from_str(&yaml).with_context(|| format!("Failed to parse {path}"))?;

    let mut fans = Vec::new();
    for cf in &config.fans {
        let (buttons, rolling_in_check, key_mask, frequency) = match cf.vendor.as_str() {
            "vendor_a" => (BUTTONS_VENDOR_A, true, 0xA, FREQUENCY_VENDOR_A),
            "vendor_b" => (BUTTONS_VENDOR_B, false, 0x0, FREQUENCY_VENDOR_B),
            other => bail!("Unknown vendor '{other}' for fan {}", cf.name),
        };
        fans.push(Fan {
            name: cf.name.clone(),
            device_id: cf.device_id,
            key: xor_nibbles(cf.device_id) ^ key_mask,
            frequency,
            buttons,
            rolling_in_check,
        });
    }
    Ok(LoadedConfig {
        fans,
        rooms: config.rooms,
    })
}

fn find_button(fan: &Fan, name: &str) -> Result<u8> {
    fan.buttons
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, id)| *id)
        .with_context(|| {
            let names: Vec<_> = fan.buttons.iter().map(|(n, _)| *n).collect();
            format!(
                "Unknown button '{name}' for {}. Available: {}",
                fan.name,
                names.join(", ")
            )
        })
}

fn make_code(fan: &Fan, button: u8, counter: u8) -> u32 {
    let hi = (button >> 1) & 0xF;
    let lo = button & 1;
    let counter = counter & 0x7;
    if fan.rolling_in_check {
        // Vendor A: rolling = (lo << 3) | counter, check = rolling ^ hi ^ key
        let rolling = (lo << 3) | counter;
        let check = rolling ^ hi ^ fan.key;
        (fan.device_id << 12) | ((hi as u32) << 8) | ((rolling as u32) << 4) | (check as u32)
    } else {
        // Vendor B: lo XORed into bit 12, rolling = counter, check = hi ^ key
        let check = hi ^ fan.key;
        ((fan.device_id << 12) ^ ((lo as u32) << 12))
            | ((hi as u32) << 8)
            | ((counter as u32) << 4)
            | (check as u32)
    }
}

#[derive(Default, Serialize, Deserialize)]
struct State {
    counter: u8,
}

impl State {
    fn path() -> PathBuf {
        let home = std::env::var("HOME").unwrap_or_else(|_| ".".into());
        PathBuf::from(home).join(".fan-remote.json")
    }

    fn load() -> Self {
        fs::read_to_string(Self::path())
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default()
    }

    fn save(&self) {
        let _ = fs::write(Self::path(), serde_json::to_string_pretty(self).unwrap());
    }

    fn next_counter(&mut self) -> u8 {
        self.counter = (self.counter + 1) & 0x7;
        self.counter
    }
}

#[derive(Parser)]
#[command(about = "Transmit fan remote OOK signal via SoapySDR")]
struct Args {
    /// Target (e.g. *, palapa, galleria1). Omit for interactive mode.
    target: Option<String>,

    /// Button to press (e.g. off, speed3, toggle_light)
    button: Option<String>,

    /// TX gain in dB
    #[arg(short, long, default_value_t = 70.0)]
    gain: f64,

    /// SoapySDR driver name
    #[arg(long, default_value = "bladerf")]
    driver: String,

    /// Path to fans.yaml config file
    #[arg(short, long, default_value = "config.yaml")]
    config: String,

    /// Number of times to repeat each command (with 1s interval)
    #[arg(long, default_value_t = 2)]
    repeat: usize,

    /// Start HTTP server on IP:PORT (e.g. 0.0.0.0:8080)
    #[arg(long)]
    http_server: Option<String>,
}

fn parse_command(s: &str) -> Result<(&str, &str)> {
    s.split_once(char::is_whitespace)
        .map(|(a, b)| (a, b.trim()))
        .with_context(|| {
            format!("Invalid command '{s}'. Use: <target> <button> (e.g. * off, palapa speed3)")
        })
}

fn generate_press(
    fan: &Fan,
    code: u32,
    repeats: usize,
    carrier_offset: f32,
    phase: &mut f32,
) -> Vec<c32> {
    let amplitude: f32 = 0.8;
    let phase_inc = TAU * carrier_offset / SAMPLE_RATE as f32;
    let mut samples = Vec::new();

    let push_on = |samples: &mut Vec<c32>, phase: &mut f32, duration: f32| {
        let n = (duration * SAMPLE_RATE as f32) as usize;
        for _ in 0..n {
            samples.push(c32::new(amplitude * phase.cos(), amplitude * phase.sin()));
            *phase += phase_inc;
        }
    };

    let push_off = |samples: &mut Vec<c32>, phase: &mut f32, duration: f32| {
        let n = (duration * SAMPLE_RATE as f32) as usize;
        for _ in 0..n {
            samples.push(c32::new(0.0, 0.0));
            *phase += phase_inc;
        }
    };

    for r in 0..repeats {
        for bit_idx in (0..32).rev() {
            let bit = (code >> bit_idx) & 1;
            if fan.rolling_in_check {
                // Vendor A: fixed pulse, variable gap
                push_on(&mut samples, phase, VA_PULSE);
                let gap = if bit == 1 { VA_LONG_GAP } else { VA_SHORT_GAP };
                push_off(&mut samples, phase, gap);
            } else {
                // Vendor B: variable pulse, variable gap (inverted)
                let (pulse, gap) = if bit == 1 {
                    (VB_SHORT_PULSE, VB_LONG_GAP)
                } else {
                    (VB_LONG_PULSE, VB_SHORT_GAP)
                };
                push_on(&mut samples, phase, pulse);
                push_off(&mut samples, phase, gap);
            }
        }

        if fan.rolling_in_check {
            push_on(&mut samples, phase, VA_PULSE);
        }

        let frame_gap = if fan.rolling_in_check {
            VA_FRAME_GAP
        } else {
            VB_FRAME_GAP
        };
        if r < repeats - 1 {
            push_off(&mut samples, phase, frame_gap);
        }
    }

    samples
}

fn open_sdr(args: &Args) -> Result<(Device, soapysdr::TxStream<c32>)> {
    let dev = Device::new(format!("driver={}", args.driver).as_str())
        .context("Failed to open SoapySDR device")?;

    let ch = 0;
    dev.set_frequency(Direction::Tx, ch, CENTER_FREQUENCY, ())
        .context("Failed to set TX frequency")?;
    dev.set_sample_rate(Direction::Tx, ch, SAMPLE_RATE)
        .context("Failed to set TX sample rate")?;
    dev.set_bandwidth(Direction::Tx, ch, BANDWIDTH)
        .context("Failed to set TX bandwidth")?;
    dev.set_gain(Direction::Tx, ch, args.gain)
        .context("Failed to set TX gain")?;

    let mut stream = dev
        .tx_stream_args::<c32, _>(&[ch], "buflen=8192")
        .context("Failed to create TX stream")?;
    stream
        .activate(None)
        .context("Failed to activate TX stream")?;
    Ok((dev, stream))
}

fn transmit(stream: &mut soapysdr::TxStream<c32>, samples: &[c32]) -> Result<()> {
    let chunk_size = 65536;
    let mut offset = 0;
    while offset < samples.len() {
        let end = (offset + chunk_size).min(samples.len());
        let written = stream
            .write(&[&samples[offset..end]], None, false, 1_000_000)
            .context("TX write failed")?;
        offset += written;
    }
    Ok(())
}

fn build_samples(fans: &[&Fan], button_name: &str, state: &mut State) -> Result<Vec<c32>> {
    let mut phase: f32 = 0.0;
    let mut samples = Vec::new();

    let silence_n = (LEAD_SILENCE * SAMPLE_RATE as f32) as usize;
    for _ in 0..silence_n {
        samples.push(c32::new(0.0, 0.0));
    }

    for fan in fans.iter() {
        let button = find_button(fan, button_name)?;
        let counter = if fan.rolling_in_check {
            state.next_counter()
        } else {
            0
        };
        let code = make_code(fan, button, counter);
        let offset_hz = (fan.frequency - CENTER_FREQUENCY) as f32;
        println!("{} {button_name}", fan.name);

        samples.extend(generate_press(fan, code, REPEATS, offset_hz, &mut phase));
    }

    for _ in 0..silence_n {
        samples.push(c32::new(0.0, 0.0));
    }
    Ok(samples)
}

fn glob_match(pattern: &str, name: &str) -> bool {
    // Simple glob: only supports trailing '*'
    if let Some(prefix) = pattern.strip_suffix('*') {
        name.starts_with(prefix)
    } else {
        name == pattern
    }
}

fn resolve_target<'a>(config: &'a LoadedConfig, target: &str) -> Result<Vec<&'a Fan>> {
    // Check if it's a room name first
    if let Some(pattern) = config.rooms.get(target) {
        let fans: Vec<_> = config
            .fans
            .iter()
            .filter(|f| glob_match(pattern, &f.name))
            .collect();
        if fans.is_empty() {
            bail!("Room '{target}' matched no fans");
        }
        return Ok(fans);
    }
    // Otherwise treat as a glob/exact fan name
    let fans: Vec<_> = config
        .fans
        .iter()
        .filter(|f| glob_match(target, &f.name))
        .collect();
    if fans.is_empty() {
        let names: Vec<_> = config.fans.iter().map(|f| f.name.as_str()).collect();
        let rooms: Vec<_> = config.rooms.keys().map(|k| k.as_str()).collect();
        bail!(
            "Unknown target '{target}'. Fans: {:?}. Rooms: {:?}",
            names,
            rooms,
        );
    }
    Ok(fans)
}

fn execute(
    config: &LoadedConfig,
    stream: &mut soapysdr::TxStream<c32>,
    state: &mut State,
    input: &str,
    repeat: usize,
) -> Result<()> {
    for i in 0..repeat {
        if i > 0 {
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
        let (target, button) = parse_command(input)?;
        let fans = resolve_target(config, target)?;
        let samples = build_samples(&fans, button, state)?;
        transmit(stream, &samples)?;
        state.save();
    }
    Ok(())
}

fn targets_json(config: &LoadedConfig) -> String {
    let fans: Vec<_> = config.fans.iter().map(|f| &f.name).collect();
    let rooms: Vec<_> = config.rooms.keys().collect();
    let commands_a: Vec<_> = BUTTONS_VENDOR_A.iter().map(|(n, _)| *n).collect();
    let commands_b: Vec<_> = BUTTONS_VENDOR_B.iter().map(|(n, _)| *n).collect();
    let mut all_commands: Vec<&str> = commands_a;
    for c in commands_b {
        if !all_commands.contains(&c) {
            all_commands.push(c);
        }
    }
    serde_json::json!({ "fans": fans, "rooms": rooms, "commands": all_commands }).to_string()
}

fn run_http_server(
    addr: &str,
    config: &LoadedConfig,
    stream: &mut soapysdr::TxStream<c32>,
    state: &mut State,
    repeat: usize,
) -> Result<()> {
    let server = tiny_http::Server::http(addr)
        .map_err(|e| anyhow::anyhow!("Failed to start HTTP server: {e}"))?;
    eprintln!("HTTP server listening on {addr}");

    for request in server.incoming_requests() {
        let (status, body) = match (request.method(), request.url()) {
            (tiny_http::Method::Get, "/targets") => (200, targets_json(config)),
            (tiny_http::Method::Post, url) if url.starts_with("/send?") => {
                let query = &url["/send?".len()..];
                let params: HashMap<&str, &str> =
                    query.split('&').filter_map(|p| p.split_once('=')).collect();
                match (params.get("target"), params.get("cmd")) {
                    (Some(target), Some(cmd)) => {
                        let input = format!("{target} {cmd}");
                        match execute(config, stream, state, &input, repeat) {
                            Ok(()) => (200, format!("OK: {input}")),
                            Err(e) => (400, format!("Error: {e}")),
                        }
                    }
                    _ => (400, "Missing target or cmd parameter".to_string()),
                }
            }
            _ => (404, "Not found".to_string()),
        };
        let response = tiny_http::Response::from_string(&body)
            .with_status_code(status)
            .with_header(
                "Content-Type: application/json"
                    .parse::<tiny_http::Header>()
                    .unwrap(),
            );
        let _ = request.respond(response);
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    let config = load_config(&args.config)?;
    let (_dev, mut stream) = open_sdr(&args)?;
    let mut state = State::load();

    if let Some(addr) = &args.http_server {
        run_http_server(addr, &config, &mut stream, &mut state, args.repeat)?;
    } else if let (Some(target), Some(button)) = (&args.target, &args.button) {
        let cmd = format!("{target} {button}");
        execute(&config, &mut stream, &mut state, &cmd, args.repeat)?;
    } else if args.target.is_some() {
        bail!("Both target and button are required (e.g. fan-tx '*' off)");
    } else {
        use std::io::BufRead;
        eprintln!("Waiting for commands...");
        let stdin = std::io::stdin();
        for line in stdin.lock().lines() {
            let line = line?;
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if let Err(e) = execute(&config, &mut stream, &mut state, line, args.repeat) {
                eprintln!("Error: {e}");
            }
        }
    }

    Ok(())
}
