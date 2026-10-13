//! atmos-render: real-time Dolby Atmos decoder for Linux, no runtime needed.
//!
//!   E-AC-3 + JOC (streaming / music Atmos) -> Cavern's decoder, ported to Rust
//!   TrueHD Atmos (Blu-ray)                 -> truehd crate + object panner
//!
//! stdin (or a file): raw E-AC-3 or TrueHD elementary stream, format detected automatically.
//! stdout: interleaved float32 LE, 12 channels FL FR FC LFE RL RR SL SR TFL TFR TRL TRR (7.1.4).
//! Drop-in for `atmos-decode` (accepts --layout 7.1.4, --gain, --positions, --info).
use std::fs::File;
use std::io::{self, Read};

const POSITIONS: &str = "FL,FR,FC,LFE,RL,RR,SL,SR,TFL,TFR,TRL,TRR";
const USAGE: &str = "\
Usage: atmos-render [options] [input|-]

Decodes Dolby Atmos (E-AC-3 + JOC, TrueHD) from a raw elementary stream and writes 7.1.4
PCM (interleaved float32 LE: FL FR FC LFE RL RR SL SR TFL TFR TRL TRR) to stdout.
  e.g.  ffmpeg -i song.m4a -map 0:a:0 -c copy -f eac3 - | atmos-render -

Options:
  -g, --gain <dB>       Output gain (default 0)
  -f, --format <fmt>    eac3 or truehd (default: detected from the stream)
  -l, --layout 7.1.4    Output layout (only 7.1.4)
      --positions       Print the PipeWire channel map and exit
  -i, --info            Print what the stream is and exit
  -h, --help            Show this help

E-AC-3 decoding is a port of Cavern by VoidX (http://en.sbence.hu, https://github.com/VoidXH/Cavern).";

enum Format { Eac3, TrueHd }

/// Position of the first E-AC-3 sync word that is confirmed by a second sync word exactly one
/// frame later (a lone 0x0B77 also occurs by chance inside other data, e.g. a TrueHD stream cut
/// mid-way). None if not (yet) confirmed.
fn eac3_sync(buf: &[u8]) -> Option<usize> {
    (0..buf.len().saturating_sub(5)).find(|&i| {
        if buf[i] != 0x0B || buf[i + 1] != 0x77 { return false; }
        let size = ((((buf[i + 2] & 7) as usize) << 8) | buf[i + 3] as usize) + 1;
        let next = i + size * 2;
        next + 1 < buf.len() && buf[next] == 0x0B && buf[next + 1] == 0x77
    })
}

/// Detects the stream type: TrueHD by two major syncs (0xF8726FBA), E-AC-3 by two consecutive
/// sync words one frame apart. Either alone can occur by chance inside the other format.
fn detect(buf: &[u8], eof: bool) -> Option<Format> {
    // TrueHD: a major sync confirmed by another one shortly after (one every 16-128 access units)
    const MAJOR: [u8; 4] = [0xF8, 0x72, 0x6F, 0xBA];
    let thd = buf.windows(4).position(|w| w == MAJOR).filter(|&p| {
        buf[p + 4..].windows(4).take(16 * 1024).any(|w| w == MAJOR)
    });
    let ac3 = eac3_sync(buf);
    match (ac3, thd) {
        (Some(a), Some(t)) => Some(if a < t { Format::Eac3 } else { Format::TrueHd }),
        (None, Some(_)) => Some(Format::TrueHd),
        // E-AC-3 needs no more evidence once confirmed; wait a little for a TrueHD major sync
        // (one every 16-128 access units, so within a few KB) before deciding.
        (Some(_), None) if buf.len() > 64 * 1024 || eof => Some(Format::Eac3),
        _ => None,
    }
}

fn main() {
    let mut gain_db = 0f32;
    let mut info = false;
    let mut forced: Option<Format> = None;
    let mut input: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "-h" | "--help" => { println!("{USAGE}"); return; }
            "--positions" => { println!("{POSITIONS}"); return; }
            "-i" | "--info" => info = true,
            "-f" | "--format" => forced = Some(match args.next().as_deref() {
                Some("eac3") | Some("ac3") => Format::Eac3,
                Some("truehd") | Some("mlp") => Format::TrueHd,
                other => fail(&format!("unknown format {other:?} (eac3 or truehd)")),
            }),
            "-g" | "--gain" => gain_db = args.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| fail("--gain needs a number")),
            "-l" | "--layout" => {
                let l = args.next().unwrap_or_default();
                if l != "7.1.4" { fail(&format!("unsupported layout '{l}' (only 7.1.4)")); }
            }
            _ if a.starts_with('-') && a != "-" => fail(&format!("unknown option {a}\n\n{USAGE}")),
            _ => input = Some(a),
        }
    }
    let gain = 10f32.powf(gain_db / 20.0);
    let mut reader: Box<dyn Read> = match input.as_deref() {
        None | Some("-") => Box::new(io::stdin().lock()),
        Some(p) => Box::new(File::open(p).unwrap_or_else(|e| fail(&format!("{p}: {e}")))),
    };

    // Read until the format is recognised (a few KB at most)
    let mut prefix = Vec::new();
    let mut chunk = [0u8; 8192];
    let format = match forced {
        Some(f) => f,
        None => loop {
            let n = reader.read(&mut chunk).unwrap_or(0);
            prefix.extend_from_slice(&chunk[..n]);
            if let Some(f) = detect(&prefix, n == 0) { break f; }
            if n == 0 || prefix.len() > 1 << 20 { fail("no E-AC-3 or TrueHD stream found in the input"); }
        },
    };
    let out = io::stdout().lock();
    let result = match format {
        Format::Eac3 => atmos_render::eac3_render::run(&prefix, reader, out, gain, info),
        Format::TrueHd => {
            if info { eprintln!("atmos-render: TrueHD"); return; }
            atmos_render::truehd::run(&prefix, reader, out, gain)
        }
    };
    if let Err(e) = result {
        if e.kind() != io::ErrorKind::BrokenPipe { fail(&e.to_string()); }
    }
}

fn fail(msg: &str) -> ! {
    eprintln!("atmos-render: {msg}");
    std::process::exit(1);
}
