//! End to end: Rust E-AC-3 decoder -> ObjectDecoder -> CavernRenderer (7.1.4), compared with the output of the
//! C# atmos-decode tool (Cavern) for the whole track.
//!
//! Usage: cargo run --release --example verify_render [track.eac3] [reference.f32]

use std::fs::File;
use std::io::{BufReader, Read};
use std::time::{Duration, Instant};

use atmos_render::cavern_render::CavernRenderer;
use atmos_render::eac3::Eac3Decoder;
use atmos_render::objects::{ObjectDecoder, ObjectKind};

const NAMES: [&str; 12] = ["FL", "FR", "FC", "LFE", "RL", "RR", "SL", "SR", "TFL", "TFR", "TRL", "TRR"];

struct Compare {
    reference: BufReader<File>,
    buf: Vec<u8>,
    max_err: [f32; 12],
    peak: [f32; 12],
    first_bad: Option<(usize, usize, f32, f32)>,
    samples: usize,
    ref_done: bool,
    time: Duration,
}

impl Compare {
    fn feed(&mut self, out: &[f32]) {
        if self.ref_done {
            return;
        }
        let t0 = Instant::now();
        self.buf.resize(out.len() * 4, 0);
        let mut filled = 0;
        while filled < self.buf.len() {
            match self.reference.read(&mut self.buf[filled..]) {
                Ok(0) | Err(_) => break,
                Ok(n) => filled += n,
            }
        }
        let n = filled / 4;
        if n < out.len() {
            self.ref_done = true;
        }
        for i in 0..n {
            let r = f32::from_le_bytes(self.buf[i * 4..i * 4 + 4].try_into().unwrap());
            let ch = i % 12;
            let e = (out[i] - r).abs();
            if !(e <= self.max_err[ch]) {
                self.max_err[ch] = e;
            }
            self.peak[ch] = self.peak[ch].max(r.abs());
            if self.first_bad.is_none() && !(e <= 1e-5) {
                self.first_bad = Some((self.samples + i / 12, ch, out[i], r));
            }
        }
        self.samples += n / 12;
        self.time += t0.elapsed();
    }
}

fn db(x: f32) -> f32 { 20.0 * x.log10() }

fn main() {
    let input = std::env::args().nth(1).unwrap_or_else(|| "/home/claude/src/run/track.eac3".into());
    let reference = std::env::args().nth(2).unwrap_or_else(|| "/home/claude/src/run/file.f32".into());
    let stream = std::fs::read(&input).expect("input");
    let mut cmp = Compare {
        reference: BufReader::with_capacity(1 << 20, File::open(&reference).expect("reference")),
        buf: Vec::new(),
        max_err: [0.0; 12],
        peak: [0.0; 12],
        first_bad: None,
        samples: 0,
        ref_done: false,
        time: Duration::ZERO,
    };

    let mut dec = Eac3Decoder::new();
    let mut objs = ObjectDecoder::new();
    let mut renderer = CavernRenderer::new_714(48000);
    let mut kinds: Vec<ObjectKind> = Vec::new();
    let (mut t_total, mut t_render) = (Duration::ZERO, Duration::ZERO);
    let mut frames = 0usize;
    let mut audio_samples = 0usize;

    let drain = |objs: &mut ObjectDecoder, renderer: &mut CavernRenderer, kinds: &mut Vec<ObjectKind>,
                     cmp: &mut Compare, t_render: &mut Duration| {
        kinds.clear();
        kinds.extend((0..objs.object_count()).map(|i| objs.object_kind(i)));
        while let Some(blk) = objs.next_block() {
            let t = Instant::now();
            let out = renderer.push_block(&blk, kinds).map(|o| o.to_vec());
            *t_render += t.elapsed();
            if let Some(out) = out {
                cmp.feed(&out);
            }
        }
    };

    for chunk in stream.chunks(1 << 16) {
        let t = Instant::now();
        dec.push(chunk);
        while let Some(fr) = dec.next_frame() {
            frames += 1;
            audio_samples += fr.frame_size;
            objs.push_frame(&fr.channel_ids, &fr.pcm, &fr.aux);
            drain(&mut objs, &mut renderer, &mut kinds, &mut cmp, &mut t_render);
        }
        t_total += t.elapsed();
    }
    let t = Instant::now();
    dec.finish();
    while let Some(fr) = dec.next_frame() {
        frames += 1;
        audio_samples += fr.frame_size;
        objs.push_frame(&fr.channel_ids, &fr.pcm, &fr.aux);
        drain(&mut objs, &mut renderer, &mut kinds, &mut cmp, &mut t_render);
    }
    objs.flush();
    drain(&mut objs, &mut renderer, &mut kinds, &mut cmp, &mut t_render);
    t_total += t.elapsed();
    t_total -= cmp.time;

    let seconds = audio_samples as f64 / 48000.0;
    println!("frames: {frames}, objects: {}, kinds: {:?}", objs.object_count(), kinds);
    println!("compared {} samples x 12 channels (reference exhausted: {})", cmp.samples, cmp.ref_done);
    println!("  ch      max|err|        peak   err re peak");
    let mut worst = f32::NEG_INFINITY;
    for ch in 0..12 {
        let rel = db(cmp.max_err[ch]) - db(cmp.peak[ch]);
        worst = worst.max(rel);
        println!("  {:<4} {:>11.3e} {:>11.3e} {:>9.1} dB", NAMES[ch], cmp.max_err[ch], cmp.peak[ch], rel);
    }
    println!("worst channel: {worst:.1} dB re its peak");
    match cmp.first_bad {
        None => println!("no sample differs by more than 1e-5"),
        Some((s, ch, a, b)) => println!("first sample with error > 1e-5: sample {s} ({:.3} s) channel {} ours {a} ref {b}",
            s as f64 / 48000.0, NAMES[ch]),
    }
    println!("speed: full pipeline {:.2} s for {:.1} s of audio ({:.0}x real time); renderer alone {:.2} s ({:.0}x)",
        t_total.as_secs_f64(), seconds, seconds / t_total.as_secs_f64(), t_render.as_secs_f64(),
        seconds / t_render.as_secs_f64());
}
