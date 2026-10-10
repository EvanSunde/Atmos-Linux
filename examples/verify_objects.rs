//! Verifies src/objects (port of Cavern's E-AC-3 object decoding: EMDF, OAMD, JOC, QMF and
//! EnhancedAC3Renderer glue) against reference dumps produced by Cavern on the same stream.
//!
//! Dump alignment: Cavern's `EnhancedAC3Decoder` decodes the first stream frame inside its constructor, so the
//! core/aux dumps (made by calling DecodeFrame after construction) start at stream frame 1, while objects.bin
//! starts at stream frame 0. Core frame k therefore covers objects.bin timeslots 24*(k+1) .. 24*(k+1)+23.
//! Stream frame 0 is not available, so the decoder is primed with a silent copy of core frame 0 (with that
//! frame's aux data). This keeps the JOC applier's 7-timeslot cycle in phase with Cavern; QMF history, JOC
//! interpolation history and object position ramps of the missing frame are approximated, so the first few
//! timeslots are reported separately as warm-up.
//!
//! Usage: cargo run --release --example verify_objects [ref_dir]

use std::fs::File;
use std::io::{BufReader, Read};
use std::time::{Duration, Instant};

use atmos_render::objects::{ObjectDecoder, ObjectKind};

const WARMUP: usize = 48;

struct CoreFrame {
    ids: Vec<i32>,
    pcm: Vec<Vec<f32>>,
}

fn rd_i32(b: &[u8], p: &mut usize) -> i32 {
    let v = i32::from_le_bytes(b[*p..*p + 4].try_into().unwrap());
    *p += 4;
    v
}

fn load_core(path: &str) -> Vec<CoreFrame> {
    let b = std::fs::read(path).expect("core.bin");
    let mut p = 0;
    let mut frames = Vec::new();
    while p < b.len() {
        let nch = rd_i32(&b, &mut p) as usize;
        let fs = rd_i32(&b, &mut p) as usize;
        let ids = (0..nch).map(|_| rd_i32(&b, &mut p)).collect();
        let mut pcm = Vec::with_capacity(nch);
        for _ in 0..nch {
            let ch: Vec<f32> = b[p..p + fs * 4].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
            p += fs * 4;
            pcm.push(ch);
        }
        frames.push(CoreFrame { ids, pcm });
    }
    frames
}

fn load_aux(path: &str) -> Vec<Vec<(i32, Vec<u8>)>> {
    let b = std::fs::read(path).expect("aux.bin");
    let mut p = 0;
    let mut frames = Vec::new();
    while p < b.len() {
        let n = rd_i32(&b, &mut p);
        let mut subs = Vec::new();
        for _ in 0..n {
            let id = rd_i32(&b, &mut p);
            let len = rd_i32(&b, &mut p) as usize;
            subs.push((id, b[p..p + len].to_vec()));
            p += len;
        }
        frames.push(subs);
    }
    frames
}

struct RefReader {
    r: BufReader<File>,
    nobj: usize,
    buf: Vec<u8>,
}

struct RefBlock {
    pos: Vec<[f32; 3]>,
    vol: Vec<f32>,
    samples: Vec<[f32; 64]>,
}

impl RefReader {
    fn open(path: &str) -> Self {
        let mut r = BufReader::with_capacity(1 << 20, File::open(path).expect("objects.bin"));
        let mut h = [0u8; 4];
        r.read_exact(&mut h).unwrap();
        let nobj = i32::from_le_bytes(h) as usize;
        Self { r, nobj, buf: vec![0; nobj * (3 + 1 + 64) * 4] }
    }
    fn next(&mut self) -> Option<RefBlock> {
        self.r.read_exact(&mut self.buf).ok()?;
        let f: Vec<f32> = self.buf.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
        let n = self.nobj;
        let pos = (0..n).map(|o| [f[o * 3], f[o * 3 + 1], f[o * 3 + 2]]).collect();
        let vol = f[n * 3..n * 4].to_vec();
        let samples = (0..n).map(|o| f[n * 4 + o * 64..n * 4 + o * 64 + 64].try_into().unwrap()).collect();
        Some(RefBlock { pos, vol, samples })
    }
}

#[derive(Default, Clone)]
struct Stats {
    max_err: f32,
    peak: f32,
    max_pos_err: f32,
    vol_mismatch: usize,
}

fn db(x: f32) -> f32 { if x <= 0.0 { f32::NEG_INFINITY } else { 20.0 * x.log10() } }

fn main() {
    let dir = std::env::args().nth(1).unwrap_or_else(|| "/home/claude/src/ref".to_string());
    let core = load_core(&format!("{dir}/core.bin"));
    let aux = load_aux(&format!("{dir}/aux.bin"));
    assert_eq!(core.len(), aux.len());
    let mut refr = RefReader::open(&format!("{dir}/objects.bin"));
    let nobj = refr.nobj;
    println!("frames: {}, reference objects: {}", core.len(), nobj);

    let mut dec = ObjectDecoder::new();
    let mut busy = Duration::ZERO;

    // Priming with a silent stand-in for the missing stream frame 0 (see header comment).
    let silent: Vec<Vec<f32>> = core[0].pcm.iter().map(|c| vec![0.0; c.len()]).collect();
    let t = Instant::now();
    dec.push_frame(&core[0].ids, &silent, &aux[0]);
    busy += t.elapsed();
    let mut ref_index = 0usize;
    while dec.next_block().is_some() {
        refr.next().unwrap();
        ref_index += 1;
    }
    println!("objects decoded: {} (has_objects = {}), primed {} timeslots", dec.object_count(), dec.has_objects(), ref_index);
    assert_eq!(dec.object_count(), nobj, "object count mismatch");
    let kinds: Vec<String> = (0..dec.object_count()).map(|i| match dec.object_kind(i) {
        ObjectKind::Lfe => "LFE".to_string(),
        ObjectKind::Bed(c) => format!("Bed({c})"),
        ObjectKind::Dynamic => "Dyn".to_string(),
    }).collect();
    println!("object kinds: {}", kinds.join(" "));

    let mut warm = vec![Stats::default(); nobj];
    let mut steady = vec![Stats::default(); nobj];
    let mut first_div: Option<(usize, String)> = None;
    let mut last_div: Option<usize> = None;
    let mut divergent = 0usize;
    let mut compared = 0usize;
    let mut skipped_pending = false;
    let rel_threshold = 10f32.powf(-90.0 / 20.0);

    'frames: for (k, frame) in core.iter().enumerate() {
        let t = Instant::now();
        dec.push_frame(&frame.ids, &frame.pcm, &aux[k]);
        busy += t.elapsed();
        loop {
            let t = Instant::now();
            let blk = dec.next_block();
            busy += t.elapsed();
            let Some(blk) = blk else { break };
            let Some(r) = refr.next() else { break 'frames };
            let idx = ref_index;
            ref_index += 1;
            if !skipped_pending {
                // Last timeslot of the missing stream frame 0 (silence was fed for it).
                skipped_pending = true;
                continue;
            }
            compared += 1;
            let in_warmup = compared <= WARMUP;
            let stats = if in_warmup { &mut warm } else { &mut steady };
            let mut why = String::new();
            for o in 0..nobj {
                let s = &mut stats[o];
                let mut err = 0f32;
                for i in 0..64 {
                    err = err.max((blk.samples[o][i] - r.samples[o][i]).abs());
                    s.peak = s.peak.max(r.samples[o][i].abs());
                }
                s.max_err = s.max_err.max(err);
                let mut perr = 0f32;
                for a in 0..3 {
                    perr = perr.max((blk.positions[o][a] - r.pos[o][a]).abs());
                }
                s.max_pos_err = s.max_pos_err.max(perr);
                if blk.volumes[o] != r.vol[o] {
                    s.vol_mismatch += 1;
                }
                if why.is_empty() {
                    if perr > 1e-4 {
                        why = format!("object {o} position {:?} vs {:?}", blk.positions[o], r.pos[o]);
                    } else if blk.volumes[o] != r.vol[o] {
                        why = format!("object {o} volume {} vs {}", blk.volumes[o], r.vol[o]);
                    } else if err > 1e-6 && err > rel_threshold * s.peak.max(1e-3) {
                        why = format!("object {o} sample error {err:e}");
                    }
                }
            }
            if !why.is_empty() {
                divergent += 1;
                last_div = Some(idx);
                if first_div.is_none() {
                    first_div = Some((idx, why));
                }
            }
        }
    }

    let audio_seconds = core.len() as f64 * 1536.0 / 48000.0;
    let print = |name: &str, st: &[Stats]| {
        println!("\n{name}:");
        println!("  obj  kind       max|err|      peak    err/peak(dB)  max pos err  vol mismatches");
        for (o, s) in st.iter().enumerate() {
            println!("  {o:>3}  {:<8} {:>11.3e} {:>10.3e} {:>12.1} {:>12.3e} {:>8}", kinds[o], s.max_err, s.peak,
                db(s.max_err) - db(s.peak), s.max_pos_err, s.vol_mismatch);
        }
        let worst_rel = st.iter().filter(|s| s.peak > 0.0).map(|s| db(s.max_err) - db(s.peak)).fold(f32::NEG_INFINITY, f32::max);
        let worst_pos = st.iter().map(|s| s.max_pos_err).fold(0f32, f32::max);
        let vols: usize = st.iter().map(|s| s.vol_mismatch).sum();
        println!("  worst relative sample error: {worst_rel:.1} dB, worst position error: {worst_pos:.3e}, volume mismatches: {vols}");
    };
    println!("\ncompared timeslots: {compared} (reference timeslots 24..{}), first {WARMUP} counted as warm-up", ref_index - 1);
    print("warm-up (first 2 frames, decoder primed without stream frame 0)", &warm);
    print("steady state", &steady);
    match first_div {
        None => println!("\nno divergent timeslots (pos > 1e-4, volume !=, or sample error > -90 dB rel. peak)"),
        Some((i, why)) => println!("\ndivergent timeslots: {divergent}; first at reference timeslot {i} ({why}); last at {}",
            last_div.unwrap()),
    }
    println!("\nprocessing time: {:.3} s for {:.1} s of audio ({:.1}x real time)", busy.as_secs_f64(), audio_seconds,
        audio_seconds / busy.as_secs_f64());
}
