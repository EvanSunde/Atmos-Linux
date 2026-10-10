//! Verifies the E-AC-3 core decoder against Cavern reference dumps (ref/core.bin, ref/aux.bin),
//! streaming the input in odd-sized chunks, then times decoding of the whole track.
//!
//! cargo run --release --example verify_core [track.eac3] [core.bin] [aux.bin]

use std::io::Read;
use std::time::Instant;

use atmos_render::eac3::{Eac3Decoder, Eac3Frame};

fn chunk() -> usize { std::env::var("VERIFY_CHUNK").ok().and_then(|v| v.parse().ok()).unwrap_or(777) }

fn i32_at(d: &[u8], p: &mut usize) -> i32 {
    let v = i32::from_le_bytes(d[*p..*p + 4].try_into().unwrap());
    *p += 4;
    v
}

/// Decode by feeding `chunk()`-byte (default 777, env VERIFY_CHUNK) pieces, calling `f` for each frame until it returns false.
fn stream_decode(data: &[u8], mut f: impl FnMut(Eac3Frame) -> bool) -> Eac3Decoder {
    let mut dec = Eac3Decoder::new();
    let mut off = 0;
    'outer: loop {
        if off < data.len() {
            let end = (off + chunk()).min(data.len());
            dec.push(&data[off..end]);
            off = end;
            if off == data.len() {
                dec.finish();
            }
        }
        while let Some(frame) = dec.next_frame() {
            if !f(frame) {
                break 'outer;
            }
        }
        if off == data.len() {
            while let Some(frame) = dec.next_frame() {
                if !f(frame) {
                    break 'outer;
                }
            }
            break;
        }
    }
    dec
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let track = args.get(1).cloned().unwrap_or("/home/claude/src/run/track.eac3".into());
    let core_path = args.get(2).cloned().unwrap_or("/home/claude/src/ref/core.bin".into());
    let aux_path = args.get(3).cloned().unwrap_or("/home/claude/src/ref/aux.bin".into());

    let mut data = Vec::new();
    std::fs::File::open(&track).expect("track").read_to_end(&mut data).unwrap();
    let core = std::fs::read(&core_path).expect("core.bin");
    let aux = std::fs::read(&aux_path).expect("aux.bin");

    // ---------- Verification ----------
    let (mut cp, mut ap) = (0usize, 0usize);
    let mut frames = 0usize;
    let mut layout_mismatch = 0usize;
    let mut aux_mismatch = 0usize;
    let mut aux_total = 0usize;
    let mut first_aux_mismatch: Option<usize> = None;
    let mut max_err: Vec<f64> = Vec::new();
    let mut peak: Vec<f64> = Vec::new();
    let mut ids: Vec<i32> = Vec::new();
    let mut worst_frame = (0usize, 0f64);
    let mut exact_samples = 0u64;
    let mut total_samples = 0u64;

    // Cavern's FrameBasedDecoder constructor (via `new BlockBuffer<float>(DecodeFrame)`) decodes stream frame 0
    // eagerly, so the harness's first RefDecodeFrame() returns stream frame 1: reference frame k = our frame k + 1.
    let mut skipped_first = false;
    let dec = stream_decode(&data, |fr| {
        if !skipped_first {
            skipped_first = true;
            return true;
        }
        if cp >= core.len() || ap >= aux.len() {
            return false;
        }
        // core.bin
        let nch = i32_at(&core, &mut cp) as usize;
        let fsz = i32_at(&core, &mut cp) as usize;
        let ref_ids: Vec<i32> = (0..nch).map(|_| i32_at(&core, &mut cp)).collect();
        if ids.is_empty() {
            ids = ref_ids.clone();
            max_err = vec![0.0; nch];
            peak = vec![0.0; nch];
        }
        if ref_ids != fr.channel_ids || fsz != fr.frame_size {
            layout_mismatch += 1;
            if layout_mismatch < 5 {
                eprintln!("frame {frames}: layout {:?}/{} vs ref {:?}/{}", fr.channel_ids, fr.frame_size, ref_ids, fsz);
            }
        }
        for c in 0..nch {
            for s in 0..fsz {
                let r = f32::from_le_bytes(core[cp..cp + 4].try_into().unwrap());
                cp += 4;
                let v = fr.pcm.get(c).and_then(|x| x.get(s)).copied().unwrap_or(f32::NAN);
                let err = (v as f64 - r as f64).abs();
                let err = if err.is_nan() { f64::INFINITY } else { err };
                if c < max_err.len() {
                    max_err[c] = max_err[c].max(err);
                    peak[c] = peak[c].max((r as f64).abs());
                }
                if err > worst_frame.1 {
                    worst_frame = (frames, err);
                }
                if v.to_bits() == r.to_bits() {
                    exact_samples += 1;
                }
                total_samples += 1;
            }
        }
        // aux.bin
        let nsub = i32_at(&aux, &mut ap) as usize;
        let mut ref_aux = Vec::new();
        for _ in 0..nsub {
            let sid = i32_at(&aux, &mut ap);
            let nb = i32_at(&aux, &mut ap) as usize;
            ref_aux.push((sid, aux[ap..ap + nb].to_vec()));
            ap += nb;
        }
        aux_total += 1;
        if ref_aux != fr.aux {
            aux_mismatch += 1;
            if first_aux_mismatch.is_none() && std::env::var("VERIFY_DEBUG").is_ok() {
                for ((rs, rb), (s, b)) in ref_aux.iter().zip(fr.aux.iter()) {
                    eprintln!("aux sub {rs}/{s}: ref {} bytes {:02x?}", rb.len(), &rb[..rb.len().min(24)]);
                    eprintln!("               ours {} bytes {:02x?}", b.len(), &b[..b.len().min(24)]);
                }
            }
            first_aux_mismatch.get_or_insert(frames);
        }
        frames += 1;
        true
    });

    println!("Verified {frames} frames (stream frames 1..={frames}) against Cavern reference");
    println!("Channel layout/frame size mismatches: {layout_mismatch}");
    let mut overall_err = 0f64;
    let mut overall_peak = 0f64;
    for c in 0..ids.len() {
        let rel = 20.0 * (max_err[c] / peak[c]).log10();
        println!("  channel {:>2}: max abs err {:.3e}, peak {:.4}, relative {:.1} dB", ids[c], max_err[c], peak[c], rel);
        overall_err = overall_err.max(max_err[c]);
        overall_peak = overall_peak.max(peak[c]);
    }
    println!("Overall: max abs err {:.3e}, peak {:.4}, relative {:.1} dB (worst at frame {})", overall_err,
        overall_peak, 20.0 * (overall_err / overall_peak).log10(), worst_frame.0);
    println!("Bit-exact samples: {exact_samples}/{total_samples} ({:.4}%)",
        100.0 * exact_samples as f64 / total_samples.max(1) as f64);
    println!("Aux: {}/{} frames identical{}", aux_total - aux_mismatch, aux_total,
        first_aux_mismatch.map(|f| format!(" (first mismatch at frame {f})")).unwrap_or_default());
    if !dec.errors.is_empty() {
        println!("Decoder errors: {:?}", &dec.errors[..dec.errors.len().min(5)]);
    }

    // ---------- Timing ----------
    let start = Instant::now();
    let mut count = 0usize;
    let mut samples = 0usize;
    let mut rate = 48000;
    let dec = stream_decode(&data, |fr| {
        count += 1;
        samples += fr.frame_size;
        rate = fr.sample_rate;
        true
    });
    let elapsed = start.elapsed().as_secs_f64();
    let audio = samples as f64 / rate as f64;
    println!("Timing: {count} frames ({audio:.1} s of audio) in {elapsed:.3} s = {:.1}x real time, errors: {}",
        audio / elapsed, dec.errors.len());
}
