//! End to end: Rust E-AC-3 decoder -> Rust object decoder, compared with Cavern's object dump
//! (objects.bin) from the very first timeslot.
use atmos_render::{eac3::Eac3Decoder, objects::ObjectDecoder};
use std::io::Read;

fn main() {
    // usage: verify_pipeline <track.eac3> <objects.bin>
    let a: Vec<String> = std::env::args().collect();
    let stream = std::fs::read(a.get(1).map(String::as_str).unwrap_or("track.eac3")).unwrap();
    let mut f = std::fs::File::open(a.get(2).map(String::as_str).unwrap_or("ref/objects.bin")).unwrap();
    let mut b = Vec::new(); f.read_to_end(&mut b).unwrap();
    let n = i32::from_le_bytes(b[0..4].try_into().unwrap()) as usize;
    let per = n * 3 + n + n * 64;
    let refv: Vec<f32> = b[4..].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
    let slots = refv.len() / per;
    let (mut dec, mut objs) = (Eac3Decoder::new(), ObjectDecoder::new());
    let (mut t, mut max_err, mut max_pos, mut peak, mut first_bad) = (0usize, 0f32, 0f32, 0f32, None);
    for chunk in stream.chunks(4096) {
        dec.push(chunk);
        while let Some(fr) = dec.next_frame() {
            objs.push_frame(&fr.channel_ids, &fr.pcm, &fr.aux);
            while let Some(blk) = objs.next_block() {
                if t >= slots { break; }
                let r = &refv[t * per..(t + 1) * per];
                for o in 0..n {
                    for k in 0..3 { max_pos = max_pos.max((blk.positions[o][k] - r[o * 3 + k]).abs()); }
                    for s in 0..64 {
                        let rv = r[n * 4 + o * 64 + s];
                        let e = (blk.samples[o][s] - rv).abs();
                        peak = peak.max(rv.abs());
                        if e > 1e-5 && first_bad.is_none() { first_bad = Some(t); }
                        max_err = max_err.max(e);
                    }
                }
                t += 1;
            }
        }
        if t >= slots { break; }
    }
    println!("compared {t} timeslots from the start: max sample error {max_err:.3e} ({:.1} dB re peak), max position error {max_pos:.3e}, first timeslot > 1e-5: {first_bad:?}",
             20.0 * (max_err / peak).log10());
}
