//! E-AC-3 (+ JOC Atmos) -> 7.1.4, entirely Cavern-ported: decoding (`eac3`, `objects`) and rendering
//! (`cavern_render`, configured exactly like the former C# `atmos-decode`).
use std::io::{self, Read, Write};

use crate::eac3::Eac3Decoder;
use crate::cavern_render::{channel::layout_714, CavernRenderer};
use crate::objects::{ObjectDecoder, ObjectKind};

/// Decodes E-AC-3 from `input` (after `prefix`) and writes interleaved f32 LE 7.1.4 to `out`.
pub fn run<R: Read, W: Write>(prefix: &[u8], input: R, out: W, gain: f32, info: bool) -> io::Result<()> {
    let mut input = io::Cursor::new(prefix.to_vec()).chain(input);
    let mut out = io::BufWriter::with_capacity(1 << 16, out);
    let mut dec = Eac3Decoder::new();
    let mut objs = ObjectDecoder::new();
    // atmos-decode: Listener.Volume = gain (x .707 only for layouts without 4 height channels)
    let mut rend: Option<CavernRenderer> = None;
    let mut kinds: Vec<ObjectKind> = Vec::new();
    let mut chunk = vec![0u8; 1 << 16];
    let mut buf = Vec::with_capacity(256 * 12 * 4 * 8);
    let mut announced = false;
    let mut eof = false;
    loop {
        if !eof {
            let n = match input.read(&mut chunk) {
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            };
            if n == 0 { eof = true; dec.finish(); } else { dec.push(&chunk[..n]); }
        }
        while let Some(f) = dec.next_frame() {
            objs.push_frame(&f.channel_ids, &f.pcm, &f.aux);
            if !announced {
                announced = true;
                let desc = if objs.has_objects() {
                    format!("E-AC-3 + JOC, {} Hz, {} sources (object-based Atmos) -> 7.1.4", f.sample_rate, objs.object_count())
                } else {
                    format!("E-AC-3, {} Hz, {} channels (channel-based) -> 7.1.4", f.sample_rate, f.channel_ids.len())
                };
                eprintln!("atmos-render: {desc}");
                if info { return Ok(()); }
                rend = Some(CavernRenderer::new(layout_714(), f.sample_rate, 256, gain));
            }
            render_blocks(&mut objs, rend.as_mut().unwrap(), &mut kinds, &mut buf);
            out.write_all(&buf)?;
            buf.clear();
        }
        if eof {
            objs.flush();
            if let Some(r) = rend.as_mut() { render_blocks(&mut objs, r, &mut kinds, &mut buf); }
            out.write_all(&buf)?;
            return out.flush();
        }
    }
}

fn render_blocks(objs: &mut ObjectDecoder, rend: &mut CavernRenderer, kinds: &mut Vec<ObjectKind>, buf: &mut Vec<u8>) {
    while let Some(b) = objs.next_block() {
        kinds.clear();
        kinds.extend((0..b.samples.len()).map(|i| objs.object_kind(i)));
        if let Some(out) = rend.push_block(&b, kinds) {
            for v in out { buf.extend_from_slice(&v.to_le_bytes()); }
        }
    }
}
