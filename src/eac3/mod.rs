//! E-AC-3 core decoder (port of Cavern.Format Transcoders/EnhancedAC3*).
//!
//! This file ports the frame loop of Cavern.Format/Decoders/EnhancedAC3Decoder.cs: substreams are decoded
//! until the next independent substream 0 header, each substream's channels (and LFE) are collected into a
//! persistent channel map, and the frame is output in ascending Cavern `ReferenceChannel` order.
//! Input is streamed: [`Eac3Decoder::push`] bytes as they arrive and poll [`Eac3Decoder::next_frame`].

pub mod body;
pub mod consts;
pub mod fft;
pub mod header;

use std::collections::BTreeMap;
use std::panic::{catch_unwind, AssertUnwindSafe};

use body::Body;
use consts::{Eac3Error, MUST_DECODE};
use header::{Header, Peek};

/// One decoded E-AC-3 frame (all substreams of a timeslot).
#[derive(Clone, Debug, Default)]
pub struct Eac3Frame {
    pub sample_rate: u32,
    /// Samples per channel (blocks * 256).
    pub frame_size: usize,
    /// Cavern ReferenceChannel values, ascending.
    pub channel_ids: Vec<i32>,
    /// `[channel][frame_size]`, same scale as Cavern.
    pub pcm: Vec<Vec<f32>>,
    /// (substream id, aux bytes) for each substream decoded in this frame, in decode order.
    pub aux: Vec<(i32, Vec<u8>)>,
}

/// Where an output channel's samples come from (Cavern stores a reference to the body's result array).
#[derive(Clone, Copy)]
struct Source {
    body: usize,
    /// Index into `frame_result`, or `None` for the LFE result.
    channel: Option<usize>,
}

enum Fetch {
    NeedData,
    End,
    Frame { bytes: usize, substream_id: i32 },
}

/// Streaming E-AC-3 decoder.
pub struct Eac3Decoder {
    buf: Vec<u8>,
    pos: usize,
    input_done: bool,
    /// Cavern's `Finished`.
    finished: bool,
    header: Header,
    /// Independently decoded substreams (ids 0..7 independent, 8..15 dependent).
    bodies: Vec<Option<Box<Body>>>,
    /// Rendered samples for each channel (persists between frames, like Cavern's `outputs`).
    outputs: BTreeMap<i32, Source>,
    /// The header of the next substream is parsed and its body is prepared.
    cur_ready: bool,
    /// At least one substream of the current frame has been decoded.
    in_frame: bool,
    aux: Vec<(i32, Vec<u8>)>,
    /// Errors encountered (Cavern would throw); decoding continues with the next syncframe.
    pub errors: Vec<String>,
}

impl Default for Eac3Decoder {
    fn default() -> Self { Self::new() }
}

impl Eac3Decoder {
    pub fn new() -> Self {
        Self {
            buf: Vec::new(),
            pos: 0,
            input_done: false,
            finished: false,
            header: Header::default(),
            bodies: (0..16).map(|_| None).collect(),
            outputs: BTreeMap::new(),
            cur_ready: false,
            in_frame: false,
            aux: Vec::new(),
            errors: Vec::new(),
        }
    }

    /// Streaming input: append raw bytes as they arrive.
    pub fn push(&mut self, data: &[u8]) {
        if self.pos > 0 && (self.pos >= 65536 || self.pos * 2 >= self.buf.len()) {
            self.buf.drain(..self.pos);
            self.pos = 0;
        }
        self.buf.extend_from_slice(data);
    }

    /// Signal end of input.
    pub fn finish(&mut self) { self.input_done = true; }

    /// True when the end of the stream was reached and no more frames will be returned.
    pub fn is_finished(&self) -> bool { self.finished }

    /// Returns the next fully decoded frame when enough bytes are buffered, else None.
    pub fn next_frame(&mut self) -> Option<Eac3Frame> {
        loop {
            if self.finished {
                return None;
            }
            if !self.cur_ready {
                if self.in_frame {
                    // Cavern reads the next header to know whether the frame is complete (next substream ID is 0).
                    // Only the first bytes are needed for that; the full parse follows on the next call.
                    match self.fetch(false) {
                        Fetch::NeedData => return None,
                        Fetch::End => return self.end_of_stream(),
                        Fetch::Frame { substream_id, .. } => {
                            if substream_id == 0 {
                                return Some(self.emit());
                            }
                        }
                    }
                }
                match self.fetch(true) {
                    Fetch::NeedData => return None,
                    Fetch::End => return self.end_of_stream(),
                    Fetch::Frame { bytes, .. } => {
                        let frame = self.buf[self.pos..self.pos + bytes].to_vec();
                        self.pos += bytes;
                        if let Err(e) = self.read_header(frame) {
                            self.report(e);
                            continue;
                        }
                        self.cur_ready = true;
                    }
                }
            }

            // Decode the prepared substream
            self.cur_ready = false;
            let id = self.header.substream_id;
            let header = &self.header;
            let body = self.bodies[id as usize].as_mut().expect("body prepared");
            let result = catch_unwind(AssertUnwindSafe(|| body.update(header)));
            match result {
                Ok(Ok(())) => {}
                Ok(Err(e)) => {
                    self.report(format!("substream {id}: {e}"));
                    continue;
                }
                Err(_) => {
                    self.report(format!("substream {id}: decoder panic (corrupt data)"));
                    continue;
                }
            }
            for i in 0..body.channels.len() {
                self.outputs.insert(body.channels[i], Source { body: id as usize, channel: Some(i) });
            }
            if self.header.lfe {
                self.outputs.insert(consts::ch::SCREEN_LFE, Source { body: id as usize, channel: None });
            }
            self.aux.push((id, body.aux_data().to_vec()));
            self.in_frame = true;
        }
    }

    fn report(&mut self, e: String) {
        if self.errors.len() < 1000 {
            self.errors.push(e);
        }
    }

    /// Stream end: Cavern still outputs the pending frame if the last header it holds is substream 0.
    fn end_of_stream(&mut self) -> Option<Eac3Frame> {
        self.finished = true;
        if self.in_frame && self.header.substream_id == 0 {
            Some(self.emit())
        } else {
            None
        }
    }

    /// Cavern's `ReadHeader`: decode the header and prepare the body of the next substream.
    fn read_header(&mut self, frame: Vec<u8>) -> Result<(), String> {
        let header = &mut self.header;
        let bodies = &mut self.bodies;
        let r = catch_unwind(AssertUnwindSafe(|| -> Result<(), Eac3Error> {
            let extractor = header.decode(frame)?;
            let body = bodies[header.substream_id as usize].get_or_insert_with(|| Box::new(Body::new()));
            body.prepare_update(header, extractor)
        }));
        match r {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(format!("header: {e}")),
            Err(_) => Err("header: decoder panic (corrupt data)".to_string()),
        }
    }

    /// Look at the next syncframe. With `full`, the whole syncframe must be buffered.
    fn fetch(&mut self, full: bool) -> Fetch {
        loop {
            let avail = self.buf.len() - self.pos;
            if avail < MUST_DECODE {
                return if self.input_done { Fetch::End } else { Fetch::NeedData };
            }
            match Header::peek(&self.buf[self.pos..self.pos + MUST_DECODE]) {
                Ok(Peek::End) => return Fetch::End,
                Ok(Peek::Frame { bytes, substream_id }) => {
                    if full && avail < bytes {
                        return if self.input_done { Fetch::End } else { Fetch::NeedData };
                    }
                    return Fetch::Frame { bytes, substream_id };
                }
                Err(e) => {
                    // Cavern throws here; for a live stream, resynchronize to the next syncword instead.
                    self.report(format!("{e}"));
                    self.pos += 1;
                    // Stops at the last byte at worst, which may be the first half of a syncword.
                    while self.pos + 1 < self.buf.len()
                        && !(self.buf[self.pos] == 0x0B && self.buf[self.pos + 1] == 0x77)
                    {
                        self.pos += 1;
                    }
                }
            }
        }
    }

    /// Collect the outputs in ascending channel order.
    fn emit(&mut self) -> Eac3Frame {
        self.in_frame = false;
        let mut channel_ids = Vec::with_capacity(self.outputs.len());
        let mut pcm = Vec::with_capacity(self.outputs.len());
        for (&ch, src) in &self.outputs {
            let body = self.bodies[src.body].as_ref().expect("output body");
            let data = match src.channel {
                Some(i) => body.frame_result[i].clone(),
                None => body.lfe_result.clone().unwrap_or_default(),
            };
            channel_ids.push(ch);
            pcm.push(data);
        }
        let frame_size = pcm.first().map(|c| c.len()).unwrap_or(self.header.blocks as usize * 256);
        Eac3Frame {
            sample_rate: self.header.sample_rate as u32,
            frame_size,
            channel_ids,
            pcm,
            aux: std::mem::take(&mut self.aux),
        }
    }
}
