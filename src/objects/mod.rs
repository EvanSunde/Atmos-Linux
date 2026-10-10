//! EMDF / OAMD / JOC / QMF object decoding (port of Cavern.Format Decoders/EnhancedAC3/*). Owned by the object port.
//!
//! This file ports the renderer glue of Cavern.Format/Renderers/EnhancedAC3Renderer.cs (constructor, `Update`,
//! `RenderNextTimeslot`) together with the frame/timeslot ordering that results from
//! Cavern.Format/Decoders/EnhancedAC3Decoder.cs (`DecodeFrame` parses a frame's aux data right after its audio),
//! Cavern.Format/Decoders/FrameBasedDecoder.cs and Cavern.Format/Utilities/BlockBuffer.cs (`Read` fetches the
//! next frame as soon as the last sample of the current one is consumed, and resets `LastFetchStart` to 0).
//!
//! Resulting timing, reproduced by [`ObjectDecoder`]: the last 64-sample timeslot of frame N is rendered with
//! the metadata (OAMD/JOC) of frame N+1 and timecode 0; timeslot j < last of frame N uses frame N's metadata
//! with timecode j*64. Hence after `push_frame(N)` the last timeslot of frame N-1 and all but the last timeslot
//! of frame N are available; [`ObjectDecoder::flush`] renders the final pending timeslot at the end of a stream
//! (Cavern then reads a zeroed frame without new metadata).

mod cavern;
mod emdf;
mod joc;
mod joc_applier;
mod joc_tables;
mod joc_tables_huff;
mod oamd;
mod qmf;
mod qmf_coeffs;

use std::collections::VecDeque;

use crate::bits::BitExtractor;

use cavern::{ch, Vec3};
use emdf::ExtensibleMetadataDecoder;
use joc_applier::JointObjectCodingApplier;
use joc_tables::INPUT_MATRIX;
use oamd::SourceState;
pub use qmf::SUBBANDS as TIMESLOT_SAMPLES;

/// One 64-sample timeslot of rendered objects, matching one Cavern `GetNextObjectSamples(64)` call.
#[derive(Clone, Debug)]
pub struct ObjectBlock {
    /// Per object, Cavern `Source.Position` (environment units, environment size (10, 7, 10)).
    pub positions: Vec<[f32; 3]>,
    /// Per object, Cavern `Source.Volume`.
    pub volumes: Vec<f32>,
    /// Per object, Cavern `Source.Size` (0 until OAMD sets it; relative to the environment, 0 = point).
    pub sizes: Vec<f32>,
    /// Per object samples (LFE inserted at its position, as Cavern).
    pub samples: Vec<[f32; 64]>,
}

/// What an output object is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjectKind {
    /// The LFE channel (render to the subwoofer).
    Lfe,
    /// A static bed channel (Cavern ReferenceChannel value).
    Bed(i32),
    /// A dynamic, positioned object.
    Dynamic,
}

/// Decodes E-AC-3 + JOC object audio from decoded core frames and their aux data, like Cavern's
/// `EnhancedAC3Renderer`.
pub struct ObjectDecoder {
    /// `EnhancedAC3Decoder.Extensions`.
    ext: ExtensibleMetadataDecoder,
    /// The renderer was constructed (first frame was pushed).
    initialized: bool,
    /// The renderer was constructed in object mode (`stream.Extensions.HasObjects` at construction).
    object_mode: bool,
    /// `Renderer.HasObjects` (updated every timeslot).
    has_objects: bool,
    applier: Option<JointObjectCodingApplier>,
    /// `Renderer.objects`.
    sources: Vec<SourceState>,
    /// `ChannelPrototype.GetStandardMatrix(stream.ChannelCount)` at construction.
    input_matrix: Vec<i32>,
    /// The core output has an LFE channel (`outputMatrix` contains ScreenLFE).
    output_has_lfe: bool,
    /// `finalResult` / `timeslotResult` (one per JOC object, or per channel in channel mode).
    timeslot_result: Vec<[f32; 64]>,
    /// Index into the input data the LFE timeslot refers to (Cavern keeps a reference to that array).
    lfe_input: Option<usize>,
    /// Applier output slots that alias the input data arrays (Cavern quirk after a channel-based fallback
    /// timeslot in object mode: `timeslotResult[i] = inputData[i]` overwrites the applier's cache entries).
    aliased: Vec<bool>,
    /// Last `objectSamples` (kept for slots Cavern doesn't overwrite).
    object_samples: Vec<[f32; 64]>,
    /// The last timeslot of the previous frame, waiting for the next frame's metadata: [channel][sample].
    pending: Option<Vec<[f32; 64]>>,
    /// Rendered timeslots.
    out: VecDeque<ObjectBlock>,
}

impl Default for ObjectDecoder {
    fn default() -> Self { Self::new() }
}

impl ObjectDecoder {
    pub fn new() -> Self {
        Self {
            ext: ExtensibleMetadataDecoder::new(),
            initialized: false,
            object_mode: false,
            has_objects: false,
            applier: None,
            sources: Vec::new(),
            input_matrix: Vec::new(),
            output_has_lfe: false,
            timeslot_result: Vec::new(),
            lfe_input: None,
            aliased: Vec::new(),
            object_samples: Vec::new(),
            pending: None,
            out: VecDeque::new(),
        }
    }

    /// Feed one decoded core frame: channel ids (Cavern ReferenceChannel values, ascending), planar PCM, and the
    /// aux byte blocks of the substreams decoded for this frame, in order.
    pub fn push_frame(&mut self, channel_ids: &[i32], pcm: &[Vec<f32>], aux: &[(i32, Vec<u8>)]) {
        // EnhancedAC3Decoder.DecodeFrame: Extensions.Decode for each decoded substream, in order.
        for (_, bytes) in aux {
            let mut e = BitExtractor::with_last_byte(bytes.clone(), bytes.len());
            self.ext.decode(&mut e);
        }

        let frame_size = pcm.first().map(|c| c.len()).unwrap_or(0);
        if !self.initialized {
            self.construct(channel_ids, frame_size);
        }

        // The previous frame's last timeslot is read after this frame was decoded (LastFetchStart = 0).
        if let Some(prev) = self.pending.take() {
            self.render_timeslot(&prev, 0);
        }

        let timeslots = frame_size / TIMESLOT_SAMPLES;
        let nch = pcm.len();
        let mut slot = vec![[0f32; 64]; nch];
        for ts in 0..timeslots {
            for (c, channel) in pcm.iter().enumerate() {
                slot[c].copy_from_slice(&channel[ts * 64..ts * 64 + 64]);
            }
            if ts + 1 == timeslots {
                self.pending = Some(slot.clone());
            } else {
                self.render_timeslot(&slot, (ts * 64) as i32);
            }
        }
    }

    /// End of stream: render the last pending timeslot (Cavern reads it with unchanged metadata and timecode 0).
    pub fn flush(&mut self) {
        if let Some(prev) = self.pending.take() {
            self.render_timeslot(&prev, 0);
        }
    }

    /// Pop the next rendered timeslot when available.
    pub fn next_block(&mut self) -> Option<ObjectBlock> { self.out.pop_front() }

    /// The stream is currently decoded as objects (Cavern `Renderer.HasObjects`).
    pub fn has_objects(&self) -> bool { self.has_objects }

    /// Number of rendered objects (Cavern `Renderer.Objects.Count`); 0 before the first frame.
    pub fn object_count(&self) -> usize { self.sources.len() }

    /// For rendering: is object i the LFE / a bed channel (and which ReferenceChannel) or dynamic.
    pub fn object_kind(&self, i: usize) -> ObjectKind {
        if self.object_mode {
            let oamd = &self.ext.oamd;
            if i < oamd.beds as usize {
                let channel = oamd.get_static_channels()[i];
                return if channel == ch::SCREEN_LFE { ObjectKind::Lfe } else { ObjectKind::Bed(channel) };
            }
            ObjectKind::Dynamic
        } else {
            match self.input_matrix.get(i) {
                Some(&c) if c == ch::SCREEN_LFE => ObjectKind::Lfe,
                Some(&c) => ObjectKind::Bed(c),
                None => ObjectKind::Dynamic,
            }
        }
    }

    /// EnhancedAC3Renderer constructor.
    fn construct(&mut self, channel_ids: &[i32], frame_size: usize) {
        self.initialized = true;
        self.input_matrix = cavern::get_standard_matrix(channel_ids.len());
        self.output_has_lfe = channel_ids.contains(&ch::SCREEN_LFE);

        if self.ext.has_objects {
            // Object-based rendering
            self.object_mode = true;
            let joc_objects = self.ext.joc.object_count;
            self.timeslot_result = vec![[0.0; 64]; joc_objects];
            self.aliased = vec![false; joc_objects];
            self.applier = Some(JointObjectCodingApplier::new(&self.ext.joc, frame_size));
            self.sources = vec![SourceState::default(); self.ext.oamd.object_count];
            self.has_objects = true;
        } else {
            // Channel-based rendering
            self.sources = channel_ids
                .iter()
                .enumerate()
                .map(|(i, &c)| SourceState {
                    lfe: self.input_matrix[i] == ch::SCREEN_LFE,
                    position: cavern::alternative_position(c),
                    ..SourceState::default()
                })
                .collect();
            self.timeslot_result = vec![[0.0; 64]; channel_ids.len()];
            self.aliased = vec![false; channel_ids.len()];
        }
        for s in &mut self.sources {
            s.volume = 0.707; // E-AC-3 + JOC has a general -3 dB gain to prevent clipping
        }
        self.object_samples = vec![[0.0; 64]; self.sources.len()];
    }

    /// `Update(64)` → `RenderNextTimeslot()` and the LFE insertion, then record the block.
    /// `input` is `inputData` (channels in decoder output order), `timecode` is `LastFetchStart / ChannelCount`.
    fn render_timeslot(&mut self, input: &[[f32; 64]], timecode: i32) {
        let mut lfe_timeslot = [0f32; 64];
        // Object-based rendering (only possible when the renderer was constructed with objects; Cavern would
        // dereference a null applier otherwise).
        self.has_objects = self.ext.has_objects && self.object_mode;
        if self.has_objects {
            self.ext.oamd.update_sources(timecode, &mut self.sources);

            let mut sources: [Option<&[f32]>; 7] = [None; 7];
            for (i, &target) in INPUT_MATRIX.iter().enumerate() {
                if let Some(j) = self.input_matrix.iter().position(|&c| c == target) {
                    sources[i] = input.get(j).map(|a| &a[..]);
                }
            }
            for (i, &c) in self.input_matrix.iter().enumerate() {
                if c == ch::SCREEN_LFE {
                    self.lfe_input = Some(i);
                }
            }

            let applier = self.applier.as_mut().unwrap();
            let result = applier.apply(&sources, &mut self.ext.joc);
            for (dst, src) in self.timeslot_result.iter_mut().zip(result.iter()) {
                *dst = *src;
            }
            if let Some(j) = self.lfe_input {
                lfe_timeslot = match self.aliased.get(j) {
                    // The applier wrote this object's output into the shared input array.
                    Some(true) => result[j],
                    _ => input.get(j).copied().unwrap_or([0.0; 64]),
                };
            }
        } else {
            // Channel-based rendering or fallback to it when OAMD or JOC can't be decoded correctly
            let n = self.input_matrix.len().min(self.timeslot_result.len()).min(input.len());
            for i in 0..n {
                if let Some(s) = self.sources.get_mut(i) {
                    s.position = cavern::alternative_position(self.input_matrix[i]);
                }
                self.timeslot_result[i] = input[i];
                if self.object_mode {
                    self.aliased[i] = true;
                }
            }
            for i in n..self.timeslot_result.len() {
                if let Some(s) = self.sources.get_mut(i) {
                    s.position = Vec3::default();
                }
                self.timeslot_result[i] = [0.0; 64];
            }
            if let Some(j) = self.lfe_input {
                lfe_timeslot = input.get(j).copied().unwrap_or([0.0; 64]);
            }
        }

        // LFE insertion (Update)
        let count = self.object_samples.len();
        let lfe = self.ext.oamd.get_lfe_position();
        if lfe != -1 {
            let lfe = lfe as usize;
            for i in 0..count {
                if i < lfe {
                    self.object_samples[i] = self.timeslot_result.get(i).copied().unwrap_or([0.0; 64]);
                } else if i == lfe {
                    if self.output_has_lfe {
                        self.object_samples[i] = lfe_timeslot;
                    }
                } else {
                    self.object_samples[i] = self.timeslot_result.get(i - 1).copied().unwrap_or([0.0; 64]);
                }
            }
        } else {
            for i in 0..count {
                self.object_samples[i] = self.timeslot_result.get(i).copied().unwrap_or([0.0; 64]);
            }
        }

        self.out.push_back(ObjectBlock {
            positions: self.sources.iter().map(|s| s.position.to_array()).collect(),
            volumes: self.sources.iter().map(|s| s.volume).collect(),
            sizes: self.sources.iter().map(|s| s.size).collect(),
            samples: self.object_samples.clone(),
        });
    }
}
