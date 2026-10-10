//! Joint object coding. Port of Cavern.Format/Decoders/EnhancedAC3/JointObjectCoding.cs,
//! JointObjectCodingDecoder.cs and JointObjectCodingCache.cs.
//!
//! Cavern computes the per-object mixing matrices on the thread pool; every object is independent, so a
//! sequential loop gives identical results.

use crate::bits::BitExtractor;

use super::joc_tables::{get_huff_code_table, parameter_band_mapping, HuffmanType, INPUT_MATRIX, JOC_NUM_BANDS};
use super::qmf::SUBBANDS;

/// Maximum number of objects to render.
const MAX_OBJECTS: usize = 64;
/// Maximum data points per object.
const MAX_DATA_POINTS: usize = 2;
/// Maximum QMF timeslots per frame (1536 / 64).
pub const MAX_TIMESLOTS: usize = 1536 / SUBBANDS;

/// Unsupported JOC feature (Cavern throws `UnsupportedFeatureException`).
#[derive(Debug)]
#[allow(dead_code)]
pub struct Unsupported(pub &'static str);

/// Per-object JOC state (the per-object slices of Cavern's reusable arrays).
#[derive(Clone)]
struct JocObject {
    /// joc_num_bands_idx.
    bands_index: u8,
    /// joc_num_bands.
    bands: u8,
    /// b_joc_sparse.
    sparse_coded: bool,
    /// Index of the used quantization table.
    quantization_table: u8,
    /// joc_slope_idx: steep (stepped) transition.
    steep_slope: bool,
    /// Number of data points.
    data_points: usize,
    /// Timeslot indexes where the source matrix changes.
    timeslot_offsets: [i32; MAX_DATA_POINTS],
    /// Source channel indexes in sparse mode [dp][band].
    joc_channel: [[i32; 23]; MAX_DATA_POINTS],
    /// Quantized data for sparse mode [dp][band].
    joc_vector: [[i32; 23]; MAX_DATA_POINTS],
    /// Quantized, differentially coded matrix [dp][ch][band].
    joc_matrix: Vec<[[i32; 23]; MAX_DATA_POINTS]>,
    /// Decoded JOC matrix [dp][ch][band (up to 64 slots)].
    mix_matrix: [Vec<[f32; SUBBANDS]>; MAX_DATA_POINTS],
    /// Decoded JOC matrix with fading [ts][ch][subband].
    pub interpolated: Vec<Vec<[f32; SUBBANDS]>>,
}

impl JocObject {
    fn new(channels: usize) -> Self {
        Self {
            bands_index: 0,
            bands: 0,
            sparse_coded: false,
            quantization_table: 0,
            steep_slope: false,
            data_points: 0,
            timeslot_offsets: [0; MAX_DATA_POINTS],
            joc_channel: [[0; 23]; MAX_DATA_POINTS],
            joc_vector: [[0; 23]; MAX_DATA_POINTS],
            // Cavern layout is [dp][ch][band]; stored here as [ch][dp][band].
            joc_matrix: vec![[[0; 23]; MAX_DATA_POINTS]; channels],
            mix_matrix: [vec![[0.0; SUBBANDS]; channels], vec![[0.0; SUBBANDS]; channels]],
            interpolated: vec![vec![[0.0; SUBBANDS]; channels]; MAX_TIMESLOTS],
        }
    }
}

/// Joint object coding decoder.
pub struct JointObjectCoding {
    /// The object is active and will have rendered audio data (b_joc_obj_present).
    pub object_active: Vec<bool>,
    /// Number of full bandwidth input channels.
    pub channel_count: usize,
    /// Number of rendered dynamic objects.
    pub object_count: usize,
    /// Multiplier for the output signal's amplitude.
    pub gain: f32,
    /// The JOC contained vector-based and not matrix-based encoded data at some point.
    pub had_sparse: bool,
    /// Previous JOC mixing matrix values [obj][ch][subband].
    prev_matrix: Vec<[[f32; SUBBANDS]; 7]>,
    objects: Vec<JocObject>,
    /// Channel count the cache was allocated for (`jocMatrix[0][0].Length`).
    cache_channels: usize,
}

impl Default for JointObjectCoding {
    fn default() -> Self { Self::new() }
}

impl JointObjectCoding {
    /// Create a JOC decoder. Always reuse a previous one as history data is required for decoding.
    pub fn new() -> Self {
        debug_assert_eq!(INPUT_MATRIX.len(), 7);
        Self {
            object_active: Vec::new(),
            channel_count: 0,
            object_count: 0,
            gain: 0.0,
            had_sparse: false,
            prev_matrix: vec![[[0.0; SUBBANDS]; 7]; MAX_OBJECTS],
            objects: Vec::new(),
            cache_channels: 0,
        }
    }

    /// Decodes a JOC frame from an EMDF payload.
    pub fn decode(&mut self, e: &mut BitExtractor) -> Result<(), Unsupported> {
        self.decode_header(e)?;
        self.decode_info(e);
        self.decode_data(e);
        Ok(())
    }

    fn decode_header(&mut self, e: &mut BitExtractor) -> Result<(), Unsupported> {
        let downmix_config = e.read(3);
        if downmix_config > 4 {
            return Err(Unsupported("joc_dmx_config_idx"));
        }
        self.channel_count = if downmix_config == 0 || downmix_config == 3 { 5 } else { 7 };
        self.object_count = e.read(6) as usize + 1;
        self.update_cache();
        if e.read(3) != 0 {
            return Err(Unsupported("joc_ext_config_idx"));
        }
        Ok(())
    }

    /// Read JOC metadata.
    fn decode_info(&mut self, e: &mut BitExtractor) {
        let gain_power = e.read(3);
        self.gain = 1.0 + (e.read(5) as f32 / 32.0) * 2f32.powf((gain_power - 4) as f32);
        e.skip(10); // Sequence counter
        for obj in 0..self.object_count {
            let active = e.read_bit();
            self.object_active[obj] = active;
            if active {
                let o = &mut self.objects[obj];
                o.bands_index = e.read(3) as u8;
                o.bands = JOC_NUM_BANDS[o.bands_index as usize];
                o.sparse_coded = e.read_bit();
                o.quantization_table = e.read_bit_int() as u8;

                // joc_data_point_info
                o.steep_slope = e.read_bit();
                o.data_points = e.read(1) as usize + 1;
                if o.steep_slope {
                    for dp in 0..o.data_points {
                        o.timeslot_offsets[dp] = e.read(5) + 1;
                    }
                }
            }
        }
    }

    /// Read JOC channels/vectors/matrices.
    fn decode_data(&mut self, e: &mut BitExtractor) {
        let channels = self.channel_count;
        for obj in 0..self.object_count {
            if !self.object_active[obj] {
                continue;
            }
            let o = &mut self.objects[obj];
            let bands = o.bands as usize;
            if o.sparse_coded {
                let channel_table = get_huff_code_table(channels as i32, HuffmanType::Idx);
                let vec_table = get_huff_code_table(o.quantization_table as i32, HuffmanType::Vec);
                for dp in 0..o.data_points {
                    o.joc_channel[dp][0] = e.read(3);
                    for pb in 1..bands {
                        o.joc_channel[dp][pb] = huffman_decode(channel_table, e);
                    }
                    for pb in 0..bands {
                        o.joc_vector[dp][pb] = huffman_decode(vec_table, e);
                    }
                }
            } else {
                let code_table = get_huff_code_table(o.quantization_table as i32, HuffmanType::Mtx);
                for dp in 0..o.data_points {
                    for ch in 0..channels {
                        for pb in 0..bands {
                            o.joc_matrix[ch][dp][pb] = huffman_decode(code_table, e);
                        }
                    }
                }
            }
        }
    }

    /// Checks if the cache is ready for the given number of objects and channels, and fixes if it's not.
    fn update_cache(&mut self) {
        if self.object_active.len() == self.object_count && self.cache_channels == self.channel_count {
            return;
        }
        self.object_active = vec![false; self.object_count];
        self.objects = vec![JocObject::new(self.channel_count); self.object_count];
        self.cache_channels = self.channel_count;
    }

    /// Get the object mixing matrices for a frame of `frame_size` samples. Results are read with [`Self::interpolated`].
    pub fn get_mixing_matrices(&mut self, frame_size: usize) {
        let timeslots = (frame_size / SUBBANDS).min(MAX_TIMESLOTS);
        for obj in 0..self.object_count {
            self.get_mixing_matrices_obj(obj, timeslots);
        }
    }

    /// The interpolated mixing matrix of an object for a timeslot: [ch][subband].
    #[inline]
    pub fn interpolated(&self, obj: usize, timeslot: usize) -> &[[f32; SUBBANDS]] {
        &self.objects[obj].interpolated[timeslot]
    }

    fn get_mixing_matrices_obj(&mut self, obj: usize, timeslots: usize) {
        let channels = self.channel_count;
        let active = self.object_active[obj];
        let o = &mut self.objects[obj];
        let prev = &mut self.prev_matrix[obj];
        let center_value = o.quantization_table as i32 * 48 + 48;
        if active {
            let gain_step = 0.2f32 - o.quantization_table as f32 * 0.1f32;
            if o.sparse_coded {
                // Cavern: "Call DecodeSparse and revert 0 to gainStep when the standard documentation is fixed".
                // The sparse matrix is never decoded, the stale matrix is multiplied by 0 instead (the 'sparse workaround').
                self.had_sparse = true;
                dequantize(o, channels, center_value, 0.0);
            } else {
                decode_coarse(o, channels, center_value, gain_step);
            }
        } else {
            // The final result is in the interpolation matrix.
            for ts in 0..timeslots {
                for ch in 0..channels {
                    o.interpolated[ts][ch] = [0.0; SUBBANDS];
                }
            }
            return;
        }

        let pb_mapping = &parameter_band_mapping()[o.bands_index as usize];
        if o.data_points == 1 {
            if o.steep_slope {
                // Cavern would throw if the split point is beyond the cached 24 timeslots.
                let split_point = (o.timeslot_offsets[0].max(0) as usize).min(MAX_TIMESLOTS);
                for ts in 0..split_point {
                    for ch in 0..channels {
                        o.interpolated[ts][ch] = prev[ch];
                    }
                }
                for ts in split_point..timeslots {
                    // Quirk kept from Cavern: with one data point, the stale second matrix is used before the
                    // (also possibly stale) second offset.
                    let source = if (ts as i32) < o.timeslot_offsets[1] { 1 } else { 0 };
                    for ch in 0..channels {
                        let src = o.mix_matrix[source][ch];
                        let dst = &mut o.interpolated[ts][ch];
                        for sb in 0..SUBBANDS {
                            dst[sb] = src[pb_mapping[sb] as usize];
                        }
                    }
                }
            } else {
                for ch in 0..channels {
                    let channel_prev = prev[ch];
                    let mix = o.mix_matrix[0][ch];
                    for ts in 0..timeslots {
                        let lerp = (ts + 1) as f32 / timeslots as f32;
                        let dst = &mut o.interpolated[ts][ch];
                        for sb in 0..SUBBANDS {
                            dst[sb] = channel_prev[sb] + (mix[pb_mapping[sb] as usize] - channel_prev[sb]) * lerp;
                        }
                    }
                }
            }
        } else if o.steep_slope {
            for ts in 0..timeslots {
                let t1 = ts as i32 + 1;
                for ch in 0..channels {
                    // Quirk kept from Cavern: the current matrix is copied without parameter band mapping.
                    let src = if t1 < o.timeslot_offsets[0] { prev[ch] } else { o.mix_matrix[0][ch] };
                    o.interpolated[ts][ch] = src;
                }
            }
        } else {
            let ts_2 = timeslots >> 1;
            for ts in 0..timeslots {
                let t1 = ts + 1;
                if t1 <= ts_2 {
                    let lerp = t1 as f32 / ts_2 as f32;
                    for ch in 0..channels {
                        // Quirk kept from Cavern: the first half interpolates towards the parameter-band matrix
                        // indexed by subband (no mapping).
                        let from = prev[ch];
                        let to = o.mix_matrix[0][ch];
                        let dst = &mut o.interpolated[ts][ch];
                        for sb in 0..SUBBANDS {
                            dst[sb] = from[sb] + (to[sb] - from[sb]) * lerp;
                        }
                    }
                } else {
                    let lerp = (t1 - ts_2) as f32 / (timeslots - ts_2) as f32;
                    for ch in 0..channels {
                        let from = o.mix_matrix[0][ch];
                        let to = o.mix_matrix[1][ch];
                        let dst = &mut o.interpolated[ts][ch];
                        for sb in 0..SUBBANDS {
                            let pb = pb_mapping[sb] as usize;
                            dst[sb] = from[pb] + (to[pb] - from[pb]) * lerp;
                        }
                    }
                }
            }
        }

        // Cavern repeats this for every timeslot, the result is the same.
        let last = o.data_points - 1;
        for ch in 0..channels {
            let mix_source = &o.mix_matrix[last][ch];
            for sb in 0..SUBBANDS {
                prev[ch][sb] = mix_source[pb_mapping[sb] as usize];
            }
        }
    }
}

/// Read a Huffman-coded value from the bitstream.
#[inline]
fn huffman_decode(code_table: &[[i16; 2]], e: &mut BitExtractor) -> i32 {
    let mut node: i32 = 0;
    loop {
        let bit = e.read_bit() as usize;
        node = code_table.get(node as usize).map(|n| n[bit] as i32).unwrap_or(0);
        if node <= 0 {
            break;
        }
    }
    !node
}

/// Decode and dequantize a complete JOC matrix around a `quantized_center` value.
fn decode_coarse(o: &mut JocObject, channels: usize, quantized_center: i32, gain_step: f32) {
    let center = quantized_center as f32 * gain_step;
    let max = center * 2.0;
    let bands = o.bands as usize;
    for dp in 0..o.data_points {
        for ch in 0..channels {
            let src = &o.joc_matrix[ch][dp];
            let dst = &mut o.mix_matrix[dp][ch];
            dst[0] = (center + src[0] as f32 * gain_step) % max;
            for i in 1..bands {
                let next = (dst[i - 1] + src[i] as f32 * gain_step) % max;
                dst[i - 1] -= center;
                dst[i] = next;
            }
            dst[bands - 1] -= center;
        }
    }
}

/// Convert the values of the decoded JOC matrix to the mixing range.
fn dequantize(o: &mut JocObject, channels: usize, center: i32, gain_step: f32) {
    let bands = o.bands as usize;
    for dp in 0..o.data_points {
        for ch in 0..channels {
            let m = &mut o.mix_matrix[dp][ch];
            for v in m.iter_mut().take(bands) {
                *v = (*v - center as f32) * gain_step;
            }
        }
    }
}
