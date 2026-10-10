//! Converts a channel-based audio stream and JOC to object output samples. Port of
//! Cavern.Format/Decoders/EnhancedAC3/JointObjectCodingApplier.cs (the non-Mono, non-CavernAmp `ProcessObject`
//! path, which is what runs under .NET on Linux). Cavern runs the per-channel forward and per-object inverse
//! transforms on the thread pool; they are independent, so running them sequentially gives the same result.

use super::joc::JointObjectCoding;
use super::joc_tables::INPUT_MATRIX;
use super::qmf::{QuadratureMirrorFilterBank, SUBBANDS};

pub struct JointObjectCodingApplier {
    /// Length of an AC-3 frame (fixed at construction).
    frame_size: usize,
    /// Recycled timeslot object output arrays.
    pub timeslot_cache: Vec<[f32; SUBBANDS]>,
    /// Forward transformation results [ch].
    results_real: Vec<[f32; SUBBANDS]>,
    results_imag: Vec<[f32; SUBBANDS]>,
    /// Recycled QMFB transform objects (shared by forward channel and inverse object transforms, as in Cavern).
    converters: Vec<QuadratureMirrorFilterBank>,
    /// Next timeslot to read in the current JOC.
    timeslot: usize,
}

impl JointObjectCodingApplier {
    pub fn new(joc: &JointObjectCoding, frame_size: usize) -> Self {
        let max_channels = INPUT_MATRIX.len();
        let objects = joc.object_count;
        let converter_count = max_channels.max(objects);
        Self {
            frame_size,
            timeslot_cache: vec![[0.0; SUBBANDS]; objects],
            results_real: vec![[0.0; SUBBANDS]; max_channels],
            results_imag: vec![[0.0; SUBBANDS]; max_channels],
            converters: (0..converter_count).map(|_| QuadratureMirrorFilterBank::new()).collect(),
            timeslot: 0,
        }
    }

    /// Gets the audio samples of each object for the next timeslot. `input` is in [`INPUT_MATRIX`] order
    /// (None where the core has no such channel; Cavern would crash on a missing channel that JOC uses,
    /// silence is used instead).
    pub fn apply(&mut self, input: &[Option<&[f32]>; 7], joc: &mut JointObjectCoding) -> &[[f32; SUBBANDS]] {
        if self.timeslot == 0 {
            joc.get_mixing_matrices(self.frame_size);
        }

        // Forward transformations
        let channels = joc.channel_count.min(INPUT_MATRIX.len());
        const SILENCE: [f32; SUBBANDS] = [0.0; SUBBANDS];
        for ch in 0..channels {
            let conv = &mut self.converters[ch];
            conv.process_forward(input[ch].unwrap_or(&SILENCE));
            self.results_real[ch] = conv.out_real;
            self.results_imag[ch] = conv.out_imag;
        }

        // Inverse transformations
        let objects = joc.object_count.min(self.timeslot_cache.len());
        let gain = joc.gain;
        let mut real = [0f32; SUBBANDS];
        let mut imag = [0f32; SUBBANDS];
        for obj in 0..objects {
            let mix = joc.interpolated(obj, self.timeslot);
            let (r0, i0, m0) = (&self.results_real[0], &self.results_imag[0], &mix[0]);
            for sb in 0..SUBBANDS {
                real[sb] = r0[sb] * m0[sb];
                imag[sb] = i0[sb] * m0[sb];
            }
            for ch in 1..channels {
                let (rc, ic, mc) = (&self.results_real[ch], &self.results_imag[ch], &mix[ch]);
                for sb in 0..SUBBANDS {
                    real[sb] = real[sb] + rc[sb] * mc[sb];
                    imag[sb] = imag[sb] + ic[sb] * mc[sb];
                }
            }
            let out = &mut self.timeslot_cache[obj];
            self.converters[obj].process_inverse(&real, &imag, out);
            if gain != 1.0 {
                for v in out.iter_mut() {
                    *v *= gain;
                }
            }
        }

        // Quirk kept from Cavern: the timeslot counter wraps at the input array length (7 JOC input channels),
        // not at the frame's timeslot count, so the mixing matrices are recalculated every 7 timeslots and only
        // their first 7 timeslots are used.
        self.timeslot += 1;
        if self.timeslot == input.len() {
            self.timeslot = 0;
        }
        &self.timeslot_cache
    }
}
