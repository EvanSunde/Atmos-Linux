//! Quadrature mirror filter bank. Port of
//! Cavern.Format/Decoders/EnhancedAC3/QuadratureMirrorFilterBank.cs and QuadratureMirrorFilterBank.Process.cs
//! (the vectorised .NET path, which is what runs under .NET with no CavernAmp), plus the
//! Cavern/Utilities/QMath.Vector.cs helpers it calls.
//!
//! Arithmetic order follows the C# helpers: element-wise products/sums are evaluated left to right without FMA.
//! `QMath.MultiplyAndAdd(float*, float*, int)` (a dot product) uses `Vector<float>` (8 lanes with AVX2) and
//! `Vector.Dot`, which the .NET 8 JIT lowers to a pairwise horizontal sum; that order is reproduced in
//! [`dot_vector8`] (tiny float differences are possible if the JIT order differs).

use std::sync::OnceLock;

use super::qmf_coeffs::COEFFS;

/// QMFB subband count.
pub const SUBBANDS: usize = 64;
/// Value needed for transforming subbands.
const DOUBLE_LENGTH: usize = SUBBANDS * 2;
/// 1 / subbands.
const SUBBAND_DIV: f32 = 1.0 / SUBBANDS as f32;

/// Rotation caches shared by all filter banks (static in Cavern).
struct Caches {
    forward_real: Vec<[f32; DOUBLE_LENGTH]>,
    forward_imag: Vec<[f32; DOUBLE_LENGTH]>,
    inverse_real: Vec<[f32; DOUBLE_LENGTH]>,
    inverse_imag: Vec<[f32; DOUBLE_LENGTH]>,
}

fn caches() -> &'static Caches {
    static C: OnceLock<Caches> = OnceLock::new();
    C.get_or_init(|| {
        let mut c = Caches {
            forward_real: vec![[0.0; DOUBLE_LENGTH]; SUBBANDS],
            forward_imag: vec![[0.0; DOUBLE_LENGTH]; SUBBANDS],
            inverse_real: vec![[0.0; DOUBLE_LENGTH]; SUBBANDS],
            inverse_imag: vec![[0.0; DOUBLE_LENGTH]; SUBBANDS],
        };
        let pi = std::f32::consts::PI; // MathF.PI
        for sb in 0..SUBBANDS {
            for j in 0..DOUBLE_LENGTH {
                let exp = pi * (sb as f32 + 0.5) * (j as f32 - 0.5) * SUBBAND_DIV;
                c.forward_real[sb][j] = exp.cos();
                c.forward_imag[sb][j] = exp.sin();
                let exp = pi * (sb as f32 + 0.5) * (j as f32 - DOUBLE_LENGTH as f32 + 0.5) * SUBBAND_DIV;
                c.inverse_real[sb][j] = exp.cos() * SUBBAND_DIV;
                c.inverse_imag[sb][j] = exp.sin() * SUBBAND_DIV;
            }
        }
        c
    })
}

/// `QMath.MultiplyAndAdd(float* lhs, float* rhs, int count)` with `Vector<float>.Count == 8`:
/// `sum += Vector.Dot(lhs[i..i+8] * rhs[i..i+8], One)` per chunk.
#[inline]
fn dot_vector8(lhs: &[f32], rhs: &[f32]) -> f32 {
    debug_assert!(lhs.len() == rhs.len() && lhs.len() % 8 == 0);
    let mut sum = 0f32;
    for (a, b) in lhs.chunks_exact(8).zip(rhs.chunks_exact(8)) {
        let p = [a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3], a[4] * b[4], a[5] * b[5], a[6] * b[6], a[7] * b[7]];
        // Vector256 sum: add the upper 128-bit half to the lower, then two horizontal adds.
        let q = [p[0] + p[4], p[1] + p[5], p[2] + p[6], p[3] + p[7]];
        sum += (q[0] + q[1]) + (q[2] + q[3]);
    }
    sum
}

/// Converts a PCM stream to a quadrature mirror filter bank and back.
pub struct QuadratureMirrorFilterBank {
    /// Input sample cache for forward transformations.
    input_stream_forward: Vec<f32>,
    /// Input sample cache for inverse transformations.
    input_stream_inverse: Vec<f32>,
    /// Summation of window groups.
    grouping: [f32; DOUBLE_LENGTH],
    /// Output cache (real).
    pub out_real: [f32; SUBBANDS],
    /// Output cache (imaginary).
    pub out_imag: [f32; SUBBANDS],
}

impl Default for QuadratureMirrorFilterBank {
    fn default() -> Self { Self::new() }
}

impl QuadratureMirrorFilterBank {
    pub fn new() -> Self {
        caches();
        Self {
            input_stream_forward: vec![0.0; COEFFS.len()],
            input_stream_inverse: vec![0.0; COEFFS.len() * 2],
            grouping: [0.0; DOUBLE_LENGTH],
            out_real: [0.0; SUBBANDS],
            out_imag: [0.0; SUBBANDS],
        }
    }

    /// Transform a timeslot of real subbands to QMFB. Results are in `out_real` / `out_imag`.
    pub fn process_forward(&mut self, input: &[f32]) {
        let len = COEFFS.len();
        self.input_stream_forward.copy_within(0..len - SUBBANDS, SUBBANDS);
        for i in 0..SUBBANDS {
            self.input_stream_forward[i] = input[SUBBANDS - 1 - i];
        }

        let stream = &self.input_stream_forward;
        let g = &mut self.grouping;
        for i in 0..DOUBLE_LENGTH {
            g[i] = stream[i] * COEFFS[i];
        }
        let mut sample = DOUBLE_LENGTH;
        while sample < len {
            let s = &stream[sample..sample + DOUBLE_LENGTH];
            let c = &COEFFS[sample..sample + DOUBLE_LENGTH];
            for i in 0..DOUBLE_LENGTH {
                g[i] = g[i] + s[i] * c[i];
            }
            sample += DOUBLE_LENGTH;
        }

        let c = caches();
        for sb in 0..SUBBANDS {
            self.out_real[sb] = dot_vector8(&c.forward_real[sb], g);
            self.out_imag[sb] = dot_vector8(&c.forward_imag[sb], g);
        }
    }

    /// Convert a timeslot of QMFB subbands to PCM samples.
    pub fn process_inverse(&mut self, in_real: &[f32; SUBBANDS], in_imag: &[f32; SUBBANDS], output: &mut [f32; SUBBANDS]) {
        let len = self.input_stream_inverse.len();
        self.input_stream_inverse.copy_within(0..len - DOUBLE_LENGTH, DOUBLE_LENGTH);

        let c = caches();
        {
            let inv = &mut self.input_stream_inverse[..DOUBLE_LENGTH];
            let (cr, ci) = (&c.inverse_real[0], &c.inverse_imag[0]);
            let (r, im) = (in_real[0], -in_imag[0]);
            for j in 0..DOUBLE_LENGTH {
                inv[j] = cr[j] * r + ci[j] * im;
            }
            for sb in 1..SUBBANDS {
                let (cr, ci) = (&c.inverse_real[sb], &c.inverse_imag[sb]);
                let (r, im) = (in_real[sb], -in_imag[sb]);
                for j in 0..DOUBLE_LENGTH {
                    inv[j] = inv[j] + cr[j] * r + ci[j] * im;
                }
            }
        }

        let inv = &self.input_stream_inverse;
        for i in 0..SUBBANDS {
            output[i] = inv[i] * COEFFS[i] + inv[SUBBANDS * 3 + i] * COEFFS[SUBBANDS + i];
        }
        for j in 1..COEFFS.len() / DOUBLE_LENGTH {
            let time_slot = SUBBANDS * 4 * j;
            let coeff_slot = DOUBLE_LENGTH * j;
            let time_pair = time_slot + SUBBANDS * 3;
            let coeff_pair = coeff_slot + SUBBANDS;
            let (a, b) = (&inv[time_slot..time_slot + SUBBANDS], &COEFFS[coeff_slot..coeff_slot + SUBBANDS]);
            let (c2, d) = (&inv[time_pair..time_pair + SUBBANDS], &COEFFS[coeff_pair..coeff_pair + SUBBANDS]);
            for i in 0..SUBBANDS {
                output[i] = output[i] + a[i] * b[i] + c2[i] * d[i];
            }
        }
    }
}
