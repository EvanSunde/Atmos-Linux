//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/Allocation.cs, AllocationParsing.cs,
//! AllocationConstants.cs and AllocationHistory.cs (decode paths only).

use std::sync::OnceLock;

use super::tables::*;
use super::{EXP_D15, EXP_D45, MAX_ALLOCATION_SIZE};
use crate::bits::BitExtractor;
use crate::eac3::fft::{convolve, in_place_ifft_unscaled, Complex, FftCache};

/// Maximum IMDCT transformation size.
const IMDCT_SIZE: usize = 512;

/// Number of LFE groups.
const NLFEGRPS: i32 = 2;
/// First mantissa of the LFE channel.
pub const LFESTRTMANT: i32 = 0;
/// Last mantissa of the LFE channel.
pub const LFEENDMANT: i32 = 7;

const BAP1_BITS: i32 = 5;
const BAP2_BITS: i32 = 7;
const BAP3_BITS: i32 = 3;
const BAP4_BITS: i32 = 7;
const BAP5_BITS: i32 = 4;

/// Number of bits to read for each value of a BAP table.
static BITS_TO_READ: [i32; 16] = [0, BAP1_BITS, BAP2_BITS, BAP3_BITS, BAP4_BITS, BAP5_BITS, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16];

/// BitConversions.int24Max
const INT24_MAX: i32 = (1 << 23) - 1;
/// BitConversions.fromInt24
const FROM_INT24: f32 = 1.0 / INT24_MAX as f32;
/// BitConversions.fromInt32 (1f / int.MaxValue, which rounds to 2^-31 in float)
const FROM_INT32: f32 = 1.0 / i32::MAX as f32;

/// Shared state between allocations of one body (AllocationHistory.cs: bap1Pos, bap1Next...).
#[derive(Default, Clone, Copy)]
pub struct MantissaHistory {
    pub bap1_pos: i32,
    pub bap2_pos: i32,
    pub bap4_pos: i32,
    pub bap1_next: [i32; 3],
    pub bap2_next: [i32; 3],
    pub bap4_next: [i32; 2],
}

struct QuantTables {
    bap1: Vec<[i32; 3]>,
    bap2: Vec<[i32; 3]>,
    bap3: Vec<i32>,
    bap4: Vec<[i32; 2]>,
    bap5: Vec<i32>,
    x512: Vec<Complex>,
    x256: Vec<Complex>,
}

/// GenerateQuantization
fn generate_quantization(levels: i32) -> Vec<i32> {
    let mut result = vec![0; levels as usize + 1];
    let mut numerator = -1 - levels;
    for i in 0..levels as usize {
        numerator += 2;
        result[i] = INT24_MAX * numerator / levels;
    }
    result
}

/// GenerateGroupedQuantization
fn generate_grouped<const G: usize>(levels: i32, group_bits: i32) -> Vec<[i32; G]> {
    let source = generate_quantization(levels);
    (0..1 << group_bits)
        .map(|i: i32| {
            let mut r = [0i32; G];
            let mut group_code = i;
            for j in (0..G).rev() {
                r[j] = source[(group_code % levels) as usize];
                group_code /= levels;
            }
            r
        })
        .collect()
}

/// CreateCoefficients
fn create_coefficients(imdct_size: i32) -> Vec<Complex> {
    let mul = 2.0 * std::f32::consts::PI / (imdct_size << 5) as f32;
    (0..imdct_size)
        .map(|i| {
            let phi = mul * (8 * i + 1) as f32;
            Complex::new(-phi.cos(), -phi.sin())
        })
        .collect()
}

fn tables() -> &'static QuantTables {
    static T: OnceLock<QuantTables> = OnceLock::new();
    T.get_or_init(|| QuantTables {
        bap1: generate_grouped::<3>(3, 5),
        bap2: generate_grouped::<3>(5, 7),
        bap3: generate_quantization(7),
        bap4: generate_grouped::<2>(11, 7),
        bap5: generate_quantization(15),
        x512: create_coefficients(128),
        x256: create_coefficients(64),
    })
}

/// Contains and decodes exponent and mantissa data for a single channel.
pub struct Allocation {
    /// absexp
    pub absolute_exponent: i32,
    pub grouped_exponents: Vec<i32>,
    pub exponents: Vec<i32>,
    pub psd: Vec<i32>,
    pub integrated_psd: Vec<i32>,
    pub excite: Vec<i32>,
    pub mask: Vec<i32>,
    pub bap: Vec<u8>,

    intermediate: Vec<Complex>,
    intermediate1: Vec<Complex>,
    intermediate2: Vec<Complex>,
    output: Vec<f32>,
    delay: Vec<f32>,
    coeff_split1: Vec<f32>,
    coeff_split2: Vec<f32>,
}

impl Allocation {
    pub fn new(max_length: usize) -> Self {
        Self {
            absolute_exponent: 0,
            grouped_exponents: vec![0; max_length],
            exponents: vec![0; max_length],
            psd: vec![0; max_length],
            integrated_psd: vec![0; max_length],
            excite: vec![0; max_length],
            mask: vec![0; max_length],
            bap: vec![0; max_length],
            intermediate: vec![Complex::default(); IMDCT_SIZE / 4],
            output: vec![0.0; IMDCT_SIZE],
            delay: vec![0.0; IMDCT_SIZE / 2],
            coeff_split1: vec![0.0; IMDCT_SIZE / 4],
            coeff_split2: vec![0.0; IMDCT_SIZE / 4],
            intermediate1: vec![Complex::default(); IMDCT_SIZE / 8],
            intermediate2: vec![Complex::default(); IMDCT_SIZE / 8],
        }
    }

    // ---------------- AllocationParsing.cs ----------------

    /// Read grouped full-range channel exponent data from the bitstream and decode it.
    pub fn read_channel_exponents(&mut self, e: &mut BitExtractor, expstr: u8, nchgrps: i32) {
        self.read_exponents(e, nchgrps);
        e.skip(2); // gainrng, unused by Cavern
        self.ungroup_exponents(nchgrps, expstr, 0, 1);
    }

    /// Read grouped coupling channel exponent data from the bitstream and decode it.
    pub fn read_coupling_exponents(&mut self, e: &mut BitExtractor, expstr: u8, start_mantissa: i32, ncplgrps: i32) {
        self.read_exponents(e, ncplgrps);
        self.absolute_exponent <<= 1;
        self.ungroup_exponents(ncplgrps, expstr, start_mantissa, start_mantissa);
    }

    /// Read grouped LFE exponent data from the bitstream and decode it.
    pub fn read_lfe_exponents(&mut self, e: &mut BitExtractor) {
        self.absolute_exponent = e.read(4);
        self.grouped_exponents[0] = e.read(7);
        self.grouped_exponents[1] = e.read(7);
        self.ungroup_exponents(NLFEGRPS, EXP_D15, LFESTRTMANT, LFESTRTMANT + 1);
    }

    #[inline]
    fn read_exponents(&mut self, e: &mut BitExtractor, ngrps: i32) {
        self.absolute_exponent = e.read(4);
        for group in 0..ngrps.max(0) as usize {
            self.grouped_exponents[group] = e.read(7);
        }
    }

    fn ungroup_exponents(&mut self, ngrps: i32, expstr: u8, start_mantissa: i32, exponent_offset: i32) {
        let grpsize = if expstr != EXP_D45 { expstr as usize } else { 4 };
        let mut absexp = self.absolute_exponent;
        let mut end_mantissa = exponent_offset as usize;
        // Cavern quirk: always writes the absolute exponent to index 0, even for the coupling channel
        self.exponents[0] = absexp;
        for grp in 0..ngrps.max(0) as usize {
            let expacc = self.grouped_exponents[grp];
            absexp += expacc / 25 - 2;
            for _ in 0..grpsize {
                self.exponents[end_mantissa] = absexp;
                end_mantissa += 1;
            }
            absexp += expacc % 25 / 5 - 2;
            for _ in 0..grpsize {
                self.exponents[end_mantissa] = absexp;
                end_mantissa += 1;
            }
            absexp += expacc % 5 - 2;
            for _ in 0..grpsize {
                self.exponents[end_mantissa] = absexp;
                end_mantissa += 1;
            }
        }

        // Exponent mapping into PSD
        for bin in start_mantissa as usize..end_mantissa {
            self.psd[bin] = 3072 - self.exponents[bin].wrapping_shl(7);
        }

        // PSD integration
        let end_mantissa = end_mantissa as i32;
        let mut i = start_mantissa as usize;
        let mut k = MASKTAB[start_mantissa as usize] as usize;
        loop {
            let lastbin = BNDTAB[k].min(end_mantissa) as usize;
            self.integrated_psd[k] = self.psd[i];
            i += 1;
            while i < lastbin {
                self.integrated_psd[k] = log_add(self.integrated_psd[k], self.psd[i]);
                i += 1;
            }
            k += 1;
            if end_mantissa <= lastbin as i32 {
                break;
            }
        }
    }

    // ---------------- Allocation.cs ----------------

    /// Read the encoded mantissa values and convert them to transform coeffs into the `target` array.
    pub fn read_transform_coeffs(&self, e: &mut BitExtractor, target: &mut [f32], start: i32, end: i32,
        host: &mut MantissaHistory) {
        let (start, end) = (start.max(0) as usize, end.max(0) as usize);
        let mut mantissa_bits: i64 = 0;
        let mut bap_reads: i64 = 0;
        for bin in start..end {
            let b = self.bap[bin];
            if b != 0 && b < 5 {
                bap_reads += 1i64 << ((b as i64 - 1) * 8);
            } else {
                mantissa_bits += BITS_TO_READ[b as usize] as i64;
            }
        }
        mantissa_bits += (host.bap1_pos as i64 + (bap_reads & 0xFF)) / 3 * BITS_TO_READ[1] as i64
            + (host.bap2_pos as i64 + ((bap_reads >> 8) & 0xFF)) / 3 * BITS_TO_READ[2] as i64
            + ((bap_reads >> 16) & 0xFF) * BITS_TO_READ[3] as i64
            + (host.bap4_pos as i64 + (bap_reads >> 24)) / 2 * BITS_TO_READ[4] as i64;
        // Cavern copies `mantissa_bits` bits into a side buffer and decodes from that; reading the same bits
        // in place and then jumping to the end of the mantissa block is equivalent.
        let begin = e.position;
        self.decode_transform_coeffs(e, target, start, end, host);
        e.position = begin + mantissa_bits as i32;
        let n = target.len();
        for v in &mut target[..start.min(n)] {
            *v = 0.0;
        }
        if end < target.len() {
            for v in &mut target[end..] {
                *v = 0.0;
            }
        }
    }

    fn decode_transform_coeffs(&self, e: &mut BitExtractor, target: &mut [f32], start: usize, end: usize,
        host: &mut MantissaHistory) {
        let t = tables();
        let mut h = *host;
        for bin in start..end {
            let exp = self.exponents[bin] as u32;
            target[bin] = match self.bap[bin] {
                1 => {
                    h.bap1_pos += 1;
                    if h.bap1_pos == 3 {
                        h.bap1_next = t.bap1[e.read(BAP1_BITS) as usize];
                        h.bap1_pos = 0;
                    }
                    h.bap1_next[h.bap1_pos as usize].wrapping_shr(exp) as f32 * FROM_INT24
                }
                2 => {
                    h.bap2_pos += 1;
                    if h.bap2_pos == 3 {
                        h.bap2_next = t.bap2[e.read(BAP2_BITS) as usize];
                        h.bap2_pos = 0;
                    }
                    h.bap2_next[h.bap2_pos as usize].wrapping_shr(exp) as f32 * FROM_INT24
                }
                3 => t.bap3[e.read(BAP3_BITS) as usize].wrapping_shr(exp) as f32 * FROM_INT24,
                4 => {
                    h.bap4_pos += 1;
                    if h.bap4_pos == 2 {
                        h.bap4_next = t.bap4[e.read(BAP4_BITS) as usize];
                        h.bap4_pos = 0;
                    }
                    h.bap4_next[h.bap4_pos as usize].wrapping_shr(exp) as f32 * FROM_INT24
                }
                5 => t.bap5[e.read(BAP5_BITS) as usize].wrapping_shr(exp) as f32 * FROM_INT24,
                b => {
                    // Asymmetric quantization (C# masks shift counts, so bap 0 gives 0 << 0)
                    let bits = BITS_TO_READ[b as usize];
                    e.read(bits).wrapping_shl((32 - bits) as u32).wrapping_shr(exp) as f32 * FROM_INT32
                }
            };
        }
        *host = h;
    }

    /// Performs a 512-sample inverse modified discrete cosine transform in-place.
    pub fn imdct512(&mut self, coeffs: &mut [f32], cache: &mut FftCache) {
        let t = tables();
        let x512 = &t.x512;
        for i in 0..self.intermediate.len() {
            self.intermediate[i] = Complex::new(coeffs[IMDCT_SIZE / 2 - 1 - 2 * i], coeffs[2 * i]).mul(x512[i]);
        }
        in_place_ifft_unscaled(&mut self.intermediate, cache);
        convolve(&mut self.intermediate, x512);

        const N8: usize = IMDCT_SIZE / 8;
        const N4: usize = IMDCT_SIZE / 4;
        const N2: usize = IMDCT_SIZE / 2;
        let w = &WINDOW;
        let it = &self.intermediate;
        let out = &mut self.output;
        for i in 0..N8 {
            out[2 * i] = -it[N8 + i].im * w[2 * i];
            out[2 * i + 1] = it[N8 - 1 - i].re * w[2 * i + 1];
            out[N4 + 2 * i] = -it[i].re * w[N4 + 2 * i];
            out[N4 + 1 + 2 * i] = it[N4 - 1 - i].im * w[N4 + 1 + 2 * i];
            out[N2 + 2 * i] = -it[N8 + i].re * w[N2 - 1 - 2 * i];
            out[N2 + 1 + 2 * i] = it[N8 - 1 - i].im * w[N2 - 2 - 2 * i];
            out[3 * N4 + 2 * i] = it[i].im * w[N4 - 1 - 2 * i];
            out[3 * N4 + 1 + 2 * i] = -it[N4 - 1 - i].re * w[N4 - 2 - 2 * i];
        }
        self.overlap_add(coeffs);
    }

    /// Performs a 256-sample inverse modified discrete cosine transform in-place.
    pub fn imdct256(&mut self, coeffs: &mut [f32], cache: &mut FftCache) {
        let t = tables();
        let x256 = &t.x256;
        for i in 0..self.coeff_split1.len() {
            self.coeff_split1[i] = coeffs[2 * i];
            self.coeff_split2[i] = coeffs[2 * i + 1];
        }
        for i in 0..self.intermediate1.len() {
            self.intermediate1[i] =
                Complex::new(self.coeff_split1[IMDCT_SIZE / 4 - 1 - 2 * i], self.coeff_split1[2 * i]).mul(x256[i]);
        }
        for i in 0..self.intermediate2.len() {
            self.intermediate2[i] =
                Complex::new(self.coeff_split2[IMDCT_SIZE / 4 - 1 - 2 * i], self.coeff_split2[2 * i]).mul(x256[i]);
        }
        in_place_ifft_unscaled(&mut self.intermediate1, cache);
        in_place_ifft_unscaled(&mut self.intermediate2, cache);
        convolve(&mut self.intermediate1, x256);
        convolve(&mut self.intermediate2, x256);

        const N8: usize = IMDCT_SIZE / 8;
        const N4: usize = IMDCT_SIZE / 4;
        const N2: usize = IMDCT_SIZE / 2;
        let w = &WINDOW;
        let i1 = &self.intermediate1;
        let i2 = &self.intermediate2;
        let out = &mut self.output;
        // Cavern indexes the 64-element arrays with n8 = 64 here (N8 - 1 - i stays in range)
        for i in 0..N8 {
            out[2 * i] = -i1[i].im * w[2 * i];
            out[2 * i + 1] = i1[N8 - 1 - i].re * w[2 * i + 1];
            out[N4 + 2 * i] = -i1[i].re * w[N4 + 2 * i];
            out[N4 + 1 + 2 * i] = i1[N8 - 1 - i].im * w[N4 + 1 + 2 * i];
            out[N2 + 2 * i] = -i2[i].re * w[N2 - 1 - 2 * i];
            out[N2 + 1 + 2 * i] = i2[N8 - 1 - i].im * w[N2 - 2 - 2 * i];
            out[3 * N4 + 2 * i] = i2[i].im * w[N4 - 1 - 2 * i];
            out[3 * N4 + 1 + 2 * i] = -i2[N8 - 1 - i].re * w[N4 - 2 - 2 * i];
        }
        self.overlap_add(coeffs);
    }

    #[inline]
    fn overlap_add(&mut self, coeffs: &mut [f32]) {
        let n = self.delay.len();
        for i in 0..n {
            coeffs[i] = 2.0 * (self.output[i] + self.delay[i]);
        }
        self.delay.copy_from_slice(&self.output[n..2 * n]);
    }
}

/// LogAdd (BitAllocation.cs)
#[inline]
pub fn log_add(a: i32, b: i32) -> i32 {
    let c = a - b;
    let address = ((c.abs() >> 1).min(255)) as usize;
    if c >= 0 {
        a + LATAB[address]
    } else {
        b + LATAB[address]
    }
}

#[allow(dead_code)]
const _: () = assert!(MAX_ALLOCATION_SIZE == 256);
