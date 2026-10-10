//! Port of Cavern's FFT: Cavern/Utilities/Complex.cs, ComplexArray.cs (Convolve, Conjugate),
//! FFTCache.cs and Measurements.FFT.cs (ProcessFFT / ProcessIFFT / ProcessFFT8), including the exact
//! single-precision operation order (the SIMD and scalar passes of Cavern compute identical values).

use std::sync::OnceLock;

#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Complex {
    #[inline(always)]
    pub fn new(re: f32, im: f32) -> Self { Self { re, im } }

    /// Cavern `operator *(Complex, Complex)`.
    #[inline(always)]
    pub fn mul(self, rhs: Complex) -> Complex {
        Complex::new(self.re * rhs.re - self.im * rhs.im, self.re * rhs.im + self.im * rhs.re)
    }
}

#[inline(always)]
fn add(a: Complex, b: Complex) -> Complex { Complex::new(a.re + b.re, a.im + b.im) }
#[inline(always)]
fn sub(a: Complex, b: Complex) -> Complex { Complex::new(a.re - b.re, a.im - b.im) }
#[inline(always)]
fn scale(a: Complex, s: f32) -> Complex { Complex::new(a.re * s, a.im * s) }

/// Cavern `FFTCache.cos` / `FFTCache.sin` (one value per band; Cavern stores each twice for SIMD).
struct Twiddles {
    cos: Vec<Vec<f32>>,
    sin: Vec<Vec<f32>>,
}

fn twiddles() -> &'static Twiddles {
    static T: OnceLock<Twiddles> = OnceLock::new();
    T.get_or_init(|| {
        let mut cos = Vec::new();
        let mut sin = Vec::new();
        for i in 0..10 {
            let elements: i32 = 2 << i;
            let step = -std::f32::consts::PI / elements as f32;
            let mut c = Vec::with_capacity(elements as usize / 2);
            let mut s = Vec::with_capacity(elements as usize / 2);
            let mut j = 0;
            while j < elements {
                let rotation = j as f32 * step;
                c.push(rotation.cos());
                s.push(rotation.sin());
                j += 2;
            }
            cos.push(c);
            sin.push(s);
        }
        Twiddles { cos, sin }
    })
}

/// Preallocated even/odd split arrays per recursion depth (Cavern's `FFTCache.Even/Odd`).
pub struct FftCache {
    split: Vec<(Vec<Complex>, Vec<Complex>)>,
}

impl FftCache {
    pub fn new(size: usize) -> Self {
        let depths = size.trailing_zeros() as usize;
        let split = (0..depths).map(|d| (vec![Complex::default(); 1 << d], vec![Complex::default(); 1 << d])).collect();
        Self { split }
    }
}

/// Cavern `Complex[].InPlaceIFFTUnscaled(FFTCache)` without CavernAmp: conjugate, FFT, conjugate.
pub fn in_place_ifft_unscaled(samples: &mut [Complex], cache: &mut FftCache) {
    for s in samples.iter_mut() {
        s.im = -s.im;
    }
    process_fft(samples, cache);
    for s in samples.iter_mut() {
        s.im = -s.im;
    }
}

/// Cavern `ProcessFFT(Complex[], FFTCache)` selector.
fn process_fft(samples: &mut [Complex], cache: &mut FftCache) {
    let n = samples.len();
    if n > 8 {
        let depth = n.trailing_zeros() as usize - 1;
        process_fft_depth(samples, &mut cache.split[..=depth], depth);
    } else if n == 8 {
        process_fft8(samples);
    } else if n == 4 {
        process_fft4(samples);
    } else if n == 2 {
        let (e, o) = (samples[0], samples[1]);
        samples[0] = add(e, o);
        samples[1] = sub(e, o);
    }
}

fn process_fft_depth(samples: &mut [Complex], split: &mut [(Vec<Complex>, Vec<Complex>)], depth: usize) {
    let (lower, cur) = split.split_at_mut(depth);
    let (even, odd) = &mut cur[0];
    for (i, pair) in samples.chunks_exact(2).enumerate() {
        even[i] = pair[0];
        odd[i] = pair[1];
    }
    if even.len() != 8 {
        process_fft_depth(even, lower, depth - 1);
        process_fft_depth(odd, lower, depth - 1);
    } else {
        process_fft8(even);
        process_fft8(odd);
    }

    let t = twiddles();
    let cos = &t.cos[depth];
    let sin = &t.sin[depth];
    let half = samples.len() >> 1;
    let (lo, hi) = samples.split_at_mut(half);
    for i in 0..half {
        let o = odd[i];
        let c = cos[i];
        let s = sin[i];
        let new_odd = Complex::new(o.re * c - o.im * s, o.re * s + o.im * c);
        lo[i] = add(even[i], new_odd);
        hi[i] = sub(even[i], new_odd);
    }
}

#[inline(always)]
fn rot(v: Complex) -> Complex { Complex::new(v.im, -v.re) }

fn process_fft4(s: &mut [Complex]) {
    let mut even_value = s[0];
    let mut odd_value = s[2];
    let even_value1 = add(even_value, odd_value);
    let even_value2 = sub(even_value, odd_value);
    even_value = s[1];
    odd_value = s[3];
    let odd_value1 = add(even_value, odd_value);
    odd_value = sub(even_value, odd_value);
    odd_value = rot(odd_value);
    s[0] = add(even_value1, odd_value1);
    s[1] = add(even_value2, odd_value);
    s[2] = sub(even_value1, odd_value1);
    s[3] = sub(even_value2, odd_value);
}

fn process_fft8(s: &mut [Complex]) {
    let mut even_value = s[0];
    let mut odd_value = s[4];
    let mut even_value1 = add(even_value, odd_value);
    let mut even_value2 = sub(even_value, odd_value);

    even_value = s[2];
    odd_value = s[6];
    let mut odd_value1 = add(even_value, odd_value);
    odd_value = sub(even_value, odd_value);
    odd_value = rot(odd_value);

    let even1 = add(even_value1, odd_value1);
    let even2 = add(even_value2, odd_value);
    let even3 = sub(even_value1, odd_value1);
    let even4 = sub(even_value2, odd_value);

    even_value = s[1];
    odd_value = s[5];
    even_value1 = add(even_value, odd_value);
    even_value2 = sub(even_value, odd_value);

    even_value = s[3];
    odd_value = s[7];
    odd_value1 = add(even_value, odd_value);
    odd_value = sub(even_value, odd_value);
    odd_value = rot(odd_value);

    let odd1 = add(even_value1, odd_value1);
    let mut odd2 = add(even_value2, odd_value);
    even_value1 = sub(even_value1, odd_value1); // odd3 from now
    even_value2 = sub(even_value2, odd_value); // odd4 from now

    const SIN: f32 = -0.70710678118; // sin(-pi/4)
    odd2 = scale(Complex::new(-odd2.re - odd2.im, odd2.re - odd2.im), SIN);
    even_value1 = rot(even_value1);
    even_value2 = scale(Complex::new(even_value2.re - even_value2.im, even_value2.re + even_value2.im), SIN);

    s[0] = add(even1, odd1);
    s[1] = add(even2, odd2);
    s[2] = add(even3, even_value1);
    s[3] = add(even4, even_value2);
    s[4] = sub(even1, odd1);
    s[5] = sub(even2, odd2);
    s[6] = sub(even3, even_value1);
    s[7] = sub(even4, even_value2);
}

/// Cavern `ComplexArray.Convolve`: in-place complex multiplication.
#[inline]
pub fn convolve(source: &mut [Complex], other: &[Complex]) {
    for (l, r) in source.iter_mut().zip(other.iter()) {
        let old_real = l.re;
        l.re = l.re * r.re - l.im * r.im;
        l.im = old_real * r.im + l.im * r.re;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ifft_matches_dft() {
        for &n in &[8usize, 16, 64, 128] {
            let input: Vec<Complex> = (0..n).map(|i| Complex::new((i as f32 * 0.37).sin(), (i as f32 * 1.3).cos())).collect();
            let mut x = input.clone();
            let mut cache = FftCache::new(n);
            in_place_ifft_unscaled(&mut x, &mut cache);
            for k in 0..n {
                let (mut re, mut im) = (0f64, 0f64);
                for (j, v) in input.iter().enumerate() {
                    let a = 2.0 * std::f64::consts::PI * (j * k) as f64 / n as f64;
                    re += v.re as f64 * a.cos() - v.im as f64 * a.sin();
                    im += v.re as f64 * a.sin() + v.im as f64 * a.cos();
                }
                assert!((re - x[k].re as f64).abs() < 1e-3 && (im - x[k].im as f64).abs() < 1e-3, "n={n} k={k}");
            }
        }
    }
}
