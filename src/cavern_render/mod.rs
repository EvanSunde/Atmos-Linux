//! Cavern's object-to-speaker renderer, as configured by the atmos-decode tool (7.1.4, `UpdateRate = 256`,
//! `AudioQuality = Perfect`, `LFESeparation = true`, `DirectLFE = true`, `Volume = 1`, default normalizer).
//!
//! Ports the parts of Cavern/Listener.cs + Listener.Interface.cs (`Render` → `Frame`: source collection,
//! mixing, per-channel volume, normalizer), Cavern/Source.cs (`Precalculate`, `Precollect`, `Collect` for a
//! mono 3D source with rolloff disabled, no Doppler, no resampling), Cavern/SpecialSources/StreamMaster.cs +
//! StreamMasterSource.cs (sample delivery), Cavern/Filters/Normalizer.cs (`new Normalizer(true)`, i.e.
//! limiter-only) and the balance-based panner (see `balance.rs`).
//!
//! Timing (StreamMaster): during `Listener.Render()`, the first source's `Precollect` calls
//! `Renderer.GetNextObjectSamples(256)`, i.e. four 64-sample object timeslots, before any source is mixed, so
//! every source is mixed with the position/volume after the 4th timeslot. Source distance (used only for the
//! `distance < Range` check) comes from `Precalculate`, which runs before the fetch, i.e. from the previous
//! update's position. [`CavernRenderer::push_block`] batches [`ObjectBlock`]s accordingly.
//!
//! Not ported (inactive in this configuration): DirectionalRenderer (asymmetric layouts), headphone
//! virtualizer, LFE lowpass (DirectLFE), Doppler/pitch, resampling (source and listener rates are equal),
//! 1D/stereo mixing (SpatialBlend = 1), rolloff (sources use `Rolloffs.Disabled`), screen locking, filters.

mod balance;
pub mod channel;

pub use channel::{Channel, REF714};
use channel::{layout_714, V3};

use crate::objects::{ObjectBlock, ObjectKind};

/// Samples per object timeslot delivered by `ObjectDecoder`.
const TIMESLOT: usize = 64;

pub struct CavernRenderer {
    /// `Listener.Channels`.
    channels: Vec<Channel>,
    sample_rate: u32,
    /// `Listener.UpdateRate`.
    update_rate: usize,
    /// `Listener.Volume`.
    volume: f32,
    /// `Listener.LFEVolume`.
    lfe_volume: f32,
    /// `Listener.LFESeparation`.
    lfe_separation: bool,
    /// `Listener.Range`.
    range: f32,
    /// `Listener.Normalizer` (adaption speed, 0 = disabled).
    normalizer: f32,
    /// `Normalizer.limiterOnly` (Listener creates `new Normalizer(true)`).
    limiter_only: bool,
    /// `Normalizer.lastGain`.
    normalizer_gain: f32,
    /// Per-source `distance` from `Precalculate` (position before this update's fetch).
    last_positions: Vec<[f32; 3]>,
    /// `renderBuffer`: interleaved output.
    render_buffer: Vec<f32>,
    /// Scratch: per-source single channel accumulation.
    channel_mix: Vec<f32>,
    /// Scratch: panner mix operations.
    ops: Vec<(usize, f32)>,
    /// `push_block` accumulation: per object samples of the current update.
    pending: Vec<Vec<f32>>,
    pending_blocks: usize,
}

impl CavernRenderer {
    /// The atmos-decode configuration on its 7.1.4 layout (output order [`REF714`]:
    /// FL FR FC LFE RL RR SL SR TFL TFR TRL TRR).
    pub fn new_714(sample_rate: u32) -> Self {
        // Volume = gain * (objects && fewer than 4 overhead channels ? .707 : 1) = 1 for 7.1.4 at 0 dB gain.
        Self::new(layout_714(), sample_rate, 256, 1.0)
    }

    /// A listener on a symmetric `layout` (only the balance-based renderer is ported).
    pub fn new(channels: Vec<Channel>, sample_rate: u32, update_rate: usize, volume: f32) -> Self {
        assert!(update_rate % TIMESLOT == 0, "update rate must be a multiple of 64");
        let n = channels.len();
        Self {
            channels,
            sample_rate,
            update_rate,
            volume,
            lfe_volume: 1.0,
            lfe_separation: true,
            range: 100.0,
            normalizer: 1.0,
            limiter_only: true,
            normalizer_gain: 1.0,
            last_positions: Vec::new(),
            render_buffer: vec![0.0; n * update_rate],
            channel_mix: vec![0.0; update_rate],
            ops: Vec::new(),
            pending: Vec::new(),
            pending_blocks: 0,
        }
    }

    /// Output channel count.
    pub fn channel_count(&self) -> usize { self.channels.len() }

    /// Samples per channel produced by one [`Self::render`] call.
    pub fn update_rate(&self) -> usize { self.update_rate }

    /// Feed one 64-sample object timeslot; every `update_rate / 64` blocks (4 for 256) an update is rendered and
    /// its interleaved output returned. `kinds[i]` is `ObjectDecoder::object_kind(i)` (LFE objects go to the LFE
    /// channel only). Position, volume and size are taken from the last block of the update, as Cavern mixes
    /// every source after all of the update's timeslots were decoded.
    pub fn push_block(&mut self, block: &ObjectBlock, kinds: &[ObjectKind]) -> Option<&[f32]> {
        let n = block.samples.len();
        if self.pending.len() != n {
            self.pending = vec![vec![0.0; self.update_rate]; n];
            self.pending_blocks = 0;
        }
        let offset = self.pending_blocks * TIMESLOT;
        for (dst, src) in self.pending.iter_mut().zip(block.samples.iter()) {
            dst[offset..offset + TIMESLOT].copy_from_slice(src);
        }
        self.pending_blocks += 1;
        if self.pending_blocks * TIMESLOT < self.update_rate {
            return None;
        }
        self.pending_blocks = 0;
        let pending = std::mem::take(&mut self.pending);
        let lfe: Vec<bool> = (0..n).map(|i| kinds.get(i) == Some(&ObjectKind::Lfe)).collect();
        let refs: Vec<&[f32]> = pending.iter().map(|v| &v[..]).collect();
        self.render_into(&refs, &block.positions, &block.volumes, &block.sizes, &lfe);
        self.pending = pending;
        Some(&self.render_buffer)
    }

    /// Render one update (`Listener.Render()` with one frame). Per source: `samples[i]` (`update_rate` samples),
    /// `positions[i]` / `volumes[i]` / `sizes[i]` (Cavern `Source.Position` / `Volume` / `Size` as they are after
    /// the update's samples were fetched) and `lfe[i]` (`Source.LFE`). Returns interleaved output
    /// (`update_rate` × channels).
    pub fn render(&mut self, samples: &[&[f32]], positions: &[[f32; 3]], volumes: &[f32], sizes: &[f32], lfe: &[bool])
        -> &[f32] {
        self.render_into(samples, positions, volumes, sizes, lfe);
        &self.render_buffer
    }

    fn render_into(&mut self, samples: &[&[f32]], positions: &[[f32; 3]], volumes: &[f32], sizes: &[f32], lfe: &[bool]) {
        let n_src = samples.len();
        let n_ch = self.channels.len();
        let update_rate = self.update_rate;
        // Precalculate: distance from the listener using the position before this update's fetch
        // (sources start at the origin).
        if self.last_positions.len() != n_src {
            self.last_positions = vec![[0.0; 3]; n_src];
        }

        // Frame(): collect and mix
        self.render_buffer.iter_mut().for_each(|v| *v = 0.0);
        for src in 0..n_src {
            let p = self.last_positions[src];
            let distance = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
            // Collect(): 3D mix if in range (Mute = false, SpatialBlend = 1, mono clip).
            if !(distance < self.range) {
                continue;
            }
            let position = V3::new(positions[src][0], positions[src][1], positions[src][2]);
            // (position - listener.Position).RotateInverse(listener.Rotation) with a listener at the origin.
            let direction = position.sub(V3::default()).rotate_inverse(V3::default());
            let rolloff = 1.0; // Rolloffs.Disabled
            let spatial_blend = 1.0;
            let gain = volumes[src] * rolloff * spatial_blend;
            self.ops.clear();
            balance::render(&self.channels, self.lfe_separation, lfe[src], sizes[src], direction, gain, &mut self.ops);

            // Apply the WaveformUtils.Mix calls to this source's `rendered` buffer (zeroed per source) in order,
            // then WaveformUtils.Mix(rendered, renderBuffer). Channels are accumulated one at a time, which is
            // the same arithmetic per element.
            let s = &samples[src][..update_rate];
            let mut done = 0u64;
            for k in 0..self.ops.len() {
                let channel = self.ops[k].0;
                if done & (1 << channel) != 0 {
                    continue;
                }
                done |= 1 << channel;
                let mix = &mut self.channel_mix;
                mix.iter_mut().for_each(|v| *v = 0.0);
                for &(c, g) in &self.ops[k..] {
                    if c == channel {
                        for (m, &x) in mix.iter_mut().zip(s) {
                            *m += x * g;
                        }
                    }
                }
                for (i, &m) in mix.iter().enumerate() {
                    self.render_buffer[i * n_ch + channel] += m;
                }
            }
        }
        for (dst, src) in self.last_positions.iter_mut().zip(positions) {
            *dst = *src;
        }

        // Volume and subwoofers' lowpass (DirectLFE: no lowpass)
        for (channel, c) in self.channels.iter().enumerate() {
            let g = if c.lfe { self.lfe_volume * self.volume } else { self.volume };
            for i in 0..update_rate {
                self.render_buffer[i * n_ch + channel] *= g;
            }
        }

        // Normalizer (limiter only)
        if self.normalizer != 0.0 {
            let decay_factor = self.normalizer * update_rate as f32 / self.sample_rate as f32;
            let buf = &mut self.render_buffer;
            let mut max = buf[0].abs();
            for &v in &buf[1..] {
                let a = v.abs();
                if max < a {
                    max = a;
                }
            }
            if max * self.normalizer_gain > 1.0 {
                // Attack
                self.normalizer_gain = 0.9 / max;
            }
            let g = self.normalizer_gain;
            buf.iter_mut().for_each(|v| *v *= g);
            self.normalizer_gain += decay_factor;
            if self.limiter_only && self.normalizer_gain > 1.0 {
                self.normalizer_gain = 1.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A source straight ahead (Cavern position (0, 0, 10) → direction (0, 0, 1), exactly the centre speaker).
    fn render_center(size: f32) -> Vec<f32> {
        let mut r = CavernRenderer::new_714(48000);
        let samples = vec![1f32; 256];
        r.render(&[&samples], &[[0.0, 0.0, 10.0]], &[1.0], &[size], &[false]).to_vec()
    }

    #[test]
    fn size_zero_snaps_to_center() {
        // BalanceBasedRenderer: Size == 0 and DistanceSquared(direction, FC.CubicalPos) <= 1e-12 → Mix with sqrt(gain).
        let out = render_center(0.0);
        for (ch, &v) in out[..12].iter().enumerate() {
            assert_eq!(v, if ch == 2 { 1.0 } else { 0.0 }, "channel {ch}");
        }
    }

    #[test]
    fn size_spreads_like_cavern() {
        // Hand-traced C# for direction (0, 0, 1), gain 1, Size 0.5: the bottom layer is all-rear (closestBR = 1:
        // FL/FR/FC), AssignLR picks FC as both left and right; the top layer resolves to TFL/TFR but layerVol = (1, 0).
        // frontVol = (0, 0) (Ratio of equal Z), innerVolume3D = 1 - .5, extraChannelVolume = sqrt(.5 / 12) to every
        // non-LFE channel first, then rearVol.X = .5: FC += sqrt(.5 * (1 - 0)), FC += sqrt(.5 * 0).
        let size = 0.5f32;
        let extra = 1.0 * (size / 12.0).sqrt();
        let inner = 1.0 * (1.0 - size);
        let fc = ((0.0 + 1.0 * extra) + 1.0 * (inner * (1.0 - 0.0f32)).sqrt()) + 1.0 * (inner * 0.0f32).sqrt();
        let out = render_center(size);
        for i in 0..256 {
            for ch in 0..12 {
                let expected = match ch {
                    2 => fc,
                    3 => 0.0, // LFE: LFESeparation and not an LFE source
                    _ => extra,
                };
                assert_eq!(out[i * 12 + ch], expected, "sample {i} channel {ch}");
            }
        }
        // And it differs from the point source.
        assert_ne!(render_center(0.0)[..12], out[..12]);
    }
}
