//! Shared 7.1.4 renderer: output layout and the object panner used for both E-AC-3 and TrueHD.
//! Output order (same as the "Atmos Headphones" PipeWire sink): FL FR FC LFE RL RR SL SR TFL TFR TRL TRR.
use std::f32::consts::FRAC_PI_2;

pub const OUT: usize = 12;
pub const MAX_CH: usize = 16;
// Output indices
pub const FL: usize = 0; pub const FR: usize = 1; pub const FC: usize = 2; pub const LFE: usize = 3;
pub const RL: usize = 4; pub const RR: usize = 5; pub const SL: usize = 6; pub const SR: usize = 7;
pub const TFL: usize = 8; pub const TFR: usize = 9; pub const TRL: usize = 10; pub const TRR: usize = 11;
pub const H: f32 = std::f32::consts::FRAC_1_SQRT_2;
/// Gains are re-evaluated every TICK samples and interpolated in between.
pub const TICK: u64 = 8;
/// How long to hold audio while waiting for the first object metadata.
pub const MAX_HOLD_SAMPLES: usize = 48000 * 2;

/// Pans a DAMF-coordinate position (x -1 left..1 right, y -1 back..1 front, z 0 ear..1 top)
/// onto 7.1.4 with power-preserving pairwise panning per axis (cube / dual-balance panner).
pub fn pan(x: f32, y: f32, z: f32, elevation: bool) -> [f32; OUT] {
    let mut g = [0f32; OUT];
    let x = x.clamp(-1.0, 1.0);
    let y = y.clamp(-1.0, 1.0);
    let z = if elevation { z.clamp(0.0, 1.0) } else { 0.0 };
    let (ear, top) = ((z * FRAC_PI_2).cos(), (z * FRAC_PI_2).sin());
    let lr = |u: f32| ((u * FRAC_PI_2).cos(), (u * FRAC_PI_2).sin()); // u 0 = left .. 1 = right
    let u = (x + 1.0) * 0.5;

    // Ear level: front row (FL FC FR), side row (SL SR), rear row (RL RR)
    let (row_a, row_b, t) = if y >= 0.0 { (0, 1, y) } else { (2, 1, -y) };
    let (w_mid, w_edge) = ((t * FRAC_PI_2).cos(), (t * FRAC_PI_2).sin());
    for (row, w) in [(row_a, w_edge), (row_b, w_mid)] {
        let w = w * ear;
        if w == 0.0 { continue; }
        match row {
            0 => {
                if x < 0.0 {
                    let (a, b) = lr(x + 1.0);
                    g[FL] += w * a; g[FC] += w * b;
                } else {
                    let (a, b) = lr(x);
                    g[FC] += w * a; g[FR] += w * b;
                }
            }
            1 => { let (a, b) = lr(u); g[SL] += w * a; g[SR] += w * b; }
            _ => { let (a, b) = lr(u); g[RL] += w * a; g[RR] += w * b; }
        }
    }
    // Top: front pair / rear pair
    if top > 0.0 {
        let t = (y + 1.0) * 0.5; // 0 back .. 1 front
        let (wb, wf) = ((t * FRAC_PI_2).cos(), (t * FRAC_PI_2).sin());
        let (a, b) = lr(u);
        g[TFL] += top * wf * a; g[TFR] += top * wf * b;
        g[TRL] += top * wb * a; g[TRR] += top * wb * b;
    }
    g
}


/// Gains for a Cavern `ReferenceChannel` bed channel, or None if it has no fixed 7.1.4 speaker.
pub fn reference_channel_gains(id: i32) -> Option<[f32; OUT]> {
    let mut g = [0f32; OUT];
    match id {
        0 => g[FL] = 1.0, 1 => g[FR] = 1.0, 2 => g[FC] = 1.0, 3 => g[LFE] = 1.0,
        4 => g[RL] = 1.0, 5 => g[RR] = 1.0, 6 => g[SL] = 1.0, 7 => g[SR] = 1.0,
        8 => { g[FL] = H; g[FC] = H; } 9 => { g[FR] = H; g[FC] = H; }        // front left/right centre
        15 => g[TFL] = 1.0, 16 => g[TFR] = 1.0,
        17 => { g[TFL] = H; g[TRL] = H; } 18 => { g[TFR] = H; g[TRR] = H; }  // top side
        21 => { g[TFL] = H; g[TFR] = H; }                                    // top front centre
        23 => { g[RL] = H; g[RR] = H; }                                      // rear centre
        24 => { g[FL] = H; g[SL] = H; } 25 => { g[FR] = H; g[SR] = H; }      // wide
        26 => g[TRL] = 1.0, 27 => g[TRR] = 1.0,
        28 => { g[TRL] = H; g[TRR] = H; }                                    // top rear centre
        _ => return None,
    }
    Some(g)
}
