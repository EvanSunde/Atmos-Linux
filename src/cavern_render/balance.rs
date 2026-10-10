//! Balance-based object panning. Port of Cavern/Rendering/BalanceBasedRenderer.cs and
//! `SourceRenderer.MixToLFE` (Cavern/Rendering/SourceRenderer.cs). This is the renderer Cavern selects for
//! symmetric layouts (`Listener.ReplaceChannels`), which the atmos-decode 7.1.4 layout is.
//!
//! Instead of mixing into the interleaved per-source buffer directly, the panner returns the ordered list of
//! `WaveformUtils.Mix(samples, rendered, channel, channels, gain)` calls; the caller applies them in order, so
//! accumulation into the same channel happens in Cavern's order.

use super::channel::{Channel, V3};

/// Environment size inverse (`Listener.EnvironmentSizeInverse`, default environment (10, 7, 10)).
pub const ENVIRONMENT_SIZE_INVERSE: V3 = V3::new(1.0 / 10.0, 1.0 / 7.0, 1.0 / 10.0);

/// Inverse lerp, but returns 0 when the values are equal.
#[inline]
fn ratio(a: f32, b: f32, x: f32) -> f32 {
    if a == b {
        return 0.0;
    }
    (x - a) / (b - a)
}

/// Check and assign a channel if it's the closest left/right from a given position.
#[inline]
fn assign_lr(channels: &[Channel], channel: i32, left: &mut i32, right: &mut i32, pos_x: f32, channel_x: f32) {
    if channel_x == pos_x {
        // Exact match
        *left = channel;
        *right = channel;
    } else if channel_x < pos_x {
        // Left
        if *left == -1 || channels[*left as usize].cubical_pos.x < channel_x {
            *left = channel;
        }
    } else if *right == -1 || channels[*right as usize].cubical_pos.x > channel_x {
        // Right
        *right = channel;
    }
}

/// For a given horizontal layer, if it's over a side of the room, fill blank speakers.
#[inline]
fn fix_incomplete_layer(front_left: &mut i32, front_right: &mut i32, rear_left: &mut i32, rear_right: &mut i32) {
    if *front_left != -1 || *front_right != -1 {
        if *front_left == -1 {
            *front_left = *front_right;
        }
        if *front_right == -1 {
            *front_right = *front_left;
        }
        if *rear_left == -1 && *rear_right == -1 {
            *rear_left = *front_left;
            *rear_right = *front_right;
        }
    }
    if *rear_left != -1 || *rear_right != -1 {
        if *rear_left == -1 {
            *rear_left = *rear_right;
        }
        if *rear_right == -1 {
            *rear_right = *rear_left;
        }
        if *front_left == -1 && *front_right == -1 {
            *front_left = *rear_left;
            *front_right = *rear_right;
        }
    }
}

/// `BalanceBasedRenderer.Render` for one source: pushes `(channel, gain)` mix operations to `ops`, in order.
/// `lfe_separation` is `Listener.LFESeparation`.
pub fn render(channels: &[Channel], lfe_separation: bool, source_lfe: bool, source_size: f32, direction: V3,
    gain: f32, ops: &mut Vec<(usize, f32)>) {
    if source_lfe || !lfe_separation {
        // SourceRenderer.MixToLFE
        for (channel, c) in channels.iter().enumerate() {
            if c.lfe {
                ops.push((channel, gain));
            }
        }
        if source_lfe {
            return;
        }
    }

    // Find a bounding box
    let (mut bottom_front_left, mut bottom_front_right, mut bottom_rear_left, mut bottom_rear_right) = (-1, -1, -1, -1);
    let (mut top_front_left, mut top_front_right, mut top_rear_left, mut top_rear_right) = (-1, -1, -1, -1);
    // Closest layers on Y and Z axes (Cavern's arbitrary initial values)
    let mut closest_top = 66f32;
    let mut closest_bottom = -69f32;
    let mut closest_tf = 82f32;
    let mut closest_tr = -84f32;
    let mut closest_bf = 65f32;
    let mut closest_br = -2665f32;

    // Find closest horizontal layers (no headphone virtualizer)
    let direction = direction.mul(ENVIRONMENT_SIZE_INVERSE);

    for (channel, c) in channels.iter().enumerate() {
        if c.lfe {
            continue;
        }
        // World-space scaling and its inverse can move an exact speaker placement by a float ULP,
        // so snap if panned very near of a channel
        if source_size == 0.0 && direction.distance_squared(c.cubical_pos) <= 1e-12 {
            ops.push((channel, gain.sqrt()));
            return;
        }
        let channel_y = c.cubical_pos.y;
        let channel_z = c.cubical_pos.z;
        if channel_y <= direction.y {
            if closest_bottom < channel_y {
                closest_bottom = channel_y;
                closest_bf = f32::INFINITY;
                closest_br = f32::NEG_INFINITY;
            }
            if closest_bottom == channel_y {
                if channel_z <= direction.z {
                    if closest_br < channel_z {
                        closest_br = channel_z;
                    }
                } else if closest_bf > channel_z {
                    closest_bf = channel_z;
                }
            }
        } else {
            if closest_top > channel_y {
                closest_top = channel_y;
                closest_tf = f32::INFINITY;
                closest_tr = f32::NEG_INFINITY;
            }
            if closest_top == channel_y {
                if channel_z <= direction.z {
                    if closest_tr < channel_z {
                        closest_tr = channel_z;
                    }
                } else if closest_tf > channel_z {
                    closest_tf = channel_z;
                }
            }
        }
    }

    for (channel, c) in channels.iter().enumerate() {
        if c.lfe {
            continue;
        }
        let p = c.cubical_pos;
        let ch = channel as i32;
        if p.y == closest_bottom {
            // Bottom layer
            if p.z == closest_bf {
                assign_lr(channels, ch, &mut bottom_front_left, &mut bottom_front_right, direction.x, p.x);
            }
            if p.z == closest_br {
                assign_lr(channels, ch, &mut bottom_rear_left, &mut bottom_rear_right, direction.x, p.x);
            }
        }
        if p.y == closest_top {
            // Top layer
            if p.z == closest_tf {
                assign_lr(channels, ch, &mut top_front_left, &mut top_front_right, direction.x, p.x);
            }
            if p.z == closest_tr {
                assign_lr(channels, ch, &mut top_rear_left, &mut top_rear_right, direction.x, p.x);
            }
        }
    }

    // Fix incomplete top layer
    fix_incomplete_layer(&mut top_front_left, &mut top_front_right, &mut top_rear_left, &mut top_rear_right);

    if bottom_front_left == -1 && bottom_front_right == -1 && bottom_rear_left == -1 && bottom_rear_right == -1 {
        // When the bottom layer is completely empty (= the source is below all channels), copy the top layer
        bottom_front_left = top_front_left;
        bottom_front_right = top_front_right;
        bottom_rear_left = top_rear_left;
        bottom_rear_right = top_rear_right;
    } else {
        // Fix incomplete bottom layer
        fix_incomplete_layer(&mut bottom_front_left, &mut bottom_front_right, &mut bottom_rear_left, &mut bottom_rear_right);
    }

    // When the top layer is completely empty (= the source is above all channels), copy the bottom layer
    if top_front_left == -1 || top_front_right == -1 || top_rear_left == -1 || top_rear_right == -1 {
        top_front_left = bottom_front_left;
        top_front_right = bottom_front_right;
        top_rear_left = bottom_rear_left;
        top_rear_right = bottom_rear_right;
    }
    if bottom_front_left < 0 || bottom_front_right < 0 || bottom_rear_left < 0 || bottom_rear_right < 0 {
        return; // Cavern would index channels[-1] and throw; only possible with an all-LFE layout.
    }
    let pos = |i: i32| channels[i as usize].cubical_pos;

    // Spatial mix gain precalculation: (bottom; top)
    let (mut layer_x, mut layer_y) = (1f32, 0f32);
    if top_front_left != bottom_front_left {
        // Height ratio calculation
        let bottom_y = pos(bottom_front_left).y;
        layer_y = (direction.y - bottom_y) / (pos(top_front_left).y - bottom_y);
        layer_x = 1.0 - layer_y;
    }

    // Length ratios (bottom; top)
    let mut front_x = ratio(pos(bottom_rear_left).z, pos(bottom_front_left).z, direction.z);
    let mut front_y = ratio(pos(top_rear_left).z, pos(top_front_left).z, direction.z);

    // Size extension
    let mut inner_volume_3d = gain;
    if source_size != 0.0 {
        inner_volume_3d *= 1.0 - source_size;
        let extra_channel_volume = gain * (source_size / channels.len() as f32).sqrt();
        for (channel, c) in channels.iter().enumerate() {
            if !c.lfe {
                ops.push((channel, extra_channel_volume));
            }
        }
    }

    // Spatial mix output
    let mut rear_x = 1.0 - front_x;
    let mut rear_y = 1.0 - front_y;
    layer_x *= inner_volume_3d;
    layer_y *= inner_volume_3d;
    front_x *= layer_x;
    front_y *= layer_y;
    rear_x *= layer_x;
    rear_y *= layer_y;
    let pair = |left: i32, right: i32, vol: f32, ops: &mut Vec<(usize, f32)>| {
        let r = ratio(pos(left).x, pos(right).x, direction.x);
        ops.push((left as usize, (vol * (1.0 - r)).sqrt()));
        ops.push((right as usize, (vol * r).sqrt()));
    };
    if front_x != 0.0 {
        pair(bottom_front_left, bottom_front_right, front_x, ops);
    }
    if rear_x != 0.0 {
        pair(bottom_rear_left, bottom_rear_right, rear_x, ops);
    }
    if front_y != 0.0 {
        pair(top_front_left, top_front_right, front_y, ops);
    }
    if rear_y != 0.0 {
        pair(top_rear_left, top_rear_right, rear_y, ops);
    }
}
