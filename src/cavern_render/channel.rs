//! Output speaker channels. Port of Cavern/Channel.cs (position calculation), the parts of
//! Cavern/Channels/ChannelPrototype.Consts.cs / ChannelPrototype.Layout.cs used by `ToLayout(ref714)`, and
//! Cavern/Utilities/VectorExtensions.cs (`MapToCube`, `RotateInverse`, `Deg2Rad`).

/// System.Numerics.Vector3 subset (component-wise single precision, as .NET 8 evaluates it, no FMA).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl V3 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }
    #[inline]
    pub fn sub(self, o: V3) -> V3 { V3::new(self.x - o.x, self.y - o.y, self.z - o.z) }
    #[inline]
    pub fn mul(self, o: V3) -> V3 { V3::new(self.x * o.x, self.y * o.y, self.z * o.z) }
    /// `Vector3.DistanceSquared`: Dot(a - b, a - b) = (dx*dx + dy*dy) + dz*dz.
    #[inline]
    pub fn distance_squared(self, o: V3) -> f32 {
        let d = self.sub(o);
        d.x * d.x + d.y * d.y + d.z * d.z
    }
    /// `VectorExtensions.MapToCube`: `vector / max(|x|, |y|, |z|)` (component-wise division).
    #[inline]
    pub fn map_to_cube(self) -> V3 {
        let max = net_max(self.x.abs(), net_max(self.y.abs(), self.z.abs()));
        V3::new(self.x / max, self.y / max, self.z / max)
    }
    /// `VectorExtensions.RotateInverse` (double precision trigonometry, as Cavern's `(float)Math.Cos`).
    pub fn rotate_inverse(self, angles: V3) -> V3 {
        let angles = V3::new(angles.x * -DEG2RAD, angles.y * -DEG2RAD, angles.z * -DEG2RAD);
        let (mut cos, mut sin) = ((angles.z as f64).cos() as f32, (angles.z as f64).sin() as f32);
        let mut x = cos * self.x - sin * self.y;
        let mut y = sin * self.x + cos * self.y;
        cos = (angles.y as f64).cos() as f32;
        sin = (angles.y as f64).sin() as f32;
        let mut z = cos * self.z - sin * x;
        x = cos * x + sin * self.z;
        cos = (angles.x as f64).cos() as f32;
        sin = (angles.x as f64).sin() as f32;
        let old_y = y;
        y = cos * y - sin * z;
        z = sin * old_y + cos * z;
        V3::new(x, y, z)
    }
}

/// .NET `Math.Max(float, float)` (NaN if either is NaN).
#[inline]
fn net_max(a: f32, b: f32) -> f32 {
    if a.is_nan() || b.is_nan() { f32::NAN } else if a > b { a } else { b }
}

/// `VectorExtensions.Deg2Rad`.
pub const DEG2RAD: f32 = 0.01745329251;

/// Spatially positioned audio output channel (Cavern `Channel`).
#[derive(Clone, Copy, Debug)]
pub struct Channel {
    /// Elevation in degrees (negative is up).
    pub x: f32,
    /// Azimuth in degrees, clockwise.
    pub y: f32,
    pub lfe: bool,
    pub spherical_pos: V3,
    pub cubical_pos: V3,
}

impl Channel {
    /// `new Channel(x, y, LFE)` → `SetPosition` → `Recalculate` (Home environment: not spherical).
    pub fn new(x: f32, y: f32, lfe: bool) -> Self {
        let x_rad = x * DEG2RAD;
        let y_rad = y * DEG2RAD;
        let (sin_x, cos_x, sin_y, cos_y) = (x_rad.sin(), x_rad.cos(), y_rad.sin(), y_rad.cos());
        let spherical_pos = V3::new(sin_y * cos_x, -sin_x, cos_y * cos_x);
        Self { x, y, lfe, spherical_pos, cubical_pos: spherical_pos.map_to_cube() }
    }
}

/// Cavern ReferenceChannel values of `ChannelPrototype.ref714`, the output channel order:
/// FL, FR, FC, LFE, RL, RR, SL, SR, TFL, TFR, TRL, TRR.
pub const REF714: [i32; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 15, 16, 26, 27];

/// The atmos-decode 7.1.4 layout: `ChannelPrototype.ToLayout(ref714)` with the side and rear channels moved to
/// the Dolby reference angles (SL/SR at ±90°, RL/RR at ±135°), as `CreateLayout` in atmos-decode's Program.cs.
pub fn layout_714() -> Vec<Channel> {
    vec![
        Channel::new(0.0, -30.0, false),   // FrontLeft (prototype)
        Channel::new(0.0, 30.0, false),    // FrontRight
        Channel::new(0.0, 0.0, false),     // FrontCenter
        Channel::new(0.0, 0.0, true),      // ScreenLFE
        Channel::new(0.0, -135.0, false),  // RearLeft (atmos-decode override)
        Channel::new(0.0, 135.0, false),   // RearRight (atmos-decode override)
        Channel::new(0.0, -90.0, false),   // SideLeft (atmos-decode override)
        Channel::new(0.0, 90.0, false),    // SideRight (atmos-decode override)
        Channel::new(-45.0, -70.0, false), // TopFrontLeft (prototype: y = -70, x = -45)
        Channel::new(-45.0, 70.0, false),  // TopFrontRight
        Channel::new(-45.0, -150.0, false), // TopRearLeft
        Channel::new(-45.0, 150.0, false), // TopRearRight
    ]
}
