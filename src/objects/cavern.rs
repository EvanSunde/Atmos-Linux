//! Minimal ports of the Cavern core helpers used by the object decoder:
//! - System.Numerics.Vector3 operations as .NET 8 implements them (component-wise, no FMA),
//! - Cavern/Utilities/VectorExtensions.cs (MapToCube),
//! - Cavern/Utilities/QMath.cs (DbToGain, float overload),
//! - Cavern/Channels/ReferenceChannel.cs (enum values),
//! - Cavern/Channels/ChannelPrototype.Consts.cs / ChannelPrototype.cs (AlternativePositions, LFE flags,
//!   home standard matrices, GetStandardMatrix),
//! - Cavern/Listener.Interface.cs (default EnvironmentSize and ScreenSize).

/// System.Numerics.Vector3 (only what's needed). All operations are component-wise single-precision.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }
    #[inline]
    pub fn add(self, o: Vec3) -> Vec3 { Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z) }
    #[inline]
    pub fn sub(self, o: Vec3) -> Vec3 { Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z) }
    #[inline]
    pub fn mul(self, o: Vec3) -> Vec3 { Vec3::new(self.x * o.x, self.y * o.y, self.z * o.z) }
    #[inline]
    pub fn scale(self, s: f32) -> Vec3 { Vec3::new(self.x * s, self.y * s, self.z * s) }
    /// `Vector3 / float` (.NET divides each component, no reciprocal multiplication).
    #[inline]
    pub fn div_scalar(self, s: f32) -> Vec3 { Vec3::new(self.x / s, self.y / s, self.z / s) }
    /// `Vector3.Dot`: (x*x + y*y) + z*z.
    #[inline]
    pub fn dot(self, o: Vec3) -> f32 { self.x * o.x + self.y * o.y + self.z * o.z }
    /// `Vector3.Length()`.
    #[inline]
    pub fn length(self) -> f32 { self.dot(self).sqrt() }
    /// .NET 8 `Vector3.Lerp`: `(value1 * (1 - amount)) + (value2 * amount)`.
    #[inline]
    pub fn lerp(a: Vec3, b: Vec3, t: f32) -> Vec3 { a.scale(1.0 - t).add(b.scale(t)) }
    /// Cavern `VectorExtensions.MapToCube`.
    #[inline]
    pub fn map_to_cube(self) -> Vec3 {
        let max = self.x.abs().max(self.y.abs().max(self.z.abs()));
        self.div_scalar(max)
    }
    #[inline]
    pub fn to_array(self) -> [f32; 3] { [self.x, self.y, self.z] }
}

/// Cavern `QMath.DbToGain(float)`: `MathF.Pow(10, gain * .05f)`.
#[inline]
pub fn db_to_gain(gain: f32) -> f32 { 10f32.powf(gain * 0.05) }

/// `Listener.EnvironmentSize` default value (no settings file is loaded by the format library).
pub const ENVIRONMENT_SIZE: Vec3 = Vec3::new(10.0, 7.0, 10.0);

/// `Listener.ScreenSize` default value.
pub const SCREEN_SIZE_X: f32 = 0.9;
pub const SCREEN_SIZE_Y: f32 = 0.486;

/// Cavern `ReferenceChannel` enum values.
#[allow(dead_code)]
pub mod ch {
    pub const FRONT_LEFT: i32 = 0;
    pub const FRONT_RIGHT: i32 = 1;
    pub const FRONT_CENTER: i32 = 2;
    pub const SCREEN_LFE: i32 = 3;
    pub const REAR_LEFT: i32 = 4;
    pub const REAR_RIGHT: i32 = 5;
    pub const SIDE_LEFT: i32 = 6;
    pub const SIDE_RIGHT: i32 = 7;
    pub const FRONT_LEFT_CENTER: i32 = 8;
    pub const FRONT_RIGHT_CENTER: i32 = 9;
    pub const HEARING_IMPAIRED: i32 = 10;
    pub const VISUALLY_IMPAIRED: i32 = 11;
    pub const UNKNOWN: i32 = 12;
    pub const MOTION_DATA: i32 = 13;
    pub const EXTERNAL_DATA: i32 = 14;
    pub const TOP_FRONT_LEFT: i32 = 15;
    pub const TOP_FRONT_RIGHT: i32 = 16;
    pub const TOP_SIDE_LEFT: i32 = 17;
    pub const TOP_SIDE_RIGHT: i32 = 18;
    pub const SIGN_LANGUAGE: i32 = 19;
    pub const BOTTOM_SURROUND: i32 = 20;
    pub const TOP_FRONT_CENTER: i32 = 21;
    pub const GODS_VOICE: i32 = 22;
    pub const REAR_CENTER: i32 = 23;
    pub const WIDE_LEFT: i32 = 24;
    pub const WIDE_RIGHT: i32 = 25;
    pub const TOP_REAR_LEFT: i32 = 26;
    pub const TOP_REAR_RIGHT: i32 = 27;
    pub const TOP_REAR_CENTER: i32 = 28;
}

/// `ChannelPrototype.AlternativePositions`, indexed by ReferenceChannel.
pub const ALTERNATIVE_POSITIONS: [Vec3; 29] = [
    Vec3::new(-1.0, 0.0, 1.0),       // FrontLeft
    Vec3::new(1.0, 0.0, 1.0),        // FrontRight
    Vec3::new(0.0, 0.0, 1.0),        // FrontCenter
    Vec3::new(-1.0, -1.0, 1.0),      // ScreenLFE
    Vec3::new(-1.0, 0.0, -1.0),      // RearLeft
    Vec3::new(1.0, 0.0, -1.0),       // RearRight
    Vec3::new(-1.0, 0.0, 0.0),       // SideLeft
    Vec3::new(1.0, 0.0, 0.0),        // SideRight
    Vec3::new(-0.5, 0.0, 1.0),       // FrontLeftCenter
    Vec3::new(0.5, 0.0, 1.0),        // FrontRightCenter
    Vec3::new(0.0, 0.0, 1.0),        // HearingImpaired
    Vec3::new(0.0, 0.0, 1.0),        // VisuallyImpaired
    Vec3::new(0.0, 0.0, 1.0),        // Unknown
    Vec3::new(0.0, 0.0, 1.0),        // MotionData
    Vec3::new(0.0, 0.0, 1.0),        // ExternalData
    Vec3::new(-1.0, 1.0, 1.0),       // TopFrontLeft
    Vec3::new(1.0, 1.0, 1.0),        // TopFrontRight
    Vec3::new(-1.0, 1.0, 0.0),       // TopSideLeft
    Vec3::new(1.0, 1.0, 0.0),        // TopSideRight
    Vec3::new(0.0, 0.0, 1.0),        // SignLanguage
    Vec3::new(0.0, -1.0, 0.0),       // BottomSurround
    Vec3::new(0.0, 1.0, 1.0),        // TopFrontCenter
    Vec3::new(0.0, 1.0, 0.0),        // GodsVoice
    Vec3::new(0.0, 0.0, -1.0),       // RearCenter
    Vec3::new(-1.0, 0.0, 0.677419),  // WideLeft
    Vec3::new(1.0, 0.0, 0.677419),   // WideRight
    Vec3::new(-1.0, 1.0, -1.0),      // TopRearLeft
    Vec3::new(1.0, 1.0, -1.0),       // TopRearRight
    Vec3::new(0.0, 1.0, -1.0),       // TopRearCenter
];

/// `ChannelPrototype.AlternativePositions[channel] * Listener.EnvironmentSize`.
#[inline]
pub fn alternative_position(channel: i32) -> Vec3 {
    let p = ALTERNATIVE_POSITIONS.get(channel as usize).copied().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    p.mul(ENVIRONMENT_SIZE)
}

/// `ChannelPrototype.Mapping[channel].LFE` (only ScreenLFE is an LFE prototype).
#[inline]
pub fn is_lfe(channel: i32) -> bool { channel == ch::SCREEN_LFE }

use ch::*;
const REF100: &[i32] = &[FRONT_CENTER];
const REF200: &[i32] = &[FRONT_LEFT, FRONT_RIGHT];
const REF300: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER];
const REF400: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, SIDE_LEFT, SIDE_RIGHT];
const REF500: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SIDE_LEFT, SIDE_RIGHT];
const REF510: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, SIDE_LEFT, SIDE_RIGHT];
const REF700: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT];
const REF710: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT];
const REF702: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_RIGHT];
const REF712: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_RIGHT];
const REF704: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_RIGHT];
const REF714: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_RIGHT];
const WAV706: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_CENTER, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_CENTER, TOP_REAR_RIGHT];
const WAV716: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    TOP_FRONT_LEFT, TOP_FRONT_CENTER, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_CENTER, TOP_REAR_RIGHT];
const WAV906: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    WIDE_LEFT, WIDE_RIGHT, TOP_FRONT_LEFT, TOP_FRONT_CENTER, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_CENTER, TOP_REAR_RIGHT];
const WAV916: &[i32] = &[FRONT_LEFT, FRONT_RIGHT, FRONT_CENTER, SCREEN_LFE, REAR_LEFT, REAR_RIGHT, SIDE_LEFT, SIDE_RIGHT,
    WIDE_LEFT, WIDE_RIGHT, TOP_FRONT_LEFT, TOP_FRONT_CENTER, TOP_FRONT_RIGHT, TOP_REAR_LEFT, TOP_REAR_CENTER, TOP_REAR_RIGHT];

/// `ChannelPrototype.HomeStandardMatrix`.
const HOME_STANDARD_MATRIX: [&[i32]; 17] = [
    &[], REF100, REF200, REF300, REF400, REF500, REF510, REF700, REF710,
    REF702, REF712, REF704, REF714, WAV706, WAV716, WAV906, WAV916,
];

/// `ChannelPrototype.GetStandardMatrix(count)` (home standard), padded with Unknown above 16 channels.
pub fn get_standard_matrix(count: usize) -> Vec<i32> {
    let sub = count.min(HOME_STANDARD_MATRIX.len() - 1);
    let mut m = HOME_STANDARD_MATRIX[sub].to_vec();
    m.resize(count, UNKNOWN);
    m
}
