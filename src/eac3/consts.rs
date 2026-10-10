//! Port of Cavern.Format/Transcoders/EnhancedAC3Consts.cs and EnhancedAC3Enums.cs.

/// First word of all AC-3 frames.
pub const SYNC_WORD: i32 = 0x0B77;

/// Bytes that must be read before determining the frame size.
pub const MUST_DECODE: usize = 7;

/// Frame size code to actual frame size in 16-bit words for 48 kHz sample rate.
pub const FRAME_SIZES: [i32; 19] =
    [64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 448, 512, 640, 768, 896, 1024, 1152, 1280];

/// Number of blocks per sync frame for each code.
pub const NUMBER_OF_BLOCKS: [i32; 4] = [1, 2, 3, 6];

/// Sample rates for each sample rate code.
pub const SAMPLE_RATES: [i32; 3] = [48000, 44100, 32000];

/// Cavern `ReferenceChannel` enum values (Cavern/Channels/ReferenceChannel.cs).
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

use ch::*;

/// Possible channel arrangements in E-AC-3, indexed by acmod. LFE is marked separately.
pub const CHANNEL_ARRANGEMENTS: [&[i32]; 8] = [
    &[FRONT_CENTER, FRONT_CENTER],                                  // 0: dual mono
    &[FRONT_CENTER],                                                // 1: mono
    &[FRONT_LEFT, FRONT_RIGHT],                                     // 2: stereo
    &[FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT],                       // 3: L, C, R
    &[FRONT_LEFT, FRONT_RIGHT, REAR_CENTER],                        // 4: L, R, S
    &[FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT, REAR_CENTER],          // 5: L, C, R, S
    &[FRONT_LEFT, FRONT_RIGHT, SIDE_LEFT, SIDE_RIGHT],              // 6: L, R, SL, SR
    &[FRONT_LEFT, FRONT_CENTER, FRONT_RIGHT, SIDE_LEFT, SIDE_RIGHT], // 7: L, C, R, SL, SR
];

/// If a custom channel mapping is present, these are the channels for each bit.
pub const CHANNEL_MAPPING_TARGETS: [&[i32]; 16] = [
    &[SCREEN_LFE],
    &[SCREEN_LFE],
    &[TOP_SIDE_LEFT, TOP_SIDE_RIGHT],
    &[TOP_FRONT_CENTER],
    &[TOP_FRONT_LEFT, TOP_FRONT_RIGHT],
    &[WIDE_LEFT, WIDE_RIGHT],
    &[SCREEN_LFE, SCREEN_LFE], // Side surround, but not used (Cavern's mapping)
    &[GODS_VOICE],
    &[REAR_CENTER],
    &[REAR_LEFT, REAR_RIGHT],
    &[FRONT_LEFT_CENTER, FRONT_RIGHT_CENTER],
    &[SIDE_RIGHT],
    &[SIDE_LEFT],
    &[FRONT_RIGHT],
    &[FRONT_CENTER],
    &[FRONT_LEFT],
];

/// Supported decoder versions (bsid).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DecoderType {
    AlternateAc3 = 6,
    #[default]
    Ac3 = 8,
    Eac3 = 16,
}

/// Types of programs in a single frame of a stream.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum StreamType {
    #[default]
    Independent = 0,
    Dependent = 1,
    Repackaged = 2,
    Reserved = 3,
}

impl StreamType {
    pub fn from_bits(v: i32) -> Self {
        match v & 3 {
            0 => StreamType::Independent,
            1 => StreamType::Dependent,
            2 => StreamType::Repackaged,
            _ => StreamType::Reserved,
        }
    }
}

/// Errors Cavern would throw as exceptions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Eac3Error {
    Sync,
    Reserved(&'static str),
    Unsupported(&'static str),
    Corruption(&'static str),
    Decoder(i32),
}

impl std::fmt::Display for Eac3Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Eac3Error::Sync => write!(f, "sync word not found"),
            Eac3Error::Reserved(s) => write!(f, "reserved value: {s}"),
            Eac3Error::Unsupported(s) => write!(f, "unsupported feature: {s}"),
            Eac3Error::Corruption(s) => write!(f, "corrupt field: {s}"),
            Eac3Error::Decoder(c) => write!(f, "decoder error {c}"),
        }
    }
}

impl std::error::Error for Eac3Error {}
