//! Hardcoded values for JOC decoding. Port of Cavern.Format/Decoders/EnhancedAC3/JointObjectCodingTables.cs
//! (the Huffman tables are in `joc_tables_huff.rs`, generated from the same file), plus the expanded
//! parameter band LUT that JointObjectCodingCache.cs `SetupStaticCache` builds.

use std::sync::OnceLock;

use super::cavern::ch;
use super::joc_tables_huff::*;
use super::qmf::SUBBANDS;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum HuffmanType {
    Mtx,
    Vec,
    Idx,
}

/// Implementation of joc_get_huff_code, gets the corresponding Huffman table.
/// Does not handle invalid modes correctly (as Cavern).
pub fn get_huff_code_table(mode: i32, ty: HuffmanType) -> &'static [[i16; 2]] {
    match ty {
        HuffmanType::Mtx => if mode == 1 { &JOC_HUFF_CODE_FINE_GENERIC } else { &JOC_HUFF_CODE_COARSE_GENERIC },
        HuffmanType::Vec => if mode == 1 { &JOC_HUFF_CODE_FINE_COEFF_SPARSE } else { &JOC_HUFF_CODE_COARSE_COEFF_SPARSE },
        HuffmanType::Idx => if mode == 7 { &JOC_HUFF_CODE_7CH_POS_INDEX_SPARSE } else { &JOC_HUFF_CODE_5CH_POS_INDEX_SPARSE },
    }
}

/// The order of input channels for JOC matrix mixing.
pub const INPUT_MATRIX: [i32; 7] = [
    ch::FRONT_LEFT, ch::FRONT_RIGHT, ch::FRONT_CENTER, ch::SIDE_LEFT, ch::SIDE_RIGHT, ch::REAR_LEFT, ch::REAR_RIGHT,
];

pub const JOC_NUM_BANDS: [u8; 8] = [1, 3, 5, 7, 9, 12, 15, 23];

const PARAMETER_BAND_MAPPING_COMPRESSED: [&[u8]; 8] = [
    &[0],
    &[0, 3, 14],
    &[0, 1, 3, 9, 23],
    &[0, 1, 2, 4, 8, 14, 23],
    &[0, 1, 2, 3, 5, 7, 9, 14, 23],
    &[0, 1, 2, 3, 4, 6, 8, 11, 14, 18, 23, 35],
    &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 14, 18, 23, 35],
    &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20, 23, 26, 30, 35, 41, 48],
];

/// Parameter band of each QMF subband for each band count index (`SetupStaticCache` expansion:
/// binary search, `~pb - 1` when not found).
pub fn parameter_band_mapping() -> &'static [[u8; SUBBANDS]; 8] {
    static M: OnceLock<[[u8; SUBBANDS]; 8]> = OnceLock::new();
    M.get_or_init(|| {
        let mut out = [[0u8; SUBBANDS]; 8];
        for (i, m) in PARAMETER_BAND_MAPPING_COMPRESSED.iter().enumerate() {
            for sb in 0..SUBBANDS {
                let pb = match m.binary_search(&(sb as u8)) {
                    Ok(p) => p as i32,
                    Err(ins) => ins as i32 - 1,
                };
                out[i][sb] = pb as u8;
            }
        }
        out
    })
}
