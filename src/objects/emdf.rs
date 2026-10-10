//! Extensible Metadata Delivery Format decoding. Port of
//! Cavern.Format/Decoders/EnhancedAC3/ExtensibleMetadataDecoder.cs and ExtensibleMetadataExtensions.cs.

use crate::bits::BitExtractor;

use super::joc::JointObjectCoding;
use super::oamd::ObjectAudioMetadata;

/// EMDF marker.
const SYNC_WORD: i32 = 0x5838;
/// Payload ID for Object Audio Metadata.
const OAMD_PAYLOAD_ID: i32 = 11;
/// Payload ID for Joint Object Coding.
const JOC_PAYLOAD_ID: i32 = 14;

/// Read variable-length values from an EMDF stream.
pub fn variable_bits(e: &mut BitExtractor, bits: i32) -> i32 {
    let mut value = 0i32;
    loop {
        value = value.wrapping_add(e.read(bits));
        if e.read_bit() {
            value = (value.wrapping_add(1)).wrapping_shl(bits as u32);
        } else {
            return value;
        }
    }
}

/// Read variable-length values from an EMDF stream with a limit on length
/// (`limit` is 1 less than the actual length, as in Cavern).
pub fn variable_bits_limited(e: &mut BitExtractor, bits: i32, mut limit: i32) -> i32 {
    let mut value = 0i32;
    loop {
        value = value.wrapping_add(e.read(bits));
        let read_more = e.read_bit();
        if read_more {
            value = (value.wrapping_add(1)).wrapping_shl(bits as u32);
        }
        // C#: while (readMore && limit-- != 0) - limit is only decremented when readMore is true.
        if !read_more {
            return value;
        }
        let l = limit;
        limit -= 1;
        if l == 0 {
            return value;
        }
    }
}

/// Decodes EMDF from the auxiliary data of an E-AC-3 frame.
#[derive(Default)]
pub struct ExtensibleMetadataDecoder {
    /// The last EMDF frame contained object (JOC) data.
    pub has_objects: bool,
    /// The Joint Object Coding data of the last frame.
    pub joc: JointObjectCoding,
    /// The Object Audio Metadata of the last frame.
    pub oamd: ObjectAudioMetadata,
}

impl ExtensibleMetadataDecoder {
    pub fn new() -> Self { Self::default() }

    /// Decode the next EMDF frame from a bitstream.
    pub fn decode(&mut self, e: &mut BitExtractor) {
        self.has_objects = false;
        let mut syncword = 0i32;
        while e.position < e.back_position - 32 {
            syncword = ((syncword << 8) & 0xFFFF) + e.read(8); // Syncwords are byte-padded
            if syncword == SYNC_WORD && self.decode_block(e) {
                break;
            }
        }
    }

    /// Tries to decode an EMDF block, returns if succeeded. Assumes the sync word was already read.
    fn decode_block(&mut self, e: &mut BitExtractor) -> bool {
        let length = e.read(16);
        let frame_end_pos = e.position + length * 8;
        if frame_end_pos > e.back_position {
            return false;
        }

        let mut version = e.read(2);
        if version == 3 {
            version += variable_bits(e, 2);
        }
        let mut key = e.read(3);
        if key == 7 {
            key += variable_bits(e, 3);
        }
        if version != 0 || key != 0 {
            return false;
        }

        while e.position < frame_end_pos {
            let mut payload_id = e.read(5);
            if payload_id == 0 {
                break;
            }
            if payload_id == 0x1F {
                payload_id += variable_bits(e, 5);
            }
            if payload_id > JOC_PAYLOAD_ID {
                return false;
            }

            let has_sample_offset = e.read_bit();
            let mut sample_offset = 0;
            if has_sample_offset {
                sample_offset = e.read(12) >> 1; // Skip 1 bit
            }

            if e.read_bit() {
                variable_bits(e, 11);
            }
            if e.read_bit() {
                variable_bits(e, 2);
            }
            if e.read_bit() {
                e.skip(8);
            }

            if !e.read_bit() {
                let mut frame_aligned = false;
                if !has_sample_offset {
                    frame_aligned = e.read_bit();
                    if frame_aligned {
                        e.skip(2);
                    }
                }
                if has_sample_offset || frame_aligned {
                    e.skip(7);
                }
            }

            let payload_end = variable_bits(e, 8) * 8 + e.position;
            if payload_end > e.back_position {
                return false;
            }
            if payload_id == JOC_PAYLOAD_ID {
                // Cavern throws on unsupported JOC configurations; the payload is skipped here instead.
                let _ = self.joc.decode(e);
                self.has_objects = true;
            } else if payload_id == OAMD_PAYLOAD_ID {
                let _ = self.oamd.decode(e, sample_offset);
            }
            e.position = payload_end;
        }
        true
    }
}
