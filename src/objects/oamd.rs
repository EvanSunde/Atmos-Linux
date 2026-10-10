//! Object Audio Metadata. Port of Cavern.Format/Decoders/EnhancedAC3/ObjectAudioMetadata.cs,
//! ObjectAudioMetadataEnums.cs, ObjectAudioElementMetadata.cs and ObjectInfoBlock.cs.

use crate::bits::BitExtractor;

use super::cavern::{self, ch, db_to_gain, Vec3, ENVIRONMENT_SIZE, SCREEN_SIZE_X, SCREEN_SIZE_Y};
use super::emdf::variable_bits_limited;
use super::qmf::SUBBANDS;

/// Unsupported OAMD feature (Cavern throws `UnsupportedFeatureException`).
#[derive(Debug)]
#[allow(dead_code)]
pub struct Unsupported(pub &'static str);

/// The parts of Cavern's `Source` that OAMD writes.
#[derive(Clone, Copy, Debug)]
pub struct SourceState {
    pub position: Vec3,
    pub volume: f32,
    pub size: f32,
    pub lfe: bool,
}

impl Default for SourceState {
    fn default() -> Self { Self { position: Vec3::default(), volume: 1.0, size: 0.0, lfe: false } }
}

/// `NonStandardBedChannel.LowFrequencyEffects`.
const NSB_LFE: usize = 3;
/// `NonStandardBedChannel.Max`.
const NSB_MAX: usize = 17;

const ISF_OBJECT_COUNT: [u8; 6] = [4, 8, 10, 14, 15, 30];

/// What each bit of the bed assignment means.
const BED_CHANNELS: [i32; NSB_MAX] = [
    ch::FRONT_LEFT, ch::FRONT_RIGHT, ch::FRONT_CENTER, ch::SCREEN_LFE, ch::SIDE_LEFT, ch::SIDE_RIGHT,
    ch::REAR_LEFT, ch::REAR_RIGHT, ch::TOP_FRONT_LEFT, ch::TOP_FRONT_RIGHT, ch::TOP_SIDE_LEFT, ch::TOP_SIDE_RIGHT,
    ch::TOP_REAR_LEFT, ch::TOP_REAR_RIGHT, ch::WIDE_LEFT, ch::WIDE_RIGHT, ch::SCREEN_LFE,
];

/// Which bed channels are set with each bit of a standard layout.
const STANDARD_BED_CHANNELS: [&[usize]; 10] =
    [&[0, 1], &[2], &[3], &[4, 5], &[6, 7], &[8, 9], &[10, 11], &[12, 13], &[14, 15], &[16]];

/// A decoded OAMD frame from an EMDF payload.
#[derive(Default)]
pub struct ObjectAudioMetadata {
    /// Count of bed channels.
    pub beds: u8,
    /// Number of audio objects in the stream, including beds.
    pub object_count: usize,
    elements: Vec<OAElementMD>,
    /// Bed channels used: [element][NonStandardBedChannel].
    bed_assignment: Vec<[bool; NSB_MAX]>,
    isf_in_use: bool,
    isf_index: u8,
    /// This payload applies this many samples later.
    offset: i32,
}

impl ObjectAudioMetadata {
    /// Decodes a OAMD frame from an EMDF payload.
    pub fn decode(&mut self, e: &mut BitExtractor, offset: i32) -> Result<(), Unsupported> {
        self.offset = offset;
        let mut version = e.read(2);
        if version == 3 {
            version += e.read(3);
        }
        if version != 0 {
            return Err(Unsupported("OAver"));
        }
        self.object_count = e.read(5) as usize + 1;
        if self.object_count == 32 {
            self.object_count += e.read(7) as usize;
        }
        self.program_assignment(e)?;
        let alternate_object_present = e.read_bit();
        let mut element_count = e.read(4) as usize;
        if element_count == 15 {
            element_count += e.read(5) as usize;
        }

        let mut bed_or_isf_objects = self.beds as usize;
        if self.isf_in_use {
            bed_or_isf_objects += ISF_OBJECT_COUNT[self.isf_index as usize] as usize;
        }

        if self.elements.len() != element_count {
            self.elements = (0..element_count).map(|_| OAElementMD::default()).collect();
        }
        for el in self.elements.iter_mut() {
            el.read(e, alternate_object_present, self.object_count, bed_or_isf_objects)?;
        }
        Ok(())
    }

    /// Get the "objects" that are just static channels (ReferenceChannel values).
    pub fn get_static_channels(&self) -> Vec<i32> {
        let mut result = vec![0; self.beds as usize];
        let mut last = 0;
        for assignment in &self.bed_assignment {
            for (j, &set) in assignment.iter().enumerate() {
                if set {
                    result[last] = BED_CHANNELS[j];
                    last += 1;
                    if last == self.beds as usize {
                        return result;
                    }
                }
            }
        }
        result
    }

    /// Gets which object is the LFE channel or -1 if it's not present.
    pub fn get_lfe_position(&self) -> i32 {
        let mut beds = 0;
        for bed in &self.bed_assignment {
            for (i, &set) in bed.iter().enumerate() {
                if set {
                    if i == NSB_LFE {
                        return beds;
                    }
                    beds += 1;
                }
            }
        }
        -1
    }

    /// Set the object properties from metadata. `timecode`: samples since the beginning of the audio frame.
    pub fn update_sources(&mut self, mut timecode: i32, sources: &mut [SourceState]) {
        timecode -= self.offset;
        let mut element = 0;
        for i in (0..self.elements.len()).rev() {
            let min_offset = self.elements[i].min_offset();
            if min_offset < 0 {
                continue;
            }
            if min_offset as i32 <= timecode {
                element = i;
                break;
            }
        }

        if let Some(el) = self.elements.get_mut(element) {
            el.update_sources(timecode, sources);
        }
        let mut checked_bed = 0usize;
        let mut checked_bed_instance = 0usize;
        for i in 0..self.beds as usize {
            // Cavern never resets checkedBed when moving to the next bed instance, which would throw for
            // multiple bed instances; this port stops assigning bed positions instead.
            loop {
                match self.bed_assignment.get(checked_bed_instance).and_then(|b| b.get(checked_bed)) {
                    Some(true) => break,
                    Some(false) => {
                        checked_bed += 1;
                        if checked_bed == NSB_MAX {
                            checked_bed_instance += 1;
                        }
                    }
                    None => return,
                }
            }
            if let Some(s) = sources.get_mut(i) {
                let channel = BED_CHANNELS[checked_bed];
                s.position = cavern::alternative_position(channel);
                s.lfe = cavern::is_lfe(channel);
            }
            checked_bed += 1;
            if checked_bed == NSB_MAX {
                checked_bed_instance += 1;
            }
        }
    }

    fn program_assignment(&mut self, e: &mut BitExtractor) -> Result<(), Unsupported> {
        if e.read_bit() {
            // Dynamic object-only program
            if e.read_bit() {
                // LFE present
                let mut a = [false; NSB_MAX];
                a[NSB_LFE] = true;
                self.bed_assignment = vec![a];
            } else {
                self.bed_assignment = Vec::new();
            }
        } else {
            let content_description = e.read(4);

            // Object(s) with speaker-anchored coordinate(s) (bed objects)
            if content_description & 1 != 0 {
                e.skip(1); // The object is distributable - Cavern will do it anyway
                let count = if e.read_bit() { e.read(3) as usize + 2 } else { 1 };
                self.bed_assignment = vec![[false; NSB_MAX]; count];
                for bed in 0..count {
                    if e.read_bit() {
                        // LFE only
                        self.bed_assignment[bed][NSB_LFE] = true;
                    } else if e.read_bit() {
                        // Standard bed assignment
                        let standard = e.read_bits(10);
                        for (i, &set) in standard.iter().enumerate() {
                            for &j in STANDARD_BED_CHANNELS[i] {
                                self.bed_assignment[bed][j] = set;
                            }
                        }
                    } else {
                        let bits = e.read_bits(NSB_MAX);
                        self.bed_assignment[bed].copy_from_slice(&bits);
                    }
                }
            }

            // Intermediate spatial format (ISF)
            self.isf_in_use = content_description & 2 != 0;
            if self.isf_in_use {
                self.isf_index = e.read(3) as u8;
                if self.isf_index as usize >= ISF_OBJECT_COUNT.len() {
                    return Err(Unsupported("ISF"));
                }
            }

            // Object(s) with room-anchored or screen-anchored coordinates
            if content_description & 4 != 0 && e.read(5) == 31 {
                e.skip(7);
            }

            // Reserved
            if content_description & 8 != 0 {
                let n = (e.read(4) + 1) * 8;
                e.skip(n);
            }
        }

        let mut beds = 0u8;
        for bed in &self.bed_assignment {
            beds += bed.iter().filter(|&&b| b).count() as u8;
        }
        self.beds = beds;
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------------------------
// OAElementMD (ObjectAudioElementMetadata.cs)
// ---------------------------------------------------------------------------------------------------------------

/// Marks an object positioning frame.
const OBJECT_ELEMENT_INDEX: i32 = 1;
const SAMPLE_OFFSET_INDEX: [u8; 4] = [8, 16, 18, 24];
const RAMP_DURATIONS: [i16; 3] = [0, 512, 1536];
const RAMP_DURATION_INDEX: [i16; 16] =
    [32, 64, 128, 256, 320, 480, 1000, 1001, 1024, 1600, 1601, 1602, 1920, 2000, 2002, 2048];

/// Decodes an object audio element metadata block.
#[derive(Default)]
struct OAElementMD {
    /// The movement data of the block was already set to motion.
    block_used: Vec<bool>,
    /// The object at index had a valid position in the last frame.
    update_last: Vec<bool>,
    /// The object at index has a valid position in the current frame.
    update_now: Vec<bool>,
    /// Global sample offset, applied to all info blocks.
    sample_offset: u8,
    /// The beginning of each info block in samples. Negative: this element doesn't contain object location data.
    block_offset_factor: Vec<i16>,
    /// Time to fade to a new position in samples for each info block.
    ramp_duration: Vec<i16>,
    /// Rendering info for each object's updates: [object][info block]. None if not an object element yet.
    info_blocks: Option<Vec<Vec<ObjectInfoBlock>>>,
    /// The position of each object to move to.
    future: Vec<Vec3>,
    /// Samples until reaching the future.
    future_distance: i32,
}

impl OAElementMD {
    /// The timecode of the first update in this block.
    fn min_offset(&self) -> i16 { self.block_offset_factor.first().copied().unwrap_or(-1) }

    fn read(&mut self, e: &mut BitExtractor, alternate_object_present: bool, object_count: usize,
        bed_or_isf_objects: usize) -> Result<(), Unsupported> {
        let element_index = e.read(4);
        // Cavern: endPos = Position + VariableBits(4, 4) + 1 (Position is evaluated before the read).
        let pos = e.position;
        let end_pos = pos + variable_bits_limited(e, 4, 4) + 1;
        e.skip(if alternate_object_present { 5 } else { 1 });
        if element_index == OBJECT_ELEMENT_INDEX {
            self.object_element(e, object_count, bed_or_isf_objects)?;
        } else {
            // Other elements are unused by encoders
            self.block_offset_factor = vec![(-1 - element_index) as i16];
        }
        e.position = end_pos; // Padding
        Ok(())
    }

    /// Set the object properties from metadata.
    fn update_sources(&mut self, timecode: i32, sources: &mut [SourceState]) {
        let Some(info_blocks) = self.info_blocks.as_mut() else { return };
        // Cavern would throw on a non-object element here (null arrays); skipping is the closest safe behaviour.
        self.update_last.copy_from_slice(&self.update_now);

        for blk in 0..self.block_used.len() {
            if self.block_used[blk] {
                continue;
            }
            if timecode > self.block_offset_factor[blk] as i32 {
                self.block_used[blk] = true;
                self.future_distance = self.ramp_duration[blk] as i32 - (timecode - self.block_offset_factor[blk] as i32);
                for (obj, block) in info_blocks.iter_mut().enumerate() {
                    self.update_now[obj] = block[blk].valid_position;
                    if let Some(s) = sources.get_mut(obj) {
                        self.future[obj] = block[blk].update_source(s);
                    }
                }
            }
        }

        if self.future_distance > 0 {
            let t = (SUBBANDS as f32 / self.future_distance as f32).min(1.0);
            for obj in 0..info_blocks.len() {
                if self.update_now[obj] {
                    if let Some(s) = sources.get_mut(obj) {
                        s.position = if self.update_last[obj] {
                            Vec3::lerp(s.position, self.future[obj], t)
                        } else {
                            self.future[obj]
                        };
                    }
                }
            }
            self.future_distance -= SUBBANDS as i32;
        }
        // Quirk kept from Cavern: when an update has no ramp (futureDistance <= 0), the new position is never applied.
    }

    fn object_element(&mut self, e: &mut BitExtractor, object_count: usize, bed_or_isf_objects: usize)
        -> Result<(), Unsupported> {
        self.md_update_info(e)?;
        if !e.read_bit() {
            // Reserved
            e.skip(5);
        }

        let blocks = self.ramp_duration.len();
        let needs_alloc = self.block_used.len() != blocks
            || self.info_blocks.as_ref().map(|b| b.len()).unwrap_or(usize::MAX) != object_count;
        if needs_alloc {
            self.block_used = vec![false; blocks];
            self.update_last = vec![false; object_count];
            self.update_now = vec![false; object_count];
            self.info_blocks = Some((0..object_count).map(|_| vec![ObjectInfoBlock::default(); blocks]).collect());
            self.future = vec![Vec3::default(); object_count];
        }

        self.block_used.iter_mut().for_each(|b| *b = false);
        let info_blocks = self.info_blocks.as_mut().unwrap();
        for (obj, blocks) in info_blocks.iter_mut().enumerate() {
            for (blk, block) in blocks.iter_mut().enumerate() {
                block.update(e, blk, obj < bed_or_isf_objects);
            }
        }
        Ok(())
    }

    fn md_update_info(&mut self, e: &mut BitExtractor) -> Result<(), Unsupported> {
        self.sample_offset = match e.read(2) {
            0 => 0,
            1 => SAMPLE_OFFSET_INDEX[e.read(2) as usize],
            2 => e.read(5) as u8,
            _ => return Err(Unsupported("mdOffset")),
        };
        let n = e.read(3) as usize + 1;
        self.block_offset_factor = vec![0; n];
        self.ramp_duration = vec![0; n];
        for blk in 0..n {
            self.block_offset_factor[blk] = (e.read(6) + self.sample_offset as i32) as i16;
            let ramp_duration_code = e.read(2);
            self.ramp_duration[blk] = if ramp_duration_code == 3 {
                if e.read_bit() {
                    RAMP_DURATION_INDEX[e.read(4) as usize]
                } else {
                    e.read(11) as i16
                }
            } else {
                RAMP_DURATIONS[ramp_duration_code as usize]
            };
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------------------------
// ObjectInfoBlock (ObjectInfoBlock.cs)
// ---------------------------------------------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ObjectAnchor {
    Room,
    Screen,
    Speaker,
}

const XY_SCALE: f32 = 1.0 / 62.0;
const Z_SCALE: f32 = 1.0 / 15.0;
const SIZE_SCALE: f32 = 1.0 / 31.0;
const ROOM_CENTER: Vec3 = Vec3::new(0.5, 0.5, 0.0);
const DISTANCE_FACTORS: [f32; 16] =
    [1.1, 1.3, 1.6, 2.0, 2.5, 3.2, 4.0, 5.0, 6.3, 7.9, 10.0, 12.6, 15.8, 20.0, 25.1, 50.1];
const DEPTH_FACTORS: [f32; 4] = [0.25, 0.5, 1.0, 2.0];

#[derive(Clone, Copy)]
struct ObjectInfoBlock {
    /// This block contained a position information update.
    valid_position: bool,
    /// This frame contains the difference from the last object position instead of an exact position.
    differential_position: bool,
    /// Object volume multiplier. Any negative value means reusing the last gain.
    gain: f32,
    /// Object distance from the center of the room.
    distance: f32,
    /// Object size. Any negative value means reusing the last size.
    size: f32,
    depth_factor: f32,
    screen_factor: f32,
    anchor: ObjectAnchor,
    /// The coded position information, either exact or differential.
    position: Vec3,
    /// Last fully transmitted position to add delta positions to.
    last_precise: Vec3,
}

impl Default for ObjectInfoBlock {
    fn default() -> Self {
        Self {
            valid_position: false,
            differential_position: false,
            gain: -1.0,
            distance: 0.0,
            size: -1.0,
            depth_factor: 0.0,
            screen_factor: 0.0,
            anchor: ObjectAnchor::Room,
            position: Vec3::default(),
            last_precise: Vec3::default(),
        }
    }
}

impl ObjectInfoBlock {
    /// Read new information for this block.
    fn update(&mut self, e: &mut BitExtractor, blk: usize, bed_or_isf_object: bool) {
        let inactive = e.read_bit();
        let basic_info_status = if inactive { 0 } else if blk == 0 { 1 } else { e.read(2) };
        if basic_info_status & 1 == 1 {
            self.object_basic_info(e, basic_info_status == 1);
        }

        let mut render_info_status = 0;
        if !inactive && !bed_or_isf_object {
            render_info_status = if blk == 0 { 1 } else { e.read(2) };
        }
        if render_info_status & 1 == 1 {
            self.object_render_info(e, blk, render_info_status == 1);
        }

        if e.read_bit() {
            // Additional table data
            let n = (e.read(4) + 1) * 8;
            e.skip(n);
        }

        if bed_or_isf_object {
            self.anchor = ObjectAnchor::Speaker;
        }
    }

    /// Sets the properties of the block, returns the final target position (in Cavern space).
    fn update_source(&mut self, source: &mut SourceState) -> Vec3 {
        if self.gain >= 0.0 {
            source.volume = self.gain;
        }
        if self.size >= 0.0 {
            source.size = self.size;
        }

        if self.valid_position && self.anchor != ObjectAnchor::Speaker {
            if self.differential_position {
                self.position = Vec3::new(
                    clamp01(self.last_precise.x + self.position.x),
                    clamp01(self.last_precise.y + self.position.y),
                    clamp01(self.last_precise.z + self.position.z),
                );
            } else {
                self.last_precise = self.position;
            }

            match self.anchor {
                ObjectAnchor::Room => {
                    if !self.distance.is_nan() {
                        let intersect = self.position.map_to_cube();
                        let distance_factor = intersect.length() / self.distance;
                        self.position = intersect.scale(distance_factor).add(ROOM_CENTER.scale(1.0 - distance_factor));
                    }
                }
                ObjectAnchor::Screen => {
                    // DO NOT set Cavern's screen locking, it's a different algorithm
                    let p = self.position;
                    let reference = Vec3::new((p.x - 0.5) * SCREEN_SIZE_X + 0.5, p.y, (p.z + 1.0) * SCREEN_SIZE_Y);
                    let sfm = Vec3::new(self.screen_factor, 1.0, self.screen_factor);
                    let depth = p.y.powf(self.depth_factor);
                    let dfm = Vec3::new(depth, 1.0, depth);
                    self.position = dfm.mul(sfm.mul(p).add(reference).sub(sfm.mul(reference)))
                        .add(reference).sub(dfm.mul(reference));
                }
                ObjectAnchor::Speaker => {}
            }
        }

        // Convert to Cavern coordinate space
        ENVIRONMENT_SIZE.mul(Vec3::new(
            self.position.x * 2.0 - 1.0,
            self.position.z,
            self.position.y * -2.0 + 1.0,
        ))
    }

    fn object_basic_info(&mut self, e: &mut BitExtractor, read_all_blocks: bool) {
        let blocks = if read_all_blocks { 3 } else { e.read(2) };

        // Gain
        if blocks & 2 != 0 {
            let gain_helper = e.read(2);
            let g = match gain_helper {
                0 => 1.0,
                1 => 0.0,
                2 => {
                    let gh = e.read(6);
                    db_to_gain(if gh < 15 { 15 - gh } else { 14 - gh } as f32)
                }
                _ => -1.0,
            };
            self.gain = g * 0.707; // 3 dB attenuation as some content clip without this
        }

        // Priority - unneccessary, everything's rendered
        if blocks & 1 != 0 && !e.read_bit() {
            e.skip(5);
        }
    }

    fn object_render_info(&mut self, e: &mut BitExtractor, blk: usize, read_all_blocks: bool) {
        let blocks = if read_all_blocks { 15 } else { e.read(4) };

        // Spatial position
        self.valid_position = blocks & 1 != 0;
        if self.valid_position {
            self.differential_position = blk != 0 && e.read_bit();
            if self.differential_position {
                // BitExtractor.ReadSigned always returns 0 in Cavern (kept for parity).
                self.position = Vec3::new(
                    e.read_signed(3) as f32 * XY_SCALE,
                    e.read_signed(3) as f32 * XY_SCALE,
                    e.read_signed(3) as f32 * Z_SCALE,
                );
            } else {
                let pos_x = e.read(6);
                let pos_y = e.read(6);
                let pos_z = ((e.read_bit_int() << 1) - 1) * e.read(4);
                self.position = Vec3::new(
                    (pos_x as f32 * XY_SCALE).min(1.0),
                    (pos_y as f32 * XY_SCALE).min(1.0),
                    (pos_z as f32 * Z_SCALE).min(1.0),
                );
            }
            if e.read_bit() {
                // Distance specified
                self.distance = if e.read_bit() {
                    100.0 // Infinite distance - close enough
                } else {
                    DISTANCE_FACTORS[e.read(4) as usize]
                };
            } else {
                self.distance = f32::NAN;
            }
        }

        // Zone constraints - the renderer is not prepared for zoning
        if blocks & 2 != 0 {
            e.skip(4);
        }

        // Scaling
        if blocks & 4 != 0 {
            self.size = match e.read(2) {
                0 => 0.0,
                1 => e.read(5) as f32 * SIZE_SCALE,
                2 => {
                    let x = e.read(5) as f32;
                    let y = e.read(5) as f32;
                    let z = e.read(5) as f32;
                    Vec3::new(x, y, z).scale(SIZE_SCALE).length()
                }
                _ => -1.0,
            };
        }

        // Screen anchoring
        if blocks & 8 != 0 && e.read_bit() {
            self.anchor = ObjectAnchor::Screen;
            self.screen_factor = (e.read(3) + 1) as f32 * 0.125;
            self.depth_factor = DEPTH_FACTORS[e.read(2) as usize];
        }

        e.skip(1); // Snap to the nearest channel - unused
    }
}

/// `Math.Clamp(v, 0, 1)` for floats.
#[inline]
fn clamp01(v: f32) -> f32 {
    if v < 0.0 { 0.0 } else if v > 1.0 { 1.0 } else { v }
}
