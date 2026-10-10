//! Port of Cavern.Format/Transcoders/EnhancedAC3Header.cs and EnhancedAC3Header/{AC3,EAC3}/*.cs
//! (decode paths only).

use super::consts::*;
use crate::bits::BitExtractor;

const CHANNEL_MAPPING_BITS: i32 = 16;

/// Read an E-AC-3 header (and keeps the fields that persist between headers, like Cavern's instance does).
#[derive(Default, Clone)]
pub struct Header {
    /// acmod
    pub channel_mode: i32,
    /// numblks
    pub blocks: i32,
    /// lfeon
    pub lfe: bool,
    pub sample_rate: i32,
    /// fscod
    pub sample_rate_code: i32,
    /// substreamid (+8 for dependent substreams)
    pub substream_id: i32,
    pub words_per_syncframe: i32,
    pub decoder: DecoderType,
    pub stream_type: StreamType,
    bsmod: i32,
    /// chanmap, None if chanmape is off
    channel_mapping: Option<i32>,
    frmsizecod: i32,

    // EAC3/BitStreamInformation.cs
    pub dialnorm: i32,
    pub compr: Option<i32>,
    pub dialnorm2: i32,
    pub compr2: Option<i32>,
    blkid: bool,
    convsync: bool,
    addbsie: bool,
    addbsi: Vec<u8>,
    addbsilen: usize,

    // EAC3/Mixing.cs
    mixing_enabled: bool,
    center_downmix: i32,
    surround_downmix: i32,
    dmixmod: i32,
    lfemixlevcod: Option<i32>,
    pgmscl: Option<i32>,
    pgmscl2: Option<i32>,
    extpgmscl: Option<i32>,
    mixdef: i32,
    mixdata: i32,
    mixdata_large: Vec<u8>,
    mixdata_large_len: usize,
    paninfoe: bool,
    panmean: i32,
    paninfo: i32,
    paninfo2e: bool,
    panmean2: i32,
    paninfo2: i32,
    frmmixcfginfoe: bool,
    blkmixcfginfo: Vec<Option<i32>>,

    // EAC3/Informational.cs
    informational_metadata_enabled: bool,
    copyright_bit: bool,
    original_bitstream: bool,
    dsurmod: i32,
    dheadphonmod: i32,
    dsurexmod: i32,
    audprodie: bool,
    mixlevel: i32,
    roomtyp: i32,
    adconvtyp: bool,
    audprodi2e: bool,
    mixlevel2: i32,
    roomtyp2: i32,
    adconvtyp2: bool,
    sourcefscod: bool,

    // AC3/BitStreamInformation.cs
    langcod: Option<i32>,
    langcod2: Option<i32>,
    additional_downmix_info: Option<i32>,
    additional_mix_info: Option<i32>,
}

fn parse_decoder(bsid: i32) -> Result<DecoderType, Eac3Error> {
    if bsid == DecoderType::AlternateAc3 as i32 {
        return Ok(DecoderType::AlternateAc3);
    }
    if bsid <= DecoderType::Ac3 as i32 {
        return Ok(DecoderType::Ac3);
    }
    if bsid == DecoderType::Eac3 as i32 {
        return Ok(DecoderType::Eac3);
    }
    Err(Eac3Error::Unsupported("decoder"))
}

/// Outcome of looking at the first [`MUST_DECODE`] bytes of a syncframe.
pub enum Peek {
    /// Syncword 0: Cavern treats this as end of stream.
    End,
    /// Valid sync: total syncframe length in bytes and the substream ID Cavern would assign.
    Frame { bytes: usize, substream_id: i32 },
}

impl Header {
    /// Look at the first 7 bytes of a syncframe to know its length and substream ID, exactly as
    /// [`Header::decode`] would compute them.
    pub fn peek(b: &[u8]) -> Result<Peek, Eac3Error> {
        let sync = ((b[0] as i32) << 8) | b[1] as i32;
        if sync != SYNC_WORD {
            if sync == 0 {
                return Ok(Peek::End);
            }
            return Err(Eac3Error::Sync);
        }
        let mut words = ((((b[2] & 7) as i32) << 8) | b[3] as i32) + 1;
        let stream_type = StreamType::from_bits((b[2] >> 6) as i32);
        let mut sid = ((b[2] >> 3) & 7) as i32;
        let decoder = parse_decoder((b[5] >> 3) as i32)?;
        if decoder != DecoderType::Eac3 {
            let src = (b[4] >> 6) as i32;
            let frmsizecod = (b[4] & 63) as i32;
            words = ac3_words(src, frmsizecod)?;
            sid = 0;
        } else if stream_type == StreamType::Dependent {
            sid += 8;
        }
        Ok(Peek::Frame { bytes: words as usize * 2, substream_id: sid })
    }

    /// Reads an E-AC-3 header from a complete syncframe (`frame` must hold the whole syncframe,
    /// as Cavern reads `WordsPerSyncframe * 2` bytes). Returns an extractor positioned at the audio frame.
    pub fn decode(&mut self, frame: Vec<u8>) -> Result<BitExtractor, Eac3Error> {
        let mut e = BitExtractor::new(frame);
        let read_sync = e.read(16);
        if read_sync != SYNC_WORD {
            return Err(Eac3Error::Sync);
        }

        self.stream_type = StreamType::from_bits(e.read(2));
        self.substream_id = e.read(3);
        self.words_per_syncframe = e.read(11) + 1;
        self.sample_rate_code = e.read(2);
        self.blocks = NUMBER_OF_BLOCKS[e.read(2) as usize];
        self.channel_mode = e.read(3);
        self.lfe = e.read_bit();
        self.decoder = parse_decoder(e.read(5))?;

        if self.decoder != DecoderType::Eac3 {
            self.stream_type = StreamType::Repackaged;
            self.substream_id = 0;
            self.blocks = 6;
            self.sample_rate_code = (e.at(4) >> 6) as i32;
            self.frmsizecod = (e.at(4) & 63) as i32;
            self.words_per_syncframe = ac3_words(self.sample_rate_code, self.frmsizecod)?;
            self.bsmod = e.read(3);
            self.channel_mode = e.read(3);
        }
        // Cavern's extractor.Expand(...) here only appends the rest of the frame; we already hold it.

        if self.stream_type == StreamType::Dependent {
            self.substream_id += 8; // There can be 8 dependent and independent substreams, both start at 0
        }
        if self.stream_type == StreamType::Reserved {
            return Err(Eac3Error::Reserved("strmtyp"));
        }
        if self.sample_rate_code == 3 {
            return Err(Eac3Error::Reserved("fscod"));
        }
        self.sample_rate = SAMPLE_RATES[self.sample_rate_code as usize];

        self.channel_mapping = None;
        match self.decoder {
            DecoderType::AlternateAc3 | DecoderType::Ac3 => self.read_bit_stream_information(&mut e),
            DecoderType::Eac3 => self.read_bit_stream_information_eac3(&mut e),
        }
        Ok(e)
    }

    /// Gets the channels contained in the stream in order.
    pub fn get_channel_arrangement(&self) -> Result<Vec<i32>, Eac3Error> {
        let mut channels: Vec<i32> = CHANNEL_ARRANGEMENTS[self.channel_mode as usize].to_vec();
        if let Some(mapping) = self.channel_mapping {
            let mut channel = 0;
            let mut i = CHANNEL_MAPPING_BITS - 1;
            while i > 0 {
                if (mapping >> i) & 1 == 1 {
                    for &target in CHANNEL_MAPPING_TARGETS[i as usize] {
                        if channel == channels.len() {
                            return Err(Eac3Error::Corruption("chanmap"));
                        }
                        channels[channel] = target;
                        channel += 1;
                    }
                }
                i -= 1;
            }
        }
        Ok(channels)
    }

    // ---- EAC3/BitStreamInformation.cs ----
    fn read_bit_stream_information_eac3(&mut self, e: &mut BitExtractor) {
        self.dialnorm = e.read(5);
        self.compr = e.read_conditional(8);
        if self.channel_mode == 0 {
            self.dialnorm2 = e.read(5);
            self.compr2 = e.read_conditional(8);
        }
        if self.stream_type == StreamType::Dependent {
            self.channel_mapping = e.read_conditional(CHANNEL_MAPPING_BITS);
        }
        self.read_mixing_metadata(e);
        self.read_info_metadata(e);
        if self.stream_type == StreamType::Independent && self.blocks != 6 {
            self.convsync = e.read_bit();
        }
        if self.stream_type == StreamType::Repackaged {
            self.blkid = self.blocks == 6 || e.read_bit();
            if self.blkid {
                self.frmsizecod = e.read(6);
            }
        }
        self.addbsie = e.read_bit();
        if self.addbsie {
            self.addbsilen = 0;
            let n = e.read(6) + 1;
            e.read_bytes_into(&mut self.addbsi, &mut self.addbsilen, n);
        }
    }

    // ---- EAC3/Mixing.cs ----
    fn read_mixing_metadata(&mut self, e: &mut BitExtractor) {
        self.mixing_enabled = e.read_bit();
        if !self.mixing_enabled {
            return;
        }
        if self.channel_mode > 2 {
            self.dmixmod = e.read(2);
        }
        if (self.channel_mode & 1) != 0 && self.channel_mode > 2 {
            self.center_downmix = e.read(6);
        }
        if (self.channel_mode & 4) != 0 {
            self.surround_downmix = e.read(6);
        }
        if self.lfe {
            self.lfemixlevcod = e.read_conditional(5);
        }

        if self.stream_type == StreamType::Independent {
            self.pgmscl = e.read_conditional(6);
            if self.channel_mode == 0 {
                self.pgmscl2 = e.read_conditional(6);
            }
            self.extpgmscl = e.read_conditional(6);

            self.mixdef = e.read(2);
            if self.mixdef == 1 {
                self.mixdata = e.read(5);
            } else if self.mixdef == 2 {
                self.mixdata = e.read(12);
            } else if self.mixdef == 3 {
                self.mixdata_large_len = 0;
                let n = e.read(5) + 2;
                e.read_bytes_into(&mut self.mixdata_large, &mut self.mixdata_large_len, n);
            }

            if self.channel_mode < 2 {
                self.paninfoe = e.read_bit();
                if self.paninfoe {
                    self.panmean = e.read(8);
                    self.paninfo = e.read(6);
                }
                if self.channel_mode == 0 {
                    self.paninfo2e = e.read_bit();
                    if self.paninfo2e {
                        self.panmean2 = e.read(8);
                        self.paninfo2 = e.read(6);
                    }
                }
            }

            self.frmmixcfginfoe = e.read_bit();
            if self.frmmixcfginfoe {
                if self.blkmixcfginfo.len() != self.blocks as usize {
                    self.blkmixcfginfo = vec![None; self.blocks as usize];
                }
                if self.blocks == 1 {
                    self.blkmixcfginfo[0] = Some(e.read(5));
                } else {
                    for block in 0..self.blocks as usize {
                        self.blkmixcfginfo[block] = e.read_conditional(5);
                    }
                }
            }
        }
    }

    // ---- EAC3/Informational.cs ----
    fn read_info_metadata(&mut self, e: &mut BitExtractor) {
        self.informational_metadata_enabled = e.read_bit();
        if !self.informational_metadata_enabled {
            return;
        }
        self.bsmod = e.read(3);
        self.copyright_bit = e.read_bit();
        self.original_bitstream = e.read_bit();
        if self.channel_mode == 2 {
            self.dsurmod = e.read(2);
            self.dheadphonmod = e.read(2);
        } else if self.channel_mode >= 6 {
            self.dsurexmod = e.read(2);
        }
        self.audprodie = e.read_bit();
        if self.audprodie {
            self.mixlevel = e.read(5);
            self.roomtyp = e.read(2);
            self.adconvtyp = e.read_bit();
        }
        if self.channel_mode == 0 {
            self.audprodi2e = e.read_bit();
            if self.audprodi2e {
                self.mixlevel2 = e.read(5);
                self.roomtyp2 = e.read(2);
                self.adconvtyp2 = e.read_bit();
            }
        }
        if self.sample_rate_code < 3 {
            self.sourcefscod = e.read_bit();
        }
    }

    // ---- AC3/BitStreamInformation.cs ----
    fn read_bit_stream_information(&mut self, e: &mut BitExtractor) {
        if (self.channel_mode & 1) != 0 && self.channel_mode != 1 {
            self.center_downmix = e.read(2);
        }
        if (self.channel_mode & 4) != 0 {
            self.surround_downmix = e.read(2);
        }
        if self.channel_mode == 2 {
            self.dsurmod = e.read(2);
        }

        self.lfe = e.read_bit();
        self.dialnorm = e.read(5);
        self.compr = e.read_conditional(8);
        self.langcod = e.read_conditional(8);
        self.audprodie = e.read_bit();
        if self.audprodie {
            self.mixlevel = e.read(5);
            self.roomtyp = e.read(2);
        }

        if self.channel_mode == 0 {
            self.dialnorm2 = e.read(5);
            self.compr2 = e.read_conditional(8);
            self.langcod2 = e.read_conditional(8);
            self.audprodi2e = e.read_bit();
            if self.audprodi2e {
                self.mixlevel2 = e.read(5);
                self.roomtyp2 = e.read(2);
            }
        }

        self.copyright_bit = e.read_bit();
        self.original_bitstream = e.read_bit();
        self.additional_downmix_info = e.read_conditional(14);
        self.additional_mix_info = e.read_conditional(14);
        self.addbsie = e.read_bit();
        if self.addbsie {
            self.addbsilen = 0;
            let n = e.read(6) + 1;
            e.read_bytes_into(&mut self.addbsi, &mut self.addbsilen, n);
        }
    }
}

/// Words per syncframe for legacy AC-3 frames.
fn ac3_words(sample_rate_code: i32, frmsizecod: i32) -> Result<i32, Eac3Error> {
    let idx = (frmsizecod >> 1) as usize;
    if idx >= FRAME_SIZES.len() {
        return Err(Eac3Error::Corruption("frmsizecod")); // Cavern: IndexOutOfRangeException
    }
    let mut words = FRAME_SIZES[idx];
    if sample_rate_code == 1 {
        // 44.1 kHz
        words = words * 1393 / 1280;
        if (frmsizecod & 1) == 1 {
            words += 1;
        }
    } else if sample_rate_code == 2 {
        words += words >> 1;
    }
    Ok(words)
}
