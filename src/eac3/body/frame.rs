//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/DecodeAudioFrame.cs.

use super::tables::FRMCPLEXPSTR_TBL;
use super::Body;
use crate::eac3::consts::{Eac3Error, StreamType};
use crate::eac3::header::Header;

/// Cavern QMath.Log2: reads the exponent of the float representation.
fn qmath_log2(val: i32) -> i32 {
    let bits = (val as f32).to_bits() as i32;
    ((bits >> 23) + 1) & 0x1F
}

/// Cavern QMath.Log2Ceil.
pub fn log2_ceil(val: i32) -> i32 {
    let log = qmath_log2(val);
    if (1i32.wrapping_shl(log as u32)) != val {
        log + 1
    } else {
        log
    }
}

impl Body {
    /// For E-AC-3, data for multiple blocks is included in an audio frame header.
    pub(super) fn decode_audio_frame(&mut self, header: &Header) -> Result<(), Eac3Error> {
        let nch = self.channels.len();
        let e = &mut self.extractor;
        self.expstre = header.blocks != 6 || e.read_bit();
        self.ahte = header.blocks == 6 && e.read_bit();
        self.snroffststr = e.read(2);
        self.transproce = e.read_bit();
        self.blkswe = e.read_bit();
        self.dithflage = e.read_bit();
        self.bamode = e.read_bit();
        self.frmfgaincode = e.read_bit();
        self.dbaflde = e.read_bit();
        self.skip_field_syntax_enabled = e.read_bit();
        self.spxattene = e.read_bit();

        if header.channel_mode > 1 {
            // Not mono
            self.cplstre[0] = true;
            self.cplinu[0] = e.read_bit();
            for block in 1..self.cplstre.len() {
                self.cplstre[block] = e.read_bit();
                if self.cplstre[block] {
                    self.cplinu[block] = e.read_bit();
                } else {
                    self.cplinu[block] = self.cplinu[block - 1];
                }
            }
        } else {
            // Cavern quirk: cplinu[0] is left untouched
            for block in 1..self.cplstre.len() {
                self.cplinu[block] = false;
            }
        }

        // Exponent strategy data init
        if self.expstre {
            for block in 0..self.cplexpstr.len() {
                if self.cplinu[block] {
                    self.cplexpstr[block] = e.read(2) as u8;
                }
                for channel in 0..nch {
                    self.chexpstr[block][channel] = e.read(2) as u8;
                }
            }
        } else {
            let ncplblks = self.cplinu.iter().filter(|&&x| x).count();
            if header.channel_mode > 1 && ncplblks > 0 {
                self.frmcplexpstr = e.read(5);
            }
            for channel in 0..nch {
                self.frmchexpstr[channel] = e.read(5);
            }
            for block in 0..self.cplexpstr.len() {
                self.cplexpstr[block] = FRMCPLEXPSTR_TBL[self.frmcplexpstr as usize][block];
                for channel in 0..nch {
                    self.chexpstr[block][channel] = FRMCPLEXPSTR_TBL[self.frmchexpstr[channel] as usize][block];
                }
            }
        }

        if header.lfe {
            for block in 0..self.lfeexpstr.len() {
                self.lfeexpstr[block] = e.read_bit();
            }
        }

        // Converter exponent strategy data
        if header.stream_type == StreamType::Independent {
            self.convexpstre = header.blocks == 6 || e.read_bit();
            if self.convexpstre {
                for channel in 0..nch {
                    self.convexpstr[channel] = e.read(5);
                }
            }
        }

        // AHT data
        if self.ahte {
            return Err(Eac3Error::Unsupported("AHT"));
        }

        // Audio frame SNR offset data
        if self.snroffststr == 0 {
            self.frmcsnroffst = e.read(6);
            self.frmfsnroffst = e.read(4);
        }

        // Transient pre-noise processing data
        if self.transproce {
            for channel in 0..nch {
                self.chintransproc[channel] = e.read_bit();
                if self.chintransproc[channel] {
                    self.transprocloc[channel] = e.read(10);
                    self.transproclen[channel] = e.read(8);
                }
            }
        }

        // Spectral extension attenuation data
        if self.spxattene {
            for ch in 0..nch {
                self.chinspxatten[ch] = e.read_bit();
                if self.chinspxatten[ch] {
                    self.spxattencod[ch] = e.read(5);
                }
            }
        }

        self.blkstrtinfoe = header.blocks != 1 && e.read_bit();
        if self.blkstrtinfoe {
            let nblkstrtbits = (header.blocks - 1) * (4 + log2_ceil(header.words_per_syncframe));
            self.blkstrtinfo = e.read(nblkstrtbits);
        }

        // Syntax state init
        for channel in 0..nch {
            self.firstspxcos[channel] = true;
            self.firstcplcos[channel] = true;
        }
        self.firstcplleak = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn log2ceil() {
        assert_eq!(super::log2_ceil(1536), 11);
        assert_eq!(super::log2_ceil(1024), 10);
        assert_eq!(super::log2_ceil(1025), 11);
    }
}
