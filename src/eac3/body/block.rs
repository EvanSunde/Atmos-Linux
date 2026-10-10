//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/DecodeAudioBlock.cs and the channel-level wrappers of
//! BitAllocation.cs (Allocate, AllocateCoupling, AllocateLFE).

use super::allocation::{LFEENDMANT, LFESTRTMANT};
use super::bit_allocation::{allocate, AllocParams, DBA_NEW_INFO_FOLLOWS};
use super::tables::FASTGAIN;
use super::{Body, EXP_REUSE};
use crate::eac3::consts::{DecoderType, Eac3Error, StreamType};
use crate::eac3::header::Header;

/// CavernFormatGlobal.Unsafe defaults to false, so the error checks throw.
const UNSAFE: bool = false;

impl Body {
    fn alloc_params(&self, header: &Header) -> AllocParams {
        AllocParams {
            sdcycod: self.sdcycod,
            fdcycod: self.fdcycod,
            sgaincod: self.sgaincod,
            dbpbcod: self.dbpbcod,
            floorcod: self.floorcod,
            sample_rate_code: header.sample_rate_code,
        }
    }

    fn allocate_channel(&mut self, header: &Header, channel: usize) {
        if self.csnroffst == 0 && self.fsnroffst[channel] == 0 {
            self.allocation[channel].bap.fill(0);
            return;
        }
        let snroffset = (((self.csnroffst - 15) << 4) + self.fsnroffst[channel]) << 2;
        let p = self.alloc_params(header);
        allocate(&p, 0, self.endmant[channel], FASTGAIN[self.fgaincod[channel] as usize], snroffset,
            &mut self.allocation[channel], &self.deltba[channel], 0, 0);
    }

    fn allocate_coupling(&mut self, header: &Header) {
        if self.csnroffst == 0 && self.cplfsnroffst == 0 {
            self.coupling_allocation.bap.fill(0);
            return;
        }
        let snroffset = (((self.csnroffst - 15) << 4) + self.cplfsnroffst) << 2;
        let p = self.alloc_params(header);
        allocate(&p, self.cplstrtmant, self.cplendmant, FASTGAIN[self.cplfgaincod as usize], snroffset,
            &mut self.coupling_allocation, &self.cpldeltba, (self.cplfleak << 8) + 768, (self.cplsleak << 8) + 768);
    }

    fn allocate_lfe(&mut self, header: &Header) {
        if self.csnroffst == 0 && self.lfefsnroffst == 0 {
            self.lfe_allocation.bap.fill(0);
            return;
        }
        let snroffset = (((self.csnroffst - 15) << 4) + self.lfefsnroffst) << 2;
        let p = self.alloc_params(header);
        allocate(&p, LFESTRTMANT, LFEENDMANT, FASTGAIN[self.lfefgaincod as usize], snroffset,
            &mut self.lfe_allocation, &self.lfedeltba, 0, 0);
    }

    /// Decode an (E-)AC-3 audio block.
    pub(super) fn decode_audio_block(&mut self, header: &Header, block: usize) -> Result<(), Eac3Error> {
        let eac3 = header.decoder == DecoderType::Eac3;
        let nch = self.channels.len();

        {
            let e = &mut self.extractor;
            if self.blkswe {
                for channel in 0..nch {
                    self.blksw[channel] = e.read_bit();
                }
            } else {
                self.blksw.fill(false);
            }
            if self.dithflage {
                for channel in 0..nch {
                    self.dithflag[channel] = e.read_bit();
                }
            } else {
                self.dithflag.fill(true);
            }

            self.dynrng = e.read_conditional(8);
            if header.channel_mode == 0 {
                self.dynrng2 = e.read_conditional(8);
            }
        }

        if eac3 {
            self.read_spx(header, block);
        } else {
            self.spxinu = false;
            self.clear_spx();
        }

        self.decode_coupling_strategy(header, eac3, block);
        if self.cplinu[block] {
            self.decode_coupling_coordinates(header, eac3)?;
        }

        if header.channel_mode == 2 {
            // Cavern doesn't implement rematrixing
            return Err(Eac3Error::Unsupported("stereo"));
        }

        let e = &mut self.extractor;
        // Exponent strategy
        if !eac3 {
            if self.cplinu[block] {
                self.cplexpstr[block] = e.read(2) as u8;
            }
            for channel in 0..nch {
                self.chexpstr[block][channel] = e.read(2) as u8;
            }
            if header.lfe {
                self.lfeexpstr[block] = e.read_bit();
            }
        }

        // Channel bandwidth code
        for channel in 0..nch {
            if self.chexpstr[block][channel] != EXP_REUSE && !self.chincpl[channel] && !self.chinspx[channel] {
                self.chbwcod[channel] = e.read(6);
            }
        }

        // Exponents
        self.parse_parametric_bit_allocation(block);
        let e = &mut self.extractor;
        if self.cplinu[block] && self.cplexpstr[block] != EXP_REUSE {
            self.coupling_allocation.read_coupling_exponents(e, self.cplexpstr[block], self.cplstrtmant, self.ncplgrps);
        }

        // Exponents for full bandwidth channels
        for channel in 0..nch {
            if self.chexpstr[block][channel] != EXP_REUSE {
                self.allocation[channel].read_channel_exponents(e, self.chexpstr[block][channel], self.nchgrps[channel]);
            }
        }

        // Exponents for LFE channel
        if header.lfe && self.lfeexpstr[block] {
            self.lfe_allocation.read_lfe_exponents(e);
        }

        // Bit allocation parametric information
        if self.bamode {
            self.baie = e.read_bit();
            if self.baie {
                self.sdcycod = e.read(2);
                self.fdcycod = e.read(2);
                self.sgaincod = e.read(2);
                self.dbpbcod = e.read(2);
                self.floorcod = e.read(3);
            }
        } else {
            self.sdcycod = 2;
            self.fdcycod = 1;
            self.sgaincod = 1;
            self.dbpbcod = 2;
            self.floorcod = 7;
        }

        if self.snroffststr == 0 {
            self.csnroffst = self.frmcsnroffst;
            if self.cplinu[block] {
                self.cplfsnroffst = self.frmfsnroffst;
            }
            for channel in 0..nch {
                self.fsnroffst[channel] = self.frmfsnroffst;
            }
            if header.lfe {
                self.lfefsnroffst = self.frmfsnroffst;
            }
        } else {
            self.snroffste = (eac3 && block == 0) || e.read_bit();
            if self.snroffste {
                self.csnroffst = e.read(6);
                if !eac3 {
                    if self.cplinu[block] {
                        self.cplfsnroffst = e.read(4);
                        self.cplfgaincod = e.read(3);
                    }
                    for channel in 0..nch {
                        self.fsnroffst[channel] = e.read(4);
                        self.fgaincod[channel] = e.read(3);
                    }
                    if header.lfe {
                        self.lfefsnroffst = e.read(4);
                        self.lfefgaincod = e.read(3);
                    }
                } else if self.snroffststr == 1 {
                    self.blkfsnroffst = e.read(4);
                    self.cplfsnroffst = self.blkfsnroffst;
                    for channel in 0..nch {
                        self.fsnroffst[channel] = self.blkfsnroffst;
                    }
                    self.lfefsnroffst = self.blkfsnroffst;
                } else if self.snroffststr == 2 {
                    if self.cplinu[block] {
                        self.cplfsnroffst = e.read(4);
                    }
                    for channel in 0..nch {
                        self.fsnroffst[channel] = e.read(4);
                    }
                    if header.lfe {
                        self.lfefsnroffst = e.read(4);
                    }
                }
            }
        }

        if eac3 {
            self.fgaincode = self.frmfgaincode && e.read_bit();
            if self.fgaincode {
                if self.cplinu[block] {
                    self.cplfgaincod = e.read(3);
                }
                for channel in 0..nch {
                    self.fgaincod[channel] = e.read(3);
                }
                if header.lfe {
                    self.lfefgaincod = e.read(3);
                }
            } else {
                if self.cplinu[block] {
                    self.cplfgaincod = 4;
                }
                for channel in 0..nch {
                    self.fgaincod[channel] = 4;
                }
                if header.lfe {
                    self.lfefgaincod = 4;
                }
            }

            if header.stream_type == StreamType::Independent {
                self.convsnroffste = e.read_bit();
                if self.convsnroffste {
                    self.convsnroffst = e.read(10);
                }
            }
        }

        if self.cplinu[block] {
            if self.firstcplleak {
                self.cplleake = true;
                self.firstcplleak = false;
            } else {
                self.cplleake = e.read_bit();
            }
            if self.cplleake {
                self.cplfleak = e.read(3);
                self.cplsleak = e.read(3);
            }
        }

        // Delta bit allocation
        if self.dbaflde || !eac3 {
            self.deltbaie = e.read_bit();
            if self.deltbaie {
                if self.cplinu[block] {
                    self.cpldeltba.enabled = e.read(2);
                }
                for channel in 0..nch {
                    self.deltba[channel].enabled = e.read(2);
                }
                if self.cplinu[block] && self.cpldeltba.enabled == DBA_NEW_INFO_FOLLOWS {
                    self.cpldeltba.read(e);
                }
                for channel in 0..nch {
                    if self.deltba[channel].enabled == DBA_NEW_INFO_FOLLOWS {
                        self.deltba[channel].read(e);
                    }
                }
            }
        }

        // Error checks
        if block == 0 {
            if !self.cplstre[block] && !UNSAFE {
                return Err(Eac3Error::Decoder(1));
            }
            if header.lfe && !self.lfeexpstr[block] && !UNSAFE {
                return Err(Eac3Error::Decoder(10));
            }
        }
        for channel in 0..nch {
            if block == 0 && self.chexpstr[0][channel] == EXP_REUSE && !UNSAFE {
                return Err(Eac3Error::Decoder(8));
            }
            if !self.chincpl[channel] && self.chbwcod[channel] > 60 && !UNSAFE {
                return Err(Eac3Error::Decoder(11));
            }
        }

        // "Unused dummy data" that might just be used to transport objects
        if self.skip_field_syntax_enabled {
            self.skip_length_enabled = e.read_bit();
            if self.skip_length_enabled {
                self.skip_length = e.read(9);
                e.read_bytes_into(&mut self.aux_data, &mut self.aux_data_pos, self.skip_length);
            }
        }

        // Quantized mantissa values - prepare for the next allocation frame
        self.hist.bap1_pos = 2;
        self.hist.bap2_pos = 2;
        self.hist.bap4_pos = 1;

        if self.cplinu[block] {
            self.allocate_coupling(header);
        }

        let mut got_cplchan = false;
        for channel in 0..nch {
            if self.chahtinu[channel] == 0 {
                self.allocate_channel(header, channel);
                let end = self.endmant[channel];
                self.allocation[channel].read_transform_coeffs(&mut self.extractor, &mut self.channel_output[channel],
                    0, end, &mut self.hist);
            } else {
                return Err(Eac3Error::Unsupported("AHT"));
            }

            if self.cplinu[block] && self.chincpl[channel] && !got_cplchan {
                if self.cplahtinu == 0 {
                    self.coupling_allocation.read_transform_coeffs(&mut self.extractor,
                        &mut self.coupling_transform_coeffs, self.cplstrtmant, self.cplendmant, &mut self.hist);
                    got_cplchan = true;
                } else {
                    return Err(Eac3Error::Unsupported("AHT"));
                }
            }
        }

        // Output
        for channel in 0..nch {
            if self.cplinu[block] && self.chincpl[channel] {
                self.apply_coupling(channel);
            }
            if self.blksw[channel] {
                self.allocation[channel].imdct256(&mut self.channel_output[channel], &mut self.fft);
            } else {
                self.allocation[channel].imdct512(&mut self.channel_output[channel], &mut self.fft);
            }
            self.frame_result[channel][block * 256..(block + 1) * 256]
                .copy_from_slice(&self.channel_output[channel][..256]);
        }

        // Combined mantissa and output handling for LFE
        if header.lfe {
            if self.lfeahtinu == 0 {
                self.allocate_lfe(header);
                self.lfe_allocation.read_transform_coeffs(&mut self.extractor, &mut self.lfe_output, LFESTRTMANT,
                    LFEENDMANT, &mut self.hist);
                self.lfe_allocation.imdct512(&mut self.lfe_output, &mut self.fft);
                if let Some(lfe) = self.lfe_result.as_mut() {
                    lfe[block * 256..(block + 1) * 256].copy_from_slice(&self.lfe_output[..256]);
                }
            } else {
                return Err(Eac3Error::Unsupported("AHT"));
            }
        }
        Ok(())
    }
}
