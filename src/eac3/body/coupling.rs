//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/Coupling.cs (decode paths).

use super::Body;
use crate::eac3::consts::Eac3Error;
use crate::eac3::header::Header;

impl Body {
    /// Decode the (enhanced) coupling strategy information.
    pub(super) fn decode_coupling_strategy(&mut self, header: &Header, eac3: bool, block: usize) {
        let nch = self.channels.len();
        let e = &mut self.extractor;
        if !eac3 {
            self.cplstre[block] = e.read_bit();
            if self.cplstre[block] {
                self.cplinu[block] = e.read_bit();
            }
        }

        if self.cplstre[block] || !eac3 {
            if self.cplinu[block] {
                self.ecplinu = eac3 && e.read_bit();
                if eac3 && header.channel_mode == 2 {
                    self.chincpl[0] = true;
                    self.chincpl[1] = true;
                } else {
                    for channel in 0..nch {
                        self.chincpl[channel] = e.read_bit();
                    }
                }
                if !self.ecplinu {
                    // Standard coupling
                    if header.channel_mode == 2 {
                        self.phsflginu = e.read_bit();
                    }
                    self.cplbegf = e.read(4);
                    if !self.spxinu {
                        self.cplendf = e.read(4);
                    } else {
                        self.cplendf = if self.spxbegf < 6 { self.spxbegf - 2 } else { self.spxbegf * 2 - 7 };
                    }
                    self.ncplsubnd = 3 + self.cplendf - self.cplbegf;
                    self.ncplbnd = self.ncplsubnd;
                    // (DecoderException(3) only in Pedantic mode, which is off by default)
                    self.cplbndstrce = !eac3 || e.read_bit();
                    if self.cplbndstrce {
                        for band in 1..self.ncplsubnd.max(1) {
                            let idx = (self.cplbegf + band) as usize;
                            self.cplbndstrc[idx] = e.read_bit();
                            if self.cplbndstrc[idx] {
                                self.ncplbnd -= 1;
                            }
                        }
                    } else {
                        for band in 1..self.ncplsubnd.max(1) {
                            if self.cplbndstrc[(self.cplbegf + band) as usize] {
                                self.ncplbnd -= 1;
                            }
                        }
                    }
                } else {
                    // Enhanced coupling
                    self.ecplbegf = e.read(4);
                    self.ecpl_begin_subbnd = if self.ecplbegf < 3 {
                        self.ecplbegf * 2
                    } else if self.ecplbegf < 13 {
                        self.ecplbegf + 2
                    } else {
                        self.ecplbegf * 2 - 10
                    };
                    if !self.spxinu {
                        self.ecplendf = e.read(4);
                        self.ecpl_end_subbnd = self.ecplendf + 7;
                    } else {
                        self.ecpl_end_subbnd = if self.spxbegf < 6 { self.spxbegf + 5 } else { self.spxbegf * 2 };
                    }
                    self.ecplbndstrce = e.read_bit();
                    if self.ecplbndstrce {
                        self.ecplbndstrc = [false; 21];
                        for sbnd in 9.max(self.ecpl_begin_subbnd + 1)..self.ecpl_end_subbnd {
                            self.ecplbndstrc[sbnd as usize] = e.read_bit();
                        }
                    }
                }
            } else if eac3 {
                for channel in 0..nch {
                    self.chincpl[channel] = false;
                    self.firstcplcos[channel] = true;
                }
                self.firstcplleak = true;
                self.phsflginu = false;
                self.ecplinu = false;
            }
        }
    }

    /// Decode coupling coordinates and phase flags.
    pub(super) fn decode_coupling_coordinates(&mut self, header: &Header, eac3: bool) -> Result<(), Eac3Error> {
        if self.ecplinu {
            return Err(Eac3Error::Unsupported("ecplinu"));
        }
        let e = &mut self.extractor;
        for channel in 0..self.channels.len() {
            if self.chincpl[channel] {
                if eac3 && self.firstcplcos[channel] {
                    self.cplcoe[channel] = true;
                    self.firstcplcos[channel] = false;
                } else {
                    self.cplcoe[channel] = e.read_bit();
                }
                if self.cplcoe[channel] {
                    self.mstrcplco[channel] = e.read(2) * 3;
                    let mstr = self.mstrcplco[channel];
                    let tcplco = &mut self.cplco[channel];
                    let cplchexp = &mut self.cplcoexp[channel];
                    let cplchmant = &mut self.cplcomant[channel];
                    for band in 0..self.ncplbnd.max(0) as usize {
                        cplchexp[band] = e.read(4);
                        cplchmant[band] = e.read(4);
                        // C# masks shift counts to 5 bits, hence wrapping_shl
                        if cplchexp[band] != 15 {
                            tcplco[band] = (cplchmant[band] + 16).wrapping_shl((15 - cplchexp[band] - mstr) as u32);
                        } else {
                            tcplco[band] = cplchmant[band].wrapping_shl((15 - mstr) as u32);
                        }
                    }
                }
                if header.channel_mode == 2 && self.phsflginu && (self.cplcoe[0] || self.cplcoe[1]) {
                    return Err(Eac3Error::Unsupported("stereo"));
                }
            } else {
                self.firstcplcos[channel] = true;
            }
        }
        Ok(())
    }

    /// Add coupling with the required coordinates to a channel.
    pub(super) fn apply_coupling(&mut self, channel: usize) {
        const ALIGNMENT: f32 = 1.0 / (1 << 17) as f32;
        let channel_mantissa = &mut self.channel_output[channel];
        let coords = &self.cplco[channel];
        let mut used_subband = 0usize;
        for subband in 0..self.ncplsubnd.max(0) {
            // Cavern quirk: the counter is incremented before use, so the first coupling band uses coordinate [1]
            if !self.cplbndstrc[(self.cplbegf + subband) as usize] {
                used_subband += 1;
            }
            let offset = ((self.cplbegf + subband) * 12 + 37) as usize;
            let coord = coords[used_subband] as f32;
            for bin in 0..12 {
                channel_mantissa[bin + offset] = self.coupling_transform_coeffs[bin + offset] * coord * ALIGNMENT;
            }
        }
    }
}
