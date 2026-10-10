//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/SPX.cs and Parsers.cs (decode paths).
//! Note: like Cavern, spectral extension metadata is only parsed, the high band is not synthesized.

use super::tables::ECPLSUBBNDTAB;
use super::Body;
use crate::eac3::header::Header;

/// Addition for each ExpStrat to calculate group sizes.
const GROUP_ADD: [i32; 3] = [-1, 2, 8];
/// Divider of each ExpStrat to calculate group sizes.
const GROUP_DIV: [i32; 3] = [3, 6, 12];

impl Body {
    /// Read spectral extension metadata for a given block.
    pub(super) fn read_spx(&mut self, header: &Header, block: usize) {
        let nch = self.channels.len();
        let e = &mut self.extractor;
        self.spxstre = block == 0 || e.read_bit();
        if self.spxstre {
            self.spxinu = e.read_bit();
            if self.spxinu {
                if header.channel_mode == 1 {
                    self.chinspx[0] = true;
                } else {
                    for channel in 0..nch {
                        self.chinspx[channel] = e.read_bit();
                    }
                }
                self.spxstrtf = e.read(2);
                self.spxbegf = e.read(3);
                self.spxendf = e.read(3);
                self.spx_begin_subbnd = if self.spxbegf < 6 { self.spxbegf + 2 } else { self.spxbegf * 2 - 3 };
                self.spx_end_subbnd = if self.spxendf < 3 { self.spxendf + 5 } else { self.spxendf * 2 + 3 };
                self.spxbndstrce = e.read_bit();
                if self.spxbndstrce {
                    // Cavern reads spxbndstrc here, but ParseSPX replaces it with a fresh all-false array
                    for _band in self.spx_begin_subbnd + 1..self.spx_end_subbnd {
                        e.read_bit();
                    }
                }
                self.parse_spx();
            } else {
                self.clear_spx();
            }
        }

        // Coordinates
        if self.spxinu {
            let e = &mut self.extractor;
            for channel in 0..nch {
                if self.chinspx[channel] {
                    if self.firstspxcos[channel] {
                        self.spxcoe[channel] = true;
                        self.firstspxcos[channel] = false;
                    } else {
                        self.spxcoe[channel] = e.read_bit();
                    }
                    if self.spxcoe[channel] {
                        self.spxblnd[channel] = e.read(5);
                        self.mstrspxco[channel] = e.read(2);
                        for band in 0..self.nspxbnds.max(0) as usize {
                            self.spxcoexp[channel][band] = e.read(4);
                            self.spxcomant[channel][band] = e.read(2);
                        }
                    }
                } else {
                    self.firstspxcos[channel] = true;
                }
            }
        }
    }

    /// Set up SPX as unused.
    pub(super) fn clear_spx(&mut self) {
        for channel in 0..self.channels.len() {
            self.chinspx[channel] = false;
            self.firstspxcos[channel] = true;
        }
    }

    /// Parse spectral extension data (Parsers.cs ParseSPX). With Cavern's fresh all-false spxbndstrc,
    /// every sub-band after the first starts a new band.
    fn parse_spx(&mut self) {
        self.nspxbnds = 1;
        for _bnd in self.spx_begin_subbnd + 1..self.spx_end_subbnd {
            self.nspxbnds += 1;
        }
        for channel in 0..self.channels.len() {
            self.spxcoexp[channel] = vec![0; self.nspxbnds as usize];
            self.spxcomant[channel] = vec![0; self.nspxbnds as usize];
        }
    }

    /// Set endmant and nchgrps (Parsers.cs ParseParametricBitAllocation).
    pub(super) fn parse_parametric_bit_allocation(&mut self, block: usize) {
        if self.cplinu[block] {
            self.cplstrtmant = 37 + 12 * self.cplbegf;
            self.cplendmant = 37 + 12 * (self.cplendf + 3);
            if self.cplexpstr[block] != super::EXP_REUSE {
                let div = GROUP_DIV[self.cplexpstr[block] as usize - 1];
                if self.ecplinu {
                    self.ecplstartmant = ECPLSUBBNDTAB[self.ecpl_begin_subbnd as usize];
                    self.ecplendmant = ECPLSUBBNDTAB[self.ecpl_end_subbnd as usize];
                    self.ncplgrps = (self.ecplendmant - self.ecplstartmant) / div;
                } else {
                    self.ncplgrps = (self.cplendmant - self.cplstrtmant) / div;
                }
            }
        }

        for channel in 0..self.channels.len() {
            if self.ecplinu {
                self.endmant[channel] = ECPLSUBBNDTAB[self.ecpl_begin_subbnd as usize];
            } else if self.spxinu && !self.cplinu[block] {
                self.endmant[channel] = self.spx_begin_subbnd * 12 + 25;
            } else if self.chincpl[channel] {
                self.endmant[channel] = self.cplstrtmant;
            } else {
                self.endmant[channel] = (self.chbwcod[channel] + 12) * 3 + 37;
            }

            let strat = self.chexpstr[block][channel] as usize;
            if strat != 0 {
                self.nchgrps[channel] = (self.endmant[channel] + GROUP_ADD[strat - 1]) / GROUP_DIV[strat - 1];
            }
        }
    }
}
