//! Port of Cavern.Format/Transcoders/EnhancedAC3Body/BitAllocation.cs and DeltaBitAllocation.cs (decode).

use super::allocation::Allocation;
use super::tables::*;
use crate::bits::BitExtractor;

/// Handling of delta bit allocation data.
pub const DBA_REUSE: i32 = 0;
pub const DBA_NEW_INFO_FOLLOWS: i32 = 1;
pub const DBA_NO_ALLOCATION: i32 = 2;
#[allow(dead_code)]
pub const DBA_MUTE_OUTPUT: i32 = 3;

/// Delta bit allocation information.
#[derive(Clone, Default)]
pub struct DeltaBitAllocation {
    pub enabled: i32,
    pub offset: Vec<i32>,
    pub length: Vec<i32>,
    pub bit_allocation: Vec<i32>,
}

impl DeltaBitAllocation {
    pub fn reset(&mut self) {
        self.enabled = DBA_NO_ALLOCATION;
        self.offset = Vec::new();
        self.length = Vec::new();
        self.bit_allocation = Vec::new();
    }

    pub fn read(&mut self, e: &mut BitExtractor) {
        let segments = (e.read(3) + 1) as usize;
        if self.offset.len() != segments {
            self.offset = vec![0; segments];
            self.length = vec![0; segments];
            self.bit_allocation = vec![0; segments];
        }
        for segment in 0..segments {
            self.offset[segment] = e.read(5);
            self.length[segment] = e.read(4);
            self.bit_allocation[segment] = e.read(3);
        }
    }
}

/// Bit allocation parametric information used by [`allocate`].
#[derive(Clone, Copy)]
pub struct AllocParams {
    pub sdcycod: i32,
    pub fdcycod: i32,
    pub sgaincod: i32,
    pub dbpbcod: i32,
    pub floorcod: i32,
    pub sample_rate_code: i32,
}

#[allow(clippy::too_many_arguments)]
pub fn allocate(p: &AllocParams, start: i32, end: i32, fgain: i32, snroffset: i32, allocation: &mut Allocation,
    dba: &DeltaBitAllocation, mut fastleak: i32, mut slowleak: i32) {
    // Initialization
    let sdecay = SLOWDEC[p.sdcycod as usize];
    let fdecay = FASTDEC[p.fdcycod as usize];
    let sgain = SLOWGAIN[p.sgaincod as usize];
    let dbknee = DBPBTAB[p.dbpbcod as usize];
    let floor = FLOORTAB[p.floorcod as usize];

    // Compute excitation function
    let bndpsd = &allocation.integrated_psd;
    let excite = &mut allocation.excite;
    let bndstrt = MASKTAB[start as usize];
    let bndend = MASKTAB[(end - 1) as usize] + 1;
    let mut begin;
    if bndstrt == 0 {
        // Full bandwidth and LFE channels
        let mut lowcomp = calc_lowcomp(0, bndpsd[0], bndpsd[1], 0);
        excite[0] = bndpsd[0] - fgain - lowcomp;
        lowcomp = calc_lowcomp(lowcomp, bndpsd[1], bndpsd[2], 1);
        excite[1] = bndpsd[1] - fgain - lowcomp;
        begin = 7;
        for bin in 2..7usize {
            if bndend != 7 || bin != 6 {
                lowcomp = calc_lowcomp(lowcomp, bndpsd[bin], bndpsd[bin + 1], bin as i32);
            }
            fastleak = bndpsd[bin] - fgain;
            slowleak = bndpsd[bin] - sgain;
            excite[bin] = fastleak - lowcomp;
            if (bndend != 7 || bin != 6) && bndpsd[bin] <= bndpsd[bin + 1] {
                begin = bin as i32 + 1;
                break;
            }
        }
        let bins = bndend.min(22);
        let mut bin = begin;
        while bin < bins {
            let b = bin as usize;
            if bndend != 7 || bin != 6 {
                lowcomp = calc_lowcomp(lowcomp, bndpsd[b], bndpsd[b + 1], bin);
            }
            fastleak = (fastleak - fdecay).max(bndpsd[b] - fgain);
            slowleak = (slowleak - sdecay).max(bndpsd[b] - sgain);
            excite[b] = (fastleak - lowcomp).max(slowleak);
            bin += 1;
        }
        begin = 22;
    } else {
        // Coupling channel
        begin = bndstrt;
    }
    let mut bin = begin;
    while bin < bndend {
        let b = bin as usize;
        fastleak = (fastleak - fdecay).max(bndpsd[b] - fgain);
        slowleak = (slowleak - sdecay).max(bndpsd[b] - sgain);
        excite[b] = fastleak.max(slowleak);
        bin += 1;
    }

    // Compute masking curve
    let mask = &mut allocation.mask;
    let hth = &HTH[p.sample_rate_code as usize];
    for b in bndstrt as usize..bndend.max(bndstrt) as usize {
        if bndpsd[b] < dbknee {
            excite[b] += (dbknee - bndpsd[b]) >> 2;
        }
        mask[b] = excite[b].max(hth[b]);
    }

    // Apply delta bit allocation
    if dba.enabled == DBA_REUSE || dba.enabled == DBA_NEW_INFO_FOLLOWS {
        let mut band = bndstrt;
        for seg in 0..dba.offset.len() {
            band += dba.offset[seg];
            let ba = dba.bit_allocation[seg];
            let delta = if ba >= 4 { (ba - 3) << 7 } else { (ba - 4) << 7 };
            for _ in 0..dba.length[seg] {
                mask[band as usize] += delta;
                band += 1;
            }
        }
    }

    // Compute bit allocation
    let psd = &allocation.psd;
    let bap = &mut allocation.bap;
    let mut i = start as usize;
    let mut j = MASKTAB[start as usize] as usize;
    loop {
        let lastbin = BNDTAB[j].min(end) as usize;
        let mut masked = mask[j] - snroffset - floor;
        if masked < 0 {
            masked = 0;
        }
        masked = (masked & 0x1fe0) + floor;
        while i < lastbin {
            let address = ((psd[i] - masked) >> 5).clamp(0, 63);
            bap[i] = BAPTAB[address as usize];
            i += 1;
        }
        j += 1;
        if end <= lastbin as i32 {
            break;
        }
    }
    for v in &mut bap[i..] {
        *v = 0;
    }
}

#[inline]
fn calc_lowcomp(a: i32, b0: i32, b1: i32, bin: i32) -> i32 {
    if bin < 7 {
        if b0 + 256 == b1 {
            return 384;
        } else if b0 > b1 {
            return (a - 64).max(0);
        }
    } else if bin < 20 {
        if b0 + 256 == b1 {
            return 320;
        } else if b0 > b1 {
            return (a - 64).max(0);
        }
    } else {
        return (a - 128).max(0);
    }
    a
}
