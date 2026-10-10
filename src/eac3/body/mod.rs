//! Port of Cavern.Format/Transcoders/EnhancedAC3Body.cs and EnhancedAC3Body/Memory.cs, Consts.cs (enums).
//! Renders a single E-AC-3 substream and holds inter-frame data. Encoder paths are not ported.

pub mod allocation;
pub mod bit_allocation;
mod block;
mod coupling;
mod frame;
mod spx;
pub mod tables;

use allocation::{Allocation, MantissaHistory};
use bit_allocation::DeltaBitAllocation;

use super::consts::{DecoderType, Eac3Error};
use super::fft::FftCache;
use super::header::Header;
use crate::bits::BitExtractor;

/// ExpStrat values.
pub const EXP_REUSE: u8 = 0;
pub const EXP_D15: u8 = 1;
#[allow(dead_code)]
pub const EXP_D25: u8 = 2;
pub const EXP_D45: u8 = 3;

/// Absolute maximum band. If arrays are allocated to this size, they can't be overrun.
pub const MAX_ALLOCATION_SIZE: usize = 256;

/// defcplbndstrc
const DEF_CPLBNDSTRC: [bool; 18] =
    [false, false, false, false, false, false, false, false, true, false, true, true, false, true, true, true, true, true];
/// defecplbndstrc
const DEF_ECPLBNDSTRC: [bool; 21] = [false, false, false, false, false, false, false, false, true, false, true, false,
    true, false, true, true, true, false, true, true, true];

/// Renders a single E-AC-3 substream and holds inter-frame data.
#[allow(dead_code)]
pub struct Body {
    /// Full bandwidth channels (Cavern ReferenceChannel values) in bitstream order.
    pub channels: Vec<i32>,
    /// Full bandwidth samples from the last decoded frame.
    pub frame_result: Vec<Vec<f32>>,
    /// LFE samples from the last decoded frame.
    pub lfe_result: Option<Vec<f32>>,

    extractor: BitExtractor,
    caches_created: bool,
    fft: FftCache,
    hist: MantissaHistory,

    allocation: Vec<Allocation>,
    coupling_allocation: Allocation,
    lfe_allocation: Allocation,

    skip_field_syntax_enabled: bool,
    skip_length_enabled: bool,
    skip_length: i32,
    pub(crate) aux_data_pos: usize,
    pub(crate) aux_data: Vec<u8>,

    ahte: bool,
    baie: bool,
    bamode: bool,
    blkstrtinfoe: bool,
    blkswe: bool,
    convexpstre: bool,
    convsnroffste: bool,
    cplbndstrce: bool,
    cplleake: bool,
    dbaflde: bool,
    deltbaie: bool,
    dithflage: bool,
    ecplbndstrce: bool,
    ecplinu: bool,
    expstre: bool,
    fgaincode: bool,
    firstcplleak: bool,
    frmfgaincode: bool,
    phsflginu: bool,
    snroffste: bool,
    spxattene: bool,
    spxbndstrce: bool,
    spxinu: bool,
    spxstre: bool,
    transproce: bool,
    blksw: Vec<bool>,
    chincpl: Vec<bool>,
    chinspx: Vec<bool>,
    chinspxatten: Vec<bool>,
    chintransproc: Vec<bool>,
    cplcoe: Vec<bool>,
    cplinu: Vec<bool>,
    cplstre: Vec<bool>,
    dithflag: Vec<bool>,
    firstcplcos: Vec<bool>,
    firstspxcos: Vec<bool>,
    lfeexpstr: Vec<bool>,
    spxcoe: Vec<bool>,
    cpldeltba: DeltaBitAllocation,
    lfedeltba: DeltaBitAllocation,
    deltba: Vec<DeltaBitAllocation>,
    cplexpstr: Vec<u8>,
    chexpstr: Vec<Vec<u8>>,
    blkfsnroffst: i32,
    blkstrtinfo: i32,
    convsnroffst: i32,
    cplahtinu: i32,
    cplbegf: i32,
    cplendf: i32,
    cplendmant: i32,
    cplfgaincod: i32,
    cplfleak: i32,
    cplfsnroffst: i32,
    cplsleak: i32,
    cplstrtmant: i32,
    csnroffst: i32,
    dbpbcod: i32,
    ecpl_begin_subbnd: i32,
    ecpl_end_subbnd: i32,
    ecplbegf: i32,
    ecplendf: i32,
    ecplendmant: i32,
    ecplstartmant: i32,
    fdcycod: i32,
    floorcod: i32,
    frmcplexpstr: i32,
    frmcsnroffst: i32,
    frmfsnroffst: i32,
    lfeahtinu: i32,
    lfefgaincod: i32,
    lfefsnroffst: i32,
    ncplbnd: i32,
    ncplgrps: i32,
    ncplsubnd: i32,
    nspxbnds: i32,
    sdcycod: i32,
    sgaincod: i32,
    snroffststr: i32,
    spx_begin_subbnd: i32,
    spx_end_subbnd: i32,
    spxbegf: i32,
    spxendf: i32,
    spxstrtf: i32,
    dynrng: Option<i32>,
    dynrng2: Option<i32>,
    chahtinu: Vec<i32>,
    chbwcod: Vec<i32>,
    convexpstr: Vec<i32>,
    endmant: Vec<i32>,
    fgaincod: Vec<i32>,
    frmchexpstr: Vec<i32>,
    fsnroffst: Vec<i32>,
    mstrcplco: Vec<i32>,
    mstrspxco: Vec<i32>,
    nchgrps: Vec<i32>,
    spxattencod: Vec<i32>,
    spxblnd: Vec<i32>,
    transproclen: Vec<i32>,
    transprocloc: Vec<i32>,
    cplco: Vec<Vec<i32>>,
    cplcoexp: Vec<Vec<i32>>,
    cplcomant: Vec<Vec<i32>>,
    spxcoexp: Vec<Vec<i32>>,
    spxcomant: Vec<Vec<i32>>,

    /// PCM output data of each channel (also holds the transform coefficients before the IMDCT).
    channel_output: Vec<Vec<f32>>,
    /// Transform coefficients for the coupling channel's last read audio block.
    coupling_transform_coeffs: Vec<f32>,
    /// PCM output data for the LFE channel's last read audio block.
    lfe_output: Vec<f32>,

    cplbndstrc: [bool; 18],
    ecplbndstrc: [bool; 21],
}

impl Body {
    pub fn new() -> Self {
        Self {
            channels: Vec::new(),
            frame_result: Vec::new(),
            lfe_result: None,
            extractor: BitExtractor::default(),
            caches_created: false,
            fft: FftCache::new(128),
            hist: MantissaHistory::default(),
            allocation: Vec::new(),
            coupling_allocation: Allocation::new(MAX_ALLOCATION_SIZE),
            lfe_allocation: Allocation::new(MAX_ALLOCATION_SIZE),
            skip_field_syntax_enabled: false,
            skip_length_enabled: false,
            skip_length: 0,
            aux_data_pos: 0,
            aux_data: Vec::new(),
            ahte: false,
            baie: false,
            bamode: false,
            blkstrtinfoe: false,
            blkswe: false,
            convexpstre: false,
            convsnroffste: false,
            cplbndstrce: false,
            cplleake: false,
            dbaflde: false,
            deltbaie: false,
            dithflage: false,
            ecplbndstrce: false,
            ecplinu: false,
            expstre: false,
            fgaincode: false,
            firstcplleak: false,
            frmfgaincode: false,
            phsflginu: false,
            snroffste: false,
            spxattene: false,
            spxbndstrce: false,
            spxinu: false,
            spxstre: false,
            transproce: false,
            blksw: Vec::new(),
            chincpl: Vec::new(),
            chinspx: Vec::new(),
            chinspxatten: Vec::new(),
            chintransproc: Vec::new(),
            cplcoe: Vec::new(),
            cplinu: Vec::new(),
            cplstre: Vec::new(),
            dithflag: Vec::new(),
            firstcplcos: Vec::new(),
            firstspxcos: Vec::new(),
            lfeexpstr: Vec::new(),
            spxcoe: Vec::new(),
            cpldeltba: DeltaBitAllocation::default(),
            lfedeltba: DeltaBitAllocation::default(),
            deltba: Vec::new(),
            cplexpstr: Vec::new(),
            chexpstr: Vec::new(),
            blkfsnroffst: 0,
            blkstrtinfo: 0,
            convsnroffst: 0,
            cplahtinu: 0,
            cplbegf: 0,
            cplendf: 0,
            cplendmant: 0,
            cplfgaincod: 0,
            cplfleak: 0,
            cplfsnroffst: 0,
            cplsleak: 0,
            cplstrtmant: 0,
            csnroffst: 0,
            dbpbcod: 0,
            ecpl_begin_subbnd: 0,
            ecpl_end_subbnd: 0,
            ecplbegf: 0,
            ecplendf: 0,
            ecplendmant: 0,
            ecplstartmant: 0,
            fdcycod: 0,
            floorcod: 0,
            frmcplexpstr: 0,
            frmcsnroffst: 0,
            frmfsnroffst: 0,
            lfeahtinu: 0,
            lfefgaincod: 0,
            lfefsnroffst: 0,
            ncplbnd: 0,
            ncplgrps: 0,
            ncplsubnd: 0,
            nspxbnds: 0,
            sdcycod: 0,
            sgaincod: 0,
            snroffststr: 0,
            spx_begin_subbnd: 0,
            spx_end_subbnd: 0,
            spxbegf: 0,
            spxendf: 0,
            spxstrtf: 0,
            dynrng: None,
            dynrng2: None,
            chahtinu: Vec::new(),
            chbwcod: Vec::new(),
            convexpstr: Vec::new(),
            endmant: Vec::new(),
            fgaincod: Vec::new(),
            frmchexpstr: Vec::new(),
            fsnroffst: Vec::new(),
            mstrcplco: Vec::new(),
            mstrspxco: Vec::new(),
            nchgrps: Vec::new(),
            spxattencod: Vec::new(),
            spxblnd: Vec::new(),
            transproclen: Vec::new(),
            transprocloc: Vec::new(),
            cplco: Vec::new(),
            cplcoexp: Vec::new(),
            cplcomant: Vec::new(),
            spxcoexp: Vec::new(),
            spxcomant: Vec::new(),
            channel_output: Vec::new(),
            coupling_transform_coeffs: vec![0.0; MAX_ALLOCATION_SIZE],
            lfe_output: vec![0.0; MAX_ALLOCATION_SIZE],
            cplbndstrc: DEF_CPLBNDSTRC,
            ecplbndstrc: DEF_ECPLBNDSTRC,
        }
    }

    /// Do the mandatory steps before reading the audio blocks.
    pub fn prepare_update(&mut self, header: &Header, extractor: BitExtractor) -> Result<(), Eac3Error> {
        self.extractor = extractor;
        self.channels = header.get_channel_arrangement()?;
        let samples = header.blocks as usize * 256;
        if self.frame_result.len() != self.channels.len() {
            self.frame_result = vec![vec![0.0; samples]; self.channels.len()];
        }
        if self.lfe_result.is_none() && header.lfe {
            self.lfe_result = Some(vec![0.0; samples]);
        }

        // Cavern creates the caches only once per body. Recreating them when the layout changes deviates
        // only where Cavern would index out of range.
        if !self.caches_created || self.blksw.len() != self.channels.len() || self.cplinu.len() != header.blocks as usize {
            self.create_cache_tables(header.blocks as usize, self.channels.len());
            self.caches_created = true;
        }
        if header.decoder == DecoderType::Eac3 {
            self.decode_audio_frame(header)?;
        } else {
            self.blkswe = true;
            self.dithflage = true;
            self.bamode = true;
            self.snroffststr = -1;
            self.frmfgaincode = false;
            self.firstcplleak = false;
            self.dbaflde = true;
            self.skip_field_syntax_enabled = true;
        }
        Ok(())
    }

    /// The auxiliary data collected during the last [`Body::update`] (`auxData[0..auxDataPos]`).
    pub fn aux_data(&self) -> &[u8] { &self.aux_data[..self.aux_data_pos] }

    /// Decode all audio blocks and the auxiliary data field.
    pub fn update(&mut self, header: &Header) -> Result<(), Eac3Error> {
        self.aux_data_pos = 0;
        for block in 0..header.blocks as usize {
            self.decode_audio_block(header, block)?;
        }
        self.read_aux();
        Ok(())
    }

    /// Read the auxiliary data field and add it to `aux_data`.
    fn read_aux(&mut self) {
        let e = &mut self.extractor;
        e.position = e.back_position - 32;
        let aux_length = e.read(14);
        if e.read_bit() {
            // Cavern quirk: auxdatal is a bit count, but Cavern reads that many bytes
            e.position = e.back_position - 32 - aux_length;
            e.read_bytes_into(&mut self.aux_data, &mut self.aux_data_pos, aux_length);
        }
    }

    fn create_cache_tables(&mut self, blocks: usize, channels: usize) {
        self.blksw = vec![false; channels];
        self.chahtinu = vec![0; channels];
        self.chbwcod = vec![0; channels];
        self.chexpstr = vec![vec![0; channels]; blocks];
        self.chincpl = vec![false; channels];
        self.chinspx = vec![false; channels];
        self.chinspxatten = vec![false; channels];
        self.chintransproc = vec![false; channels];
        self.channel_output = vec![vec![0.0; MAX_ALLOCATION_SIZE]; channels];
        self.convexpstr = vec![0; channels];
        self.cplcoe = vec![false; channels];
        self.cplco = vec![vec![0; DEF_CPLBNDSTRC.len()]; channels];
        self.cplcoexp = vec![vec![0; DEF_CPLBNDSTRC.len()]; channels];
        self.cplcomant = vec![vec![0; DEF_CPLBNDSTRC.len()]; channels];
        self.cplexpstr = vec![0; blocks];
        self.cplinu = vec![false; blocks];
        self.cplstre = vec![false; blocks];
        self.deltba = vec![DeltaBitAllocation::default(); channels];
        self.dithflag = vec![false; channels];
        self.endmant = vec![0; channels];
        self.fgaincod = vec![0; channels];
        self.firstcplcos = vec![false; channels];
        self.firstspxcos = vec![false; channels];
        self.frmchexpstr = vec![0; channels];
        self.fsnroffst = vec![0; channels];
        self.lfeexpstr = vec![false; blocks];
        self.mstrcplco = vec![0; channels];
        self.mstrspxco = vec![0; channels];
        self.nchgrps = vec![0; channels];
        self.spxattencod = vec![0; channels];
        self.spxblnd = vec![0; channels];
        self.spxcoe = vec![false; channels];
        self.spxcoexp = vec![Vec::new(); channels];
        self.spxcomant = vec![Vec::new(); channels];
        self.transproclen = vec![0; channels];
        self.transprocloc = vec![0; channels];

        self.allocation = (0..channels).map(|_| Allocation::new(MAX_ALLOCATION_SIZE)).collect();
        self.coupling_allocation = Allocation::new(MAX_ALLOCATION_SIZE);
        self.lfe_allocation = Allocation::new(MAX_ALLOCATION_SIZE);
        self.cpldeltba.reset();
        self.lfedeltba.reset();
        for d in &mut self.deltba {
            d.reset();
        }
    }
}

impl Default for Body {
    fn default() -> Self { Self::new() }
}
