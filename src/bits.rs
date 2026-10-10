//! Bit reader, a port of Cavern's `BitExtractor` (Cavern.Format/Utilities/BitExtractor.cs).
//! MSB-first. Semantics (including quirks) match Cavern exactly so decoded output is identical.

#[derive(Clone, Default)]
pub struct BitExtractor {
    /// Current bit position.
    pub position: i32,
    /// Bit position after the last valid bit.
    pub back_position: i32,
    pub source: Vec<u8>,
}

impl BitExtractor {
    pub fn new(source: Vec<u8>) -> Self {
        let back = (source.len() * 8) as i32;
        Self { position: 0, back_position: back, source }
    }

    /// Cavern's `BitExtractor(byte[] source, int lastByte)`.
    pub fn with_last_byte(source: Vec<u8>, last_byte: usize) -> Self {
        Self { position: 0, back_position: (last_byte * 8) as i32, source }
    }

    pub fn readable(&self) -> bool { true }

    pub fn clear(&mut self) { self.position = self.back_position; }

    #[inline]
    fn byte(&self, i: i32) -> i32 {
        // Cavern would throw on out-of-range reads; reading zeros keeps decoding going.
        *self.source.get(i as usize).unwrap_or(&0) as i32
    }

    #[inline]
    pub fn read(&mut self, mut bits: i32) -> i32 {
        let mut result: i32 = 0;
        while bits > 0 {
            let removed_left = self.position & 7;
            let mut removed_right = 0;
            if removed_left + bits < 8 {
                removed_right = 8 - removed_left - bits;
            }
            let shift_back = removed_left + removed_right;
            let read_bits = 8 - shift_back;
            result = (result << read_bits)
                .wrapping_add(((self.byte(self.position >> 3) << removed_left) & 0xFF) >> shift_back);
            bits -= read_bits;
            self.position += read_bits;
        }
        result
    }

    pub fn read_conditional(&mut self, bits: i32) -> Option<i32> {
        if self.read_bit_int() != 0 { Some(self.read(bits)) } else { None }
    }

    /// Port of Cavern's `ReadSigned`: `sign << (31 - bits) + value - sign` where C# `+` binds
    /// tighter than `<<`, and `sign = value & (1 << bits)` is always 0 for a `bits`-wide value.
    /// So Cavern always returns 0 here. Kept identical for output parity with Cavern.
    pub fn read_signed(&mut self, bits: i32) -> i32 {
        let value = self.read(bits);
        let sign = value & (1 << bits);
        let shift = (31 - bits) + value - sign;
        sign.wrapping_shl(shift as u32 & 31)
    }

    #[inline]
    pub fn read_bit(&mut self) -> bool { self.read_bit_int() == 1 }

    #[inline]
    pub fn read_bit_int(&mut self) -> i32 {
        let v = (self.byte(self.position >> 3) >> (7 - (self.position & 7))) & 1;
        self.position += 1;
        v
    }

    /// Cavern's `ReadBits`: fills from the end (result[bits-1] is read first).
    pub fn read_bits(&mut self, bits: usize) -> Vec<bool> {
        let mut result = vec![false; bits];
        let mut b = bits;
        while b > 0 {
            b -= 1;
            result[b] = self.read_bit();
        }
        result
    }

    pub fn read_bits_into(&mut self, buffer: &mut Vec<u8>, count: i32) {
        let full = count >> 3;
        let rem = count & 7;
        let needed = (full + if rem != 0 { 1 } else { 0 }) as usize;
        if needed > buffer.len() { buffer.resize(needed, 0); }
        for i in 0..full as usize { buffer[i] = self.read(8) as u8; }
        if rem != 0 { buffer[full as usize] = (self.read(rem) << (8 - rem)) as u8; }
    }

    /// Cavern's `ReadBytesInto`: reads `count` whole bytes and appends them at `offset`.
    pub fn read_bytes_into(&mut self, buffer: &mut Vec<u8>, offset: &mut usize, count: i32) {
        if *offset + count.max(0) as usize > buffer.len() { buffer.resize(*offset + count.max(0) as usize, 0); }
        for _ in 0..count.max(0) {
            buffer[*offset] = self.read(8) as u8;
            *offset += 1;
        }
    }

    #[inline]
    pub fn skip(&mut self, count: i32) { self.position += count; }

    pub fn at(&self, index: usize) -> u8 { self.source[index] }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reads() {
        let mut b = BitExtractor::new(vec![0b1011_0011, 0b0101_1100]);
        assert_eq!(b.read(3), 0b101);
        assert_eq!(b.read(7), 0b1_0011_01);
        assert!(!b.read_bit()); // 11th bit is 0
        assert_eq!(b.read_signed(3), 0);
    }
}
