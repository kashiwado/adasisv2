//! Bit reader for extracting bit fields from ADASIS v2 CAN frames.
//!
//! The ADASIS v2 protocol uses Big-Endian (Motorola) byte order as described
//! in section 4.2.1 of the specification. In this format:
//! - Byte 0 bit 7 is the MSB of the 64-bit frame
//! - Byte 0 bit 0 is the LSB of byte 0
//! - Byte 7 bit 0 is the LSB of the frame
//!
//! Multi-bit signals are stored MSB-first within each byte and wrap from
//! the LSB of one byte to the MSB of the next byte.

/// A bit reader that extracts fields from an 8-byte CAN frame using
/// Big-Endian (Motorola) byte order.
///
/// Bit indexing follows the Motorola convention where bit position 0 is the MSB
/// of byte 0 (byte 0, bit 7 in standard notation). Position increments
/// toward LSB within each byte, then wraps to the MSB of the next byte.
pub struct BitReader<'a> {
    data: &'a [u8; 8],
}

impl<'a> BitReader<'a> {
    /// Creates a new BitReader from an 8-byte CAN frame.
    pub fn new(data: &'a [u8; 8]) -> Self {
        Self { data }
    }

    /// Reads `len` bits starting from the given Motorola bit position.
    ///
    /// Bit position 0 = byte 0, bit 7 (MSB of the frame).
    /// Bit position 7 = byte 0, bit 0 (LSB of byte 0).
    /// Bit position 8 = byte 1, bit 7 (MSB of byte 1).
    pub fn read_bits_be(&self, start_bit: usize, len: usize) -> u32 {
        let mut value: u32 = 0;
        let mut remaining = len;
        let mut current = start_bit;

        while remaining > 0 {
            let byte_index = current / 8;
            let bit_offset = current % 8; // 0 = MSB within byte
            let bits_available_in_this_byte = 8 - bit_offset;
            let bits_to_take = remaining.min(bits_available_in_this_byte);

            // The actual bit position within the byte (7 = MSB in standard notation)
            let bit_in_byte = 7 - bit_offset;
            let shift = (bit_in_byte + 1 - bits_to_take) as u32;
            let mask = ((1u32 << bits_to_take) - 1) << shift;
            let extracted = ((self.data[byte_index] as u32) & mask) >> shift;

            value = (value << bits_to_take) | extracted;
            remaining -= bits_to_take;
            current += bits_to_take;
        }

        value
    }

    /// Reads a single bit at the given Motorola bit position.
    pub fn read_bit_be(&self, start_bit: usize) -> bool {
        let byte_index = start_bit / 8;
        let bit_offset = start_bit % 8; // 0 = MSB
        let bit_in_byte = 7 - bit_offset;
        (self.data[byte_index] & (1 << bit_in_byte)) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_single_byte() {
        let data = [0xB2u8, 0, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        assert_eq!(reader.read_bits_be(0, 3), 5);
        assert_eq!(reader.read_bits_be(3, 2), 2);
        assert!(!reader.read_bit_be(5));
        assert_eq!(reader.read_bits_be(6, 2), 2);
    }

    #[test]
    fn test_read_cross_byte() {
        let data = [0xFFu8, 0x80, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        // Read 9 bits starting from bit 7 (byte 0 bit 0 = LSB)
        // Bit 7 = byte 0 bit 0 = 1, then byte 1 bits 7-0 = 10000000
        // So 9 bits: 1_10000000 = 0x180 = 384
        assert_eq!(reader.read_bits_be(7, 9), 0x180);
    }

    #[test]
    fn test_read_full_byte() {
        let data = [0xA5u8, 0, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        assert_eq!(reader.read_bits_be(0, 8), 0xA5);
    }

    #[test]
    fn test_read_all_64_bits() {
        let data = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
        let reader = BitReader::new(&data);
        let val = reader.read_bits_be(0, 32);
        assert_eq!(val, 0x01234567);
    }
}
