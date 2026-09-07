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
//!
//! Little-Endian (Intel) byte order is also supported for interoperability.
//! In Intel byte order:
//! - Byte 0 bit 0 is the LSB of the 64-bit frame
//! - Byte 0 bit 7 is the MSB of byte 0
//! - Byte 7 bit 7 is the MSB of the frame

/// A bit reader that extracts fields from an 8-byte CAN frame.
///
/// Supports both Big-Endian (Motorola) and Little-Endian (Intel) byte order.
///
/// Bit indexing follows the Motorola convention where bit position 0 is the MSB
/// of byte 0 (byte 0, bit 7 in standard notation). Position increments
/// toward LSB within each byte, then wraps to the MSB of the next byte.
///
/// For Little-Endian, bit position 0 is the LSB of byte 0.
pub struct BitReader<'a> {
    data: &'a [u8; 8],
}

impl<'a> BitReader<'a> {
    /// Creates a new BitReader from an 8-byte CAN frame.
    pub fn new(data: &'a [u8; 8]) -> Self {
        Self { data }
    }

    /// Reads `len` bits starting from the given Motorola bit position (Big-Endian).
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

    /// Reads a single bit at the given Motorola bit position (Big-Endian).
    pub fn read_bit_be(&self, start_bit: usize) -> bool {
        let byte_index = start_bit / 8;
        let bit_offset = start_bit % 8; // 0 = MSB
        let bit_in_byte = 7 - bit_offset;
        (self.data[byte_index] & (1 << bit_in_byte)) != 0
    }

    /// Reads `len` bits starting from the given Intel bit position (Little-Endian).
    ///
    /// Bit position 0 = byte 0, bit 0 (LSB of the frame).
    /// Bit position 7 = byte 0, bit 7 (MSB of byte 0).
    /// Bit position 8 = byte 1, bit 0 (LSB of byte 1).
    pub fn read_bits_le(&self, start_bit: usize, len: usize) -> u32 {
        let mut value: u32 = 0;
        let mut remaining = len;
        let mut current = start_bit;

        while remaining > 0 {
            let byte_index = current / 8;
            let bit_offset = current % 8; // 0 = LSB within byte
            let bits_available_in_this_byte = 8 - bit_offset;
            let bits_to_take = remaining.min(bits_available_in_this_byte);

            // Extract bits from the byte: bit_offset is the LSB position within byte
            let mask = ((1u32 << bits_to_take) - 1) << bit_offset;
            let extracted = ((self.data[byte_index] as u32) & mask) >> bit_offset;

            value |= extracted << (len - remaining);
            remaining -= bits_to_take;
            current += bits_to_take;
        }

        value
    }

    /// Reads a single bit at the given Intel bit position (Little-Endian).
    pub fn read_bit_le(&self, start_bit: usize) -> bool {
        let byte_index = start_bit / 8;
        let bit_offset = start_bit % 8; // 0 = LSB
        (self.data[byte_index] & (1 << bit_offset)) != 0
    }

    /// Reads `len` bits starting from the given bit position.
    /// Uses the specified byte order (big_endian=true for Motorola, false for Intel).
    pub fn read_bits(&self, start_bit: usize, len: usize, big_endian: bool) -> u32 {
        if big_endian {
            self.read_bits_be(start_bit, len)
        } else {
            self.read_bits_le(start_bit, len)
        }
    }

    /// Reads a single bit at the given bit position with the specified byte order.
    pub fn read_bit(&self, start_bit: usize, big_endian: bool) -> bool {
        if big_endian {
            self.read_bit_be(start_bit)
        } else {
            self.read_bit_le(start_bit)
        }
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

    #[test]
    fn test_read_bits_le() {
        // Little-endian: byte 0 = 0xB2 = 0b10110010
        // LE bit positions: bit0=0(LSB), bit1=1, bit2=0, bit3=0, bit4=1, bit5=1, bit6=0, bit7=1(MSB)
        let data = [0xB2u8, 0, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        // Read 3 bits at position 0 (LE): bits 0,1,2 = 0,1,0 = 0b010 = 2
        assert_eq!(reader.read_bits_le(0, 3), 2);
        // Read 2 bits at position 3 (LE): bits 3,4 = 0,1 → result bit 0 = 0, bit 1 = 1 → 0b10 = 2
        assert_eq!(reader.read_bits_le(3, 2), 2);
        // Read 2 bits at position 5 (LE): bits 5,6 = 1,0 → result bit 0 = 1, bit 1 = 0 → 0b01 = 1
        assert_eq!(reader.read_bits_le(5, 2), 1);
    }

    #[test]
    fn test_read_bits_le_cross_byte() {
        // Little-endian cross-byte read
        let data = [0xFFu8, 0x01, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        // Read 9 bits starting at bit 7 (LE): bit 7 of byte 0 + byte 1 bits 0-7
        // byte 0 = 0xFF, bit 7 (MSB) = 1
        // byte 1 = 0x01, bit 0 (LSB) = 1, rest = 0
        // LE result: bit 0 = 1 (byte 0 bit 7), bit 1 = 1 (byte 1 bit 0), rest 0
        // = 0b11 = 3
        assert_eq!(reader.read_bits_le(7, 9), 3);
    }

    #[test]
    fn test_read_bit_le() {
        let data = [0x01u8, 0, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        // LE: bit 0 is the LSB of byte 0 = 1
        assert!(reader.read_bit_le(0));
        // bit 1 is 0
        assert!(!reader.read_bit_le(1));
    }

    #[test]
    fn test_read_bits_dispatch() {
        let data = [0xB2u8, 0, 0, 0, 0, 0, 0, 0];
        let reader = BitReader::new(&data);
        // Using the dispatch method with big_endian=true should match read_bits_be
        assert_eq!(reader.read_bits(0, 3, true), reader.read_bits_be(0, 3));
        // Using the dispatch method with big_endian=false should match read_bits_le
        assert_eq!(reader.read_bits(0, 3, false), reader.read_bits_le(0, 3));
    }
}
