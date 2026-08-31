//! Bit writer for serializing ADASIS v2 messages to CAN frames.
//!
//! Writes fields in Big-Endian (Motorola) byte order as specified in section 4.2.1
//! of the ADASIS v2 specification.

/// A bit writer that serializes fields into an 8-byte CAN frame using
/// Big-Endian (Motorola) byte order.
///
/// Bit indexing follows the Motorola convention where bit position 0 is the MSB
/// of byte 0 (byte 0, bit 7 in standard notation). Position increments
/// toward LSB within each byte, then wraps to the MSB of the next byte.
pub struct BitWriter {
    data: [u8; 8],
    current_bit: usize,
}

impl BitWriter {
    /// Creates a new BitWriter initialized to all zeros.
    pub fn new() -> Self {
        Self {
            data: [0u8; 8],
            current_bit: 0,
        }
    }

    /// Writes a 1-bit boolean value at the current position.
    pub fn write_bit_be(&mut self, value: bool) {
        self.write_bits_be(if value { 1u32 } else { 0u32 }, 1);
    }

    /// Writes `len` bits of `value` starting at the current position, MSB-first.
    /// Accepts any type that can be converted to u32.
    pub fn write_bits_be<V: Into<u32>>(&mut self, value: V, len: usize) {
        let mut val: u32 = value.into();
        let mut remaining = len;

        while remaining > 0 {
            let byte_index = self.current_bit / 8;
            let bit_offset = self.current_bit % 8; // 0 = MSB within byte
            let bits_available_in_this_byte = 8 - bit_offset;
            let bits_to_write = remaining.min(bits_available_in_this_byte);

            // Extract the top bits_to_write bits from val (relative to remaining)
            // If remaining > bits_to_write, we need the top bits
            // If remaining == bits_to_write, we take all of val
            let shift = remaining - bits_to_write;
            let bits_to_write_now = bits_to_write;
            let extracted = if shift >= 32 {
                0
            } else if shift == 0 {
                val & ((1u32 << bits_to_write_now) - 1)
            } else {
                (val >> shift) & ((1u32 << bits_to_write_now) - 1)
            };

            // Write into the byte at the correct position
            let bit_in_byte = 7 - bit_offset; // actual bit position (7=MSB)
            let shift_in_byte = (bit_in_byte + 1 - bits_to_write_now) as u32;
            let mask: u32 = !(((1u32 << bits_to_write_now) - 1) << shift_in_byte);
            let byte_val = self.data[byte_index] as u32;
            let updated = (byte_val & mask) | (extracted << shift_in_byte);
            self.data[byte_index] = updated as u8;

            // Advance
            remaining -= bits_to_write;
            val &= (1u32 << shift).wrapping_sub(1); // clear the bits we wrote
            if shift == 0 {
                val = 0; // no more bits to process
            }
            self.current_bit += bits_to_write;
        }
    }

    /// Converts the writer into the final 8-byte CAN frame.
    pub fn into_bytes(self) -> [u8; 8] {
        self.data
    }
}

impl Default for BitWriter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_single_byte() {
        let mut writer = BitWriter::new();
        writer.write_bits_be(5u8, 3); // 101
        writer.write_bits_be(2u8, 2); // 10
        writer.write_bit_be(false); // 0
        writer.write_bits_be(2u8, 2); // 10
        let bytes = writer.into_bytes();
        assert_eq!(bytes[0], 0xB2);
    }

    #[test]
    fn test_write_all_64_bits() {
        // u32 can only hold 32 bits, so test with 32-bit values
        let mut writer2 = BitWriter::new();
        writer2.write_bits_be(0x01234567u32, 32);
        let bytes2 = writer2.into_bytes();
        assert_eq!(bytes2[0], 0x01);
        assert_eq!(bytes2[1], 0x23);
        assert_eq!(bytes2[2], 0x45);
        assert_eq!(bytes2[3], 0x67);
    }

    #[test]
    fn test_write_bit_positions() {
        let mut writer = BitWriter::new();
        writer.write_bits_be(1u8, 1); // bit 0 (byte 0, bit 7)
        writer.write_bits_be(0u8, 1); // bit 1
        writer.write_bits_be(1u8, 1); // bit 2
        let bytes = writer.into_bytes();
        // 101xxxxx = 0b10100000 = 0xA0
        assert_eq!(bytes[0], 0xA0);
    }

    #[test]
    fn test_roundtrip() {
        let original = [0xDEu8, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78];
        let reader = crate::bitreader::BitReader::new(&original);
        let val = reader.read_bits_be(0, 32);

        let mut writer = BitWriter::new();
        writer.write_bits_be(val, 32);
        writer.write_bits_be(reader.read_bits_be(32, 32), 32);
        let result = writer.into_bytes();

        assert_eq!(result, original);
    }

    #[test]
    fn test_write_u8_and_u16() {
        let mut writer = BitWriter::new();
        writer.write_bits_be(0u8, 5); // 5 bits zero
        writer.write_bits_be(300u16, 9); // 9 bits
        let bytes = writer.into_bytes();
        // First 5 bits: 00000
        // Next 9 bits: 100101100 (300 in binary)
        // Byte 0: 00000100 = 0x04
        // Byte 1: 10110000 = 0xB0
        assert_eq!(bytes[0], 0x04);
        assert_eq!(bytes[1], 0xB0);
    }
}
