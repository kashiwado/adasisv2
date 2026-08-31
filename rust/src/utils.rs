#![allow(dead_code)]

//! Utility functions for ADASIS v2 protocol value conversions.

/// Converts a 9-bit speed value from the POSITION message to m/s.
///
/// Value mapping:
/// - 0: ≤ -12.8 m/s
/// - 1..=63: -12.6 to -0.2 m/s (value = (raw - 64) * 0.2)
/// - 64: Standing still (0.0)
/// - 65..=509: +0.2 to +89.0 m/s (value = (raw - 64) * 0.2)
/// - 510: ≥ +89.2 m/s
/// - 511: N/A (None)
pub fn interpret_speed(speed_raw: u16) -> Option<f32> {
    match speed_raw {
        511 => None,
        0 => Some(-12.8),
        64 => Some(0.0),
        510 => Some(89.2),
        1..=63 | 65..=509 => Some((speed_raw as f32 - 64.0) * 0.2),
        _ => None,
    }
}

/// Converts a 9-bit position age value from the POSITION message to milliseconds.
///
/// - 0..=509: value * 5 ms
/// - 510: >= 2545 ms
/// - 511: N/A (None)
pub fn interpret_position_age(age_raw: u16) -> Option<f32> {
    match age_raw {
        511 => None,
        510 => Some(2545.0),
        _ => Some(age_raw as f32 * 5.0),
    }
}

/// Converts an 8-bit relative heading value to degrees.
///
/// Formula: raw * (360 / 254) degrees.
/// - 255: N/A (None)
pub fn interpret_relative_heading(heading_raw: u16) -> Option<f32> {
    if heading_raw == 255 {
        None
    } else {
        Some(heading_raw as f32 * 360.0 / 254.0)
    }
}

/// Converts a 5-bit probability value to a percentage.
///
/// Formula: raw * (100 / 30) percent.
/// - 0: unknown
/// - 31: N/A (None)
pub fn interpret_probability(prob_raw: u8) -> Option<f32> {
    match prob_raw {
        31 => None,
        _ => Some(prob_raw as f32 * 100.0 / 30.0),
    }
}

/// Converts an 8-bit turn angle value from the STUB message to degrees.
///
/// Formula: raw * (360 / 254) degrees.
/// - 255: N/A (None)
pub fn interpret_turn_angle(turn_angle_raw: u8) -> Option<f32> {
    if turn_angle_raw == 255 {
        None
    } else {
        Some(turn_angle_raw as f32 * 360.0 / 254.0)
    }
}

/// Converts a 10-bit heading change value to degrees.
///
/// Formula: raw * (360 / 254) degrees.
/// - 254: unknown
pub fn interpret_heading_change(heading_change_raw: u16) -> Option<f32> {
    if heading_change_raw == 254 {
        None
    } else {
        Some(heading_change_raw as f32 * 360.0 / 254.0)
    }
}

/// Converts a 10-bit slope value to percent using multiplier 0.1 and offset -51.1.
///
/// Formula: (raw * 0.1) - 51.1
/// - 1023: N/A (None)
pub fn interpret_slope(slope_raw: u16) -> Option<f32> {
    if slope_raw == 1023 {
        None
    } else {
        Some(slope_raw as f32 * 0.1 - 51.1)
    }
}

/// Converts a 32-bit longitude value to degrees.
///
/// Formula: (raw * 0.0000001) - 180.0
/// Uses f64 precision as specified.
/// - 0xFFFFFFFF: N/A (None)
pub fn interpret_longitude(lon_raw: u32) -> Option<f64> {
    if lon_raw == 0xFFFFFFFF {
        None
    } else {
        Some(lon_raw as f64 * 0.0000001 - 180.0)
    }
}

/// Converts a 32-bit latitude value to degrees.
///
/// Formula: (raw * 0.0000001) - 90.0
/// Uses f64 precision as specified.
/// - 0xFFFFFFFF: N/A (None)
pub fn interpret_latitude(lat_raw: u32) -> Option<f64> {
    if lat_raw == 0xFFFFFFFF {
        None
    } else {
        Some(lat_raw as f64 * 0.0000001 - 90.0)
    }
}

/// Converts a 32-bit altitude value to meters above WGS84.
///
/// Formula: (raw * 0.01) - 1000.0
/// Allowed range: -1000m to +10000m.
/// - 0xFFFFFFFF: N/A (None)
pub fn interpret_altitude(alt_raw: u32) -> Option<f32> {
    if alt_raw == 0xFFFFFFFF {
        None
    } else {
        Some(alt_raw as f32 * 0.01 - 1000.0)
    }
}

/// Converts a 6-bit map version year raw value to the full year.
///
/// Formula: year = 2000 + raw_value (mod 63)
/// - 63: N/A (None)
pub fn interpret_map_version_year(year_raw: u8) -> Option<u16> {
    if year_raw == 63 {
        None
    } else {
        Some(2000 + year_raw as u16)
    }
}

/// Decodes a 10-bit curvature profile value into a curvature in 1/m.
///
/// Uses the piecewise linear decoding method from section 10.1.2:
/// - A value of 1023 means "unknown" (returns 0.0 as a default).
/// - C = value - 511 (signed integer)
/// - Different scaling factors apply based on the magnitude of C.
pub fn decode_curvature(value: i32) -> f32 {
    // Value 1023 means "unknown"
    if value == 1023 {
        return 0.0;
    }

    let c = value - 511;

    // SIGN(C): -1 if C < 0, +1 if C >= 0
    let sign = if c < 0 { -1.0f32 } else { 1.0f32 };
    let abs_c = c.unsigned_abs() as f32;

    if abs_c <= 64.0 {
        // band 1: C / 100000
        c as f32 / 100000.0
    } else if abs_c <= 128.0 {
        // band 2: 2 * (C - SIGN(C)*32) / 100000
        2.0 * (c as f32 - sign * 32.0) / 100000.0
    } else if abs_c <= 192.0 {
        // band 3: 4 * (C - SIGN(C)*80) / 100000
        4.0 * (c as f32 - sign * 80.0) / 100000.0
    } else if abs_c <= 256.0 {
        // band 4: 8 * (C - SIGN(C)*136) / 100000
        8.0 * (c as f32 - sign * 136.0) / 100000.0
    } else if abs_c <= 320.0 {
        // band 5: 16 * (C - SIGN(C)*196) / 100000
        16.0 * (c as f32 - sign * 196.0) / 100000.0
    } else if abs_c <= 384.0 {
        // band 6: 32 * (C - SIGN(C)*258) / 100000
        32.0 * (c as f32 - sign * 258.0) / 100000.0
    } else if abs_c <= 448.0 {
        // band 7: 64 * (C - SIGN(C)*321) / 100000
        64.0 * (c as f32 - sign * 321.0) / 100000.0
    } else {
        // band 8: 128 * (C - SIGN(C)*384.5) / 100000
        // abs_c <= 511
        128.0 * (c as f32 - sign * 384.5) / 100000.0
    }
}

/// Encodes a curvature value in 1/m into a 10-bit profile value.
///
/// Uses the piecewise linear encoding method from section 10.1.1.
/// A curvature of 0.0 encodes to value 511.
/// Values with |c| >= 0.16192 are clamped to 0 or 1022.
pub fn encode_curvature(curvature: f32) -> u16 {
    let c = curvature * 100000.0;

    if curvature.abs() >= 0.16192 {
        // At the extremes
        if curvature >= 0.0 {
            1022
        } else {
            0
        }
    } else if curvature.abs() < 0.00064 {
        // band 1: ROUND(c * 100000)
        (511.0 + c.round()).round() as u16
    } else if curvature.abs() < 0.00192 {
        // band 2: ROUND((c * 100000) / 2 + SIGN(c) * 32)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 2.0 + sign * 32.0).round()).round() as u16
    } else if curvature.abs() < 0.00448 {
        // band 3: ROUND((c * 100000) / 4 + SIGN(c) * 80)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 4.0 + sign * 80.0).round()).round() as u16
    } else if curvature.abs() < 0.00960 {
        // band 4: ROUND((c * 100000) / 8 + SIGN(c) * 136)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 8.0 + sign * 136.0).round()).round() as u16
    } else if curvature.abs() < 0.01984 {
        // band 5: ROUND((c * 100000) / 16 + SIGN(c) * 196)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 16.0 + sign * 196.0).round()).round() as u16
    } else if curvature.abs() < 0.04032 {
        // band 6: ROUND((c * 100000) / 32 + SIGN(c) * 258)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 32.0 + sign * 258.0).round()).round() as u16
    } else if curvature.abs() < 0.08128 {
        // band 7: ROUND((c * 100000) / 64 + SIGN(c) * 321)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 64.0 + sign * 321.0).round()).round() as u16
    } else {
        // band 8: ROUND((c * 100000) / 128 + SIGN(c) * 384.5)
        let sign = if curvature < 0.0 { -1.0f32 } else { 1.0f32 };
        (511.0 + (c / 128.0 + sign * 384.5).round()).round() as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_curvature_zero() {
        assert_eq!(decode_curvature(511), 0.0);
    }

    #[test]
    fn test_curvature_small_positive() {
        // Band 1: C = 512 - 511 = 1, curvature = 1/100000 = 0.00001
        let c = decode_curvature(512);
        assert!((c - 0.00001).abs() < 1e-8);
    }

    #[test]
    fn test_curvature_small_negative() {
        // C = 510 - 511 = -1, curvature = -1/100000 = -0.00001
        let c = decode_curvature(510);
        assert!((c - (-0.00001)).abs() < 1e-8);
    }

    #[test]
    fn test_curvature_unknown() {
        assert_eq!(decode_curvature(1023), 0.0);
    }

    #[test]
    fn test_curvature_extreme_positive() {
        // Value 1022: C = 1022 - 511 = 511, which is in band 8 (|C| > 448)
        // curvature = 128 * (511 - 384.5) / 100000 = 128 * 126.5 / 100000 = 0.16192
        let c = decode_curvature(1022);
        assert!((c - 0.16192).abs() < 1e-5);
    }

    #[test]
    fn test_curvature_extreme_negative() {
        // Value 0: C = 0 - 511 = -511, which is in band 8 (|C| > 448)
        // curvature = 128 * (-511 - (-384.5)) / 100000 = 128 * (-126.5) / 100000 = -0.16192
        let c = decode_curvature(0);
        assert!((c - (-0.16192)).abs() < 1e-5);
    }
}
