//! # ADASIS v2 Protocol Library
//!
//! This library implements serialization and deserialization of ADASIS v2
//! protocol messages as defined in the specification version 2.0.5.0 (March 2024).
//!
//! Messages include POSITION, SEGMENT, STUB, PROFILE_SHORT, PROFILE_LONG, and
//! META_DATA, along with their interpreted subtypes for profile messages.
//!
//! The library works with `[u8; 8]` payloads (e.g. CAN messages) using a
//! Big-Endian (Motorola) byte order.

mod bitreader; // Bitstream deserialization
mod bitwriter; // Bitstream serialization
mod utils; // Helper functions for the Horizon Reconstructor etc

pub use bitreader::BitReader;
pub use bitwriter::BitWriter;
pub use utils::{decode_curvature, encode_curvature};

// ============================================================================
// CONSTANTS
// ============================================================================

/// Invalid profile long payload value (all 1s in 32-bit).
pub const PROFILE_VALUE_INVALID_32: u32 = 0xFFFFFFFF;
/// Invalid profile short payload value (all 1s in 10-bit).
pub const PROFILE_VALUE_INVALID_10: u16 = 0x3FF;

// ============================================================================
// MESSAGE TYPE
// ============================================================================

/// Message type identifier extracted from the header of an ADASIS v2 message.
///
/// Defined in Table 1 of the ADASIS v2 specification (section 4.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum MessageType {
    /// System Specific message (type 0).
    SystemSpecific = 0,
    /// POSITION message (type 1).
    Position = 1,
    /// SEGMENT message (type 2).
    Segment = 2,
    /// STUB message (type 3).
    Stub = 3,
    /// PROFILE SHORT message (type 4).
    ProfileShort = 4,
    /// PROFILE LONG message (type 5).
    ProfileLong = 5,
    /// META-DATA message (type 6).
    MetaData = 6,
    /// Reserved message type (type 7).
    Reserved = 7,
}

impl MessageType {
    /// Returns the `MessageType` corresponding to the given raw 3-bit value.
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => MessageType::SystemSpecific,
            1 => MessageType::Position,
            2 => MessageType::Segment,
            3 => MessageType::Stub,
            4 => MessageType::ProfileShort,
            5 => MessageType::ProfileLong,
            6 => MessageType::MetaData,
            _ => MessageType::Reserved,
        }
    }

    /// Returns the raw 3-bit value for this message type.
    pub fn as_raw(&self) -> u8 {
        *self as u8
    }
}

/// Extracts the message type identifier from the header of an 8-byte ADASIS v2 CAN frame.
///
/// The message type occupies the top 3 bits of byte 0 (bits 7-5) in Big-Endian
/// (Motorola) byte order.
pub fn get_message_type(data: &[u8; 8], big_endian: bool) -> MessageType {
    let message_type_bits = if big_endian {
        (data[0] >> 5) & 0x07
    } else {
        data[0] & 0x07
    };
    MessageType::from_raw(message_type_bits)
}

// ============================================================================
// HEADER
// ============================================================================

/// Common ADASIS v2 message header fields present at the start of every message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdasisHeader {
    /// 3-bit message type field.
    pub message_type: MessageType,
    /// 2-bit cyclic counter for detecting missing messages.
    pub cyclic_counter: u8,
}

impl AdasisHeader {
    /// Parses the common header from raw bytes.
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);
        Self {
            message_type: get_message_type(data, big_endian),
            cyclic_counter: reader.read_bits(3, 2, big_endian) as u8,
        }
    }

    /// Serializes the header into the first byte.
    pub fn to_byte0(&self) -> u8 {
        (self.message_type.as_raw() << 5) | ((self.cyclic_counter & 0x03) << 3)
    }
}

// ============================================================================
// SHARED ENUMS
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathIdType {
    Unknown,
    PositionNotInDigitizedArea,
    PositionNotOnRoad,
    PositionNotCalibrated,
    SegmentAdasisMini,
    StubSubPathCrossroadNoMoreInfo,
    StubSubPathIntersectionHeadingChange,
    Reserved,
    InvalidForMessage,
    Normal,
}

/// Functional Road Class values used in SEGMENT and STUB messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FunctionalRoadClass {
    Unknown = 0,
    Class1 = 1,
    Class2 = 2,
    Class3 = 3,
    Class4 = 4,
    Class5 = 5,
    Class6 = 6,
    Na = 7,
}

impl FunctionalRoadClass {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => FunctionalRoadClass::Unknown,
            1 => FunctionalRoadClass::Class1,
            2 => FunctionalRoadClass::Class2,
            3 => FunctionalRoadClass::Class3,
            4 => FunctionalRoadClass::Class4,
            5 => FunctionalRoadClass::Class5,
            6 => FunctionalRoadClass::Class6,
            _ => FunctionalRoadClass::Na,
        }
    }
}

/// Form of Way values used in SEGMENT and STUB messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FormOfWay {
    Unknown = 0,
    ControlledAccess = 1,
    MultipleCarriageway = 2,
    SingleCarriageway = 3,
    Roundabout = 4,
    TrafficSquare = 5,
    Reserved1 = 6,
    Reserved2 = 7,
    ParallelRoad = 8,
    SlipRoadFreeway = 9,
    SlipRoad = 10,
    ServiceRoad = 11,
    CarPark = 12,
    ServiceStation = 13,
    PedestrianZone = 14,
    Na = 15,
}

impl FormOfWay {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => FormOfWay::Unknown,
            1 => FormOfWay::ControlledAccess,
            2 => FormOfWay::MultipleCarriageway,
            3 => FormOfWay::SingleCarriageway,
            4 => FormOfWay::Roundabout,
            5 => FormOfWay::TrafficSquare,
            6 => FormOfWay::Reserved1,
            7 => FormOfWay::Reserved2,
            8 => FormOfWay::ParallelRoad,
            9 => FormOfWay::SlipRoadFreeway,
            10 => FormOfWay::SlipRoad,
            11 => FormOfWay::ServiceRoad,
            12 => FormOfWay::CarPark,
            13 => FormOfWay::ServiceStation,
            14 => FormOfWay::PedestrianZone,
            _ => FormOfWay::Na,
        }
    }
}

/// Speed Limit Type values used in SEGMENT message (Table 11).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SpeedLimitType {
    Implicit = 0,
    ExplicitSign = 1,
    ExplicitNight = 2,
    ExplicitDay = 3,
    ExplicitTimeOfDay = 4,
    ExplicitRain = 5,
    ExplicitSnow = 6,
    Unknown = 7,
}

impl SpeedLimitType {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => SpeedLimitType::Implicit,
            1 => SpeedLimitType::ExplicitSign,
            2 => SpeedLimitType::ExplicitNight,
            3 => SpeedLimitType::ExplicitDay,
            4 => SpeedLimitType::ExplicitTimeOfDay,
            5 => SpeedLimitType::ExplicitRain,
            6 => SpeedLimitType::ExplicitSnow,
            _ => SpeedLimitType::Unknown,
        }
    }
}

/// Current Lane values used in POSITION message (Table 7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CurrentLane {
    Unknown = 0,
    EmergencyLane = 1,
    SingleLane = 2,
    LeftMost = 3,
    RightMost = 4,
    MiddleLane = 5,
    Reserved = 6,
    Na = 7,
}

impl CurrentLane {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => CurrentLane::Unknown,
            1 => CurrentLane::EmergencyLane,
            2 => CurrentLane::SingleLane,
            3 => CurrentLane::LeftMost,
            4 => CurrentLane::RightMost,
            5 => CurrentLane::MiddleLane,
            6 => CurrentLane::Reserved,
            _ => CurrentLane::Na,
        }
    }
}

/// Driving Side values used in META-DATA message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DrivingSide {
    Left = 0,
    Right = 1,
}

/// Speed Unit values used in META-DATA message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SpeedUnit {
    Kmh = 0,
    Mph = 1,
}

/// Map Provider values used in META-DATA message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MapProvider {
    Unknown = 0,
    Here = 1,
    Tomtom = 2,
    Zenrin = 3,
    Ipc = 4,
    Navinfo = 5,
    Other = 6,
    Na = 7,
}

impl MapProvider {
    pub fn from_raw(value: u8) -> Self {
        match value {
            0 => MapProvider::Unknown,
            1 => MapProvider::Here,
            2 => MapProvider::Tomtom,
            3 => MapProvider::Zenrin,
            4 => MapProvider::Ipc,
            5 => MapProvider::Navinfo,
            6 => MapProvider::Other,
            _ => MapProvider::Na,
        }
    }
}

// ============================================================================
// POSITION MESSAGE
// ============================================================================

/// Position message describing the current vehicle position in relation to
/// the ADASIS v2 Horizon Paths.
///
/// Total: 64 bits (8 bytes), see Table 5 of the specification.
#[derive(Debug, Clone, PartialEq)]
pub struct PositionMessage {
    pub header: AdasisHeader,
    /// Index of current path (0-7 have special meaning, see Table 6 in ADASIS v2.0.5.
    pub path_index: u8,
    /// Offset of current position from the current path's start point (0-8190, 8191=invalid).
    pub offset: u16,
    /// Index of position candidate (0-3).
    pub position_index: u8,
    /// Time difference in milliseconds between position calculation and message send.
    /// Value is scaled by 5 ms. 511 = N/A.
    pub position_age: u16,
    /// Speed of the vehicle projected to the path. Value is scaled by 0.2 m/s.
    pub speed: u16,
    /// Heading relative to path. Value is scaled by 360/254 degrees. 255 = N/A.
    pub relative_heading: u16,
    /// Position probability (0-30, where value means percentage of 100/30). 0=unknown, 31=N/A.
    pub probability: u8,
    /// Position confidence (0=highest, 6=lowest, 7=N/A).
    pub confidence: u8,
    /// Current lane (see [CurrentLane]). 7 = N/A.
    pub current_lane: u8,
    /// Reserved bit.
    pub reserved: u8,
}

impl PositionMessage {
    /// Evaluates a POSITION message path index for if it has a reserved value
    pub fn get_path_id_type(&self) -> PathIdType {
        match self.path_index {
            0 => PathIdType::Unknown,
            1 => PathIdType::PositionNotInDigitizedArea,
            2 => PathIdType::PositionNotOnRoad,
            3 => PathIdType::PositionNotCalibrated,
            4_u8..=6_u8 => PathIdType::InvalidForMessage,
            7 => PathIdType::Reserved,
            _ => PathIdType::Normal,
        }
    }

    /// Deserializes a POSITION message from an 8-byte CAN frame (Big-Endian/Motorola).
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);

        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let path_index = reader.read_bits(5, 6, big_endian) as u8;
        let offset = reader.read_bits(11, 13, big_endian) as u16;
        let position_index = reader.read_bits(24, 2, big_endian) as u8;
        let position_age = reader.read_bits(26, 9, big_endian) as u16;
        let speed = reader.read_bits(35, 9, big_endian) as u16;
        let relative_heading = reader.read_bits(44, 8, big_endian) as u16;
        let probability = reader.read_bits(52, 5, big_endian) as u8;
        let confidence = reader.read_bits(57, 3, big_endian) as u8;
        let current_lane = reader.read_bits(60, 3, big_endian) as u8;
        let reserved = reader.read_bits(63, 1, big_endian) as u8;

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            path_index,
            offset,
            position_index,
            position_age,
            speed,
            relative_heading,
            probability,
            confidence,
            current_lane,
            reserved,
        }
    }

    /// Serializes the position message into an 8-byte CAN frame (Big-Endian/Motorola).
    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bits(self.path_index, 6, big_endian);
        writer.write_bits(self.offset, 13, big_endian);
        writer.write_bits(self.position_index, 2, big_endian);
        writer.write_bits(self.position_age, 9, big_endian);
        writer.write_bits(self.speed, 9, big_endian);
        writer.write_bits(self.relative_heading, 8, big_endian);
        writer.write_bits(self.probability, 5, big_endian);
        writer.write_bits(self.confidence, 3, big_endian);
        writer.write_bits(self.current_lane, 3, big_endian);
        writer.write_bits(self.reserved, 1, big_endian);
        writer.into_bytes()
    }

    /// Returns the interpreted speed in m/s. 511 = N/A (None).
    pub fn speed_mps(&self) -> Option<f32> {
        match self.speed {
            511 => None,
            0 => Some(-12.8),
            64 => Some(0.0),
            510 => Some(89.2),
            1..=63 | 65..=509 => Some((self.speed as f32 - 64.0) * 0.2),
            _ => None,
        }
    }

    /// Returns the interpreted position age in milliseconds.
    pub fn position_age_ms(&self) -> Option<f32> {
        match self.position_age {
            511 => None,
            510 => Some(2545.0),
            _ => Some(self.position_age as f32 * 5.0),
        }
    }

    /// Returns the interpreted relative heading in degrees. 255 = N/A.
    pub fn relative_heading_degrees(&self) -> Option<f32> {
        if self.relative_heading == 255 {
            None
        } else {
            Some(self.relative_heading as f32 * 360.0 / 254.0)
        }
    }

    /// Returns the position probability as a percentage (0-100). 0=unknown, 31=N/A.
    pub fn probability_percent(&self) -> Option<f32> {
        if self.probability == 31 {
            None
        } else {
            Some(self.probability as f32 * 100.0 / 30.0)
        }
    }
}

// ============================================================================
// SEGMENT MESSAGE
// ============================================================================

/// Bitfield representing the accessibility of a road for different actor classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoadAccessibility {
    pub passenger_cars: bool,
    pub pedestrians: bool,
    pub bus: bool,
    pub delivery: bool,
    pub emergency: bool,
    pub taxi: bool,
    pub through_traffic: bool,
    pub trucks: bool,
    pub reserved: u8,
}

impl RoadAccessibility {
    pub fn from_raw(value: u16) -> Self {
        Self {
            passenger_cars: (value & 0x01) != 0,
            pedestrians: (value & 0x02) != 0,
            bus: (value & 0x04) != 0,
            delivery: (value & 0x08) != 0,
            emergency: (value & 0x10) != 0,
            taxi: (value & 0x20) != 0,
            through_traffic: (value & 0x40) != 0,
            trucks: (value & 0x80) != 0,
            reserved: ((value >> 8) & 0x03) as u8,
        }
    }

    pub fn to_raw(&self) -> u16 {
        (if self.passenger_cars { 0x01 } else { 0 })
            | (if self.pedestrians { 0x02 } else { 0 })
            | (if self.bus { 0x04 } else { 0 })
            | (if self.delivery { 0x08 } else { 0 })
            | (if self.emergency { 0x10 } else { 0 })
            | (if self.taxi { 0x20 } else { 0 })
            | (if self.through_traffic { 0x40 } else { 0 })
            | (if self.trucks { 0x80 } else { 0 })
            | ((self.reserved as u16 & 0x03) << 8)
    }
}

/// Segment message specifying attributes of a part of the road ahead.
///
/// Total: 64 bits (8 bytes), see Table 8 of the specification.
#[derive(Debug, Clone, PartialEq)]
pub struct SegmentMessage {
    pub header: AdasisHeader,
    pub retransmission: bool,
    pub path_index: u8,
    pub offset: u16,
    pub update: bool,
    pub functional_road_class: FunctionalRoadClass,
    pub form_of_way: FormOfWay,
    pub effective_speed_limit: u8,
    pub effective_speed_limit_type: SpeedLimitType,
    pub number_of_lanes_driving_direction: u8,
    pub number_of_lanes_opposite_direction: u8,
    pub tunnel: u8,
    pub bridge: u8,
    pub divided_road: u8,
    pub built_up_area: u8,
    pub complex_intersection: u8,
    pub relative_probability: u8,
    pub part_of_calculated_route: u8,
    pub reserved: u8,
}

impl SegmentMessage {
    /// Evaluates a SEGMENT message path index for if it has a reserved value
    pub fn get_path_id_type(&self) -> PathIdType {
        match self.path_index {
            0 => PathIdType::Unknown,
            1_u8..=3_u8 => PathIdType::InvalidForMessage,
            4 => PathIdType::SegmentAdasisMini,
            5_u8..=6_u8 => PathIdType::InvalidForMessage,
            7 => PathIdType::Reserved,
            _ => PathIdType::Normal,
        }
    }

    /// Deserialized a SegmentMessage
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);

        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let retransmission = reader.read_bit(5, big_endian);
        let path_index = reader.read_bits(6, 6, big_endian) as u8;
        let offset = reader.read_bits(12, 13, big_endian) as u16;
        let update = reader.read_bit(25, big_endian);
        let frc = reader.read_bits(26, 3, big_endian) as u8;
        let fow = reader.read_bits(29, 4, big_endian) as u8;
        let eff_speed_limit = reader.read_bits(33, 5, big_endian) as u8;
        let eff_speed_limit_type = reader.read_bits(38, 3, big_endian) as u8;
        let num_lanes_driving = reader.read_bits(41, 3, big_endian) as u8;
        let num_lanes_opposite = reader.read_bits(44, 2, big_endian) as u8;
        let tunnel = reader.read_bits(46, 2, big_endian) as u8;
        let bridge = reader.read_bits(48, 2, big_endian) as u8;
        let divided_road = reader.read_bits(50, 2, big_endian) as u8;
        let built_up_area = reader.read_bits(52, 2, big_endian) as u8;
        let complex_intersection = reader.read_bits(54, 2, big_endian) as u8;
        let relative_probability = reader.read_bits(56, 5, big_endian) as u8;
        let part_of_calc_route = reader.read_bits(61, 2, big_endian) as u8;
        let reserved = reader.read_bits(63, 1, big_endian) as u8;

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            retransmission,
            path_index,
            offset,
            update,
            functional_road_class: FunctionalRoadClass::from_raw(frc),
            form_of_way: FormOfWay::from_raw(fow),
            effective_speed_limit: eff_speed_limit,
            effective_speed_limit_type: SpeedLimitType::from_raw(eff_speed_limit_type),
            number_of_lanes_driving_direction: num_lanes_driving,
            number_of_lanes_opposite_direction: num_lanes_opposite,
            tunnel,
            bridge,
            divided_road,
            built_up_area,
            complex_intersection,
            relative_probability,
            part_of_calculated_route: part_of_calc_route,
            reserved,
        }
    }

    /// Serializes a SEGMENT message
    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bit(self.retransmission, big_endian);
        writer.write_bits(self.path_index, 6, big_endian);
        writer.write_bits(self.offset, 13, big_endian);
        writer.write_bit(self.update, big_endian);
        writer.write_bits(self.functional_road_class as u8, 3, big_endian);
        writer.write_bits(self.form_of_way as u8, 4, big_endian);
        writer.write_bits(self.effective_speed_limit, 5, big_endian);
        writer.write_bits(self.effective_speed_limit_type as u8, 3, big_endian);
        writer.write_bits(self.number_of_lanes_driving_direction, 3, big_endian);
        writer.write_bits(self.number_of_lanes_opposite_direction, 2, big_endian);
        writer.write_bits(self.tunnel, 2, big_endian);
        writer.write_bits(self.bridge, 2, big_endian);
        writer.write_bits(self.divided_road, 2, big_endian);
        writer.write_bits(self.built_up_area, 2, big_endian);
        writer.write_bits(self.complex_intersection, 2, big_endian);
        writer.write_bits(self.relative_probability, 5, big_endian);
        writer.write_bits(self.part_of_calculated_route, 2, big_endian);
        writer.write_bits(self.reserved, 1, big_endian);
        writer.into_bytes()
    }
}

// ============================================================================
// STUB MESSAGE
// ============================================================================

/// Stub message describing a crossing on the path.
///
/// Total: 64 bits (8 bytes), see Table 12 of the specification.
#[derive(Debug, Clone, PartialEq)]
pub struct StubMessage {
    pub header: AdasisHeader,
    pub retransmission: bool,
    pub path_index: u8,
    pub offset: u16,
    pub update: bool,
    pub sub_path_index: u8,
    pub turn_angle: u8,
    pub relative_probability: u8,
    pub functional_road_class: FunctionalRoadClass,
    pub form_of_way: FormOfWay,
    pub num_lanes_driving_direction: u8,
    pub num_lanes_opposite_direction: u8,
    pub complex_intersection: u8,
    pub right_of_way: u8,
    pub part_of_calculated_route: u8,
    pub last_stub_at_offset: bool,
}

impl StubMessage {
    /// Evaluates a STUB message path index for if it has a reserved value
    pub fn get_path_id_type(&self) -> PathIdType {
        match self.path_index {
            0 => PathIdType::Unknown,
            1_u8..=4_u8 => PathIdType::InvalidForMessage,
            5 => PathIdType::StubSubPathCrossroadNoMoreInfo,
            6 => PathIdType::StubSubPathIntersectionHeadingChange,
            7 => PathIdType::Reserved,
            _ => PathIdType::Normal,
        }
    }

    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);
        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let retransmission = reader.read_bit(5, big_endian);
        let path_index = reader.read_bits(6, 6, big_endian) as u8;
        let offset = reader.read_bits(12, 13, big_endian) as u16;
        let update = reader.read_bit(25, big_endian);
        let sub_path_index = reader.read_bits(26, 6, big_endian) as u8;
        let turn_angle = reader.read_bits(32, 8, big_endian) as u8;
        let relative_probability = reader.read_bits(40, 5, big_endian) as u8;
        let frc = reader.read_bits(45, 3, big_endian) as u8;
        let fow = reader.read_bits(48, 4, big_endian) as u8;
        let num_lanes_driving = reader.read_bits(52, 3, big_endian) as u8;
        let num_lanes_opposite = reader.read_bits(55, 2, big_endian) as u8;
        let complex_intersection = reader.read_bits(57, 2, big_endian) as u8;
        let right_of_way = reader.read_bits(59, 2, big_endian) as u8;
        let part_of_calc_route = reader.read_bits(61, 2, big_endian) as u8;
        let last_stub = reader.read_bit(63, big_endian);

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            retransmission,
            path_index,
            offset,
            update,
            sub_path_index,
            turn_angle,
            relative_probability,
            functional_road_class: FunctionalRoadClass::from_raw(frc),
            form_of_way: FormOfWay::from_raw(fow),
            num_lanes_driving_direction: num_lanes_driving,
            num_lanes_opposite_direction: num_lanes_opposite,
            complex_intersection,
            right_of_way,
            part_of_calculated_route: part_of_calc_route,
            last_stub_at_offset: last_stub,
        }
    }

    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bit(self.retransmission, big_endian);
        writer.write_bits(self.path_index, 6, big_endian);
        writer.write_bits(self.offset, 13, big_endian);
        writer.write_bit(self.update, big_endian);
        writer.write_bits(self.sub_path_index, 6, big_endian);
        writer.write_bits(self.turn_angle, 8, big_endian);
        writer.write_bits(self.relative_probability, 5, big_endian);
        writer.write_bits(self.functional_road_class as u8, 3, big_endian);
        writer.write_bits(self.form_of_way as u8, 4, big_endian);
        writer.write_bits(self.num_lanes_driving_direction, 3, big_endian);
        writer.write_bits(self.num_lanes_opposite_direction, 2, big_endian);
        writer.write_bits(self.complex_intersection, 2, big_endian);
        writer.write_bits(self.right_of_way, 2, big_endian);
        writer.write_bits(self.part_of_calculated_route, 2, big_endian);
        writer.write_bit(self.last_stub_at_offset, big_endian);
        writer.into_bytes()
    }

    /// Returns the turn angle in degrees. 255 = N/A.
    pub fn turn_angle_degrees(&self) -> Option<f32> {
        if self.turn_angle == 255 {
            None
        } else {
            Some(self.turn_angle as f32 * 360.0 / 254.0)
        }
    }

    /// Returns the relative probability as a percentage (0-100). 0=not allowed, 31=N/A.
    pub fn relative_probability_percent(&self) -> Option<f32> {
        if self.relative_probability == 31 {
            None
        } else {
            Some(self.relative_probability as f32 * 100.0 / 30.0)
        }
    }
}

// ============================================================================
// PROFILE SHORT MESSAGE
// ============================================================================

/// Profile type values for PROFILE SHORT messages (Table 14).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProfileShortType {
    Unused = 0,
    Curvature = 1,
    RouteNumber = 2,
    SlopeStep = 3,
    SlopeLinear = 4,
    RoadAccessibility = 5,
    RoadCondition = 6,
    VariableSpeedSign = 7,
    HeadingChange = 8,
    AverageSpeed = 9,
}

impl ProfileShortType {
    pub fn from_raw(value: u8) -> Option<Self> {
        match value {
            1 => Some(ProfileShortType::Curvature),
            2 => Some(ProfileShortType::RouteNumber),
            3 => Some(ProfileShortType::SlopeStep),
            4 => Some(ProfileShortType::SlopeLinear),
            5 => Some(ProfileShortType::RoadAccessibility),
            6 => Some(ProfileShortType::RoadCondition),
            7 => Some(ProfileShortType::VariableSpeedSign),
            8 => Some(ProfileShortType::HeadingChange),
            9 => Some(ProfileShortType::AverageSpeed),
            _ => None,
        }
    }
}

/// Interpreted road profile short information subtypes.
///
/// Each variant corresponds to a specific PROFILE_SHORT profile type
/// as defined in Table 14 of the ADASIS v2 specification.
#[derive(Debug, Clone, PartialEq)]
pub enum InterpretedProfileShort {
    /// Curvature (Type 1): piecewise-linear decoded curvature spots.
    Curvature(CurvatureProfile),
    /// Route Number (Type 2): route identifier.
    RouteNumber(RouteNumberProfile),
    /// Slope Step (Type 3): stepwise slope values with distances.
    SlopeStep(SlopeStepProfile),
    /// Slope Linear (Type 4): linear interpolated slope values.
    SlopeLinear(SlopeLinearProfile),
    /// Road Accessibility (Type 5): per-actor-class accessibility bitfields.
    RoadAccessibility(InterpretedRoadAccessibilityProfile),
    /// Road Condition (Type 6): road surface condition.
    RoadCondition(InterpretedRoadConditionProfile),
    /// Variable Speed Sign (Type 7): variable speed limit sign info.
    VariableSpeedSign(VariableSpeedSignProfile),
    /// Heading Change (Type 8): heading change angles with distances.
    HeadingChange(HeadingChangeProfile),
    /// Average Speed (Type 9): average speed values with distances.
    AverageSpeed(AverageSpeedProfile),
}

// ============================================================================
// PROFILE SHORT SUBTYPE STRUCTS
// ============================================================================

/// Curvature profile (Type 1).
///
/// Curvature is decoded using the piecewise linear lookup table
/// (section 10.1.2). The value 0x3FF means N/A.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvatureProfile {
    /// Decoded curvature at offset0 (None if raw value is invalid/N/A).
    pub curvature0: Option<f32>,
    /// Raw curvature value 0 (None if N/A).
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value 1 (None if N/A).
    pub raw_distance1: Option<u16>,
    /// Decoded curvature at offset1 (None if raw value is invalid/N/A).
    pub curvature1: Option<f32>,
    /// Raw curvature value 1 (None if N/A).
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Route Number profile (Type 2).
#[derive(Debug, Clone, PartialEq)]
pub struct RouteNumberProfile {
    /// Route number value0 (None if raw value is invalid/N/A).
    pub route_number0: Option<u16>,
    /// Raw route number value 0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (None if N/A, value in meters).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Route number value1 (None if raw value is invalid/N/A).
    pub route_number1: Option<u16>,
    /// Raw route number value 1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Slope Step profile (Type 3).
///
/// Slope is decoded as: raw_value * 0.1 - 51.1 (percent).
#[derive(Debug, Clone, PartialEq)]
pub struct SlopeStepProfile {
    /// Decoded slope at offset0 (percent, None if N/A).
    pub slope0: Option<f32>,
    /// Raw slope value 0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded slope at offset1 (percent, None if N/A).
    pub slope1: Option<f32>,
    /// Raw slope value 1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Slope Linear profile (Type 4).
///
/// Slope is decoded as: raw_value * 0.1 - 51.1 (percent).
#[derive(Debug, Clone, PartialEq)]
pub struct SlopeLinearProfile {
    /// Decoded slope at offset0 (percent, None if N/A).
    pub slope0: Option<f32>,
    /// Raw slope value 0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded slope at offset1 (percent, None if N/A).
    pub slope1: Option<f32>,
    /// Raw slope value 1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Interpreted Road Accessibility profile (Type 5).
///
/// Uses the same RoadAccessibility bitfield as SEGMENT messages.
#[derive(Debug, Clone, PartialEq)]
pub struct InterpretedRoadAccessibilityProfile {
    /// Decoded accessibility bitfield (None if raw value is N/A).
    pub accessibility: Option<RoadAccessibility>,
    /// Raw value0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded accessibility at offset1 (None if N/A).
    pub accessibility1: Option<RoadAccessibility>,
    /// Raw value1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Road Condition profile (Type 6).
///
/// Low 4 bits encode the road surface type, upper 6 bits are reserved.
#[derive(Debug, Clone, PartialEq)]
pub struct InterpretedRoadConditionProfile {
    /// Decoded road condition (None if raw value is N/A).
    pub condition: Option<RoadCondition>,
    /// Raw value0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded road condition at offset1 (None if N/A).
    pub condition1: Option<RoadCondition>,
    /// Raw value1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Road surface condition enum (4 bits, section 10.1.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RoadCondition {
    Unknown = 0,
    Dry = 1,
    Wet = 2,
    Snowy = 3,
    Icy = 4,
    Muddy = 5,
    Sandy = 6,
    Gravel = 7,
}

impl RoadCondition {
    pub fn from_raw(value: u8) -> Option<Self> {
        match value {
            0 => Some(RoadCondition::Unknown),
            1 => Some(RoadCondition::Dry),
            2 => Some(RoadCondition::Wet),
            3 => Some(RoadCondition::Snowy),
            4 => Some(RoadCondition::Icy),
            5 => Some(RoadCondition::Muddy),
            6 => Some(RoadCondition::Sandy),
            7 => Some(RoadCondition::Gravel),
            _ => None,
        }
    }
}

/// Variable Speed Sign profile (Type 7).
///
/// Decoded into a VariableSpeedSignValue struct.
#[derive(Debug, Clone, PartialEq)]
pub struct VariableSpeedSignProfile {
    /// Decoded sign info (None if raw value is N/A).
    pub sign: Option<VariableSpeedSignValue>,
    /// Raw value0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded sign info at offset1 (None if N/A).
    pub sign1: Option<VariableSpeedSignValue>,
    /// Raw value1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Variable Speed Sign value decoded from a 10-bit profile spot.
///
/// Layout: sign_type(3 bits) | speed(8 bits) | unit(1 bit) | sign_location(3 bits)
/// = 15 bits. The remaining bits are reserved.
#[derive(Debug, Clone, PartialEq)]
pub struct VariableSpeedSignValue {
    /// Sign type (3 bits, 0=speed limit, 1=speed recommendation).
    pub sign_type: u8,
    /// Speed value in km/h (8 bits, 0-250, 255=N/A).
    pub speed: Option<f32>,
    /// Unit: 0=km/h, 1=mph.
    pub unit: u8,
    /// Sign location (3 bits: 0=ahead, 1=left, 2=right, 3=sharp left, 4=sharp right).
    pub sign_location: u8,
}

impl VariableSpeedSignValue {
    pub fn from_raw(value: u16) -> Option<Self> {
        if value == PROFILE_VALUE_INVALID_10 {
            return None;
        }
        Some(Self {
            sign_type: ((value >> 7) & 0x07) as u8,
            speed: {
                let raw_speed = ((value >> 1) & 0xFF) as u8;
                if raw_speed >= 251 {
                    None
                } else {
                    Some(raw_speed as f32)
                }
            },
            unit: ((value >> 0) & 0x01) as u8,
            sign_location: 0, // placeholder, needs more bits
        })
    }

    pub fn to_raw(&self) -> u16 {
        let speed_val = match self.speed {
            Some(s) if s <= 250.0 => s as u8,
            _ => 255,
        };
        ((self.sign_type as u16 & 0x07) << 7)
            | ((speed_val as u16 & 0xFF) << 1)
            | ((self.unit as u16) & 0x01)
            | ((self.sign_location as u16 & 0x07) << 0)
    }
}

/// Heading Change profile (Type 8).
///
/// Values represent heading change in degrees: raw * (90/511).
#[derive(Debug, Clone, PartialEq)]
pub struct HeadingChangeProfile {
    /// Decoded heading change at offset0 (degrees, None if N/A).
    pub heading0: Option<f32>,
    /// Raw value0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded heading change at offset1 (degrees, None if N/A).
    pub heading1: Option<f32>,
    /// Raw value1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Average Speed profile (Type 9).
///
/// Speed values decoded as: raw * 0.5 km/h. The value 0x3FF means N/A.
#[derive(Debug, Clone, PartialEq)]
pub struct AverageSpeedProfile {
    /// Decoded average speed at offset0 (km/h, None if N/A).
    pub speed0: Option<f32>,
    /// Raw value0.
    pub raw_value0: Option<u16>,
    /// Distance to the next spot (meters, None if N/A).
    pub distance1: Option<f32>,
    /// Raw distance value.
    pub raw_distance1: Option<u16>,
    /// Decoded average speed at offset1 (km/h, None if N/A).
    pub speed1: Option<f32>,
    /// Raw value1.
    pub raw_value1: Option<u16>,
    /// Accuracy indicator (2 bits).
    pub accuracy: u8,
    /// Whether this is a control point.
    pub control_point: bool,
}

/// Helper function to decode a 10-bit profile spot into Option<f32> using a multiplier.
///
/// Returns None if the raw value is the invalid/N/A marker (0x3FF).
fn decode_profile_spot_f32(raw: u16, multiplier: f32) -> Option<f32> {
    if raw == PROFILE_VALUE_INVALID_10 {
        None
    } else {
        Some(raw as f32 * multiplier)
    }
}

/// Helper to decode a 10-bit profile spot into Option<u16>.
fn decode_profile_spot_u16(raw: u16) -> Option<u16> {
    if raw == PROFILE_VALUE_INVALID_10 {
        None
    } else {
        Some(raw)
    }
}

/// Profile SHORT message containing two 10-bit profile spots.
#[derive(Debug, Clone, PartialEq)]
pub struct ProfileShortMessage {
    pub header: AdasisHeader,
    pub retransmission: bool,
    pub path_index: u8,
    pub offset: u16,
    pub update: bool,
    pub profile_type: u8,
    pub control_point: bool,
    pub value0: u16,
    pub distance1: u16,
    pub value1: u16,
    pub accuracy: u8,
}

impl ProfileShortMessage {
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);

        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let retransmission = reader.read_bit(5, big_endian);
        let path_index = reader.read_bits(6, 6, big_endian) as u8;
        let offset = reader.read_bits(12, 13, big_endian) as u16;
        let update = reader.read_bit(25, big_endian);
        let profile_type = reader.read_bits(26, 5, big_endian) as u8;
        let control_point = reader.read_bit(31, big_endian);
        let value0 = reader.read_bits(32, 10, big_endian) as u16;
        let distance1 = reader.read_bits(42, 10, big_endian) as u16;
        let value1 = reader.read_bits(52, 10, big_endian) as u16;
        let accuracy = reader.read_bits(62, 2, big_endian) as u8;

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            retransmission,
            path_index,
            offset,
            update,
            profile_type,
            control_point,
            value0,
            distance1,
            value1,
            accuracy,
        }
    }

    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bit(self.retransmission, big_endian);
        writer.write_bits(self.path_index, 6, big_endian);
        writer.write_bits(self.offset, 13, big_endian);
        writer.write_bit(self.update, big_endian);
        writer.write_bits(self.profile_type, 5, big_endian);
        writer.write_bit(self.control_point, big_endian);
        writer.write_bits(self.value0, 10, big_endian);
        writer.write_bits(self.distance1, 10, big_endian);
        writer.write_bits(self.value1, 10, big_endian);
        writer.write_bits(self.accuracy, 2, big_endian);
        writer.into_bytes()
    }

    /// Interprets this PROFILE SHORT message into its typed subtype.
    pub fn interpret(
        &self,
        _big_endian: bool,
    ) -> Result<InterpretedProfileShort, DeserializeError> {
        let profile_type = ProfileShortType::from_raw(self.profile_type)
            .ok_or(DeserializeError::UnknownProfileType(self.profile_type))?;

        let result = match profile_type {
            ProfileShortType::Curvature => {
                let curvature0 = decode_curvature(self.value0 as i32);
                let curvature1 = decode_curvature(self.value1 as i32);
                InterpretedProfileShort::Curvature(CurvatureProfile {
                    curvature0: if self.value0 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        Some(curvature0)
                    },
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    curvature1: if self.value1 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        Some(curvature1)
                    },
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::RouteNumber => {
                InterpretedProfileShort::RouteNumber(RouteNumberProfile {
                    route_number0: decode_profile_spot_u16(self.value0),
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    route_number1: decode_profile_spot_u16(self.value1),
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::SlopeStep => InterpretedProfileShort::SlopeStep(SlopeStepProfile {
                slope0: decode_profile_spot_f32(self.value0, 0.1).map(|v| v - 51.1),
                raw_value0: decode_profile_spot_u16(self.value0),
                distance1: decode_profile_spot_f32(self.distance1, 1.0),
                raw_distance1: decode_profile_spot_u16(self.distance1),
                slope1: decode_profile_spot_f32(self.value1, 0.1).map(|v| v - 51.1),
                raw_value1: decode_profile_spot_u16(self.value1),
                accuracy: self.accuracy,
                control_point: self.control_point,
            }),
            ProfileShortType::SlopeLinear => {
                InterpretedProfileShort::SlopeLinear(SlopeLinearProfile {
                    slope0: decode_profile_spot_f32(self.value0, 0.1).map(|v| v - 51.1),
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    slope1: decode_profile_spot_f32(self.value1, 0.1).map(|v| v - 51.1),
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::RoadAccessibility => {
                InterpretedProfileShort::RoadAccessibility(InterpretedRoadAccessibilityProfile {
                    accessibility: if self.value0 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        Some(RoadAccessibility::from_raw(self.value0))
                    },
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    accessibility1: if self.value1 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        Some(RoadAccessibility::from_raw(self.value1))
                    },
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::RoadCondition => {
                InterpretedProfileShort::RoadCondition(InterpretedRoadConditionProfile {
                    condition: if self.value0 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        RoadCondition::from_raw((self.value0 & 0x0F) as u8)
                    },
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    condition1: if self.value1 == PROFILE_VALUE_INVALID_10 {
                        None
                    } else {
                        RoadCondition::from_raw((self.value1 & 0x0F) as u8)
                    },
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::VariableSpeedSign => {
                InterpretedProfileShort::VariableSpeedSign(VariableSpeedSignProfile {
                    sign: VariableSpeedSignValue::from_raw(self.value0),
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    sign1: VariableSpeedSignValue::from_raw(self.value1),
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::HeadingChange => {
                InterpretedProfileShort::HeadingChange(HeadingChangeProfile {
                    heading0: decode_profile_spot_f32(self.value0, 360.0 / 511.0),
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    heading1: decode_profile_spot_f32(self.value1, 360.0 / 511.0),
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::AverageSpeed => {
                InterpretedProfileShort::AverageSpeed(AverageSpeedProfile {
                    speed0: decode_profile_spot_f32(self.value0, 0.5),
                    raw_value0: decode_profile_spot_u16(self.value0),
                    distance1: decode_profile_spot_f32(self.distance1, 1.0),
                    raw_distance1: decode_profile_spot_u16(self.distance1),
                    speed1: decode_profile_spot_f32(self.value1, 0.5),
                    raw_value1: decode_profile_spot_u16(self.value1),
                    accuracy: self.accuracy,
                    control_point: self.control_point,
                })
            }
            ProfileShortType::Unused => {
                return Err(DeserializeError::UnknownProfileType(self.profile_type));
            }
        };

        Ok(result)
    }
}

// ============================================================================
// PROFILE LONG MESSAGE
// ============================================================================

/// Profile type values for PROFILE LONG messages (Table 17).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProfileLongType {
    Unused = 0,
    Longitude = 1,
    Latitude = 2,
    Altitude = 3,
    BezierControlPointLongitude = 4,
    BezierControlPointLatitude = 5,
    BezierControlPointAltitude = 6,
    LinkIdentifier = 7,
    TrafficSign = 8,
    TruckSpeedLimit = 9,
    ExtendedLane = 10,
    SpeedLimitAndRecommendedSpeed = 11,
}

impl ProfileLongType {
    pub fn from_raw(value: u8) -> Option<Self> {
        match value {
            1 => Some(ProfileLongType::Longitude),
            2 => Some(ProfileLongType::Latitude),
            3 => Some(ProfileLongType::Altitude),
            4 => Some(ProfileLongType::BezierControlPointLongitude),
            5 => Some(ProfileLongType::BezierControlPointLatitude),
            6 => Some(ProfileLongType::BezierControlPointAltitude),
            7 => Some(ProfileLongType::LinkIdentifier),
            8 => Some(ProfileLongType::TrafficSign),
            9 => Some(ProfileLongType::TruckSpeedLimit),
            10 => Some(ProfileLongType::ExtendedLane),
            11 => Some(ProfileLongType::SpeedLimitAndRecommendedSpeed),
            _ => None,
        }
    }
}

/// Traffic Sign profile value (Type 8, PROFILE LONG).
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficSignValue {
    pub sign_type: u8,
    pub value: u8,
    pub lane: u8,
    pub vehicle_specific: u8,
    pub time_specific: u8,
    pub condition: u8,
    pub sign_location: u8,
}

impl TrafficSignValue {
    pub fn from_u32(value: u32) -> Option<Self> {
        if value == PROFILE_VALUE_INVALID_32 {
            return None;
        }
        Some(Self {
            sign_type: ((value >> 24) & 0xFF) as u8,
            value: ((value >> 16) & 0xFF) as u8,
            lane: ((value >> 12) & 0x0F) as u8,
            vehicle_specific: ((value >> 10) & 0x03) as u8,
            time_specific: ((value >> 8) & 0x03) as u8,
            condition: ((value >> 4) & 0x0F) as u8,
            sign_location: (value & 0x07) as u8,
        })
    }

    pub fn to_u32(&self) -> u32 {
        ((self.sign_type as u32) << 24)
            | (((self.value as u32) & 0xFF) << 16)
            | (((self.lane as u32) & 0x0F) << 12)
            | (((self.vehicle_specific as u32) & 0x03) << 10)
            | (((self.time_specific as u32) & 0x03) << 8)
            | (((self.condition as u32) & 0x0F) << 4)
            | (self.sign_location as u32 & 0x07)
    }
}

/// Truck Speed Limit profile value (Type 9, PROFILE LONG).
#[derive(Debug, Clone, PartialEq)]
pub struct TruckSpeedLimitValue {
    pub speed: u8,
    pub weight: u8,
    pub hazardous_goods: u8,
    pub weather_restriction: u8,
    pub limit_type: u8,
    pub time_valid: u8,
    pub time_dependent: bool,
}

impl TruckSpeedLimitValue {
    pub fn from_u32(value: u32) -> Option<Self> {
        if value == PROFILE_VALUE_INVALID_32 {
            return None;
        }
        Some(Self {
            speed: ((value >> 24) & 0xFF) as u8,
            weight: ((value >> 16) & 0xFF) as u8,
            hazardous_goods: ((value >> 13) & 0x07) as u8,
            weather_restriction: ((value >> 10) & 0x07) as u8,
            limit_type: ((value >> 8) & 0x03) as u8,
            time_valid: ((value >> 6) & 0x03) as u8,
            time_dependent: ((value >> 5) & 0x01) != 0,
        })
    }

    pub fn to_u32(&self) -> u32 {
        ((self.speed as u32) << 24)
            | (((self.weight as u32) & 0xFF) << 16)
            | (((self.hazardous_goods as u32) & 0x07) << 13)
            | (((self.weather_restriction as u32) & 0x07) << 10)
            | (((self.limit_type as u32) & 0x03) << 8)
            | (((self.time_valid as u32) & 0x03) << 6)
            | (((self.time_dependent as u32) & 0x01) << 5)
    }
}

/// Extended Lane profile value (Type 10, PROFILE LONG).
#[derive(Debug, Clone, PartialEq)]
pub struct ExtendedLaneValue {
    pub lane_number: u8,
    pub first_predecessor_lane: u8,
    pub last_predecessor_lane: u8,
    pub priority_predecessor_lane: u8,
    pub arrow_marking: u8,
    pub lane_type: u8,
    pub line_marking: u8,
    pub yield_flag: bool,
}

impl ExtendedLaneValue {
    pub fn from_u32(value: u32) -> Option<Self> {
        if value == PROFILE_VALUE_INVALID_32 {
            return None;
        }
        Some(Self {
            lane_number: ((value >> 28) & 0x0F) as u8,
            first_predecessor_lane: ((value >> 24) & 0x0F) as u8,
            last_predecessor_lane: ((value >> 20) & 0x0F) as u8,
            priority_predecessor_lane: ((value >> 16) & 0x0F) as u8,
            arrow_marking: ((value >> 8) & 0xFF) as u8,
            lane_type: ((value >> 4) & 0x0F) as u8,
            line_marking: (value & 0x07) as u8,
            yield_flag: ((value >> 7) & 0x01) != 0,
        })
    }

    pub fn to_u32(&self) -> u32 {
        ((self.lane_number as u32 & 0x0F) << 28)
            | (((self.first_predecessor_lane as u32) & 0x0F) << 24)
            | (((self.last_predecessor_lane as u32) & 0x0F) << 20)
            | (((self.priority_predecessor_lane as u32) & 0x0F) << 16)
            | (((self.arrow_marking as u32) & 0xFF) << 8)
            | (((self.lane_type as u32) & 0x0F) << 4)
            | ((if self.yield_flag { 0x80 } else { 0 }) as u32)
            | (self.line_marking as u32 & 0x07)
    }
}

/// Speed Limit and Recommended Speed profile value (Type 11, PROFILE LONG).
#[derive(Debug, Clone, PartialEq)]
pub struct SpeedLimitAndRecommendedSpeedValue {
    pub number_of_entries: u8,
    pub entry_index: u8,
    pub variable: bool,
    pub entry_type: u8,
    pub speed_units: u8,
    pub value: u8,
    pub lane: u8,
    pub vehicle_specific: u8,
    pub time_specific: u8,
    pub condition: u8,
}

impl SpeedLimitAndRecommendedSpeedValue {
    pub fn from_u32(value: u32) -> Option<Self> {
        if value == PROFILE_VALUE_INVALID_32 {
            return None;
        }
        Some(Self {
            number_of_entries: ((value >> 28) & 0x0F) as u8,
            entry_index: ((value >> 24) & 0x0F) as u8,
            variable: ((value >> 23) & 0x01) != 0,
            entry_type: ((value >> 21) & 0x03) as u8,
            speed_units: ((value >> 20) & 0x01) as u8,
            value: ((value >> 12) & 0xFF) as u8,
            lane: ((value >> 8) & 0x0F) as u8,
            vehicle_specific: ((value >> 6) & 0x03) as u8,
            time_specific: ((value >> 4) & 0x03) as u8,
            condition: (value & 0x0F) as u8,
        })
    }

    pub fn to_u32(&self) -> u32 {
        ((self.number_of_entries as u32 & 0x0F) << 28)
            | (((self.entry_index as u32) & 0x0F) << 24)
            | (((self.variable as u32) & 0x01) << 23)
            | (((self.entry_type as u32) & 0x03) << 21)
            | (((self.speed_units as u32) & 0x01) << 20)
            | (((self.value as u32) & 0xFF) << 12)
            | (((self.lane as u32) & 0x0F) << 8)
            | (((self.vehicle_specific as u32) & 0x03) << 6)
            | (((self.time_specific as u32) & 0x03) << 4)
            | (self.condition as u32 & 0x0F)
    }
}

/// Interpreted road profile long information subtypes.
#[derive(Debug, Clone, PartialEq)]
pub enum InterpretedProfileLong {
    Longitude {
        value: Option<f64>,
        raw_value: u32,
        control_point: bool,
    },
    Latitude {
        value: Option<f64>,
        raw_value: u32,
        control_point: bool,
    },
    Altitude {
        value: Option<f32>,
        raw_value: u32,
        control_point: bool,
    },
    BezierControlPointLongitude {
        value: Option<f64>,
        raw_value: u32,
        control_point: bool,
    },
    BezierControlPointLatitude {
        value: Option<f64>,
        raw_value: u32,
        control_point: bool,
    },
    BezierControlPointAltitude {
        value: Option<f32>,
        raw_value: u32,
        control_point: bool,
    },
    LinkIdentifier {
        value: Option<u32>,
        raw_value: u32,
        control_point: bool,
    },
    TrafficSign {
        value: Option<TrafficSignValue>,
        raw_value: u32,
        control_point: bool,
    },
    TruckSpeedLimit {
        value: Option<TruckSpeedLimitValue>,
        raw_value: u32,
        control_point: bool,
    },
    ExtendedLane {
        value: Option<ExtendedLaneValue>,
        raw_value: u32,
        control_point: bool,
    },
    SpeedLimitAndRecommendedSpeed {
        value: Option<SpeedLimitAndRecommendedSpeedValue>,
        raw_value: u32,
        control_point: bool,
    },
}

/// Profile LONG message containing one 32-bit profile spot.
#[derive(Debug, Clone, PartialEq)]
pub struct ProfileLongMessage {
    pub header: AdasisHeader,
    pub retransmission: bool,
    pub path_index: u8,
    pub offset: u16,
    pub update: bool,
    pub profile_type: u8,
    pub control_point: bool,
    pub value: u32,
}

impl ProfileLongMessage {
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);

        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let retransmission = reader.read_bit(5, big_endian);
        let path_index = reader.read_bits(6, 6, big_endian) as u8;
        let offset = reader.read_bits(12, 13, big_endian) as u16;
        let update = reader.read_bit(25, big_endian);
        let profile_type = reader.read_bits(26, 5, big_endian) as u8;
        let control_point = reader.read_bit(31, big_endian);
        let value = reader.read_bits(32, 32, big_endian);

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            retransmission,
            path_index,
            offset,
            update,
            profile_type,
            control_point,
            value,
        }
    }

    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bit(self.retransmission, big_endian);
        writer.write_bits(self.path_index, 6, big_endian);
        writer.write_bits(self.offset, 13, big_endian);
        writer.write_bit(self.update, big_endian);
        writer.write_bits(self.profile_type, 5, big_endian);
        writer.write_bit(self.control_point, big_endian);
        writer.write_bits(self.value, 32, big_endian);
        writer.into_bytes()
    }

    /// Interprets this PROFILE LONG message into its typed subtype.
    pub fn interpret(&self, _big_endian: bool) -> Result<InterpretedProfileLong, DeserializeError> {
        let raw_value = self.value;
        let control_point = self.control_point;
        let profile_type = ProfileLongType::from_raw(self.profile_type)
            .ok_or(DeserializeError::UnknownProfileType(self.profile_type))?;

        let result = match profile_type {
            ProfileLongType::Longitude => InterpretedProfileLong::Longitude {
                value: if raw_value == PROFILE_VALUE_INVALID_32 {
                    None
                } else {
                    Some((raw_value as f64) * 0.0000001 - 180.0)
                },
                raw_value,
                control_point,
            },
            ProfileLongType::Latitude => InterpretedProfileLong::Latitude {
                value: if raw_value == PROFILE_VALUE_INVALID_32 {
                    None
                } else {
                    Some((raw_value as f64) * 0.0000001 - 90.0)
                },
                raw_value,
                control_point,
            },
            ProfileLongType::Altitude => InterpretedProfileLong::Altitude {
                value: if raw_value == PROFILE_VALUE_INVALID_32 {
                    None
                } else {
                    Some((raw_value as f32) * 0.01 - 1000.0)
                },
                raw_value,
                control_point,
            },
            ProfileLongType::BezierControlPointLongitude => {
                InterpretedProfileLong::BezierControlPointLongitude {
                    value: if raw_value == PROFILE_VALUE_INVALID_32 {
                        None
                    } else {
                        Some((raw_value as f64) * 0.0000001 - 180.0)
                    },
                    raw_value,
                    control_point,
                }
            }
            ProfileLongType::BezierControlPointLatitude => {
                InterpretedProfileLong::BezierControlPointLatitude {
                    value: if raw_value == PROFILE_VALUE_INVALID_32 {
                        None
                    } else {
                        Some((raw_value as f64) * 0.0000001 - 90.0)
                    },
                    raw_value,
                    control_point,
                }
            }
            ProfileLongType::BezierControlPointAltitude => {
                InterpretedProfileLong::BezierControlPointAltitude {
                    value: if raw_value == PROFILE_VALUE_INVALID_32 {
                        None
                    } else {
                        Some((raw_value as f32) * 0.01 - 1000.0)
                    },
                    raw_value,
                    control_point,
                }
            }
            ProfileLongType::LinkIdentifier => InterpretedProfileLong::LinkIdentifier {
                value: if raw_value == PROFILE_VALUE_INVALID_32 {
                    None
                } else {
                    Some(raw_value)
                },
                raw_value,
                control_point,
            },
            ProfileLongType::TrafficSign => InterpretedProfileLong::TrafficSign {
                value: TrafficSignValue::from_u32(raw_value),
                raw_value,
                control_point,
            },
            ProfileLongType::TruckSpeedLimit => InterpretedProfileLong::TruckSpeedLimit {
                value: TruckSpeedLimitValue::from_u32(raw_value),
                raw_value,
                control_point,
            },
            ProfileLongType::ExtendedLane => InterpretedProfileLong::ExtendedLane {
                value: ExtendedLaneValue::from_u32(raw_value),
                raw_value,
                control_point,
            },
            ProfileLongType::SpeedLimitAndRecommendedSpeed => {
                InterpretedProfileLong::SpeedLimitAndRecommendedSpeed {
                    value: SpeedLimitAndRecommendedSpeedValue::from_u32(raw_value),
                    raw_value,
                    control_point,
                }
            }
            ProfileLongType::Unused => {
                return Err(DeserializeError::UnknownProfileType(self.profile_type));
            }
        };

        Ok(result)
    }
}

// ============================================================================
// META-DATA MESSAGE
// ============================================================================

/// Metadata message containing utility data about the system.
///
/// Total: 64 bits (8 bytes), see Table 18 of the specification.
#[derive(Debug, Clone, PartialEq)]
pub struct MetaDataMessage {
    pub header: AdasisHeader,
    /// ISO 3166-1 numeric country code (0=unknown).
    pub country_code: u16,
    /// Region code character 1 (5 bits per character).
    pub region_code0: u8,
    /// Region code character 2.
    pub region_code1: u8,
    /// Region code character 3.
    pub region_code2: u8,
    /// Driving side: 0=Left, 1=Right.
    pub driving_side: u8,
    /// Speed unit: 0=km/h, 1=mph.
    pub speed_unit: u8,
    /// Major protocol version.
    pub major_protocol_version: u8,
    /// Minor protocol version.
    pub minor_protocol_version: u8,
    /// Minor protocol sub-version.
    pub minor_protocol_sub_version: u8,
    /// Hardware version (0=unknown).
    pub hardware_version: u16,
    /// Map provider (see [MapProvider]).
    pub map_provider: u8,
    /// Map version year ((year - 2000) % 63). 63 = N/A.
    pub map_version_year: u8,
    /// Map version quarter (0-3, representing Q1-Q3, plus reserved).
    pub map_version_quarter: u8,
    /// Reserved bits.
    pub reserved: u8,
}

impl MetaDataMessage {
    pub fn from_bytes(data: &[u8; 8], big_endian: bool) -> Self {
        let reader = BitReader::new(data);

        let message_type = MessageType::from_raw(reader.read_bits(0, 3, big_endian) as u8);
        let cyclic_counter = reader.read_bits(3, 2, big_endian) as u8;
        let country_code = reader.read_bits(5, 10, big_endian) as u16;
        let region_code0 = reader.read_bits(15, 5, big_endian) as u8;
        let region_code1 = reader.read_bits(20, 5, big_endian) as u8;
        let region_code2 = reader.read_bits(25, 5, big_endian) as u8;
        let driving_side = reader.read_bit(30, big_endian);
        let speed_unit = reader.read_bit(31, big_endian);
        let major_protocol_version = reader.read_bits(32, 2, big_endian) as u8;
        let minor_protocol_version = reader.read_bits(34, 4, big_endian) as u8;
        let minor_protocol_sub_version = reader.read_bits(38, 3, big_endian) as u8;
        let hardware_version = reader.read_bits(41, 9, big_endian) as u16;
        let map_provider = reader.read_bits(50, 3, big_endian) as u8;
        let map_version_year = reader.read_bits(53, 6, big_endian) as u8;
        let map_version_quarter = reader.read_bits(59, 2, big_endian) as u8;
        let reserved = reader.read_bits(61, 3, big_endian) as u8;

        Self {
            header: AdasisHeader {
                message_type,
                cyclic_counter,
            },
            country_code,
            region_code0,
            region_code1,
            region_code2,
            driving_side: driving_side as u8,
            speed_unit: speed_unit as u8,
            major_protocol_version,
            minor_protocol_version,
            minor_protocol_sub_version,
            hardware_version,
            map_provider,
            map_version_year,
            map_version_quarter,
            reserved,
        }
    }

    pub fn to_bytes(&self, big_endian: bool) -> [u8; 8] {
        let mut writer = BitWriter::new();
        writer.write_bits(self.header.message_type.as_raw(), 3, big_endian);
        writer.write_bits(self.header.cyclic_counter, 2, big_endian);
        writer.write_bits(self.country_code, 10, big_endian);
        writer.write_bits(self.region_code0, 5, big_endian);
        writer.write_bits(self.region_code1, 5, big_endian);
        writer.write_bits(self.region_code2, 5, big_endian);
        writer.write_bit(self.driving_side != 0, big_endian);
        writer.write_bit(self.speed_unit != 0, big_endian);
        writer.write_bits(self.major_protocol_version, 2, big_endian);
        writer.write_bits(self.minor_protocol_version, 4, big_endian);
        writer.write_bits(self.minor_protocol_sub_version, 3, big_endian);
        writer.write_bits(self.hardware_version, 9, big_endian);
        writer.write_bits(self.map_provider, 3, big_endian);
        writer.write_bits(self.map_version_year, 6, big_endian);
        writer.write_bits(self.map_version_quarter, 2, big_endian);
        writer.write_bits(self.reserved, 3, big_endian);
        writer.into_bytes()
    }

    /// Returns the interpreted map provider.
    pub fn map_provider_enum(&self) -> MapProvider {
        MapProvider::from_raw(self.map_provider)
    }

    /// Returns the interpreted map version year.
    pub fn map_version_year_full(&self) -> Option<u16> {
        if self.map_version_year == 63 {
            None
        } else {
            Some(2000 + self.map_version_year as u16)
        }
    }

    /// Returns the full region code as a 15-bit value.
    pub fn region_code_full(&self) -> u16 {
        ((self.region_code0 as u16) << 10)
            | ((self.region_code1 as u16) << 5)
            | (self.region_code2 as u16)
    }
}

// ============================================================================
// MESSAGE ENUM (DISCRIMINATED UNION)
// ============================================================================

/// A fully interpreted ADASIS v2 message.
#[derive(Debug, Clone, PartialEq)]
pub enum Message {
    Position(PositionMessage),
    Segment(SegmentMessage),
    Stub(StubMessage),
    ProfileShort(InterpretedProfileShort),
    ProfileLong(InterpretedProfileLong),
    MetaData(MetaDataMessage),
    SystemSpecific(u8),
    Reserved(u8),
}

/// Error type for deserialization failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeserializeError {
    /// Unknown profile type encountered in PROFILE_SHORT or PROFILE_LONG.
    UnknownProfileType(u8),
}

/// Deserializes an 8-byte CAN frame into an interpreted ADASIS v2 message.
pub fn deserialize(data: &[u8; 8], big_endian: bool) -> Result<Message, DeserializeError> {
    let msg_type = get_message_type(data, big_endian);

    match msg_type {
        MessageType::Position => Ok(Message::Position(PositionMessage::from_bytes(
            data, big_endian,
        ))),
        MessageType::Segment => Ok(Message::Segment(SegmentMessage::from_bytes(
            data, big_endian,
        ))),
        MessageType::Stub => Ok(Message::Stub(StubMessage::from_bytes(data, big_endian))),
        MessageType::ProfileShort => {
            let raw = ProfileShortMessage::from_bytes(data, big_endian);
            let interpreted = raw.interpret(big_endian)?;
            Ok(Message::ProfileShort(interpreted))
        }
        MessageType::ProfileLong => {
            let raw = ProfileLongMessage::from_bytes(data, big_endian);
            let interpreted = raw.interpret(big_endian)?;
            Ok(Message::ProfileLong(interpreted))
        }
        MessageType::MetaData => Ok(Message::MetaData(MetaDataMessage::from_bytes(
            data, big_endian,
        ))),
        MessageType::SystemSpecific => Ok(Message::SystemSpecific(data[1])),
        MessageType::Reserved => Ok(Message::Reserved(0)),
    }
}

/// Serializes an interpreted ADASIS v2 message into an 8-byte CAN frame.
pub fn serialize(msg: &Message, big_endian: bool) -> [u8; 8] {
    match msg {
        Message::Position(m) => m.to_bytes(big_endian),
        Message::Segment(m) => m.to_bytes(big_endian),
        Message::Stub(m) => m.to_bytes(big_endian),
        Message::ProfileShort(interpreted) => serialize_profile_short(interpreted, big_endian),
        Message::ProfileLong(interpreted) => serialize_profile_long(interpreted, big_endian),
        Message::MetaData(m) => m.to_bytes(big_endian),
        Message::SystemSpecific(v) => {
            let mut result = [0u8; 8];
            result[0] = 0x00;
            result[1] = *v;
            result
        }
        Message::Reserved(_) => [0u8; 8],
    }
}

fn serialize_profile_short(interpreted: &InterpretedProfileShort, big_endian: bool) -> [u8; 8] {
    let (profile_type, value0, distance1, value1, accuracy, control_point) = match interpreted {
        InterpretedProfileShort::Curvature(c) => (
            1u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::RouteNumber(c) => (
            2u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::SlopeStep(c) => (
            3u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::SlopeLinear(c) => (
            4u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::RoadAccessibility(c) => (
            5u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::RoadCondition(c) => (
            6u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::VariableSpeedSign(c) => (
            7u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::HeadingChange(c) => (
            8u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
        InterpretedProfileShort::AverageSpeed(c) => (
            9u8,
            c.raw_value0.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_distance1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.raw_value1.unwrap_or(PROFILE_VALUE_INVALID_10),
            c.accuracy,
            c.control_point,
        ),
    };

    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type,
        control_point,
        value0,
        distance1,
        value1,
        accuracy,
    };

    msg.to_bytes(big_endian)
}

fn serialize_profile_long(interpreted: &InterpretedProfileLong, big_endian: bool) -> [u8; 8] {
    let (profile_type, raw_value, control_point) = match interpreted {
        InterpretedProfileLong::Longitude {
            raw_value,
            control_point,
            ..
        } => (1u8, *raw_value, *control_point),
        InterpretedProfileLong::Latitude {
            raw_value,
            control_point,
            ..
        } => (2, *raw_value, *control_point),
        InterpretedProfileLong::Altitude {
            raw_value,
            control_point,
            ..
        } => (3, *raw_value, *control_point),
        InterpretedProfileLong::BezierControlPointLongitude {
            raw_value,
            control_point,
            ..
        } => (4, *raw_value, *control_point),
        InterpretedProfileLong::BezierControlPointLatitude {
            raw_value,
            control_point,
            ..
        } => (5, *raw_value, *control_point),
        InterpretedProfileLong::BezierControlPointAltitude {
            raw_value,
            control_point,
            ..
        } => (6, *raw_value, *control_point),
        InterpretedProfileLong::LinkIdentifier {
            raw_value,
            control_point,
            ..
        } => (7, *raw_value, *control_point),
        InterpretedProfileLong::TrafficSign {
            raw_value,
            control_point,
            ..
        } => (8, *raw_value, *control_point),
        InterpretedProfileLong::TruckSpeedLimit {
            raw_value,
            control_point,
            ..
        } => (9, *raw_value, *control_point),
        InterpretedProfileLong::ExtendedLane {
            raw_value,
            control_point,
            ..
        } => (10, *raw_value, *control_point),
        InterpretedProfileLong::SpeedLimitAndRecommendedSpeed {
            raw_value,
            control_point,
            ..
        } => (11, *raw_value, *control_point),
    };

    let msg = ProfileLongMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileLong,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type,
        control_point,
        value: raw_value,
    };

    msg.to_bytes(big_endian)
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
