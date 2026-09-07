//! Comprehensive tests for ADASIS v2 message serialization and deserialization.

use adasisv2::*;

/*#[test]
fn test_parse_position() {
    let data: [u8; 8] = [0b00110001, 0b00000000, 0b01010010, 0b00111111, 0b11000101 ,0b10011111, 0b11011101, 0b00101110];  
//    let data0: u64 = 0x20528835fe2e59fd;
//    let data0: [u8; 8] = [0x20, 0x52, 0x88, 0x35, 0xfe, 0x2e, 0x59, 0xfd];
    //let data1 = data0.swap_bytes();
    //let data = data1.to_ne_bytes();
    let t: adasisv2::MessageType = adasisv2::get_message_type(&data);
    for elem in &data {
        print!("{:x?}", elem);
    }
    println!("");
    assert_eq!(t, adasisv2::MessageType::Position);
    let m: adasisv2::PositionMessage = adasisv2::PositionMessage::from_bytes(&data);
    assert_eq!(m.header.message_type, MessageType::Position);
    assert_eq!(m.header.cyclic_counter, 2, "Cyclic counter");
    assert_eq!(m.path_index, 8, "path index");
    assert_eq!(m.offset, 82, "path offset");
    assert_eq!(m.position_index, 0, "position index");
    assert_eq!(m.position_age, 510, "position age");
    assert_eq!(m.speed, 89, "speed");
    assert_eq!(m.relative_heading, 253, "heading");
    assert_eq!(m.probability, 26, "probability");
    assert_eq!(m.confidence, 2, "confidence");
    assert_eq!(m.current_lane, 7, "current lane");

//                            01 89 03 90 41 03 e0 41
    let data2: [u8; 8] = [0x01, 0x89, 0x03, 0x90, 0x41, 0x03, 0xe0, 0x41];
    //let data2: [u8; 8] = [0x41, 0xe0, 0x03, 0x41, 0x90, 0x03, 0x89, 0x01];
    assert_eq!(t, adasisv2::MessageType::Position);
    let m2: adasisv2::PositionMessage = adasisv2::PositionMessage::from_bytes(&data2);
    assert_eq!(m2.header.message_type, MessageType::Position);
    assert_eq!(m2.header.cyclic_counter, 0, "Cyclic counter");
    assert_eq!(m2.path_index, 8, "path index");
    assert_eq!(m2.offset, 113, "path offset");
    assert_eq!(m2.position_index, 0, "position index");
    assert_eq!(m2.position_age, 100, "position age");
    assert_eq!(m2.speed, 104, "speed");
    assert_eq!(m2.relative_heading, 0, "heading");
    assert_eq!(m2.probability, 30, "probability");
    assert_eq!(m2.confidence, 0, "confidence");
    assert_eq!(m2.current_lane, 4, "current lane");

}*/


/// Tests that get_message_type correctly extracts the message type from byte 0.
#[test]
fn test_get_message_type() {
    for msg_type in 0u8..=7u8 {
        let data = [msg_type << 5, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(get_message_type(&data), MessageType::from_raw(msg_type));
    }
}

/// Tests POSITION message roundtrip serialization.
#[test]
fn test_position_roundtrip() {
    let data: [u8; 8] = [0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
    let msg = PositionMessage::from_bytes(&data);
    assert_eq!(msg.header.message_type, MessageType::Position);
    assert_eq!(msg.header.cyclic_counter, 0);
    let serialized = msg.to_bytes();
    assert_eq!(serialized, data);
}

/// Tests POSITION message with actual field values.
#[test]
fn test_position_fields() {
    let mut msg = PositionMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Position;
    msg.header.cyclic_counter = 3;
    msg.path_index = 42;
    msg.offset = 1000;
    msg.position_index = 2;
    msg.position_age = 300;
    msg.speed = 200;
    msg.relative_heading = 128;
    msg.probability = 20;
    msg.confidence = 5;
    msg.current_lane = 3;
    msg.reserved = 1;

    let serialized = msg.to_bytes();
    let deserialized = PositionMessage::from_bytes(&serialized);

    assert_eq!(deserialized.header.message_type, MessageType::Position);
    assert_eq!(deserialized.header.cyclic_counter, 3);
    assert_eq!(deserialized.path_index, 42);
    assert_eq!(deserialized.offset, 1000);
    assert_eq!(deserialized.position_index, 2);
    assert_eq!(deserialized.position_age, 300);
    assert_eq!(deserialized.speed, 200);
    assert_eq!(deserialized.relative_heading, 128);
    assert_eq!(deserialized.probability, 20);
    assert_eq!(deserialized.confidence, 5);
    assert_eq!(deserialized.current_lane, 3);
    assert_eq!(deserialized.reserved, 1);
}

/// Tests speed interpretation.
#[test]
fn test_position_speed_interpretation() {
    let mut msg = PositionMessage::from_bytes(&[0u8; 8]);
    msg.speed = 64;
    assert_eq!(msg.speed_mps(), Some(0.0));

    msg.speed = 511;
    assert_eq!(msg.speed_mps(), None);

    msg.speed = 65;
    assert!((msg.speed_mps().unwrap() - 0.2).abs() < 1e-5);

    msg.speed = 510;
    assert_eq!(msg.speed_mps(), Some(89.2));
}

/// Tests SEGMENT message roundtrip.
#[test]
fn test_segment_roundtrip() {
    let mut msg = SegmentMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Segment;
    msg.header.cyclic_counter = 1;
    msg.retransmission = true;
    msg.path_index = 5;
    msg.offset = 2000;
    msg.update = true;
    msg.functional_road_class = FunctionalRoadClass::Class3;
    msg.form_of_way = FormOfWay::ControlledAccess;
    msg.effective_speed_limit = 15;
    msg.effective_speed_limit_type = SpeedLimitType::ExplicitSign;
    msg.number_of_lanes_driving_direction = 3;
    msg.number_of_lanes_opposite_direction = 1;
    msg.tunnel = 1;
    msg.bridge = 0;
    msg.divided_road = 1;
    msg.built_up_area = 0;
    msg.complex_intersection = 0;
    msg.relative_probability = 20;
    msg.part_of_calculated_route = 1;
    msg.reserved = 1;

    let serialized = msg.to_bytes();
    let deserialized = SegmentMessage::from_bytes(&serialized);

    assert_eq!(deserialized.header.message_type, MessageType::Segment);
    assert_eq!(deserialized.header.cyclic_counter, 1);
    assert_eq!(deserialized.retransmission, true);
    assert_eq!(deserialized.path_index, 5);
    assert_eq!(deserialized.offset, 2000);
    assert_eq!(deserialized.update, true);
    assert_eq!(
        deserialized.functional_road_class,
        FunctionalRoadClass::Class3
    );
    assert_eq!(deserialized.form_of_way, FormOfWay::ControlledAccess);
    assert_eq!(deserialized.effective_speed_limit, 15);
    assert_eq!(
        deserialized.effective_speed_limit_type,
        SpeedLimitType::ExplicitSign
    );
    assert_eq!(deserialized.number_of_lanes_driving_direction, 3);
    assert_eq!(deserialized.number_of_lanes_opposite_direction, 1);
    assert_eq!(deserialized.tunnel, 1);
    assert_eq!(deserialized.bridge, 0);
    assert_eq!(deserialized.divided_road, 1);
    assert_eq!(deserialized.built_up_area, 0);
    assert_eq!(deserialized.complex_intersection, 0);
    assert_eq!(deserialized.relative_probability, 20);
    assert_eq!(deserialized.part_of_calculated_route, 1);
    assert_eq!(deserialized.reserved, 1);
}

/// Tests STUB message roundtrip.
#[test]
fn test_stub_roundtrip() {
    let mut msg = StubMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Stub;
    msg.header.cyclic_counter = 2;
    msg.retransmission = false;
    msg.path_index = 3;
    msg.offset = 500;
    msg.update = true;
    msg.sub_path_index = 7;
    msg.turn_angle = 128;
    msg.relative_probability = 15;
    msg.functional_road_class = FunctionalRoadClass::Class1;
    msg.form_of_way = FormOfWay::Roundabout;
    msg.num_lanes_driving_direction = 2;
    msg.num_lanes_opposite_direction = 0;
    msg.complex_intersection = 1;
    msg.right_of_way = 0;
    msg.part_of_calculated_route = 1;
    msg.last_stub_at_offset = true;

    let serialized = msg.to_bytes();
    let deserialized = StubMessage::from_bytes(&serialized);

    assert_eq!(deserialized.header.message_type, MessageType::Stub);
    assert_eq!(deserialized.header.cyclic_counter, 2);
    assert_eq!(deserialized.retransmission, false);
    assert_eq!(deserialized.path_index, 3);
    assert_eq!(deserialized.offset, 500);
    assert_eq!(deserialized.update, true);
    assert_eq!(deserialized.sub_path_index, 7);
    assert_eq!(deserialized.turn_angle, 128);
    assert_eq!(deserialized.relative_probability, 15);
    assert_eq!(
        deserialized.functional_road_class,
        FunctionalRoadClass::Class1
    );
    assert_eq!(deserialized.form_of_way, FormOfWay::Roundabout);
    assert_eq!(deserialized.num_lanes_driving_direction, 2);
    assert_eq!(deserialized.num_lanes_opposite_direction, 0);
    assert_eq!(deserialized.complex_intersection, 1);
    assert_eq!(deserialized.right_of_way, 0);
    assert_eq!(deserialized.part_of_calculated_route, 1);
    assert_eq!(deserialized.last_stub_at_offset, true);
}

/// Tests STUB turn angle interpretation.
#[test]
fn test_stub_turn_angle() {
    let mut msg = StubMessage::from_bytes(&[0u8; 8]);
    msg.turn_angle = 255;
    assert_eq!(msg.turn_angle_degrees(), None);

    msg.turn_angle = 127;
    let expected = 127.0 * 360.0 / 254.0;
    assert!((msg.turn_angle_degrees().unwrap() - expected).abs() < 1e-5);
}

/// Tests PROFILE SHORT Curvature roundtrip with interpretation.
#[test]
fn test_profile_short_curvature_roundtrip() {
    let mut msg = ProfileShortMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::ProfileShort;
    msg.profile_type = 1;
    msg.path_index = 2;
    msg.offset = 500;
    msg.update = false;
    msg.control_point = true;
    msg.value0 = 512;
    msg.distance1 = 100;
    msg.value1 = 511;
    msg.accuracy = 2;

    let serialized = msg.to_bytes();
    let deserialized = ProfileShortMessage::from_bytes(&serialized);

    assert_eq!(deserialized.profile_type, 1);
    assert_eq!(deserialized.value0, 512);
    assert_eq!(deserialized.value1, 511);
    assert_eq!(deserialized.distance1, 100);
    assert_eq!(deserialized.accuracy, 2);
    assert_eq!(deserialized.control_point, true);

    let interpreted = deserialized.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::Curvature(c) => {
            assert_eq!(c.raw_value0, Some(512));
            assert_eq!(c.raw_value1, Some(511));
            assert_eq!(c.raw_distance1, Some(100));
            assert_eq!(c.accuracy, 2);
            assert_eq!(c.control_point, true);
        }
        _ => panic!("Expected Curvature profile"),
    }
}

/// Tests all PROFILE SHORT subtypes roundtrip using a helper function.
#[test]
fn test_profile_short_all_subtypes() {
    fn check_short(profile_type: u8, is_expected: fn(&InterpretedProfileShort) -> bool) {
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
            control_point: false,
            value0: 500,
            distance1: 50,
            value1: 501,
            accuracy: 0,
        };

        let serialized = msg.to_bytes();
        let deserialized = ProfileShortMessage::from_bytes(&serialized);
        assert_eq!(deserialized.profile_type, profile_type);
        let interpreted = deserialized.interpret().unwrap();
        assert!(
            is_expected(&interpreted),
            "Failed for profile type {}",
            profile_type
        );
    }

    check_short(1, |i| matches!(i, InterpretedProfileShort::Curvature(_)));
    check_short(2, |i| matches!(i, InterpretedProfileShort::RouteNumber(_)));
    check_short(3, |i| matches!(i, InterpretedProfileShort::SlopeStep(_)));
    check_short(4, |i| matches!(i, InterpretedProfileShort::SlopeLinear(_)));
    check_short(5, |i| {
        matches!(i, InterpretedProfileShort::RoadAccessibility(_))
    });
    check_short(6, |i| {
        matches!(i, InterpretedProfileShort::RoadCondition(_))
    });
    check_short(7, |i| {
        matches!(i, InterpretedProfileShort::VariableSpeedSign(_))
    });
    check_short(8, |i| {
        matches!(i, InterpretedProfileShort::HeadingChange(_))
    });
    check_short(9, |i| matches!(i, InterpretedProfileShort::AverageSpeed(_)));
}

/// Tests PROFILE LONG Longitude roundtrip with interpretation.
#[test]
fn test_profile_long_longitude_roundtrip() {
    let msg = ProfileLongMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileLong,
            cyclic_counter: 1,
        },
        retransmission: true,
        path_index: 3,
        offset: 1000,
        update: true,
        profile_type: 1,
        control_point: false,
        value: 2000000000,
    };

    let serialized = msg.to_bytes();
    let deserialized = ProfileLongMessage::from_bytes(&serialized);

    assert_eq!(deserialized.profile_type, 1);
    assert_eq!(deserialized.value, 2000000000);

    let interpreted = deserialized.interpret().unwrap();
    match interpreted {
        InterpretedProfileLong::Longitude {
            value,
            raw_value,
            control_point,
        } => {
            assert_eq!(value, Some(20.0_f64));
            assert_eq!(raw_value, 2000000000);
            assert_eq!(control_point, false);
        }
        _ => panic!("Expected Longitude profile"),
    }
}

/// Tests all PROFILE LONG subtypes roundtrip using a helper function.
#[test]
fn test_profile_long_all_subtypes() {
    fn check_long(profile_type: u8, is_expected: fn(&InterpretedProfileLong) -> bool) {
        let value = if profile_type == 2 {
            1000000000
        } else {
            500000000
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
            control_point: false,
            value,
        };

        let serialized = msg.to_bytes();
        let deserialized = ProfileLongMessage::from_bytes(&serialized);
        assert_eq!(deserialized.profile_type, profile_type);
        let interpreted = deserialized.interpret().unwrap();
        assert!(
            is_expected(&interpreted),
            "Failed for profile type {}",
            profile_type
        );
    }

    check_long(1, |i| matches!(i, InterpretedProfileLong::Longitude { .. }));
    check_long(2, |i| matches!(i, InterpretedProfileLong::Latitude { .. }));
    check_long(3, |i| matches!(i, InterpretedProfileLong::Altitude { .. }));
    check_long(4, |i| {
        matches!(
            i,
            InterpretedProfileLong::BezierControlPointLongitude { .. }
        )
    });
    check_long(5, |i| {
        matches!(i, InterpretedProfileLong::BezierControlPointLatitude { .. })
    });
    check_long(6, |i| {
        matches!(i, InterpretedProfileLong::BezierControlPointAltitude { .. })
    });
    check_long(7, |i| {
        matches!(i, InterpretedProfileLong::LinkIdentifier { .. })
    });
    check_long(8, |i| {
        matches!(i, InterpretedProfileLong::TrafficSign { .. })
    });
    check_long(9, |i| {
        matches!(i, InterpretedProfileLong::TruckSpeedLimit { .. })
    });
    check_long(10, |i| {
        matches!(i, InterpretedProfileLong::ExtendedLane { .. })
    });
    check_long(11, |i| {
        matches!(
            i,
            InterpretedProfileLong::SpeedLimitAndRecommendedSpeed { .. }
        )
    });
}

/// Tests PROFILE LONG with invalid value (0xFFFFFFFF = N/A).
#[test]
fn test_profile_long_invalid_value() {
    let msg = ProfileLongMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileLong,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 1,
        control_point: false,
        value: 0xFFFFFFFF,
    };

    let serialized = msg.to_bytes();
    let deserialized = ProfileLongMessage::from_bytes(&serialized);
    let interpreted = deserialized.interpret().unwrap();
    match interpreted {
        InterpretedProfileLong::Longitude { value, .. } => {
            assert_eq!(value, None);
        }
        _ => panic!("Expected Longitude profile"),
    }
}

/// Tests META-DATA message roundtrip.
#[test]
fn test_metadata_roundtrip() {
    let mut msg = MetaDataMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::MetaData;
    msg.header.cyclic_counter = 2;
    msg.country_code = 840;
    msg.region_code0 = 1;
    msg.region_code1 = 2;
    msg.region_code2 = 3;
    msg.driving_side = 1;
    msg.speed_unit = 0;
    msg.major_protocol_version = 2;
    msg.minor_protocol_version = 3;
    msg.minor_protocol_sub_version = 1;
    msg.hardware_version = 100;
    msg.map_provider = 1;
    msg.map_version_year = 26;
    msg.map_version_quarter = 2;
    msg.reserved = 0;

    let serialized = msg.to_bytes();
    let deserialized = MetaDataMessage::from_bytes(&serialized);

    assert_eq!(deserialized.header.message_type, MessageType::MetaData);
    assert_eq!(deserialized.header.cyclic_counter, 2);
    assert_eq!(deserialized.country_code, 840);
    assert_eq!(deserialized.region_code0, 1);
    assert_eq!(deserialized.region_code1, 2);
    assert_eq!(deserialized.region_code2, 3);
    assert_eq!(deserialized.driving_side, 1);
    assert_eq!(deserialized.speed_unit, 0);
    assert_eq!(deserialized.major_protocol_version, 2);
    assert_eq!(deserialized.minor_protocol_version, 3);
    assert_eq!(deserialized.minor_protocol_sub_version, 1);
    assert_eq!(deserialized.hardware_version, 100);
    assert_eq!(deserialized.map_provider, 1);
    assert_eq!(deserialized.map_version_year, 26);
    assert_eq!(deserialized.map_version_quarter, 2);
    assert_eq!(deserialized.reserved, 0);

    assert_eq!(deserialized.map_provider_enum(), MapProvider::Here);
    assert_eq!(deserialized.map_version_year_full(), Some(2026));
    assert_eq!(
        deserialized.region_code_full(),
        ((1u16 << 10) | (2u16 << 5) | 3)
    );
}

/// Tests the high-level deserialize function for POSITION.
#[test]
fn test_deserialize_position() {
    let mut msg = PositionMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Position;
    msg.header.cyclic_counter = 1;
    msg.path_index = 42;
    msg.offset = 5000;
    msg.speed = 100;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::Position(p) => {
            assert_eq!(p.header.cyclic_counter, 1);
            assert_eq!(p.path_index, 42);
            assert_eq!(p.offset, 5000);
            assert_eq!(p.speed, 100);
        }
        _ => panic!("Expected Position message"),
    }
}

/// Tests the high-level deserialize function for SEGMENT.
#[test]
fn test_deserialize_segment() {
    let mut msg = SegmentMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Segment;
    msg.retransmission = true;
    msg.path_index = 3;
    msg.offset = 1000;
    msg.functional_road_class = FunctionalRoadClass::Class2;
    msg.form_of_way = FormOfWay::SingleCarriageway;
    msg.effective_speed_limit = 20;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::Segment(s) => {
            assert_eq!(s.retransmission, true);
            assert_eq!(s.path_index, 3);
            assert_eq!(s.offset, 1000);
            assert_eq!(s.functional_road_class, FunctionalRoadClass::Class2);
            assert_eq!(s.form_of_way, FormOfWay::SingleCarriageway);
            assert_eq!(s.effective_speed_limit, 20);
        }
        _ => panic!("Expected Segment message"),
    }
}

/// Tests the high-level deserialize function for STUB.
#[test]
fn test_deserialize_stub() {
    let mut msg = StubMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::Stub;
    msg.retransmission = false;
    msg.path_index = 1;
    msg.offset = 200;
    msg.sub_path_index = 5;
    msg.turn_angle = 64;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::Stub(s) => {
            assert_eq!(s.retransmission, false);
            assert_eq!(s.path_index, 1);
            assert_eq!(s.offset, 200);
            assert_eq!(s.sub_path_index, 5);
            assert_eq!(s.turn_angle, 64);
        }
        _ => panic!("Expected Stub message"),
    }
}

/// Tests the high-level deserialize function for PROFILE_SHORT.
#[test]
fn test_deserialize_profile_short() {
    let mut msg = ProfileShortMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::ProfileShort;
    msg.profile_type = 3;
    msg.path_index = 2;
    msg.offset = 500;
    msg.value0 = 300;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::ProfileShort(InterpretedProfileShort::SlopeStep(s)) => {
            assert_eq!(s.raw_value0, Some(300));
        }
        _ => panic!("Expected SlopeStep profile short"),
    }
}

/// Tests the high-level deserialize function for PROFILE_LONG.
#[test]
fn test_deserialize_profile_long() {
    let mut msg = ProfileLongMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::ProfileLong;
    msg.profile_type = 3;
    msg.path_index = 1;
    msg.offset = 300;
    msg.value = 100000;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::ProfileLong(InterpretedProfileLong::Altitude {
            value, raw_value, ..
        }) => {
            assert_eq!(raw_value, 100000);
            assert!((value.unwrap() - 0.0).abs() < 1e-3);
        }
        _ => panic!("Expected Altitude profile long"),
    }
}

/// Tests the high-level deserialize function for META-DATA.
#[test]
fn test_deserialize_metadata() {
    let mut msg = MetaDataMessage::from_bytes(&[0u8; 8]);
    msg.header.message_type = MessageType::MetaData;
    msg.country_code = 276;

    let serialized = msg.to_bytes();
    let deserialized = deserialize(&serialized).unwrap();

    match deserialized {
        Message::MetaData(m) => {
            assert_eq!(m.country_code, 276);
        }
        _ => panic!("Expected MetaData message"),
    }
}

/// Tests serialize of interpreted ProfileShort.
#[test]
fn test_serialize_interpreted_profile_short() {
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 9,
        control_point: false,
        value0: 50,
        distance1: 10,
        value1: 55,
        accuracy: 0,
    };

    let raw = msg.to_bytes();
    let deserialized = ProfileShortMessage::from_bytes(&raw);
    let interpreted = deserialized.interpret().unwrap();

    let serialized = serialize(&Message::ProfileShort(interpreted));
    assert_eq!(serialized, raw);
}

/// Tests serialize of interpreted ProfileLong.
#[test]
fn test_serialize_interpreted_profile_long() {
    let msg = ProfileLongMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileLong,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 7,
        control_point: false,
        value: 12345,
    };

    let raw = msg.to_bytes();
    let deserialized = ProfileLongMessage::from_bytes(&raw);
    let interpreted = deserialized.interpret().unwrap();

    let serialized = serialize(&Message::ProfileLong(interpreted));
    assert_eq!(serialized, raw);
}

/// Tests curvature decode/encode roundtrip.
#[test]
fn test_curvature_roundtrip() {
    for raw in 0u16..=1022u16 {
        let decoded = decode_curvature(raw as i32);
        let encoded = encode_curvature(decoded);
        assert_eq!(encoded, raw, "Failed for raw value {}", raw);
    }
}

/// Tests curvature decode for unknown value.
#[test]
fn test_curvature_unknown() {
    let decoded = decode_curvature(1023);
    assert_eq!(decoded, 0.0);
}

/// Tests curvature decode for extreme values.
#[test]
fn test_curvature_extremes() {
    let c = decode_curvature(0);
    assert!((c - (-0.16192)).abs() < 1e-4);

    let c = decode_curvature(1022);
    assert!((c - 0.16192).abs() < 1e-4);
}

/// Tests that curvature interpretation uses None for N/A raw value.
#[test]
fn test_curvature_na_interpretation() {
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 1, // Curvature
        control_point: false,
        value0: PROFILE_VALUE_INVALID_10,
        distance1: PROFILE_VALUE_INVALID_10,
        value1: PROFILE_VALUE_INVALID_10,
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::Curvature(c) => {
            assert_eq!(c.raw_value0, None);
            assert_eq!(c.raw_value1, None);
            assert_eq!(c.raw_distance1, None);
            assert_eq!(c.curvature0, None);
            assert_eq!(c.curvature1, None);
            assert_eq!(c.distance1, None);
        }
        _ => panic!("Expected Curvature profile"),
    }
}

/// Tests SlopeStep interpretation with actual decoded slope values.
#[test]
fn test_slope_step_interpretation() {
    // raw value 1000 -> slope = 1000 * 0.1 - 51.1 = 100.0 - 51.1 = 48.9%
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 3, // Slope Step
        control_point: false,
        value0: 1000,
        distance1: 50,
        value1: 500,
        accuracy: 1,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::SlopeStep(s) => {
            assert!((s.slope0.unwrap() - 48.9).abs() < 1e-4);
            assert_eq!(s.raw_value0, Some(1000));
            assert!((s.distance1.unwrap() - 50.0).abs() < 1e-4);
            assert!((s.slope1.unwrap() - (-1.1)).abs() < 1e-4); // 500 * 0.1 - 51.1 = -1.1
            assert_eq!(s.raw_value1, Some(500));
        }
        _ => panic!("Expected SlopeStep profile"),
    }
}

/// Tests RoadAccessibility interpretation (Type 5) using the shared RoadAccessibility struct.
#[test]
fn test_road_accessibility_interpretation() {
    // raw value with passenger_cars and trucks accessible
    let raw = 0x01 | 0x80; // passenger_cars + trucks
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 5, // Road Accessibility
        control_point: false,
        value0: raw,
        distance1: 30,
        value1: 0x02 | 0x04, // pedestrians + bus
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::RoadAccessibility(r) => {
            assert!(r.accessibility.as_ref().unwrap().passenger_cars);
            assert!(r.accessibility.as_ref().unwrap().trucks);
            assert!(!r.accessibility.as_ref().unwrap().pedestrians);
            assert!(r.accessibility1.as_ref().unwrap().pedestrians);
            assert!(r.accessibility1.as_ref().unwrap().bus);
        }
        _ => panic!("Expected RoadAccessibility profile"),
    }
}

/// Tests RoadCondition interpretation (Type 6).
#[test]
fn test_road_condition_interpretation() {
    // raw value with condition bits = 2 (Wet), upper bits reserved
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 6, // Road Condition
        control_point: false,
        value0: 0x02,
        distance1: 40,
        value1: 0x04,
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::RoadCondition(r) => {
            assert_eq!(r.condition, Some(RoadCondition::Wet));
            assert_eq!(r.condition1, Some(RoadCondition::Icy));
        }
        _ => panic!("Expected RoadCondition profile"),
    }
}

/// Tests VariableSpeedSign interpretation (Type 7).
#[test]
fn test_variable_speed_sign_interpretation() {
    // Just test that it deserializes correctly with N/A values
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 7, // Variable Speed Sign
        control_point: false,
        value0: PROFILE_VALUE_INVALID_10,
        distance1: 50,
        value1: 200,
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::VariableSpeedSign(v) => {
            assert_eq!(v.sign, None); // N/A
            assert_eq!(v.raw_value0, None);
        }
        _ => panic!("Expected VariableSpeedSign profile"),
    }
}

/// Tests HeadingChange interpretation (Type 8).
#[test]
fn test_heading_change_interpretation() {
    // raw 256 -> 256 * (360/511) = ~179.88 degrees
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 8, // Heading Change
        control_point: false,
        value0: 256,
        distance1: 100,
        value1: 510,
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::HeadingChange(h) => {
            let expected0 = 256.0 * 360.0 / 511.0;
            assert!((h.heading0.unwrap() - expected0).abs() < 1e-3);
            let expected1 = 510.0 * 360.0 / 511.0;
            assert!((h.heading1.unwrap() - expected1).abs() < 1e-3);
        }
        _ => panic!("Expected HeadingChange profile"),
    }
}

/// Tests AverageSpeed interpretation (Type 9).
#[test]
fn test_average_speed_interpretation() {
    // raw 100 -> 100 * 0.5 = 50.0 km/h
    let msg = ProfileShortMessage {
        header: AdasisHeader {
            message_type: MessageType::ProfileShort,
            cyclic_counter: 0,
        },
        retransmission: false,
        path_index: 0,
        offset: 0,
        update: false,
        profile_type: 9, // Average Speed
        control_point: false,
        value0: 100,
        distance1: 200,
        value1: 200,
        accuracy: 0,
    };

    let interpreted = msg.interpret().unwrap();
    match interpreted {
        InterpretedProfileShort::AverageSpeed(a) => {
            assert!((a.speed0.unwrap() - 50.0).abs() < 1e-4);
            assert!((a.speed1.unwrap() - 100.0).abs() < 1e-4);
        }
        _ => panic!("Expected AverageSpeed profile"),
    }
}
