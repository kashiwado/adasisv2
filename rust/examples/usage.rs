use adasisv2::{
    deserialize, get_message_type, serialize, AdasisHeader, MessageType, PositionMessage,
};

fn main() {
    // Example: create a POSITION message and serialize it
    let msg = PositionMessage {
        header: AdasisHeader {
            message_type: MessageType::Position,
            cyclic_counter: 3,
        },
        path_index: 8,
        offset: 82,
        position_index: 0,
        position_age: 510,
        speed: 89,
        relative_heading: 253,
        probability: 26,
        confidence: 2,
        current_lane: 7,
        reserved: 0,
    };

    // Serialize to bytes (big_endian=true)
    let bytes = msg.to_bytes(true);
    println!("Serialized POSITION message: {:02X?}", bytes);

    // Get message type from raw bytes
    let msg_type = get_message_type(&bytes, true);
    println!("Message type: {:?}", msg_type);

    // Deserialize
    let deserialized = deserialize(&bytes, true).unwrap();
    println!("Deserialized: {:?}", deserialized);

    // Serialize back
    let reserialized = serialize(&deserialized, true);
    println!("Reserialized: {:02X?}", reserialized);
    assert_eq!(bytes, reserialized);
    println!("Roundtrip successful!");
}
