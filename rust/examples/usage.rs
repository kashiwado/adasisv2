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
        path_index: 42,
        offset: 1000,
        position_index: 1,
        position_age: 300,
        speed: 100,
        relative_heading: 128,
        probability: 15,
        confidence: 5,
        current_lane: 3,
        reserved: 0,
    };

    // Serialize to bytes
    let bytes = msg.to_bytes();
    println!("Serialized POSITION message: {:02X?}", bytes);

    // Get message type from raw bytes
    let msg_type = get_message_type(&bytes);
    println!("Message type: {:?}", msg_type);

    // Deserialize
    let deserialized = deserialize(&bytes).unwrap();
    println!("Deserialized: {:?}", deserialized);

    // Serialize back
    let reserialized = serialize(&deserialized);
    println!("Reserialized: {:02X?}", reserialized);
    assert_eq!(bytes, reserialized);
    println!("Roundtrip successful!");
}
