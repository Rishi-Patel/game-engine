#include "NetworkMessage.h"

std::array<uint8_t, MESSAGE_SIZE> Serialize(const NetworkMessage& msg) {
  std::array<uint8_t, MESSAGE_SIZE> buffer{};
  size_t offset = 0;

  offset +=
      SerializeNum(buffer.data() + offset, static_cast<uint8_t>(msg.Type));
  offset += SerializeNum(buffer.data() + offset, msg.Size);

  std::memcpy(buffer.data() + offset, msg.Data.data(),
              std::max(0, static_cast<int>(msg.Size - offset)));
  offset += static_cast<int>(msg.Size - offset);

  return buffer;
}

NetworkMessage Deserialize(const std::array<uint8_t, MESSAGE_SIZE>& buffer) {
  NetworkMessage msg{};
  size_t offset = 0;

  // Read type
  msg.Type = static_cast<MessageType>(buffer[offset]);
  offset += sizeof(uint8_t);

  // Read size
  std::memcpy(&msg.Size, buffer.data() + offset, sizeof(msg.Size));
  msg.Size = NetworkToHost(msg.Size);
  offset += sizeof(msg.Size);

  // Validate size
  if (msg.Size < offset || msg.Size > MESSAGE_SIZE) {
    throw std::runtime_error("Invalid message size");
  }

  size_t payloadSize = msg.Size - offset;

  if (payloadSize > MESSAGE_DATA_SIZE) {
    throw std::runtime_error("Payload too large");
  }

  std::memcpy(msg.Data.data(), buffer.data() + offset, payloadSize);

  return msg;
}