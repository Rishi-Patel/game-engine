#include "NetworkMessage.h"

#include <stdexcept>

std::array<uint8_t, MESSAGE_SIZE> Serialize(const NetworkMessage& msg) {
  std::array<uint8_t, MESSAGE_SIZE> buffer{};
  size_t offset = 0;
  
  // Serialize type
  offset +=
      SerializeNum(buffer.data() + offset, static_cast<uint8_t>(msg.Type));

  // Serialize size
  offset += SerializeNum(buffer.data() + offset, msg.Size);

  // Serialize data
  std::memcpy(buffer.data() + offset, msg.Data.data(),
              std::max(0, static_cast<int>(std::min(msg.Size, MESSAGE_DATA_SIZE))));

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
  
  // Read data
  std::memcpy(msg.Data.data(), buffer.data() + offset, std::min(msg.Size, MESSAGE_DATA_SIZE));

  return msg;
}