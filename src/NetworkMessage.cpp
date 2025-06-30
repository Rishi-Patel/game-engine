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
  NetworkMessage msg;
  size_t offset = 0;

  msg.Type = static_cast<MessageType>(buffer[0]);
  std::memcpy(&msg.Size, buffer.data() + sizeof(msg.Type), sizeof(msg.Size));
  msg.Size = NetworkToHost(msg.Size);
  std::memcpy(msg.Data.data(),
              buffer.data() + sizeof(msg.Type) + sizeof(msg.Size),
              msg.Size - sizeof(msg.Type) + sizeof(msg.Size));
  return msg;
}