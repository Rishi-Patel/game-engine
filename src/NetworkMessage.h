#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <string>
#include <ostream>

/* HostToNetwork: Converts integer from host endian to
 *                network endian
 *
 * @num:    An integer-type number
 * @return: Unsigned type of equal byte size as input type
 *          in network endian order
 */
template <std::integral T>
constexpr std::make_unsigned_t<T> HostToNetwork(const T& num) {
  using U = std::make_unsigned_t<T>;
  // 1 byte integers or hosts with big endian encodings dont
  // need to be converted
  if constexpr (sizeof(U) == 1 || std::endian::native == std::endian::big) {
    return static_cast<U>(num);
  } else {
    return std::byteswap(static_cast<U>(num));
  }
}

/* NetworkToHost: Converts integer from network endian to
 *                host endian
 *
 * @num:    An integer-type number
 * @return: Unsigned type of equal byte size as input type
 *          in host endian order
 */
template <std::integral T>
constexpr T NetworkToHost(const T& num) {
  // 1 byte integers or hosts with big endian encodings dont
  // need to be converted
  if constexpr (sizeof(T) == 1 || std::endian::native == std::endian::big) {
    return num;
  } else {
    return std::byteswap(num);
  }
}

template <std::integral T>
uint32_t SerializeNum(uint8_t* buffer, const T& num) {
  std::make_unsigned_t<T> serializedNum = HostToNetwork(num);
  std::memcpy(buffer, &serializedNum, sizeof(serializedNum));
  return sizeof(serializedNum);
}

template <std::floating_point T>
uint32_t SerializeNum(uint8_t* buffer, const T& num) {
  // Annoyingly, I cant think of a better way to map floating points
  // to their integral sizes than to write them here
  // At least if I dont have a type in the map I can throw a compile time
  // error and add it here.
  using U = std::make_unsigned_t<
      std::conditional_t<sizeof(T) == 4, uint32_t,
                         std::conditional_t<sizeof(T) == 8, uint64_t, void>>>;
  static_assert(!std::is_void_v<U>,
                "Unsupported floating-point size. Only support up to 8 byte "
                "floating point numbers. Please add type above.");
  return SerializeNum(buffer, std::bit_cast<U>(num));
}

enum class MessageType : uint8_t {
  UNKNOWN = 0,
  CONNECT,
  DISCONNECT,
  HEARTBEAT,
  USER
};

constexpr uint32_t MESSAGE_DATA_SIZE = 1024;
constexpr uint32_t MESSAGE_HEADER_SIZE = sizeof(MessageType) + sizeof(uint32_t);
constexpr uint32_t MESSAGE_SIZE =
    MESSAGE_HEADER_SIZE + MESSAGE_DATA_SIZE;

struct NetworkMessage {
  MessageType Type;
  uint32_t Size;
  std::array<uint8_t, MESSAGE_DATA_SIZE> Data;
};

static constexpr auto kHeartbeatMessage = NetworkMessage{.Type = MessageType::HEARTBEAT, .Size = 0, .Data = {}};
static constexpr auto kDisconnectMessage = NetworkMessage{.Type = MessageType::DISCONNECT, .Size = 0, .Data = {}};

inline std::string ToString(const MessageType& type) {
  switch (type) {
    case MessageType::CONNECT:
      return "CONNECT";
    case MessageType::DISCONNECT:
      return "DISCONNECT";
    case MessageType::HEARTBEAT:
      return "HEARTBEAT";
    case MessageType::USER:
      return "USER";
    default:
      return "UNKNOWN";
  }
}

inline std::ostream& operator<<(std::ostream& os, const NetworkMessage& msg) {
  os << "Header: Type=" << ToString(msg.Type) << ", Size=" << msg.Size << "\nData=[";
  for (size_t i = 0; i < std::min(msg.Size, MESSAGE_DATA_SIZE); i++) {
    os << static_cast<int>(msg.Data[i]) << " ";
  }
  os << "]";
  return os;
}

std::array<uint8_t, MESSAGE_SIZE> Serialize(const NetworkMessage& msg);
NetworkMessage DeserializeHeader(const std::array<uint8_t, MESSAGE_SIZE>& buffer);