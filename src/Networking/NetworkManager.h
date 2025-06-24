#pragma once

#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace quill {
class Logger;
}
struct Session;

constexpr auto MESSAGE_SIZE = 1024;

enum class MessageType : uint8_t {
  UNKNOWN = 0,
  CONNECT,
  DISCONNECT,
  HEARTBEAT,
  USER
};

struct NetworkMessage {
  MessageType Type;
  uint32_t Size;
  std::array<uint8_t, MESSAGE_SIZE> Data;
};

class NetworkManager {
public:
  NetworkManager(boost::asio::io_context &ioContext, int port);

  ~NetworkManager();

  void SendMessage(uint32_t sessionId, NetworkMessage &msg);
  void BroadcastMessage(const NetworkMessage &msg);
  void RemoveSession(uint32_t sessionId);

private:
  void AsyncAccept();

  void AsyncSend(std::shared_ptr<Session> &session);
  void StartSession(std::shared_ptr<Session> &session);
  void AcceptSessionMessage(std::shared_ptr<Session> &session);
  void HandleSessionMessage(uint32_t sessionId, const NetworkMessage &msg);

private:
  uint32_t _nextSessionId;
  boost::asio::io_context &_ioContext;
  boost::asio::ip::tcp::acceptor _acceptor;
  quill::Logger *_logger;

  std::unordered_map<uint32_t, std::shared_ptr<Session>> _sessions;
  std::mutex _sessionLock;
};
