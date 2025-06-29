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

#include "quill/Frontend.h"
#include "quill/Logger.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"

static constexpr auto kLoggerName = "NetworkThread";

// Game message types
enum class MessageType : uint8_t {
  UNKNOWN = 0,
  CONNECT,
  DISCONNECT,
  HEARTBEAT,
  USER
};

constexpr auto MESSAGE_SIZE = 1024;
// Network message structure
struct NetworkMessage {
  MessageType Type;
  uint32_t Size;
  std::array<uint8_t, MESSAGE_SIZE> Data;
};

std::array<uint8_t, sizeof(NetworkMessage)> Serialize(
    const NetworkMessage &msg) {
  std::array<uint8_t, sizeof(NetworkMessage)> buffer{};
  size_t offset = 0;
  // Copy Type: 1 byte so no endian worries
  std::memcpy(buffer.data() + offset, &msg.Type, sizeof(msg.Type));
  offset += sizeof(msg.Type);

  // Copy Size
  uint32_t net_size =
      boost::asio::detail::socket_ops::host_to_network_long(msg.Size);
  std::memcpy(buffer.data() + offset, &net_size, sizeof(net_size));
  offset += sizeof(net_size);

  std::memcpy(buffer.data() + offset, msg.Data.data(), msg.Data.size());

  return buffer;
}

class NetworkManager {
 public:
  NetworkManager(Logger *logger, boost::asio::io_context &ioContext, int port)
      : _ioContext(ioContext),
        _acceptor(_ioContext, boost::asio::ip::tcp::endpoint(
                                  boost::asio::ip::tcp::v4(), port)),
        _nextSessionId(1) {
    auto console_sink =
        quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1");
    _logger = quill::Frontend::create_or_get_logger(kLoggerName,
                                                    std::move(console_sink));

    LOG_INFO(_logger->GetLogger(), "TCP Server started at {}:{}",
             _acceptor.local_endpoint().address(),
             _acceptor.local_endpoint().port());
    AsyncAccept();
  }

  ~NetworkManager() { quill::Frontend::remove_logger_blocking(_logger); }

  void SendMessage(uint32_t sessionId, NetworkMessage &msg) {
    boost::asio::post(_ioContext, [this, sessionId, msg]() {
      if (_sessions.find(sessionId) == _sessions.end()) {
        return;
      }
      auto &session = _sessions[sessionId];
      // If there is something in the queue, it means there is an active write
      // going on and we are backed up
      bool isCurrentlyWriting = !session.PendingWrites.empty();
      session.PendingWrites.emplace(std::move(msg));
      AsyncSend(session);
    });
  }

  void BroadcastMessage(const NetworkMessage &msg) {
      boost::asio::post(_ioContext, [this, msg]() {
      for (auto &[sessionId, session] : _sessions) {
        // If there is something in the queue, it means there is an active write
        // going on and we are backed up
        bool isCurrentlyWriting = !session.PendingWrites.empty();
        session.PendingWrites.emplace(msg);
        AsyncSend(session);
      }
      }
  }

  void RemoveSession(uint32_t sessionId) {
    {
      std::lock_guard<std::mutex> lock(_sessionLock);
      if (_sessions.find(sessionId) == _sessions.end()) {
        return;
      }
      _sessions[sessionId]->IsActive = false;
      _sessions.erase(sessionId);
    }

    // Notify other sessions
    NetworkMessage leave_msg(MessageType::DISCONNECT);
    memcpy(leave_msg.data, &sessionId, sizeof(uint32_t));
    leave_msg.size = sizeof(uint32_t);
    BroadcastMessage(leave_msg);
  }

 private:
  struct Session {
    bool IsActive;
    boost::asio::ip::tcp::socket Socket;
    uint32_t Id;
    NetworkMessage LastMessage;
    std::queue<NetworkMessage> PendingWrites;
  };

  void AsyncAccept() {
    _acceptor.async_accept(
        [this](std::error_code ec, boost::asio::ip::tcp::socket socket) {
          if (!ec) {
            uint32_t sessionId = _nextSessionId++;
            auto session =
                std::make_shared<Session>(false, std::move(socket), sessionId);
            {
              std::lock_guard<std::mutex> lock(_sessionLock);
              _sessions.emplace(sessionId, session);
            }
            LOG_INFO(_logger->GetLogger(), "New Session created, ID: {}",
                     sessionId);

            StartSession(session);
          }
          AsyncAccept(session);
        });
  }

  void AsyncSend(std::shared_ptr<Session> &session) {
    boost::asio::async_write(
        session->Socket,
        boost::asio::buffer(Serialize(session->PendingWrites.front()),
        [this](boost::system::error_code ec, std::size_t /*length*/) {
      if (!ec) {
        session->PendingWrites.pop();
        if (!session->PendingWrites.empty()) {
          AsyncSend(session);
        }
      } else {
        RemoveSession(session->Id);
      }
        });
  }

  void StartSession(std::shared_ptr<Session> &session) {
    session->IsActive = true;
    AcceptSessionMessage(session);
  }

  void AcceptSessionMessage(std::shared_ptr<Session> &session) {
    asio::async_read(
        session->Socket,
        /* asio::buffer(&session->LastMessage, sizeof(NetworkMessage))*/,
        [this](std::error_code ec, std::size_t) {
          msg.Size =
              boost::asio::detail::socket_ops::network_to_host_long(msg.Size);

          LOG_DEBUG(_logger->GetLogger(),
                    "Message Received; Session={}; Type={}; Size={}",
                    session->Id, session->LastMessage.Type,
                    session->LastMessage.Size);
          if (!ec) {
            HandleSessionMessage(session->Id, session->LastMessage);
            AcceptSessionMessage();
          } else {
            RemoveSession(session->Id);
          }
        });
  }

  void HandleSessionMessage(uint32_t sessionId, const NetworkMessage &msg) {
    switch (msg.Type) {
      case MessageType::CONNECT:
        break;
      case MessageType::DISCONNECT:
        RemoveSession(sessionId);
        break;
      case MessageType::HEARTBEAT:
        AckHeartbeat();
        break;
      case MessageType::USER:
        break;
      default:
        break;
    }
  }

 private:
  uint32_t _nextSessionId;
  boost::asio::io_context &_ioContext;
  boost::asio::ip::tcp::acceptor _acceptor;
  Logger *_logger;

  std::unordered_map<uint32_t, std::shared_ptr<Session>> _sessions;
  std::mutex _sessionLock;
};
