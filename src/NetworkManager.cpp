#include "NetworkManager.h"

#include <array>
#include <chrono>
#include <cstring>
#include <queue>
#include <ranges>
#include <string>
#include <thread>
#include <vector>

#include "NetworkMessage.h"
#include "quill/Frontend.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/std/Vector.h"

static constexpr auto kLoggerName = "NetworkThread";

struct Session {
  bool IsActive;
  boost::asio::ip::tcp::socket Socket;
  uint32_t Id;
  boost::asio::steady_timer HeartbeatTimer;
  NetworkMessage LastMessage;
  std::queue<NetworkMessage> PendingWrites;
  std::array<uint8_t, MESSAGE_SIZE> RawByteBuffer;
};

NetworkManager::NetworkManager(const std::optional<std::string> &hostname,
                               int port)
    : _ioContext(boost::asio::io_context()), _nextSessionId(1) {
  _logger = quill::Frontend::create_or_get_logger(
      kLoggerName,
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1"));
  _networkThread =
      std::jthread(&NetworkManager::StartNetworkThread, this, hostname, port);
}

NetworkManager::~NetworkManager() {
  _ioContext.stop();
  _networkThread.join();
  quill::Frontend::remove_logger_blocking(_logger);
}

void NetworkManager::StartNetworkThread(
    const std::optional<std::string> &hostname, int port) {
  boost::asio::ip::tcp::acceptor acceptor(
      _ioContext,
      boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));

  LOG_INFO(_logger, "TCP Server started at {}:{}",
           acceptor.local_endpoint().address().to_string(),
           acceptor.local_endpoint().port());

  AsyncAccept(acceptor);
  _ioContext.run();
}

void NetworkManager::SendNetworkMessage(uint32_t sessionId,
                                        NetworkMessage &msg) {
  boost::asio::post(_ioContext, [this, sessionId, msg]() {
    std::shared_ptr<Session> session = nullptr;

    {
      std::lock_guard<std::mutex> lock(_sessionLock);
      auto sessionItr = _sessions.find(sessionId);
      if (sessionItr == _sessions.end()) {
        return;
      }
      session = sessionItr->second;
    }

    // If there is something in the queue, it means there is an active write
    // going on and we are backed up
    session->PendingWrites.emplace(std::move(msg));
    AsyncSend(session);
  });
}

void NetworkManager::BroadcastMessage(const NetworkMessage &msg) {
  boost::asio::post(_ioContext, [this, msg]() {
    // Copy all the sessions in case its deleted. Worst case we send a message
    // to a session thats been terminated but thats not a big deal
    std::vector<std::shared_ptr<Session>> sessions;
    {
      std::lock_guard<std::mutex> lock(_sessionLock);
      std::ranges::transform(_sessions.begin(), _sessions.end(),
                             std::back_inserter(sessions),
                             [](const auto &elm) { return elm.second; });
    }
    for (auto &session : sessions) {
      if (!session->IsActive) {
        continue;
      }
      // If there is something in the queue, it means there is an active write
      // going on and we are backed up
      session->PendingWrites.emplace(msg);
      AsyncSend(session);
    }
  });
}

void NetworkManager::RemoveSession(uint32_t sessionId) {
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
  std::memcpy(leave_msg.Data.data(), &sessionId, sizeof(uint32_t));
  leave_msg.Size = sizeof(uint32_t);
  BroadcastMessage(leave_msg);
}

void NetworkManager::AsyncAccept(boost::asio::ip::tcp::acceptor &acceptor) {
  acceptor.async_accept([this, &acceptor](std::error_code ec,
                                          boost::asio::ip::tcp::socket socket) {
    if (!ec) {
      uint32_t sessionId = _nextSessionId++;
      auto session =
          std::make_shared<Session>(false, std::move(socket), sessionId,
                                    boost::asio::steady_timer(_ioContext));
      {
        std::lock_guard<std::mutex> lock(_sessionLock);
        _sessions.emplace(sessionId, session);
      }
      LOG_INFO(_logger, "New Session created, ID: {}", sessionId);

      StartSession(session);
    }
    AsyncAccept(acceptor);
  });
}

void NetworkManager::AsyncSend(std::shared_ptr<Session> session) {
  if (session->PendingWrites.empty()) {
    return;
  }

  boost::asio::async_write(
      session->Socket,
      boost::asio::buffer(Serialize(session->PendingWrites.front()),
                          session->PendingWrites.front().Size),
      [this, session](boost::system::error_code ec, std::size_t bytesSent) {
        LOG_DEBUG(_logger,
                  "Message Sent; Session={}; Type={}; Size={}; "
                  "BytesSent={}",
                  session->Id,
                  static_cast<uint32_t>(session->PendingWrites.front().Type),
                  session->PendingWrites.front().Size, bytesSent);
        if (!ec) {
          session->PendingWrites.pop();
          if (!session->PendingWrites.empty()) {
            AsyncSend(session);
          }
        } else {
          LOG_WARNING(_logger,
                      "Failed to send message to session {}: [{}] Error {}",
                      session->Id, ec.value(), ec.message());
          RemoveSession(session->Id);
        }
      });
}

void NetworkManager::StartSession(std::shared_ptr<Session> &session) {
  session->IsActive = true;
  StartHeartbeatTimer(session);
  AcceptSessionMessage(session);
}

void NetworkManager::StartHeartbeatTimer(std::shared_ptr<Session> session) {
  session->HeartbeatTimer.expires_after(std::chrono::seconds(1));
  session->HeartbeatTimer.async_wait([this, session](
                                         const boost::system::error_code &ec) {
    if (!session->IsActive || ec) {
      if (ec) {
        LOG_WARNING(_logger,
                    "Failed to accept hearbeat from session {}: [{}] Error {}",
                    session->Id, ec.value(), ec.message());
      } else {
        LOG_WARNING(_logger, "Session {} failed heartbeat check", session->Id);
      }
      RemoveSession(session->Id);
      return;
    }
    session->IsActive = false;
    StartHeartbeatTimer(session);
  });
}

void NetworkManager::AckHeartbeat(uint32_t sessionId) {
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    auto it = _sessions.find(sessionId);
    if (it == _sessions.end()) {
      return;
    }
    it->second->IsActive = true;
  }
  LOG_DEBUG(_logger, "Heartbeat received from session {}", sessionId);
  NetworkMessage msg;
  msg.Size = MESSAGE_SIZE;
  msg.Type = MessageType::HEARTBEAT;
  msg.Data.fill(0);
  SendNetworkMessage(sessionId, msg);
}

void NetworkManager::HandleSessionMessage(uint32_t sessionId,
                                          const NetworkMessage &msg) {
  switch (msg.Type) {
    case MessageType::CONNECT:
      break;
    case MessageType::DISCONNECT:
      RemoveSession(sessionId);
      break;
    case MessageType::HEARTBEAT:
      AckHeartbeat(sessionId);
      break;
    case MessageType::USER:
      HandleUserMessage(msg);
      break;
    default:
      break;
  }
}

void NetworkManager::HandleUserMessage(const NetworkMessage &msg) {
  if (_handleUserPacketCallback) {
    std::vector<uint8_t> debug(msg.Data.begin(), msg.Data.end());
    for (auto &v : msg.Data) {
      LOG_INFO(_logger, "DEBUG MESSAGE = {}; ", debug);
    }
    _handleUserPacketCallback(msg.Data);
  }
}

void NetworkManager::AcceptSessionMessage(std::shared_ptr<Session> session) {
  boost::asio::async_read(
      session->Socket,
      boost::asio::buffer(&session->RawByteBuffer, MESSAGE_SIZE),
      [this, session](std::error_code ec, std::size_t bytesRecv) {
        session->LastMessage = Deserialize(session->RawByteBuffer);
        LOG_INFO(_logger,
                 "Message Received; Session={}; Type={}; Size={}; "
                 "BytesRecv={}",
                 session->Id, static_cast<uint32_t>(session->LastMessage.Type),
                 session->LastMessage.Size, bytesRecv);
        if (!ec) {
          HandleSessionMessage(session->Id, session->LastMessage);
          AcceptSessionMessage(session);
        } else {
          LOG_WARNING(_logger,
                      "Failed to accept message from session {}: [{}] Error {}",
                      session->Id, ec.value(), ec.message());
          RemoveSession(session->Id);
        }
      });
}