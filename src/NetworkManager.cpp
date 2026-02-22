#include "NetworkManager.h"

#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <ranges>
#include <string>
#include <thread>
#include <vector>

#include "NetworkMessage.h"
#include "quill/Frontend.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"

static constexpr auto kLoggerName = "NetworkThread";

class Session : public std::enable_shared_from_this<Session> {
public:
  Session(bool isClient, bool isActive, boost::asio::ip::tcp::socket socket,
          uint32_t id, boost::asio::steady_timer timer)
      : IsClient(isClient), IsActive(isActive), Socket(std::move(socket)),
        Id(id), HeartbeatTimer(std::move(timer)) {}
  ~Session() { Socket.close(); }
  bool IsClient;
  bool IsActive;
  boost::asio::ip::tcp::socket Socket;
  uint32_t Id;
  boost::asio::steady_timer HeartbeatTimer;
  NetworkMessage LastMessage;
  std::queue<NetworkMessage> PendingWrites;
  std::array<uint8_t, MESSAGE_SIZE> RawByteBuffer;
};

NetworkManager::NetworkManager(const std::optional<Endpoint> &endpoint)
    : _ioContext(boost::asio::io_context()),
      _workGuard(boost::asio::make_work_guard(_ioContext)),
      _acceptor(_ioContext), _resolver(_ioContext), _nextSessionId(1) {
  _logger = quill::Frontend::create_or_get_logger(
      kLoggerName,
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1"));
  _networkThread =
      std::jthread(&NetworkManager::StartNetworkThread, this, endpoint);
}

NetworkManager::~NetworkManager() {
  _workGuard.reset();
  _ioContext.stop();
  _networkThread.join();
  quill::Frontend::remove_logger_blocking(_logger);
}

void NetworkManager::StartNetworkThread(
    const std::optional<Endpoint> &endpoint) {
  if (!endpoint.has_value()) {
    LOG_INFO(_logger, "NetworkManager operating in client mode");
  } else {
    _acceptor = boost::asio::ip::tcp::acceptor(
        _ioContext, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),
                                                   endpoint.value().Port));

    LOG_INFO(_logger, "TCP Listen Server started at {}:{}",
             _acceptor.local_endpoint().address().to_string(),
             _acceptor.local_endpoint().port());

    AsyncAccept();
  }
  _ioContext.run();
}

bool NetworkManager::SendNetworkMessage(uint32_t sessionId,
                                        const NetworkMessage &msg) {

  std::shared_ptr<Session> session = nullptr;
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    auto sessionItr = _sessions.find(sessionId);
    if (sessionItr == _sessions.end()) {
      return false;
    }
    session = sessionItr->second;
  }

  boost::asio::post(_ioContext, [this, session = std::move(session), msg]() {
    // If there is something in the queue, it means there is an active write
    // going on and we are backed up
    session->PendingWrites.emplace(std::move(msg));
    AsyncSend(std::move(session));
  });
  return true;
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
  // // Notify all sessions
  // NetworkMessage leave_msg = kDisconnectMessage;
  // std::memcpy(leave_msg.Data.data(), &sessionId, sizeof(uint32_t));
  // leave_msg.Size += sizeof(uint32_t);
  // BroadcastMessage(leave_msg);
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    if (_sessions.find(sessionId) == _sessions.end()) {
      return;
    }
    _sessions[sessionId]->IsActive = false;
    _sessions[sessionId]->HeartbeatTimer.cancel();

    _sessions.erase(sessionId);
  }
  LOG_INFO(_logger, "Deleted Session, ID: {}", sessionId);
  if (_onDisconnectCallback) {
    _onDisconnectCallback(sessionId);
  }
}

void NetworkManager::AsyncAccept() {
  // Lambda executed when a client connects
  // Socket where communication with the client will happen
  _acceptor.async_accept([this](std::error_code ec,
                                boost::asio::ip::tcp::socket socket) {
    if (!ec) {
      uint32_t sessionId = _nextSessionId++;
      auto session =
          std::make_shared<Session>(false, false, std::move(socket), sessionId,
                                    boost::asio::steady_timer(_ioContext));
      {
        std::lock_guard<std::mutex> lock(_sessionLock);
        _sessions.emplace(sessionId, session);
      }
      LOG_INFO(_logger, "New Session created, ID: {}", sessionId);

      StartSession(std::move(session));
    } else {
      LOG_ERROR(_logger, "Error accepting client connection: {}", ec.message());
    }
    AsyncAccept();
  });
}

void NetworkManager::AsyncSend(std::shared_ptr<Session> session) {
  if (session->PendingWrites.empty()) {
    return;
  }

  boost::asio::async_write(
      session->Socket,
      boost::asio::buffer(Serialize(session->PendingWrites.front()),
                          session->PendingWrites.front().Size +
                              MESSAGE_HEADER_SIZE),
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
            AsyncSend(std::move(session));
          }
        } else {

          LOG_WARNING(_logger,
                      "Failed to send message to session {}: [{}] Error {}",
                      session->Id, ec.value(), ec.message());
          RemoveSession(session->Id);
        }
      });
}

void NetworkManager::StartSession(std::shared_ptr<Session> session) {
  LOG_INFO(_logger, "Session {} started", session->Id);
  session->IsActive = true;
  // StartHeartbeatTimer(session);
  AcceptSessionMessage(session);

  if (_onConnectCallback) {
    _onConnectCallback(session->Id);
  }
}

void NetworkManager::StartHeartbeatTimer(std::shared_ptr<Session> session) {
  session->HeartbeatTimer.expires_after(std::chrono::seconds(1));
  session->HeartbeatTimer.async_wait([this, session](
                                         const boost::system::error_code &ec) {
    if (session->IsClient) {
      LOG_INFO(_logger, "Sending heartbeat to server from client session {}",
               session->Id);
      auto msg = kHeartbeatMessage;
      std::ignore = SendNetworkMessage(session->Id, msg);
    }

    if (!session->IsActive || ec) {
      if (ec == boost::asio::error::operation_aborted) {
        return;
      }

      if (ec) {
        LOG_WARNING(_logger,
                    "Heartbeat timer for session {} failed: [{}] Error {}",
                    session->Id, ec.value(), ec.message());
      } else {
        LOG_WARNING(_logger, "Session {} failed heartbeat check", session->Id);
      }
      RemoveSession(session->Id);
      return;
    }
    session->IsActive = false;
    StartHeartbeatTimer(std::move(session));
  });
}

void NetworkManager::AckHeartbeat(uint32_t sessionId) {
  LOG_INFO(_logger, "Heartbeat received from session {}", sessionId);
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    auto it = _sessions.find(sessionId);
    if (it == _sessions.end()) {
      return;
    }
    it->second->IsActive = true;
    if (it->second->IsClient) {
      return;
    }
  }
  auto msg = kHeartbeatMessage;
  std::ignore = SendNetworkMessage(sessionId, msg);
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
    std::vector<uint8_t> data(msg.Data.begin(),
                              msg.Data.begin() +
                                  std::min(msg.Size, MESSAGE_DATA_SIZE));
    LOG_INFO(_logger, "Sending user packet to game thread");
    _handleUserPacketCallback(std::move(data));
  }
}

void NetworkManager::AcceptSessionMessage(std::shared_ptr<Session> session) {
  boost::asio::async_read(
      session->Socket,
      boost::asio::buffer(session->RawByteBuffer, MESSAGE_HEADER_SIZE),
      [this, session](std::error_code ec, std::size_t bytesRecv) {
        if ((ec.value() == boost::asio::error::eof && !session->IsActive) ||
            ec.value() == boost::asio::error::operation_aborted) {
          return;
        }
        if (ec || bytesRecv < MESSAGE_HEADER_SIZE) {
          LOG_WARNING(
              _logger,
              "Failed to read message header from session {}: [{}] Error {}",
              session->Id, ec.value(), ec.message());
          RemoveSession(session->Id);
          return;
        }

        if (session->IsActive == false) {
          return;
        }

        session->LastMessage = DeserializeHeader(session->RawByteBuffer);

        boost::asio::async_read(
            session->Socket,
            boost::asio::buffer(
                session->RawByteBuffer.data() + MESSAGE_HEADER_SIZE,
                std::min(session->LastMessage.Size, MESSAGE_DATA_SIZE)),
            [this, session](std::error_code ec, std::size_t bytesRecv) {
              if (ec) {
                LOG_WARNING(_logger,
                            "Failed to read message data from session {}: [{}] "
                            "Error {}",
                            session->Id, ec.value(), ec.message());
                RemoveSession(session->Id);
                return;
              }
              std::memcpy(
                  session->LastMessage.Data.data(),
                  session->RawByteBuffer.data() + MESSAGE_HEADER_SIZE,
                  std::min(session->LastMessage.Size, MESSAGE_DATA_SIZE));
              LOG_DEBUG(_logger,
                        "Message Received; Session={}; Type={}; Size={}; "
                        "BytesRecv={}",
                        session->Id,
                        static_cast<uint32_t>(session->LastMessage.Type),
                        session->LastMessage.Size, bytesRecv);
              HandleSessionMessage(session->Id, session->LastMessage);
              AcceptSessionMessage(std::move(session));
            });
      });
}

void NetworkManager::Connect(const Endpoint &endpoint) {
  boost::asio::ip::tcp::resolver::results_type endpoints =
      _resolver.resolve(endpoint.Host, std::to_string(endpoint.Port));

  auto session = std::make_shared<Session>(
      true, false, boost::asio::ip::tcp::socket(_ioContext), _nextSessionId++,
      boost::asio::steady_timer(_ioContext));

  boost::system::error_code ec;
  boost::asio::connect(session->Socket, endpoints, ec);

  if (ec) {
    LOG_ERROR(_logger, "Error connecting to {}:{} [{}] {}", endpoint.Host,
              endpoint.Port, ec.value(), ec.message());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    _sessions.emplace(session->Id, session);
  }
  LOG_INFO(_logger, "Connected to server at {}:{}", endpoint.Host,
           endpoint.Port);
  StartSession(std::move(session));
}

void NetworkManager::Disconnect(uint32_t sessionId) {
  std::ignore = SendNetworkMessage(sessionId, kDisconnectMessage);
  RemoveSession(sessionId);
}
