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
#include "quill/std/Vector.h"

static constexpr auto kLoggerName = "NetworkThread";

class Session : public std::enable_shared_from_this<Session> {
public:
  Session(bool isClient, bool isActive, boost::asio::ip::tcp::socket socket,
          uint32_t id, boost::asio::steady_timer timer)
      : IsClient(isClient), IsActive(isActive), HeartbeatMissCount(0),
        Socket(std::move(socket)), Id(id), HeartbeatTimer(std::move(timer)) {}
  ~Session() { Socket.close(); }
  bool IsClient;
  bool IsActive;
  int HeartbeatMissCount;
  boost::asio::ip::tcp::socket Socket;
  uint32_t Id;
  boost::asio::steady_timer HeartbeatTimer;
  NetworkMessage LastMessage;
  std::queue<std::vector<uint8_t>> PendingWrites;
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

bool NetworkManager::Send(uint32_t sessionId, std::span<const uint8_t> data) {
  std::shared_ptr<Session> session = GetSession(sessionId);
  if (!session) {
    return false;
  }
  for (std::size_t i = 0; i < data.size(); i += MESSAGE_DATA_SIZE) {
    NetworkMessage msg{.Type = MessageType::USER, .Size = 0, .Data = {}};
    msg.Size =
        std::min<std::size_t>(std::size_t(MESSAGE_DATA_SIZE), data.size() - i);
    std::memcpy(msg.Data.data(), data.data() + i, msg.Size);

    SendNetworkMessage(session, msg);
  }
  return true;
}

void NetworkManager::SendNetworkMessage(std::shared_ptr<Session> session,
                                        NetworkMessage msg) {
  boost::asio::post(
      _ioContext, [this, session = std::move(session), msg = std::move(msg)]() {
        // If there is something in the queue, it means there is an active write
        // going on and we are backed up
        auto serializedMsg = Serialize(msg);
        bool writeInProgress = !session->PendingWrites.empty();
        session->PendingWrites.emplace(serializedMsg.begin(),
                                       serializedMsg.begin() +
                                           MESSAGE_HEADER_SIZE + msg.Size);
        if (!writeInProgress) {
          AsyncSend(std::move(session));
        }
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
    auto serializedMsg = Serialize(msg);
    for (auto &session : sessions) {
      if (!session->IsActive) {
        continue;
      }
      // If there is something in the queue, it means there is an active write
      // going on and we are backed up
      bool writeInProgress = !session->PendingWrites.empty();
      session->PendingWrites.emplace(serializedMsg.begin(),
                                     serializedMsg.begin() +
                                         MESSAGE_HEADER_SIZE + msg.Size);
      if (!writeInProgress) {
        AsyncSend(std::move(session));
      }
    }
  });
}

void NetworkManager::RemoveSession(std::shared_ptr<Session> session) {
  if (!session->IsActive) {
    return;
  }
  // // Notify all sessions
  // NetworkMessage leave_msg = kDisconnectMessage;
  // std::memcpy(leave_msg.Data.data(), &sessionId, sizeof(uint32_t));
  // leave_msg.Size += sizeof(uint32_t);
  // BroadcastMessage(leave_msg);
  session->IsActive = false;
  session->HeartbeatTimer.cancel();
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    _sessions.erase(session->Id);
  }
  LOG_INFO(_logger, "Deleted Session, ID: {}", session->Id);
  if (_onDisconnectCallback) {
    _onDisconnectCallback(session->Id);
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
      session->Socket, boost::asio::buffer(session->PendingWrites.front()),
      [this, session](boost::system::error_code ec, std::size_t bytesSent) {
        LOG_INFO(_logger, "Message Sent; Session={}; BytesSent={}", session->Id,
                 bytesSent);
        if (!ec) {
          session->PendingWrites.pop();
          if (!session->PendingWrites.empty()) {
            AsyncSend(std::move(session));
          }
        } else {

          LOG_WARNING(_logger,
                      "Failed to send message to session {}: [{}] Error {}",
                      session->Id, ec.value(), ec.message());
          RemoveSession(session);
        }
      });
}

void NetworkManager::StartSession(std::shared_ptr<Session> session) {
  LOG_INFO(_logger, "Session {} started", session->Id);
  session->IsActive = true;
  session->HeartbeatMissCount = 0;
  StartHeartbeatTimer(session);
  AcceptSessionMessage(session);
  if (session->IsClient) {
    LOG_INFO(_logger, "Sending heartbeat to server from client session {}",
             session->Id);
    auto msg = kHeartbeatMessage;
    SendNetworkMessage(session, msg);
  }

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
      SendNetworkMessage(session, msg);
    }

    if (ec == boost::asio::error::operation_aborted) {
      return;
    }

    if (ec) {
      LOG_WARNING(_logger,
                  "Heartbeat timer for session {} failed: [{}] Error {}",
                  session->Id, ec.value(), ec.message());
      RemoveSession(session);
      return;
    }

    session->HeartbeatMissCount++;
    if (session->HeartbeatMissCount >= 2) {
      LOG_WARNING(_logger, "Session {} failed heartbeat check", session->Id);
      RemoveSession(session);
      return;
    }
    StartHeartbeatTimer(std::move(session));
  });
}

void NetworkManager::AckHeartbeat(uint32_t sessionId) {
  LOG_INFO(_logger, "Heartbeat received from session {}", sessionId);
  std::shared_ptr<Session> session = GetSession(sessionId);
  if (!session) {
    return;
  }

  session->HeartbeatMissCount = 0;
  if (session->IsClient) {
    return;
  }

  auto msg = kHeartbeatMessage;
  SendNetworkMessage(session, msg);
}

void NetworkManager::HandleSessionMessage(uint32_t sessionId,
                                          const NetworkMessage &msg) {
  switch (msg.Type) {
  case MessageType::CONNECT:
    break;
  case MessageType::DISCONNECT:
    Disconnect(sessionId);
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
    LOG_INFO(_logger, "Sending user packet to game thread: {}", data);
    _handleUserPacketCallback(std::move(data));
  }
}

void NetworkManager::AcceptSessionMessage(std::shared_ptr<Session> session) {
  boost::asio::async_read(
      session->Socket,
      boost::asio::buffer(session->RawByteBuffer, MESSAGE_HEADER_SIZE),
      [this, session](std::error_code ec, std::size_t bytesRecv) {
        if (ec.value() == boost::asio::error::operation_aborted) {
          return;
        }
        if (ec.value() == boost::asio::error::eof) {
          LOG_INFO(_logger, "Session {} disconnected", session->Id);
          RemoveSession(session);
          return;
        }
        if (ec || bytesRecv < MESSAGE_HEADER_SIZE) {
          LOG_WARNING(
              _logger,
              "Failed to read message header from session {}: [{}] Error {}",
              session->Id, ec.value(), ec.message());
          RemoveSession(session);
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
                RemoveSession(session);
                return;
              }
              std::memcpy(
                  session->LastMessage.Data.data(),
                  session->RawByteBuffer.data() + MESSAGE_HEADER_SIZE,
                  std::min(session->LastMessage.Size, MESSAGE_DATA_SIZE));
              LOG_INFO(_logger,
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
  boost::asio::post(_ioContext, [this, session = std::move(session)]() mutable {
    StartSession(std::move(session));
  });
}

void NetworkManager::Disconnect(uint32_t sessionId) {
  auto session = GetSession(sessionId);
  if (!session) {
    return;
  }

  SendNetworkMessage(session, kDisconnectMessage);
  RemoveSession(session);
}

std::shared_ptr<Session> NetworkManager::GetSession(uint32_t sessionId) {
  std::shared_ptr<Session> session = nullptr;
  {
    std::lock_guard<std::mutex> lock(_sessionLock);
    auto it = _sessions.find(sessionId);
    if (it != _sessions.end()) {
      session = it->second;
    }
  }
  return session;
}