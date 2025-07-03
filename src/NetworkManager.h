#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "NetworkMessage.h"
#include "quill/Logger.h"

struct Session;

class NetworkManager {
 public:
  NetworkManager(const std::optional<std::string> &hostname, int port);
  ~NetworkManager();

  void SendNetworkMessage(uint32_t sessionId, NetworkMessage &msg);
  void BroadcastMessage(const NetworkMessage &msg);
  void RemoveSession(uint32_t sessionId);
  void SetHandlePacketCallback(
      std::function<void(const std::array<uint8_t, MESSAGE_DATA_SIZE> &)>
          &callback) {
    _handleUserPacketCallback = callback;
  }

 private:
  uint32_t _nextSessionId;
  boost::asio::io_context _ioContext;
  quill::Logger *_logger;

  std::unordered_map<uint32_t, std::shared_ptr<Session>> _sessions;
  std::mutex _sessionLock;
  std::jthread _networkThread;
  std::function<void(const std::array<uint8_t, MESSAGE_DATA_SIZE> &)>
      _handleUserPacketCallback;

  void AsyncAccept(boost::asio::ip::tcp::acceptor &acceptor);
  void AsyncSend(std::shared_ptr<Session> session);
  void StartSession(std::shared_ptr<Session> &session);
  void StartHeartbeatTimer(std::shared_ptr<Session> session);
  void AckHeartbeat(uint32_t sessionId);
  void HandleSessionMessage(uint32_t sessionId, const NetworkMessage &msg);
  void AcceptSessionMessage(std::shared_ptr<Session> session);
  void StartNetworkThread(const std::optional<std::string> &hostname, int port);
  void HandleUserMessage(const NetworkMessage &message);
};
