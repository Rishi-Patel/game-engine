#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "NetworkMessage.h"
#include "quill/Logger.h"

struct Session;

struct Endpoint {
  std::string_view Host;
  int Port;
};

class NetworkManager {
public:
  explicit NetworkManager(const std::optional<Endpoint> &endpoint);
  ~NetworkManager();

  void Connect(const Endpoint &endpoint);
  void Disconnect(uint32_t sessionId);
  [[nodiscard]] bool SendNetworkMessage(uint32_t sessionId,
                                        const NetworkMessage &msg);
  void BroadcastMessage(const NetworkMessage &msg);
  void RemoveSession(uint32_t sessionId);
  void SetHandlePacketCallback(
      std::function<void(const std::vector<uint8_t> &)> &callback) {
    _handleUserPacketCallback = callback;
  }

  void SetOnConnectCallback(std::function<void(uint32_t)> &callback) {
    _onConnectCallback = callback;
  }

  void SetOnDisconnectCallback(std::function<void(uint32_t)> &callback) {
    _onDisconnectCallback = callback;
  }

private:
  boost::asio::io_context _ioContext;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
      _workGuard;
  boost::asio::ip::tcp::acceptor _acceptor;
  boost::asio::ip::tcp::resolver _resolver;
  uint32_t _nextSessionId;
  quill::Logger *_logger;

  std::unordered_map<uint32_t, std::shared_ptr<Session>> _sessions;
  std::mutex _sessionLock;
  std::jthread _networkThread;
  std::function<void(const std::vector<uint8_t> &)> _handleUserPacketCallback;
  std::function<void(uint32_t)> _onConnectCallback;
  std::function<void(uint32_t)> _onDisconnectCallback;

  void AsyncAccept();
  void AsyncSend(std::shared_ptr<Session> session);
  void StartSession(std::shared_ptr<Session> session);
  void StartHeartbeatTimer(std::shared_ptr<Session> session);
  void AckHeartbeat(uint32_t sessionId);
  void HandleSessionMessage(uint32_t sessionId, const NetworkMessage &msg);
  void AcceptSessionMessage(std::shared_ptr<Session> session);
  void StartNetworkThread(const std::optional<Endpoint> &endpoint);
  void HandleUserMessage(const NetworkMessage &message);
};
