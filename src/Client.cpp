#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "NetworkManager.h"
#include "NetworkMessage.h"

#include "quill/Backend.h"
#include "quill/Frontend.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/std/Vector.h"

using boost::asio::ip::tcp;

int main(int argc, char *argv[]) {
  quill::Backend::start();
  auto logger = quill::Frontend::create_or_get_logger(
      "Client",
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1"));

  NetworkManager networkManager(std::nullopt);
  std::function<void(const std::vector<uint8_t> &)> callback =
      [logger](const std::vector<uint8_t> &data) {
        LOG_INFO(logger, "Received packet in client callback with data: {}",
                 data);
      };

  networkManager.SetHandlePacketCallback(callback);

  std::function<void(uint32_t)> connectCallback = [logger](uint32_t sessionId) {
    LOG_INFO(logger, "Connected to server with session ID: {}", sessionId);
  };
  networkManager.SetOnConnectCallback(connectCallback);

  std::function<void(uint32_t)> disconnectCallback = [logger](
                                                         uint32_t sessionId) {
    LOG_INFO(logger, "Disconnected from server with session ID: {}", sessionId);
  };
  networkManager.SetOnDisconnectCallback(disconnectCallback);

  networkManager.Connect(Endpoint{"127.0.0.1", 1330});

  std::vector<uint8_t> data(2 * sizeof(unsigned int));
  unsigned int x = 1;
  unsigned int y = 7;
  x = HostToNetwork(x);
  y = HostToNetwork(y);
  std::memcpy(data.data(), &x, sizeof(unsigned int));
  std::memcpy(data.data() + sizeof(unsigned int), &y, sizeof(unsigned int));

  for (int i = 0; i < 10; i++) {
    // std::cout << "Message size = " << data.size() << std::endl;
    // std::cout << "Sending message\n" << std::endl;
    if (!networkManager.Send(1, data)) {
      std::cout << "Failed to send message\n";
      return 0;
    }
    // std::cout << "Sent message\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  networkManager.Disconnect(1);

  return 0;
}