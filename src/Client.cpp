#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include "NetworkMessage.h"

using boost::asio::ip::tcp;

int main(int argc, char* argv[]) {
  try {
    boost::asio::io_context io_context;
    tcp::resolver resolver(io_context);
    tcp::resolver::results_type endpoints =
        resolver.resolve("127.0.0.1", "1330");
    tcp::socket socket(io_context);
    boost::asio::connect(socket, endpoints);
    NetworkMessage msg;
    msg.Data.fill(0);
    msg.Data[0] = 123;
    msg.Data[1023] = 210;
    msg.Size = MESSAGE_SIZE;
    msg.Type = MessageType::USER;

    for (int i = 0; i < 100; i++) {
      std::cout << "Sending message\n";
      msg.Type = MessageType::USER;
      auto data = Serialize(msg);
      int bytesSent = socket.send(boost::asio::buffer(data));
      std::cout << "Message sent " << bytesSent << "\n";

      msg.Type = MessageType::HEARTBEAT;
      std::cout << "Sending heartbeat\n";
      data = Serialize(msg);
      bytesSent = socket.send(boost::asio::buffer(data));
      std::cout << "Heartbeat sent " << bytesSent << "\n";
      std::array<uint8_t, MESSAGE_SIZE> recv_buf{};
      size_t len = socket.receive(boost::asio::buffer(recv_buf));
      auto rmsg = Deserialize(recv_buf);
      std::cout << "Len: " << len << std::endl;
      for (uint8_t v : recv_buf) {
        std::cout << static_cast<int>(v) << ",";
      }
      std::println(std::cout);

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}