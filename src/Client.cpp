#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using boost::asio::ip::udp;

int main(int argc, char* argv[]) {
  try {
    boost::asio::io_context io_context;
    udp::resolver resolver(io_context);
    udp::endpoint receiver_endpoint =
        *resolver.resolve(udp::v4(), "127.0.0.1", "1330").begin();
    udp::socket socket(io_context);
    socket.open(udp::v4());
    for (int i = 0; i < 10; i++) {
      std::array<char, 1> send_buf = {{0}};
      socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
      std::array<char, 128> recv_buf;
      udp::endpoint sender_endpoint;
      size_t len =
          socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);

      std::cout.write(recv_buf.data(), len);
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}