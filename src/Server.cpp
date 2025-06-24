#include "Server.h"

#include "quill/Frontend.h"
#include "quill/Logger.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/std/Array.h"

using boost::asio::ip::udp;

static constexpr auto kLoggerName = "NetworkThread";

std::string make_daytime_string() {
  using namespace std;  // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

udp_server::udp_server(boost::asio::io_context& io_context)
    : socket_(io_context, udp::endpoint(udp::v4(), 13)) {
  auto console_sink =
      quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1");
  quill::Frontend::create_or_get_logger(kLoggerName, std::move(console_sink));
  quill::Frontend::get_logger(kLoggerName)
      ->set_log_level(quill::LogLevel::TraceL3);
  start_receive();
}

udp_server::~udp_server() {}

void udp_server::start_receive() {
  LOG_INFO(quill::Frontend::get_logger(kLoggerName),
           "Waiting for request...\n");
  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
      std::bind(&udp_server::handle_receive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
}

void udp_server::handle_receive(const boost::system::error_code& error,
                                std::size_t /*bytes_transferred*/) {
  if (!error) {
    std::shared_ptr<std::string> message(
        new std::string(make_daytime_string()));
    LOG_INFO(quill::Frontend::get_logger(kLoggerName), "Received request\n");

    socket_.async_send_to(
        boost::asio::buffer(*message), remote_endpoint_,
        std::bind(&udp_server::handle_send, this, message,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
    LOG_INFO(quill::Frontend::get_logger(kLoggerName), "Sent message: {}\n",
             *message);

    start_receive();
  }
}

void udp_server::handle_send(std::shared_ptr<std::string> /*message*/,
                             const boost::system::error_code& /*error*/,
                             std::size_t /*bytes_transferred*/) {}