#pragma once

#include <string>

#include "quill/Backend.h"
#include "quill/Frontend.h"
#include "quill/Logger.h"
#include "quill/LogMacros.h"
#include "quill/sinks/ConsoleSink.h"
#include "quill/std/Array.h"

// class Logger {
//  public:
//   Logger() {
//     quill::Backend::start();
//     auto console_sink =
//         quill::Frontend::create_or_get_sink<quill::ConsoleSink>("sink_id_1");
//     _logger =
//         quill::Frontend::create_or_get_logger("root",
//         std::move(console_sink));
//     // Change the LogLevel to print everything
//     _logger->set_log_level(quill::LogLevel::TraceL3);
//   }
//
//   template <typename... Args>
//   void LogInfo(const std::string& fmt, Args&&... args) {
//     LOG_DYNAMIC(_logger, quill::LogLevel::Info, fmt.c_str(), args...);
//   }
//   quill::Logger* GetLogger() { return _logger; }
//
//  private:
//   quill::Logger* _logger;
// };
