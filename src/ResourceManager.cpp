#include "ResourceManager.h"

#include <array>
#include <filesystem>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using Resource::GameConfig;
using Resource::RenderConfig;
using Resource::ResourceManager;

static const std::filesystem::path RESOURCE_DIR_PATH =
    std::filesystem::path("resources");
static const std::filesystem::path GAME_CONFIG_PATH =
    RESOURCE_DIR_PATH / std::filesystem::path("game.config");
static const std::filesystem::path RENDER_CONFIG_PATH =
    RESOURCE_DIR_PATH / std::filesystem::path("rendering.config");
static const std::filesystem::path NETWORK_CONFIG_PATH =
    RESOURCE_DIR_PATH / std::filesystem::path("network.config");

static void ReadJsonFile(const std::string& path,
                         rapidjson::Document& out_document) {
  FILE* file_pointer = nullptr;
#ifdef _WIN32
  fopen_s(&file_pointer, path.c_str(), "rb");
#else
  file_pointer = fopen(path.c_str(), "rb");
#endif
  char buffer[65536];
  rapidjson::FileReadStream stream(file_pointer, buffer, sizeof(buffer));
  out_document.ParseStream(stream);
  std::fclose(file_pointer);

  if (out_document.HasParseError()) {
    rapidjson::ParseErrorCode errorCode = out_document.GetParseError();
    throw std::runtime_error("error parsing json at [" + path + "]");
  }
}

ResourceManager::ResourceManager() {
  if (!std::filesystem::exists(RESOURCE_DIR_PATH)) {
    throw std::runtime_error("error: resources/ missing");
  }

  if (!std::filesystem::exists(GAME_CONFIG_PATH)) {
    throw std::runtime_error("error: resources/game.config missing");
  }

  rapidjson::Document tmp;

  ReadJsonFile(GAME_CONFIG_PATH.string(), tmp);
  if (tmp.HasMember("initial_scene") == false) {
    throw std::runtime_error("error: initial_scene unspecified");
  }

  _gameConfigs.InitialScene = tmp["initial_scene"].GetString();

  if (tmp.HasMember("game_title")) {
    _gameConfigs.GameTitle = tmp["game_title"].GetString();
  }

  if (!std::filesystem::exists(RENDER_CONFIG_PATH)) {
    throw std::runtime_error("error: resources/rendering.config missing");
  }
  ReadJsonFile(RENDER_CONFIG_PATH.string(), tmp);
  if (tmp.HasMember("x_resolution")) {
    _renderConfigs.Width = tmp["x_resolution"].GetInt();
  }
  if (tmp.HasMember("y_resolution")) {
    _renderConfigs.Height = tmp["y_resolution"].GetInt();
  }
  if (tmp.HasMember("clear_color_r")) {
    std::get<0>(_renderConfigs.Color) = tmp["clear_color_r"].GetInt();
  }
  if (tmp.HasMember("clear_color_g")) {
    std::get<1>(_renderConfigs.Color) = tmp["clear_color_g"].GetInt();
  }
  if (tmp.HasMember("clear_color_b")) {
    std::get<2>(_renderConfigs.Color) = tmp["clear_color_b"].GetInt();
  }

  if (!std::filesystem::exists(NETWORK_CONFIG_PATH)) {
    _networkConfigs = std::nullopt;
    return;
  }
  _networkConfigs = NetworkConfig{};
  ReadJsonFile(NETWORK_CONFIG_PATH.string(), tmp);
  if (tmp.HasMember("hostname")) {
    _networkConfigs->Hostname = tmp["hostname"].GetString();
  }
  if (tmp.HasMember("port")) {
    _networkConfigs->Port = tmp["port"].GetInt();
  }
}
