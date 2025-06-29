#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>

enum class GameMode { Introduction, Level, GameOver, Win };

namespace Graphics {

using RGBA = std::tuple<uint8_t, uint8_t, uint8_t, uint8_t>;

}  // namespace Graphics

namespace Resource {

struct GameConfig {
  std::string InitialScene{};
  std::string GameTitle{};
};

struct RenderConfig {
  int Width = 640;
  int Height = 360;
  Graphics::RGBA Color{255, 255, 255, 255};
};

}  // namespace Resource