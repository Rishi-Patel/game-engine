#pragma once

#include <optional>
#include <string>
#include <vector>

#include "Defines.h"

namespace Resource {

class ResourceManager {
 public:
  ResourceManager();

  const GameConfig &GetGameConfig() const { return _gameConfigs; }

  const RenderConfig &GetRenderConfig() const { return _renderConfigs; }

  const std::optional<NetworkConfig> &GetNetworkConfig() const {
    return _networkConfigs;
  }

 private:
  GameConfig _gameConfigs;
  RenderConfig _renderConfigs;
  std::optional<NetworkConfig> _networkConfigs;
};

}  // namespace Resource
