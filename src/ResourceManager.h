#pragma once

#include <optional>
#include <string>
#include <vector>

#include "Defines.h"

namespace Resource
{

class ResourceManager
{

  public:
    ResourceManager();

    const GameConfig &GetGameConfig() const
    {
        return _gameConfigs;
    }

    const RenderConfig &GetRenderConfig() const
    {
        return _renderConfigs;
    }

    const std::vector<std::string> &GetIntroImages() const
    {
        return _introImages;
    }
    const std::vector<std::string> &GetIntroText() const
    {
        return _introText;
    }

  private:
    GameConfig _gameConfigs;
    RenderConfig _renderConfigs;

    std::vector<std::string> _introImages;
    std::vector<std::string> _introText;
};

} // namespace Resource
