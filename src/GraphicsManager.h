#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Defines.h"

namespace Graphics {
struct SpriteInfo {
  std::string SpriteName;
  std::pair<float, float> Position;
  std::pair<float, float> ScaleFactor;
  std::pair<float, float> PivotPoint;
  float RotationDegrees;
  Graphics::RGBA Color;
  std::pair<int, int> DrawPriority;
};

class GraphicsManager {
 public:
  GraphicsManager(const std::string& windowName, int screenWidth,
                  int screenHeight, const Graphics::RGBA& color);
  // Move constructor
  GraphicsManager(GraphicsManager&& other) noexcept;
  // Move assignment
  GraphicsManager& operator=(GraphicsManager&& other) noexcept;
  ~GraphicsManager();
  // No copy constructor or assignment b/c of internal unique ptrs
  GraphicsManager(const GraphicsManager&) = delete;
  GraphicsManager& operator=(const GraphicsManager&) = delete;

  void LoadImage(const std::string& imageName);
  void LoadParticle(const std::string& particleName);
  void AddFont(const std::string& font, unsigned int fontSize);
  void ClearScreen();
  void RefreshScreen();

  void DrawPoint(int x, int y, const Graphics::RGBA& color);
  void DrawSprite(const SpriteInfo& sprite);
  std::pair<float, float> GetSpriteDimension(const std::string& imageName);
  void RenderText(const std::string& font, unsigned int fontSize,
                  const std::string& text, int x, int y,
                  const Graphics::RGBA& color);
  void SetRenderScale(float scale);

  std::pair<int, int> GetScreenDimension();

 private:
  class GraphicsManagerImpl;
  std::unique_ptr<GraphicsManagerImpl> _graphicsManagerImpl;
};

}  // namespace Graphics

extern std::pair<float, float>* static_camera_pos;
