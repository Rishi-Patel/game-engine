#include "GraphicsManager.h"

#include <stdexcept>
#include <string>
#include <tuple>

#include "Defines.h"
#include "Helper.h"
#include "SDL2/SDL.h"
#include "SDL_ttf.h"
#include "glm/glm.hpp"
#include "GL/glew.h"
#include "glm/ext/matrix_clip_space.hpp"
#include "SpriteRenderer.h"
#include "Shader.h"
#include "Texture.h"

#include <ft2build.h>
#include FT_FREETYPE_H  


using Graphics::GraphicsManager;
using Graphics::RGBA;


struct TextDrawRequest {
  std::string Text;
  std::pair<float, float> Position;
  std::string Font;
  unsigned int FontSize;
  Graphics::RGBA Color;
};

struct ImageDrawRequest {
    std::string SpriteName;
    SDL_FRect SpriteDestination;
    SDL_FPoint SpriteCenter;
    float RotationInDegrees;
    Graphics::RGBA Color;
    std::tuple<int, int> Priority;
};

struct PixelDrawRequest {
    int x, y;
    Graphics::RGBA Color;
};

constexpr auto BLACK = RGBA(0, 0, 0, 255);
constexpr auto WHITE = RGBA(255, 255, 255, 255);
constexpr auto RED = RGBA(255, 0, 0, 255);

static bool ImageDrawRequestPriority(const ImageDrawRequest& lhs, const ImageDrawRequest& rhs) {
    return lhs.Priority < rhs.Priority;
};

static const std::filesystem::path IMAGE_CONFIG_DIR =
std::filesystem::path("resources") / std::filesystem::path("images");
static const std::filesystem::path FONT_CONFIG_DIR =
std::filesystem::path("resources") / std::filesystem::path("fonts");
static const std::filesystem::path SHADER_CONFIG_DIR =
    std::filesystem::path("resources") / std::filesystem::path("shaders");


class GraphicsManager::GraphicsManagerImpl
{
public:
    GraphicsManagerImpl(const std::string& windowName, int screenWidth, int screenHeight, const RGBA& color);
    ~GraphicsManagerImpl();

    void SetRenderScale(float scale);

    void ClearScreen();
    void RefreshScreen();
    void DrawPoint(int x, int y, const Graphics::RGBA& color);
    void LoadImage(const std::string& imageName);
    void LoadShader(const std::string& shaderName);
    void LoadParticle(const std::string& particleName);
    void AddFont(const std::string& font, unsigned int fontSize);
    void DrawSprite(const SpriteInfo& sprite);
    void DrawText(const std::string& font, unsigned int fontSize, const std::string& text, int x, int y, const Graphics::RGBA& color);
    std::pair<float, float> GetSpriteDimension(const std::string& imageName);
    std::pair<int, int> GetScreenDimension();

private:
    struct SDL_GLContext_ {
       SDL_GLContext Context;
       SDL_GLContext_(SDL_GLContext context) : Context(SDL_GLContext(nullptr)) {}
       SDL_GLContext_() : Context(SDL_GLContext()) {}
       ~SDL_GLContext_() { 
           if (Context != nullptr) 
               SDL_GL_DeleteContext(Context); 
       }
    };

    struct Character {
      Texture2D Texture;  // ID handle of the glyph texture
      glm::ivec2 Size;         // Size of glyph
      glm::ivec2 Bearing;      // Offset from baseline to left/top of glyph
      unsigned int Advance;    // Offset to advance to next glyph
    };

    struct Font {
      std::unordered_map<unsigned int, std::unordered_map<char, Character>> Characters;
    };


    std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> _window;
    SDL_GLContext_ _context;
    FT_Library _ft;

    std::unordered_map<std::string, Shader> _shaders;
    std::unordered_map<std::string, Texture2D> _textures;
    std::unordered_map<std::string, Font> _fonts;
    std::unique_ptr<SpriteRenderer> _spriteRenderer;

    unsigned int _screenWidth, _screenHeight;
    float _renderScale;
    RGBA _color;
    glm::mat4 _projectionMatrix;

    std::queue<TextDrawRequest> _textRequestQueue;
    std::vector<ImageDrawRequest> _imageRequestQueue;
    size_t _imageRequestSize{ 0 };
    std::queue<PixelDrawRequest> _pixelRequestQueue;

    void HandleRequests();
};

GraphicsManager::GraphicsManagerImpl::GraphicsManagerImpl(const std::string& windowName, int screenWidth,
    int screenHeight, const RGBA& color)
    // Cant initialize sdl objects in initalize list b/c we can only call SDL functions after SDL_Init & IMG_Init.
    //   'Technically' SDL Create Window initializes video if it wasnt done, but this shouldnt be relied upon as the
    //    SDL documents say to call SDL_Init before any SDL functions
    : _window(nullptr, &SDL_DestroyWindow),
    _screenWidth(screenWidth), _screenHeight(screenHeight), _color(color), 
    _projectionMatrix(glm::ortho<float>(0.f, screenWidth, screenHeight, 0.f, -1.f, 1.f)),
    _renderScale(1.0f)
{
    if (SDL_WasInit(SDL_INIT_VIDEO) == false && SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        throw std::runtime_error("SDL could not initialize! SDL_Error: " + std::string(SDL_GetError()));
    }
    if (IMG_Init(IMG_INIT_PNG) == 0)
    {
        std::runtime_error("SDL Image could not initialize!");
    }
    if (TTF_Init() < 0)
    {
        std::runtime_error("SDL TFF could not initialize!");
    }

    _window.reset(Helper::SDL_CreateWindow(windowName.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        _screenWidth, _screenHeight, SDL_WINDOW_OPENGL));
    if (nullptr == _window)
    {
        throw std::runtime_error("Window could not be created! SDL_Error: " + std::string(SDL_GetError()));
    }
    _context.Context = SDL_GL_CreateContext(_window.get());
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    if ((void*)_context.Context == nullptr) {
      throw std::runtime_error("Cannot create OpenGL context");
    }

    GLenum err = glewInit();
    if (GLEW_OK != err) {

      throw std::runtime_error("glewInit failed, Error: " + std::string((const char*)glewGetErrorString(err)));
    }
    LoadShader("sprite");

    _shaders["sprite"].Use().SetInteger("image", 0);
    _shaders["sprite"].Use().SetMatrix4("projection", _projectionMatrix);
    _spriteRenderer = std::make_unique<SpriteRenderer>(_shaders["sprite"]);


    std::apply([&](auto &...args) { glClearColor(args...); }, _color);

    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (FT_Init_FreeType(&_ft)) {
      throw std::runtime_error(
          "ERROR::FREETYPE: Could not init FreeType Library");
    }
};

void GraphicsManager::GraphicsManagerImpl::ClearScreen()
{
    std::apply([&](auto &...args) { glClearColor(args...); }, _color);
    glClear(GL_COLOR_BUFFER_BIT);
}

void GraphicsManager::GraphicsManagerImpl::RefreshScreen()
{
    HandleRequests();
    SDL_GL_SwapWindow(_window.get());
}



void GraphicsManager::GraphicsManagerImpl::DrawPoint(int x, int y, const Graphics::RGBA& color)
{
    _pixelRequestQueue.emplace(PixelDrawRequest{ x,y, color });
}


void GraphicsManager::GraphicsManagerImpl::LoadImage(const std::string& imageName) {
    const auto imagePath = IMAGE_CONFIG_DIR / (imageName + ".png");
    if (!std::filesystem::exists(imagePath))
    {
        throw std::runtime_error("error: missing image " + imageName);
    }
    auto* surface = IMG_Load(imagePath.string().c_str());
    Texture2D& texture = _textures[imageName];
    texture.Internal_Format = GL_RGBA;
    texture.Image_Format = surface->format->Amask != 0 ? GL_RGBA : GL_RGB;
    texture.Generate(surface->w, surface->h, surface->pixels);
    SDL_FreeSurface(surface);
}

void GraphicsManager::GraphicsManagerImpl::LoadShader(
    const std::string& shaderName) {
  
  const auto vShaderPath = SHADER_CONFIG_DIR / (shaderName + ".vs");
  const auto fShaderPath = SHADER_CONFIG_DIR / (shaderName + ".frag");
  if (!std::filesystem::exists(vShaderPath)) {
    throw std::runtime_error("error: missing vertex shader " + shaderName);
  }
  if (!std::filesystem::exists(fShaderPath)) {
    throw std::runtime_error("error: missing fragment shader " + shaderName);
  }

  std::string vertexCode;
  std::string fragmentCode;
  std::ifstream vertexShaderFile(vShaderPath);
  std::ifstream fragmentShaderFile(fShaderPath);
  std::stringstream vShaderStream, fShaderStream;
  // read file's buffer contents into streams
  vShaderStream << vertexShaderFile.rdbuf();
  fShaderStream << fragmentShaderFile.rdbuf();
  // close file handlers
  vertexShaderFile.close();
  fragmentShaderFile.close();
  // convert stream into string
  vertexCode = vShaderStream.str();
  fragmentCode = fShaderStream.str();

  const char* vShaderCode = vertexCode.c_str();
  const char* fShaderCode = fragmentCode.c_str();
  // 2. now create shader object from source code
  Shader& shader = _shaders[shaderName];
  shader.Compile(vShaderCode, fShaderCode);
}

void GraphicsManager::GraphicsManagerImpl::LoadParticle(const std::string& particleName) {
    if (_textures.find(particleName) != _textures.end()) {
        return;
    }
    // Load default white particle if empty name
    if (particleName == "") {

        auto* surface = SDL_CreateRGBSurfaceWithFormat(0, 8, 8, 32, SDL_PIXELFORMAT_RGBA8888);
        Uint32 white_color = SDL_MapRGBA(surface->format, 255, 255, 255, 255);
        SDL_FillRect(surface, NULL, white_color);
        Texture2D& texture = _textures[particleName];
        texture.Internal_Format = GL_RGBA;
        texture.Image_Format = GL_RGBA;
        texture.Generate(surface->w, surface->h, surface->pixels);
        SDL_FreeSurface(surface);
        return;
    }

    LoadImage(particleName);
}

void GraphicsManager::GraphicsManagerImpl::AddFont(const std::string& font, unsigned int fontSize)
{
    auto fontPath = FONT_CONFIG_DIR / (font + ".ttf");
    _fonts.emplace(font, Font());
    FT_Face face;
    if (FT_New_Face(_ft, fontPath.string().c_str(), 0, &face)){
    std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
    return;
    }
    FT_Set_Pixel_Sizes(face, 0, fontSize);  

    glPixelStorei(GL_UNPACK_ALIGNMENT,
                  1); 

    for (unsigned char c = 0; c < 128; c++) {
      // load character glyph
      if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
        std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
        continue;
      }
      // generate texture
      Texture2D texture;
      texture.Image_Format = GL_RED;
      texture.Internal_Format = GL_RGBA;
      texture.Generate(face->glyph->bitmap.width, face->glyph->bitmap.rows,
                       face->glyph->bitmap.buffer);
      // now store character for later use
      Character character = {
          texture,
          glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
          glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
          face->glyph->advance.x};
      _fonts.at(font).Characters[fontSize].insert(
            std::pair<char, Character>(c, character));
    }
    FT_Done_Face(face);
}

void GraphicsManager::GraphicsManagerImpl::DrawSprite(const SpriteInfo& sprite)
{
    if (_textures.find(sprite.SpriteName) == _textures.end()) {
        LoadImage(sprite.SpriteName);
    }

    const auto& spriteDims = GetSpriteDimension(sprite.SpriteName);
    SDL_FRect dst{ sprite.Position.first, sprite.Position.second,
        spriteDims.first * sprite.ScaleFactor.first, spriteDims.second * sprite.ScaleFactor.second };
    SDL_FPoint center{ sprite.PivotPoint.first, sprite.PivotPoint.second };
    if (_imageRequestSize < _imageRequestQueue.size()) {
        _imageRequestQueue[_imageRequestSize++] = ImageDrawRequest{ sprite.SpriteName, dst, center, sprite.RotationDegrees, sprite.Color, {sprite.DrawPriority.first, sprite.DrawPriority.second} };
    }
    else {
        _imageRequestQueue.emplace_back(ImageDrawRequest{ sprite.SpriteName, dst, center, sprite.RotationDegrees, sprite.Color, {sprite.DrawPriority.first, sprite.DrawPriority.second} });
        _imageRequestSize++;
    }
}

void GraphicsManager::GraphicsManagerImpl::HandleRequests() {
    std::stable_sort(_imageRequestQueue.begin(), _imageRequestQueue.begin() + _imageRequestSize, ImageDrawRequestPriority );
    for (int i = 0; i < _imageRequestSize; i++) {
      auto& request = _imageRequestQueue[i];
        if (std::get<0>(request.Priority) == 0) {
            // Scale Image:
            request.SpriteDestination.x *= _renderScale;
            request.SpriteDestination.w *= _renderScale;
            request.SpriteDestination.y *= _renderScale;
            request.SpriteDestination.h *= _renderScale;
            // Apply camera offset
            request.SpriteDestination.x -= static_camera_pos->first * 100;
            request.SpriteDestination.y -= static_camera_pos->second * 100;
        }
      _spriteRenderer->DrawSprite(
        _textures.at(request.SpriteName),
        glm::vec2(request.SpriteDestination.x, request.SpriteDestination.y),
        glm::vec2(request.SpriteDestination.w, request.SpriteDestination.h),
        request.RotationInDegrees, 
        glm::vec4(std::get<0>(request.Color) / 255.0f,
                      std::get<1>(request.Color) / 255.0f,
                      std::get<2>(request.Color) / 255.0f,
                      std::get<3>(request.Color) / 255.0f)
      );
    }
    _imageRequestSize = 0;

    while (_textRequestQueue.empty() == false) {
        auto request = _textRequestQueue.front();
        _textRequestQueue.pop();
        std::string::const_iterator c;

        auto& chars = _fonts[request.Font].Characters[request.FontSize];
        glm::vec4 color = {1.0f,1.0f,1.0f,1.0f};
        for (c = request.Text.begin(); c != request.Text.end(); c++) {
          Character ch = chars[*c];

          float xpos = request.Position.first + ch.Bearing.x;
          float ypos = request.Position.second - (ch.Size.y - ch.Bearing.y);

          float w = ch.Size.x;
          float h = ch.Size.y;

          auto color = glm::vec4(std::get<0>(request.Color) / 255.0f,
                                 std::get<1>(request.Color) / 255.0f,
                                 std::get<2>(request.Color) / 255.0f,
                                 std::get<3>(request.Color) / 255.0f);
          
          _spriteRenderer->DrawSprite(ch.Texture, {xpos, ypos}, {w, h}, 0.0f, color);

          request.Position.first +=
              (ch.Advance >>
               6);  // bitshift by 6 to get value in pixels (2^6 = 64)
        }
    }

    while (_pixelRequestQueue.empty() == false) {
        auto request = _pixelRequestQueue.front();
        std::apply([&](auto &...args) { glClearColor(args...); }, request.Color);
        glBegin(GL_POINTS);
        glVertex3f(static_cast<float>(request.x), static_cast<float>(request.y), 0.0f);
        glEnd();
    }
}

void GraphicsManager::GraphicsManagerImpl::DrawText(
    const std::string& font, unsigned int fontSize, const std::string& text,
    int x, int y, const Graphics::RGBA& color) {
  if (_fonts.find(font) == _fonts.end() ||
      _fonts.at(font).Characters.find(fontSize) ==
          _fonts.at(font).Characters.end()) {
    AddFont(font, fontSize);
  }

    _textRequestQueue.emplace(TextDrawRequest{text, {x, y}, font,
                              fontSize, color});
}

std::pair<float, float> GraphicsManager::GraphicsManagerImpl::GetSpriteDimension(const std::string& imageName)
{
    if (_textures.find(imageName) == _textures.end()) {
        LoadImage(imageName);
    }
    return {_textures[imageName].Width, _textures[imageName].Height};
}

std::pair<int, int> GraphicsManager::GraphicsManagerImpl::GetScreenDimension()
{
    return { _screenWidth, _screenHeight };
}

GraphicsManager::GraphicsManagerImpl::~GraphicsManagerImpl()
{
    SDL_Quit();
}

void GraphicsManager::GraphicsManagerImpl::SetRenderScale(float scale) {
    _renderScale = scale;
}

void GraphicsManager::ClearScreen()
{
    _graphicsManagerImpl->ClearScreen();
}

GraphicsManager::GraphicsManager(const std::string& windowName, int screenWidth, int screenHeight, const RGBA& color)
    : _graphicsManagerImpl(
        std::make_unique<GraphicsManager::GraphicsManagerImpl>(windowName, screenWidth, screenHeight, color))
{
}

GraphicsManager::GraphicsManager(GraphicsManager&& other) noexcept
    : _graphicsManagerImpl(std::move(other._graphicsManagerImpl))
{
}

GraphicsManager& GraphicsManager::operator=(GraphicsManager&& other) noexcept
{
    _graphicsManagerImpl = std::move(other._graphicsManagerImpl);
    return *this;
}

void GraphicsManager::RefreshScreen()
{
    _graphicsManagerImpl->RefreshScreen();
}

void GraphicsManager::DrawPoint(int x, int y, const Graphics::RGBA& color)
{
    _graphicsManagerImpl->DrawPoint(x, y, color);
}

void GraphicsManager::LoadImage(const std::string& imageName)
{
    _graphicsManagerImpl->LoadImage(imageName);
}

void GraphicsManager::LoadParticle(const std::string& particleName) {
    _graphicsManagerImpl->LoadParticle(particleName);
}

void GraphicsManager::AddFont(const std::string& font, unsigned int fontSize)
{
    _graphicsManagerImpl->AddFont(font, fontSize);
}


void GraphicsManager::DrawSprite(const SpriteInfo& sprite)
{
    _graphicsManagerImpl->DrawSprite(sprite);
}


void GraphicsManager::DrawText(const std::string& font, unsigned int fontSize, const std::string& text, int x, int y, const Graphics::RGBA& color)
{
    _graphicsManagerImpl->DrawText(font, fontSize, text, x, y, color);
}

std::pair<float, float> GraphicsManager::GetSpriteDimension(const std::string& imageName)
{
    return _graphicsManagerImpl->GetSpriteDimension(imageName);
}

std::pair<int, int> GraphicsManager::GetScreenDimension()
{
    return _graphicsManagerImpl->GetScreenDimension();
}

void GraphicsManager::SetRenderScale(float scale) {
    _graphicsManagerImpl->SetRenderScale(scale);
}

GraphicsManager::~GraphicsManager() = default;
