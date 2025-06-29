#include <algorithm>
#include <array>
#include <boost/asio.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "AudioManager.h"
#include "GraphicsManager.h"
#include "InputManager.h"
#include "quill/Backend.h"
#include "ResourceManager.h"
#include "SceneManager.h"
#include "Server.h"

constexpr auto BACKGROUND_SFX_CHANNEL = 0;
constexpr auto SCORE_SFX_CHANNEL = 1;

SceneManager* scene;

void RunServer(boost::asio::io_context& io_context) {
  try {
    NetworkManager server(io_context, 1330);
    io_context.run();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  bool running = true;

  std::unique_ptr<Resource::ResourceManager> resourceManager(nullptr);
  std::unique_ptr<Graphics::GraphicsManager> graphicsManager(nullptr);
  std::unique_ptr<AudioManager> audioManager(nullptr);
  std::unique_ptr<SceneManager> sceneManager(nullptr);
  Input::InputManager inputManager;
  quill::Backend::start();

  // Initalize Server
  boost::asio::io_context io_context;
  std::jthread serverThread(RunServer, std::ref(io_context));

  try {
    resourceManager = std::make_unique<Resource::ResourceManager>();
  } catch (std::runtime_error& err) {
    std::cout << err.what();
    return 0;
  }
  const auto& gameConfig = resourceManager->GetGameConfig();
  const auto& renderConfig = resourceManager->GetRenderConfig();

  try {
    graphicsManager = std::make_unique<Graphics::GraphicsManager>(
        gameConfig.GameTitle, renderConfig.Width, renderConfig.Height,
        renderConfig.Color);
  } catch (std::runtime_error& err) {
    std::cout << err.what();
    return 0;
  }

  try {
    audioManager = std::make_unique<AudioManager>(50);
  } catch (std::runtime_error& err) {
    std::cout << err.what();
    return 0;
  }

  // Context 0 exists by default, it maps to Intro controls
  std::vector<int> inputContexts(1, 0);

  Input::Action Exit{"Exit", [&running]() { running = false; }, []() {}};
  inputManager.AddActionToEvent(inputContexts[0], Input::Event::Exit,
                                std::move(Exit));

  try {
    sceneManager = std::make_unique<SceneManager>(
        graphicsManager.get(), audioManager.get(), &inputManager);
  } catch (std::runtime_error& err) {
    std::cout << err.what();
    return 0;
  }

  scene = sceneManager.get();

  while (running) {
    graphicsManager->ClearScreen();
    inputManager.CheckForEvents();

    sceneManager->UpdateSceneActors();
    graphicsManager->RefreshScreen();
  }

  io_context.stop();
  serverThread.join();

  return 0;
}
