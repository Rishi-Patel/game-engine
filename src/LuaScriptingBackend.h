#pragma once

#include <filesystem>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// clang-format off
#include "lua.hpp"
#include "LuaBridge/LuaBridge.h"
// clang-format on

#include "ScriptingBackend.h"

class LuaScriptingBackend : public ScriptingBackend {
 public:
  LuaScriptingBackend();
  ~LuaScriptingBackend() override;

  void Initialize(SceneManager* scene, Graphics::GraphicsManager* graphics,
                  AudioManager* audio, Input::InputManager* input,
                  NetworkManager* network, b2World* physics) override;
  void ProcessFrameEndQueues() override;

  ScriptRef LoadComponentTemplate(const std::string& componentType) override;
  ScriptRef CreateComponentInstance(const std::string& key,
                                    const ScriptRef& templateRef) override;
  void DestroyNativeComponent(ScriptRef& ref,
                              const std::string& componentType) override;

  ScriptRef WrapActor(Actor* actor) override;

  bool IsRefValid(const ScriptRef&) override;
  bool IsMethod(const ScriptRef&, const std::string& key) override;
  bool HasActorProperty(const ScriptRef&, const std::string& key) override;

  bool GetBool(const ScriptRef&, const std::string& key) override;
  int GetInt(const ScriptRef&, const std::string& key) override;
  float GetFloat(const ScriptRef&, const std::string& key) override;
  std::string GetString(const ScriptRef&, const std::string& key) override;
  Actor* GetActor(const ScriptRef&, const std::string& key) override;

  void SetBool(ScriptRef&, const std::string& key, bool) override;
  void SetInt(ScriptRef&, const std::string& key, int) override;
  void SetFloat(ScriptRef&, const std::string& key, float) override;
  void SetString(ScriptRef&, const std::string& key,
                 const std::string&) override;
  void SetActor(ScriptRef&, const std::string& key, Actor*) override;

  void CallSelfMethod(ScriptRef&, const std::string& method,
                      std::string_view actorName) override;
  void CallCollisionMethod(ScriptRef&, const std::string& method,
                           const Collision&,
                           std::string_view actorName) override;

  RigidBody* AsRigidBody(const ScriptRef&) override;
  ParticleSystem* AsParticleSystem(const ScriptRef&) override;

  // Extract the underlying LuaRef from a ScriptRef. Asserts on bad cast.
  static luabridge::LuaRef& Unwrap(ScriptRef& ref);
  static const luabridge::LuaRef& Unwrap(const ScriptRef& ref);

  lua_State* GetLuaState() { return _luaState; }

 private:
  lua_State* _luaState;
  SceneManager* _scene;
  Graphics::GraphicsManager* _graphics;
  AudioManager* _audio;
  Input::InputManager* _input;
  NetworkManager* _network;
  b2World* _physics;

  std::unordered_map<std::string, luabridge::LuaRef> _componentTemplates;

  std::unordered_map<std::string,
                     std::vector<std::pair<luabridge::LuaRef, luabridge::LuaRef>>>
      _subscriptions;
  std::queue<std::tuple<std::string, luabridge::LuaRef, luabridge::LuaRef>>
      _newSubscriptions;
  std::queue<std::tuple<std::string, luabridge::LuaRef, luabridge::LuaRef>>
      _removeSubscriptions;

  std::mutex _networkLock;
  std::queue<std::vector<uint8_t>> _networkMessageQueue;

  void InitializeClassAPI();
  void InitializeDebugAPI();
  void InitializeActorClassAPI();
  void InitializeActorAPI();
  void InitializeApplicationAPI();
  void InitializeInputAPI();
  void InitializeTextAPI();
  void InitializeAudioAPI();
  void InitializeDrawAPI();
  void InitializeCameraAPI();
  void InitializeSceneAPI();
  void InitializePhysicsAPI();
  void InitializeEventsAPI();
  void InitializeNetworkAPI();

  luabridge::LuaRef RunFile(const std::filesystem::path& filename);
  void EstablishInheritance(luabridge::LuaRef& instance,
                            const luabridge::LuaRef& template_component);
};
