#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ScriptingBackend.h"

class PythonScriptingBackend : public ScriptingBackend {
 public:
  PythonScriptingBackend();
  ~PythonScriptingBackend() override;

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

  static pybind11::object& Unwrap(ScriptRef& ref);
  static const pybind11::object& Unwrap(const ScriptRef& ref);

  // Called from file-scope PYBIND11_EMBEDDED_MODULE lambdas (which are not
  // members, so they need these via a public path).
  pybind11::object ActorAddComponent(Actor* actor,
                                     const std::string& templateType);
  void ActorRemoveComponent(Actor* actor, pybind11::object component);

 private:
  std::unique_ptr<pybind11::scoped_interpreter> _interp;
  SceneManager* _scene;
  Graphics::GraphicsManager* _graphics;
  AudioManager* _audio;
  Input::InputManager* _input;
  NetworkManager* _network;
  b2World* _physics;

  pybind11::module_ _engine;

  std::unordered_map<std::string, pybind11::object> _componentTemplates;

  std::unordered_map<std::string,
                     std::vector<std::pair<pybind11::object, pybind11::object>>>
      _subscriptions;
  std::queue<std::tuple<std::string, pybind11::object, pybind11::object>>
      _newSubscriptions;
  std::queue<std::tuple<std::string, pybind11::object, pybind11::object>>
      _removeSubscriptions;

  std::mutex _networkLock;
  std::queue<std::vector<uint8_t>> _networkMessageQueue;

  void BuildEngineModule();
  void InstallBuiltins();
  pybind11::object LoadPythonClass(const std::string& componentName);
  void EstablishInheritance(pybind11::object& instance,
                            const pybind11::object& template_component);
};
