#pragma once

#include <string>
#include <string_view>

#include "EngineComponents.h"
#include "ScriptRef.h"

struct Actor;
class SceneManager;
class AudioManager;
class NetworkManager;
class b2World;

namespace Graphics {
class GraphicsManager;
}
namespace Input {
class InputManager;
}

class ScriptingBackend {
 public:
  virtual ~ScriptingBackend() = default;

  virtual void Initialize(SceneManager* scene,
                          Graphics::GraphicsManager* graphics,
                          AudioManager* audio, Input::InputManager* input,
                          NetworkManager* network, b2World* physics) = 0;

  // Called once per frame after component updates, to flush queued subscription
  // and event-like state. Backends that do not need this can leave it empty.
  virtual void ProcessFrameEndQueues() = 0;

  // Component templates and instances
  virtual ScriptRef LoadComponentTemplate(const std::string& componentType) = 0;
  virtual ScriptRef CreateComponentInstance(
      const std::string& key, const ScriptRef& templateRef) = 0;
  virtual void DestroyNativeComponent(ScriptRef& ref,
                                      const std::string& componentType) = 0;

  // Wrap a native Actor* so it can be handed back to script code.
  virtual ScriptRef WrapActor(Actor* actor) = 0;

  // Reference introspection
  virtual bool IsRefValid(const ScriptRef&) = 0;
  virtual bool IsMethod(const ScriptRef&, const std::string& key) = 0;
  virtual bool HasActorProperty(const ScriptRef&, const std::string& key) = 0;

  // Property read
  virtual bool GetBool(const ScriptRef&, const std::string& key) = 0;
  virtual int GetInt(const ScriptRef&, const std::string& key) = 0;
  virtual float GetFloat(const ScriptRef&, const std::string& key) = 0;
  virtual std::string GetString(const ScriptRef&, const std::string& key) = 0;
  virtual Actor* GetActor(const ScriptRef&, const std::string& key) = 0;

  // Property write
  virtual void SetBool(ScriptRef&, const std::string& key, bool) = 0;
  virtual void SetInt(ScriptRef&, const std::string& key, int) = 0;
  virtual void SetFloat(ScriptRef&, const std::string& key, float) = 0;
  virtual void SetString(ScriptRef&, const std::string& key,
                         const std::string&) = 0;
  virtual void SetActor(ScriptRef&, const std::string& key, Actor*) = 0;

  // Method invocation. Component is passed as "self" implicitly.
  virtual void CallSelfMethod(ScriptRef&, const std::string& method,
                              std::string_view actorName) = 0;
  virtual void CallCollisionMethod(ScriptRef&, const std::string& method,
                                   const Collision&,
                                   std::string_view actorName) = 0;

  // Backends that expose engine-native components (RigidBody, ParticleSystem)
  // implement these. A backend that stores these as native pointers directly
  // returns them; others may return nullptr.
  virtual RigidBody* AsRigidBody(const ScriptRef&) = 0;
  virtual ParticleSystem* AsParticleSystem(const ScriptRef&) = 0;
};
