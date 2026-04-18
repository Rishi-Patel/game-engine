#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/hash.hpp"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "box2d/box2d.h"

#include "ScriptRef.h"

namespace Graphics {
class GraphicsManager;
}

namespace Input {
class InputManager;
}

class AudioManager;
class NetworkManager;
class ScriptingBackend;
class LuaScriptingBackend;

class ContactListener : public b2ContactListener {
 public:
  explicit ContactListener(ScriptingBackend* backend = nullptr)
      : _backend(backend) {}
  void SetBackend(ScriptingBackend* backend) { _backend = backend; }

 private:
  void BeginContact(b2Contact* contact) final;
  void EndContact(b2Contact* contact) final;
  ScriptingBackend* _backend;
};

struct Actor {
  size_t id = 0;
  std::string name = "";
  bool dontDestroy = false;
  std::map<std::string, ScriptRef> components;
  std::unordered_map<std::string, std::set<std::string>>
      _componentKeysByTemplateType;
  std::map<std::string, std::map<std::string, ScriptRef>::iterator>
      onUpdateComponents;
  std::set<std::string> onLateUpdateComponents;
  std::set<std::string> onStartComponents;
  std::set<std::string> onContactBeginComponents;
  std::set<std::string> onContactEndComponents;
  std::set<std::string> onTriggerBeginComponents;
  std::set<std::string> onTriggerEndComponents;
  std::set<std::string> onDestroyComponents;
  std::unordered_set<std::string> pendingDeletion;
};

struct HitResult {
  Actor* actor;
  b2Vec2 point;
  b2Vec2 normal;
  bool is_trigger;
  float fraction;
  friend bool operator<(const HitResult& lhs, const HitResult& rhs) {
    return lhs.fraction < rhs.fraction;
  }
};

class RaycastListener : public b2RayCastCallback {
 public:
  float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                      const b2Vec2& normal, float fraction) final;
  const std::vector<HitResult>& GetAllRayCast() {
    std::sort(hitResults.begin(), hitResults.end());
    return hitResults;
  }
  void Reset() { hitResults.clear(); }

 private:
  std::vector<HitResult> hitResults;
};

class SceneManager {
 public:
  SceneManager(Graphics::GraphicsManager* graphicsManager,
               AudioManager* audioManager, Input::InputManager* inputManager,
               NetworkManager* networkManager);
  ~SceneManager();

  void UpdateSceneActors();
  void SetScene(const std::string& sceneName);

  ScriptRef CreateActor(const std::string& actorTemplate);
  ScriptRef GetComponent(Actor* actor, const std::string& templateType);

  std::pair<float, float> ConvertCordsToScreen(float x, float y);

 private:
  friend class LuaScriptingBackend;

  Graphics::GraphicsManager* _graphicsManager;
  AudioManager* _audioManager;
  Input::InputManager* _inputManager;
  NetworkManager* _networkManager;

  std::unique_ptr<ScriptingBackend> _backend;

  std::unordered_map<int, Actor> actors;

  std::unordered_map<std::string, Actor> templateActors;
  std::unordered_map<std::string, std::set<int>> _actorsByName;
  std::queue<std::pair<int, ScriptRef>> newComponents;
  std::queue<std::pair<int, ScriptRef>> newComponentsBuffer;
  std::queue<std::pair<int, std::string>> removedComponents;
  std::map<int, Actor*> onUpdateActors;
  std::set<int> onLateUpdateActors;
  std::unordered_set<int> deactivedActors;
  std::queue<std::pair<int, bool>> actorChanges;
  size_t _actorCounter;
  size_t _componentCounter;

  std::unique_ptr<b2World> _physicsWorld;
  ContactListener _contactListener;
  RaycastListener _raycastListener;
  bool _physicsInitialized{false};

  std::pair<float, float> _cameraPos;
  float _cameraZoom;

  std::optional<std::string> _nextScene;
  std::string _currentScene;
  unsigned int _frameNumber;

  bool IsOnScreen(Actor& actor, const glm::ivec4& screenCoords);

  void LoadActorFromConfig(Actor& actor, const rapidjson::Value& actorConfig);
  void LoadTemplateFromConfig(Actor& actor,
                              const rapidjson::Document& templateConfig);
  Actor* GetTemplateActor(const std::string& templateActorName);
  void LoadActorFromTemplate(Actor& actor, Actor& templateActor);
  void SetParameter(Actor& actor, const std::string& configName,
                    const rapidjson::Value& value);

  void RunOnUpdate();
  void HandleActorChanges();
  void RunOnLateUpdate();
  void AddComponents();
  void RemoveComponents();
  void CleanupActorComponents(Actor& actor);

  void AttachComponentToActor(Actor& actor, ScriptRef& component);
  void DetachComponentFromActor(Actor& actor, const std::string& componentName);

  void InitializePhysics();
};

extern SceneManager* scene;
extern std::pair<float, float>* static_camera_pos;
