#include "SceneManager.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <vector>

#include "AudioManager.h"
#include "Defines.h"
#include "EngineComponents.h"
#include "GraphicsManager.h"
#include "Helper.h"
#include "InputManager.h"
#include "NetworkManager.h"
#include "ScriptingBackend.h"

static const std::filesystem::path SCENE_PATH =
    std::filesystem::path{"resources"} / std::filesystem::path{"scenes"};
static const std::filesystem::path TEMPLATE_PATH =
    std::filesystem::path{"resources"} /
    std::filesystem::path{"actor_templates"};

RaycastListener* static_raycast_listener = nullptr;
b2World* static_physics_world = nullptr;
Graphics::GraphicsManager* static_graphics_manager = nullptr;
std::pair<float, float>* static_camera_pos = nullptr;
float* static_camera_zoom = nullptr;

void ContactListener::BeginContact(b2Contact* contact) {
  auto* actorA =
      reinterpret_cast<Actor*>(contact->GetFixtureA()->GetUserData().pointer);
  auto* actorB =
      reinterpret_cast<Actor*>(contact->GetFixtureB()->GetUserData().pointer);
  b2WorldManifold worldManifold;
  contact->GetWorldManifold(&worldManifold);
  Collision collision{
      actorB, worldManifold.points[0],
      contact->GetFixtureA()->GetBody()->GetLinearVelocity() -
          contact->GetFixtureB()->GetBody()->GetLinearVelocity(),
      worldManifold.normal};

  auto& actorAComponentsToUpdate = actorA->onContactBeginComponents;
  auto& actorBComponentsToUpdate = actorB->onContactBeginComponents;
  const char* functionName = "OnCollisionEnter";
  bool isTrigger =
      (contact->GetFixtureA()->GetFilterData().categoryBits == 0x0002);
  if (isTrigger) {
    actorAComponentsToUpdate = actorA->onTriggerBeginComponents;
    actorBComponentsToUpdate = actorB->onTriggerBeginComponents;
    functionName = "OnTriggerEnter";
    collision.point = b2Vec2(-999.0f, -999.0f);
    collision.normal = b2Vec2(-999.0f, -999.0f);
  }

  for (const auto& componentName : actorAComponentsToUpdate) {
    auto& component = actorA->components.at(componentName);
    if (_backend->GetBool(component, "enabled") == false) {
      continue;
    }
    _backend->CallCollisionMethod(component, functionName, collision,
                                  actorA->name);
  }

  collision.other = actorA;
  for (const auto& componentName : actorBComponentsToUpdate) {
    auto& component = actorB->components.at(componentName);
    if (_backend->GetBool(component, "enabled") == false) {
      continue;
    }
    _backend->CallCollisionMethod(component, functionName, collision,
                                  actorB->name);
  }
}
void ContactListener::EndContact(b2Contact* contact) {
  auto* actorA =
      reinterpret_cast<Actor*>(contact->GetFixtureA()->GetUserData().pointer);
  auto* actorB =
      reinterpret_cast<Actor*>(contact->GetFixtureB()->GetUserData().pointer);
  Collision collision{
      actorB, b2Vec2(-999.0f, -999.0f),
      contact->GetFixtureA()->GetBody()->GetLinearVelocity() -
          contact->GetFixtureB()->GetBody()->GetLinearVelocity(),
      b2Vec2(-999.0f, -999.0f)};

  auto& actorAComponentsToUpdate = actorA->onContactEndComponents;
  auto& actorBComponentsToUpdate = actorB->onContactEndComponents;
  const char* functionName = "OnCollisionExit";
  bool isTrigger =
      (contact->GetFixtureA()->GetFilterData().categoryBits == 0x0002);
  if (isTrigger) {
    actorAComponentsToUpdate = actorA->onTriggerEndComponents;
    actorBComponentsToUpdate = actorB->onTriggerEndComponents;
    functionName = "OnTriggerExit";
  }

  for (const auto& componentName : actorAComponentsToUpdate) {
    auto& component = actorA->components.at(componentName);
    if (_backend->GetBool(component, "enabled") == false) {
      continue;
    }
    _backend->CallCollisionMethod(component, functionName, collision,
                                  actorA->name);
  }

  collision.other = actorA;
  for (const auto& componentName : actorBComponentsToUpdate) {
    auto& component = actorB->components.at(componentName);
    if (_backend->GetBool(component, "enabled") == false) {
      continue;
    }
    _backend->CallCollisionMethod(component, functionName, collision,
                                  actorB->name);
  }
}

float RaycastListener::ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                                     const b2Vec2& normal, float fraction) {
  if (reinterpret_cast<Actor*>(fixture->GetUserData().pointer) == nullptr) {
    return -1;
  }
  HitResult hitResult{};
  hitResult.actor = reinterpret_cast<Actor*>(fixture->GetUserData().pointer);
  hitResult.is_trigger = (fixture->GetFilterData().categoryBits == 0x0002);
  hitResult.normal = normal;
  hitResult.point = point;
  hitResult.fraction = fraction;
  hitResults.emplace_back(std::move(hitResult));
  return 1.0f;
}

ScriptRef SceneManager::CreateActor(const std::string& actorTemplate) {
  auto* actor = &actors[_actorCounter];
  actor->id = _actorCounter++;
  auto* templateActor = GetTemplateActor(actorTemplate);
  LoadActorFromTemplate(*actor, *templateActor);
  _actorsByName[actor->name].emplace(actor->id);
  for (auto& [componentName, component] : actor->components) {
    newComponents.emplace(actor->id, component);
  }
  actorChanges.emplace(actor->id, true);

  return _backend->WrapActor(actor);
}

ScriptRef SceneManager::GetComponent(Actor* actor,
                                     const std::string& templateType) {
  if (actor->_componentKeysByTemplateType.find(templateType) ==
      actor->_componentKeysByTemplateType.end()) {
    return ScriptRef();
  }
  for (const auto& componentName :
       actor->_componentKeysByTemplateType.at(templateType)) {
    if (actor->pendingDeletion.count(componentName) == 0) {
      return actor->components.at(componentName);
    }
  }
  return ScriptRef();
}

static std::optional<std::string> obtainWordAfterPhrase(
    const std::string& input, const std::string& phrase) {
  auto pos = input.find(phrase);

  if (pos == std::string::npos) {
    return std::nullopt;
  }

  pos += phrase.length();
  while (pos < input.size() && std::isspace(input[pos])) {
    ++pos;
  }

  if (pos == input.size()) {
    return std::nullopt;
  }

  auto endPos = pos;
  while (endPos < input.size() && !std::isspace(input[endPos])) {
    ++endPos;
  }
  return input.substr(pos, endPos - pos);
}

static void ReadJsonFile(const std::string& path,
                         rapidjson::Document& out_document) {
  FILE* file_pointer = nullptr;
#ifdef _WIN32
  fopen_s(&file_pointer, path.c_str(), "rb");
#else
  file_pointer = fopen(path.c_str(), "rb");
#endif
  char buffer[65536];
  rapidjson::FileReadStream stream(file_pointer, buffer, sizeof(buffer));
  out_document.ParseStream(stream);
  std::fclose(file_pointer);

  if (out_document.HasParseError()) {
    rapidjson::ParseErrorCode errorCode = out_document.GetParseError();
    throw std::runtime_error("error parsing json at [" + path + "]");
  }
}

static void AddComponentProperty(ScriptingBackend* backend,
                                 ScriptRef& component,
                                 const std::string& propertyName,
                                 const rapidjson::Value& value) {
  switch (value.GetType()) {
    case rapidjson::Type::kFalseType:
    case rapidjson::Type::kTrueType:
      backend->SetBool(component, propertyName, value.GetBool());
      break;
    case rapidjson::Type::kNumberType: {
      if (value.IsInt()) {
        backend->SetInt(component, propertyName, value.GetInt());
      } else {
        backend->SetFloat(component, propertyName, value.GetFloat());
      }
    } break;
    case rapidjson::Type::kStringType:
      backend->SetString(component, propertyName, value.GetString());
      break;
    case rapidjson::Type::kNullType:
    case rapidjson::Type::kArrayType:
    case rapidjson::Type::kObjectType:
      std::cout << "Unimplemented component property type: " << value.GetType()
                << std::endl;
      exit(0);
  }
}

void SceneManager::AttachComponentToActor(Actor& actor, ScriptRef& component) {
  std::string key = _backend->GetString(component, "key");
  auto [itr, _] = actor.components.emplace(key, component);
  actor._componentKeysByTemplateType[_backend->GetString(component, "type")]
      .emplace(key);
  _backend->SetActor(itr->second, "actor", &actor);
  _backend->SetBool(itr->second, "enabled", true);

  if (_backend->IsMethod(itr->second, "OnStart")) {
    actor.onStartComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnUpdate")) {
    actor.onUpdateComponents.emplace(key, itr);
  }
  if (_backend->IsMethod(itr->second, "OnLateUpdate")) {
    actor.onLateUpdateComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnCollisionEnter")) {
    actor.onContactBeginComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnCollisionExit")) {
    actor.onContactEndComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnTriggerEnter")) {
    actor.onTriggerBeginComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnTriggerExit")) {
    actor.onTriggerEndComponents.emplace(key);
  }
  if (_backend->IsMethod(itr->second, "OnDestroy")) {
    actor.onDestroyComponents.emplace(key);
  }
}

void SceneManager::DetachComponentFromActor(Actor& actor,
                                            const std::string& componentName) {
  auto& component = actor.components.at(componentName);
  std::string componentType = _backend->GetString(component, "type");
  actor.pendingDeletion.erase(componentName);
  actor.onStartComponents.erase(componentName);
  actor.onLateUpdateComponents.erase(componentName);
  actor.onUpdateComponents.erase(componentName);
  actor.onContactBeginComponents.erase(componentName);
  actor.onContactEndComponents.erase(componentName);
  actor.onTriggerBeginComponents.erase(componentName);
  actor.onTriggerEndComponents.erase(componentName);
  if (_backend->IsMethod(component, "OnDestroy")) {
    _backend->CallSelfMethod(component, "OnDestroy", actor.name);
  }
  if (componentType == "Rigidbody") {
    _physicsWorld->DestroyBody(_backend->AsRigidBody(component)->body);
  }
  _backend->DestroyNativeComponent(component, componentType);
  actor.onDestroyComponents.erase(componentName);
  actor.components.erase(componentName);
  actor._componentKeysByTemplateType[componentType].erase(componentName);
  if (actor._componentKeysByTemplateType[componentType].empty()) {
    actor._componentKeysByTemplateType.erase(componentType);
  }
}

void SceneManager::InitializePhysics() {
  if (_physicsInitialized) return;
  _physicsInitialized = true;
  _physicsWorld = std::make_unique<b2World>(b2Vec2(0.0f, 9.8f));
  _physicsWorld->SetContactListener(&_contactListener);
  static_physics_world = _physicsWorld.get();
}

void SceneManager::SetParameter(Actor& actor, const std::string& configName,
                                const rapidjson::Value& value) {
  if (configName == "name") {
    actor.name = value.GetString();
  }
  if (configName == "components") {
    for (auto componentItr = value.MemberBegin();
         componentItr != value.MemberEnd(); componentItr++) {
      if (actor.components.find(componentItr->name.GetString()) ==
          actor.components.end()) {
        const std::string componentType = componentItr->value["type"].GetString();
        if (componentType == "Rigidbody") {
          InitializePhysics();
        }
        auto templateComponent = _backend->LoadComponentTemplate(componentType);
        ScriptRef componentInstance = _backend->CreateComponentInstance(
            componentItr->name.GetString(), templateComponent);
        AttachComponentToActor(actor, componentInstance);
      }
      for (auto propertyItr = componentItr->value.MemberBegin();
           propertyItr != componentItr->value.MemberEnd(); propertyItr++) {
        AddComponentProperty(
            _backend.get(),
            actor.components.at(componentItr->name.GetString()),
            propertyItr->name.GetString(), propertyItr->value);
      }
    }
  }
}

void SceneManager::LoadActorFromTemplate(Actor& actor, Actor& templateActor) {
  actor.name = templateActor.name;
  for (auto& [componentName, templateActorComponent] :
       templateActor.components) {
    auto actorComponent =
        _backend->CreateComponentInstance(componentName, templateActorComponent);
    AttachComponentToActor(actor, actorComponent);
  }
}

Actor* SceneManager::GetTemplateActor(const std::string& templateActorName) {
  if (templateActors.find(templateActorName) == templateActors.end()) {
    auto templatePath = TEMPLATE_PATH / (templateActorName + ".template");
    if (std::filesystem::exists(templatePath) == false) {
      std::cout << "error: template " << templateActorName << " is missing";
      exit(0);
    }
    rapidjson::Document templateConfig;
    ReadJsonFile(templatePath.string(), templateConfig);
    LoadTemplateFromConfig(templateActors[templateActorName], templateConfig);
  }
  return &templateActors[templateActorName];
}

void SceneManager::LoadActorFromConfig(Actor& actor,
                                       const rapidjson::Value& actorConfig) {
  std::string configName{};
  for (auto itr = actorConfig.MemberBegin(); itr != actorConfig.MemberEnd();
       itr++) {
    configName = itr->name.GetString();
    SetParameter(actor, configName, itr->value);
  }
}

void SceneManager::LoadTemplateFromConfig(
    Actor& actor, const rapidjson::Document& templateConfig) {
  std::string configName{};
  for (auto itr = templateConfig.MemberBegin();
       itr != templateConfig.MemberEnd(); itr++) {
    configName = itr->name.GetString();
    SetParameter(actor, configName, itr->value);
  }
}

SceneManager::SceneManager(Graphics::GraphicsManager* graphicsManager,
                           AudioManager* audioManager,
                           Input::InputManager* inputManager,
                           NetworkManager* networkManager,
                           std::unique_ptr<ScriptingBackend> backend)
    : _graphicsManager(graphicsManager),
      _audioManager(audioManager),
      _inputManager(inputManager),
      _networkManager(networkManager),
      _backend(std::move(backend)),
      _actorCounter(0),
      _componentCounter(0),
      _cameraPos(0.0f, 0.0f),
      _cameraZoom(1.0f),
      _nextScene("basic"),
      _currentScene(""),
      _frameNumber(0) {
  _contactListener.SetBackend(_backend.get());
  static_raycast_listener = &_raycastListener;
  static_graphics_manager = _graphicsManager;
  static_camera_pos = &_cameraPos;
  static_camera_zoom = &_cameraZoom;

  _backend->Initialize(this, _graphicsManager, _audioManager, _inputManager,
                       _networkManager, _physicsWorld.get());
}

SceneManager::~SceneManager() = default;

bool SceneManager::IsOnScreen(Actor& actor, const glm::ivec4& screenCoords) {
  return true;
}

void SceneManager::RunOnUpdate() {
  for (auto& [_, actor] : onUpdateActors) {
    for (auto& [_, itr] : actor->onUpdateComponents) {
      auto& component = itr->second;
      if (_backend->GetBool(component, "enabled") == false) {
        continue;
      }
      _backend->CallSelfMethod(component, "OnUpdate", actor->name);
    }
  }
}

void SceneManager::CleanupActorComponents(Actor& actor) {
  for (auto& componentName : actor.onDestroyComponents) {
    auto& component = actor.components.at(componentName);
    _backend->CallSelfMethod(component, "OnDestroy", actor.name);
  }
  auto rbIt = actor._componentKeysByTemplateType.find("Rigidbody");
  if (rbIt != actor._componentKeysByTemplateType.end()) {
    for (auto& componentName : rbIt->second) {
      auto& component = actor.components.at(componentName);
      _physicsWorld->DestroyBody(_backend->AsRigidBody(component)->body);
      _backend->DestroyNativeComponent(component, "Rigidbody");
    }
  }
  auto psIt = actor._componentKeysByTemplateType.find("ParticleSystem");
  if (psIt != actor._componentKeysByTemplateType.end()) {
    for (auto& componentName : psIt->second) {
      auto& component = actor.components.at(componentName);
      _backend->DestroyNativeComponent(component, "ParticleSystem");
    }
  }
}

void SceneManager::HandleActorChanges() {
  while (actorChanges.empty() == false) {
    auto [actorId, isNewActor] = actorChanges.front();
    actorChanges.pop();
    if (isNewActor) {
      auto& actor = actors[actorId];
      if (actor.onUpdateComponents.empty() == false) {
        onUpdateActors.emplace(actorId, &actor);
      }
      if (actor.onLateUpdateComponents.empty() == false) {
        onLateUpdateActors.insert(actorId);
      }
    } else {
      auto it = actors.find(actorId);
      if (it == actors.end()) {
        continue;
      }
      auto& actor = it->second;
      CleanupActorComponents(actor);

      onUpdateActors.erase(actorId);
      onLateUpdateActors.erase(actorId);
      _actorsByName[actor.name].erase(actorId);
      actors.erase(it);
      deactivedActors.erase(actorId);
    }
  }
}

void SceneManager::RunOnLateUpdate() {
  for (auto actorId : onLateUpdateActors) {
    auto& actor = actors[actorId];
    for (auto& componentName : actor.onLateUpdateComponents) {
      auto& component = actor.components.at(componentName);
      if (_backend->GetBool(component, "enabled") == false) {
        continue;
      }
      _backend->CallSelfMethod(component, "OnLateUpdate", actor.name);
    }
  }
}

void SceneManager::AddComponents() {
  std::swap(newComponents, newComponentsBuffer);
  while (newComponentsBuffer.empty() == false) {
    auto [actorId, component] = newComponentsBuffer.front();
    newComponentsBuffer.pop();
    if (actors.find(actorId) == actors.end()) {
      continue;
    }
    auto& actor = actors[actorId];
    if (!_backend->HasActorProperty(component, "actor")) {
      AttachComponentToActor(actor, component);
      if (actor.onUpdateComponents.empty() == false &&
          onUpdateActors.count(actor.id) == 0) {
        onUpdateActors.emplace(actor.id, &actor);
      }
      if (actor.onLateUpdateComponents.empty() == false &&
          onLateUpdateActors.count(actor.id) == 0) {
        onLateUpdateActors.insert(actor.id);
      }
    }
    const std::string componentKey = _backend->GetString(component, "key");
    if (_backend->GetBool(component, "enabled") == false ||
        actor.onStartComponents.count(componentKey) == 0) {
      continue;
    }
    _backend->CallSelfMethod(component, "OnStart", actor.name);
  }
}
void SceneManager::RemoveComponents() {
  while (removedComponents.empty() == false) {
    const auto [actorId, componentName] = removedComponents.front();
    removedComponents.pop();
    if (actors.find(actorId) == actors.end()) {
      continue;
    }
    DetachComponentFromActor(actors[actorId], componentName);
  }
}

void SceneManager::UpdateSceneActors() {
  if (_nextScene.has_value()) {
    SetScene(_nextScene.value());
    _nextScene.reset();
  }

  AddComponents();
  RunOnUpdate();
  RunOnLateUpdate();
  RemoveComponents();
  HandleActorChanges();

  _backend->ProcessFrameEndQueues();

  if (_physicsWorld) {
    _physicsWorld->Step(1.0f / 60.0f, 8, 3);
  }
  _frameNumber++;
}

void SceneManager::SetScene(const std::string& sceneName) {
  auto p = SCENE_PATH / sceneName;
  p += std::filesystem::path(".scene");
  if (std::filesystem::exists(p) == false) {
    std::cout << "error: scene " << sceneName << " is missing";
    exit(0);
  }

  rapidjson::Document sceneConfig;
  ReadJsonFile(p.string(), sceneConfig);

  const rapidjson::Value& actorList = sceneConfig["actors"];

  if (_currentScene.empty()) {
    actors = std::unordered_map<int, Actor>();
    actors.reserve(actorList.Size() > 8192 ? actorList.Size() : 8192);
  } else {
    for (auto itr = actors.begin(); itr != actors.end();) {
      if (itr->second.dontDestroy) {
        itr++;
      } else {
        auto& actor = itr->second;
        CleanupActorComponents(actor);

        _actorsByName[actor.name].erase(actor.id);
        onUpdateActors.erase(actor.id);
        onLateUpdateActors.erase(actor.id);
        deactivedActors.erase(actor.id);
        if (_actorsByName[actor.name].empty()) {
          _actorsByName.erase(actor.name);
        }
        itr = actors.erase(itr);
      }
    }
    deactivedActors.clear();
  }
  _currentScene = sceneName;

  for (auto& actorConfig : actorList.GetArray()) {
    auto& actor = actors[_actorCounter];
    actor.id = _actorCounter++;

    if (actorConfig.HasMember("template")) {
      std::string templateActorName = actorConfig["template"].GetString();
      auto* templateActor = GetTemplateActor(templateActorName);
      LoadActorFromTemplate(actor, *templateActor);
    }

    LoadActorFromConfig(actor, actorConfig);
    for (auto& [componentName, component] : actor.components) {
      newComponents.emplace(actor.id, component);
    }
    _actorsByName[actor.name].emplace(actor.id);
    if (actor.onUpdateComponents.empty() == false) {
      onUpdateActors.emplace(actor.id, &actor);
    }
    if (actor.onLateUpdateComponents.empty() == false) {
      onLateUpdateActors.insert(actor.id);
    }
  }
}

std::pair<float, float> SceneManager::ConvertCordsToScreen(float x, float y) {
  auto [screenWidth, screenHeight] = _graphicsManager->GetScreenDimension();
  return {x * PIXELS_PER_METER + (screenWidth * 0.5f) * (1.0f / _cameraZoom),
          y * PIXELS_PER_METER + (screenHeight * 0.5f) * (1.0f / _cameraZoom)};
}
