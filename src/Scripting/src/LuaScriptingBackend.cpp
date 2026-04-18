#include "LuaScriptingBackend.h"

#include <LuaBridge/Optional.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "AudioManager.h"
#include "Defines.h"
#include "GraphicsManager.h"
#include "InputManager.h"
#include "NetworkManager.h"
#include "SceneManager.h"

namespace luabridge {
namespace detail {

// Extend LuaBridge to support more than 9 parameters in bound functions.
// These specializations follow the same pattern as the built-in Caller<_, 9>.

template <class ReturnType>
struct Caller<ReturnType, 10> {
  template <class Fn, class Params>
  static ReturnType f(Fn& fn, TypeListValues<Params>& tvl) {
    return fn(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }

  template <class T, class MemFn, class Params>
  static ReturnType f(T* obj, MemFn& fn, TypeListValues<Params>& tvl) {
    return (obj->*fn)(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }
};

template <class ReturnType>
struct Caller<ReturnType, 11> {
  template <class Fn, class Params>
  static ReturnType f(Fn& fn, TypeListValues<Params>& tvl) {
    return fn(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }

  template <class T, class MemFn, class Params>
  static ReturnType f(T* obj, MemFn& fn, TypeListValues<Params>& tvl) {
    return (obj->*fn)(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
                      tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }
};

template <class ReturnType>
struct Caller<ReturnType, 12> {
  template <class Fn, class Params>
  static ReturnType f(Fn& fn, TypeListValues<Params>& tvl) {
    return fn(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }

  template <class T, class MemFn, class Params>
  static ReturnType f(T* obj, MemFn& fn, TypeListValues<Params>& tvl) {
    return (obj->*fn)(
        tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }
};

template <class ReturnType>
struct Caller<ReturnType, 13> {
  template <class Fn, class Params>
  static ReturnType f(Fn& fn, TypeListValues<Params>& tvl) {
    return fn(tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }

  template <class T, class MemFn, class Params>
  static ReturnType f(T* obj, MemFn& fn, TypeListValues<Params>& tvl) {
    return (obj->*fn)(
        tvl.hd, tvl.tl.hd, tvl.tl.tl.hd, tvl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
        tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd);
  }
};

}  // namespace detail
}  // namespace luabridge

// Globals used by physics raycast helpers (defined in SceneManager.cpp).
extern RaycastListener* static_raycast_listener;
extern b2World* static_physics_world;

static const std::filesystem::path COMPONENT_DIR =
    std::filesystem::path("resources") /
    std::filesystem::path("component_types");

static void ReportError(const std::string& actorName,
                        const luabridge::LuaException& e) {
  std::string error_message = e.what();
  std::replace(error_message.begin(), error_message.end(), '\\', '/');
  std::cout << "\033[31m" << actorName << " : " << error_message << "\033[0m"
            << std::endl;
}

luabridge::LuaRef& LuaScriptingBackend::Unwrap(ScriptRef& ref) {
  return *std::any_cast<luabridge::LuaRef>(&ref);
}
const luabridge::LuaRef& LuaScriptingBackend::Unwrap(const ScriptRef& ref) {
  return *std::any_cast<luabridge::LuaRef>(&ref);
}

LuaScriptingBackend::LuaScriptingBackend()
    : _luaState(nullptr),
      _scene(nullptr),
      _graphics(nullptr),
      _audio(nullptr),
      _input(nullptr),
      _network(nullptr),
      _physics(nullptr) {}

LuaScriptingBackend::~LuaScriptingBackend() {
  if (_luaState) {
    lua_close(_luaState);
  }
}

void LuaScriptingBackend::Initialize(SceneManager* scene,
                                     Graphics::GraphicsManager* graphics,
                                     AudioManager* audio,
                                     Input::InputManager* input,
                                     NetworkManager* network,
                                     b2World* physics) {
  _scene = scene;
  _graphics = graphics;
  _audio = audio;
  _input = input;
  _network = network;
  _physics = physics;

  _luaState = luaL_newstate();
  luaL_openlibs(_luaState);

  InitializeClassAPI();
  InitializeDebugAPI();
  InitializeActorClassAPI();
  InitializeActorAPI();
  InitializeApplicationAPI();
  InitializeInputAPI();
  InitializeTextAPI();
  InitializeAudioAPI();
  InitializeDrawAPI();
  InitializeCameraAPI();
  InitializeSceneAPI();
  InitializePhysicsAPI();
  InitializeEventsAPI();
  InitializeNetworkAPI();
}

void LuaScriptingBackend::ProcessFrameEndQueues() {
  while (_newSubscriptions.empty() == false) {
    auto [eventType, component, func] = _newSubscriptions.front();
    _newSubscriptions.pop();
    _subscriptions[eventType].emplace_back(component, func);
  }
  while (_removeSubscriptions.empty() == false) {
    auto [eventType, component, func] = _removeSubscriptions.front();
    _removeSubscriptions.pop();
    auto subIt = _subscriptions.find(eventType);
    if (subIt == _subscriptions.end()) {
      continue;
    }
    auto& callbacks = subIt->second;
    for (auto it = callbacks.begin(); it != callbacks.end(); ++it) {
      if (it->first == component && it->second == func) {
        callbacks.erase(it);
        break;
      }
    }
    if (callbacks.empty()) {
      _subscriptions.erase(subIt);
    }
  }
}

luabridge::LuaRef LuaScriptingBackend::RunFile(
    const std::filesystem::path& filename) {
  if (luaL_dofile(_luaState, filename.string().c_str()) != LUA_OK) {
    std::cout << "problem with lua file " << filename.stem().string().c_str();
    exit(0);
  }
  return luabridge::getGlobal(_luaState, filename.stem().string().c_str());
}

void LuaScriptingBackend::EstablishInheritance(
    luabridge::LuaRef& instance,
    const luabridge::LuaRef& template_component) {
  if (template_component["type"].cast<std::string>() == "Rigidbody") {
    auto* parent = template_component.cast<RigidBody*>();
    instance = luabridge::LuaRef(_luaState, new RigidBody(*parent));
  } else if (template_component["type"].cast<std::string>() ==
             "ParticleSystem") {
    auto* parent = template_component.cast<ParticleSystem*>();
    instance = luabridge::LuaRef(_luaState, new ParticleSystem(*parent));
  } else {
    luabridge::LuaRef new_metatable = luabridge::newTable(_luaState);
    new_metatable["__index"] = template_component;

    instance.push(_luaState);
    new_metatable.push(_luaState);
    lua_setmetatable(_luaState, -2);
    lua_pop(_luaState, 1);
  }
}

ScriptRef LuaScriptingBackend::LoadComponentTemplate(
    const std::string& componentName) {
  if (_componentTemplates.find(componentName) != _componentTemplates.end()) {
    return ScriptRef(_componentTemplates.at(componentName));
  }

  if (componentName == "Rigidbody") {
    auto* rigidBody = new RigidBody();
    auto [itr, _] = _componentTemplates.emplace(
        componentName, luabridge::LuaRef(_luaState, rigidBody));
    itr->second["type"] = componentName;
  } else if (componentName == "ParticleSystem") {
    auto* particleSystem = new ParticleSystem();
    auto [itr, _] = _componentTemplates.emplace(
        componentName, luabridge::LuaRef(_luaState, particleSystem));
    itr->second["type"] = componentName;
  } else {
    const auto& componentPath =
        COMPONENT_DIR / (componentName + std::string(".lua"));
    if (!std::filesystem::exists(componentPath)) {
      std::cout << "error: failed to locate component " + componentName;
      exit(0);
    }
    RunFile(componentPath.string());
    auto [itr, _] = _componentTemplates.emplace(
        componentName, luabridge::getGlobal(_luaState, componentName.c_str()));
    itr->second["type"] = componentName;
  }

  return ScriptRef(_componentTemplates.at(componentName));
}

ScriptRef LuaScriptingBackend::CreateComponentInstance(
    const std::string& componentInstanceName, const ScriptRef& templateRef) {
  luabridge::LuaRef componentInstance = luabridge::newTable(_luaState);
  EstablishInheritance(componentInstance, Unwrap(templateRef));
  componentInstance["key"] = componentInstanceName;
  return ScriptRef(componentInstance);
}

void LuaScriptingBackend::DestroyNativeComponent(
    ScriptRef& ref, const std::string& componentType) {
  auto& lref = Unwrap(ref);
  if (componentType == "Rigidbody") {
    delete lref.cast<RigidBody*>();
  } else if (componentType == "ParticleSystem") {
    delete lref.cast<ParticleSystem*>();
  }
}

ScriptRef LuaScriptingBackend::WrapActor(Actor* actor) {
  return ScriptRef(luabridge::LuaRef(_luaState, actor));
}

bool LuaScriptingBackend::IsRefValid(const ScriptRef& ref) {
  if (!ref.has_value()) return false;
  auto* lref = std::any_cast<luabridge::LuaRef>(&ref);
  return lref && !lref->isNil();
}

bool LuaScriptingBackend::IsMethod(const ScriptRef& ref,
                                   const std::string& key) {
  return Unwrap(ref)[key].isFunction();
}

bool LuaScriptingBackend::HasActorProperty(const ScriptRef& ref,
                                           const std::string& key) {
  return !Unwrap(ref)[key].isNil();
}

bool LuaScriptingBackend::GetBool(const ScriptRef& ref,
                                  const std::string& key) {
  return Unwrap(ref)[key].cast<bool>();
}
int LuaScriptingBackend::GetInt(const ScriptRef& ref, const std::string& key) {
  return Unwrap(ref)[key].cast<int>();
}
float LuaScriptingBackend::GetFloat(const ScriptRef& ref,
                                    const std::string& key) {
  return Unwrap(ref)[key].cast<float>();
}
std::string LuaScriptingBackend::GetString(const ScriptRef& ref,
                                           const std::string& key) {
  return Unwrap(ref)[key].cast<std::string>();
}
Actor* LuaScriptingBackend::GetActor(const ScriptRef& ref,
                                     const std::string& key) {
  auto prop = Unwrap(ref)[key];
  if (prop.isNil()) return nullptr;
  return prop.cast<Actor*>();
}

void LuaScriptingBackend::SetBool(ScriptRef& ref, const std::string& key,
                                  bool v) {
  Unwrap(ref)[key] = v;
}
void LuaScriptingBackend::SetInt(ScriptRef& ref, const std::string& key,
                                 int v) {
  Unwrap(ref)[key] = v;
}
void LuaScriptingBackend::SetFloat(ScriptRef& ref, const std::string& key,
                                   float v) {
  Unwrap(ref)[key] = v;
}
void LuaScriptingBackend::SetString(ScriptRef& ref, const std::string& key,
                                    const std::string& v) {
  Unwrap(ref)[key] = v;
}
void LuaScriptingBackend::SetActor(ScriptRef& ref, const std::string& key,
                                   Actor* actor) {
  Unwrap(ref)[key] = actor;
}

void LuaScriptingBackend::CallSelfMethod(ScriptRef& ref,
                                         const std::string& method,
                                         std::string_view actorName) {
  auto& component = Unwrap(ref);
  try {
    component[method](component);
  } catch (const luabridge::LuaException& e) {
    ReportError(std::string(actorName), e);
  }
}

void LuaScriptingBackend::CallCollisionMethod(ScriptRef& ref,
                                              const std::string& method,
                                              const Collision& collision,
                                              std::string_view actorName) {
  auto& component = Unwrap(ref);
  try {
    component[method](component, collision);
  } catch (const luabridge::LuaException& e) {
    ReportError(std::string(actorName), e);
  }
}

RigidBody* LuaScriptingBackend::AsRigidBody(const ScriptRef& ref) {
  return Unwrap(ref).cast<RigidBody*>();
}
ParticleSystem* LuaScriptingBackend::AsParticleSystem(const ScriptRef& ref) {
  return Unwrap(ref).cast<ParticleSystem*>();
}

// ============================================================================
// API registration
// ============================================================================

void LuaScriptingBackend::InitializeClassAPI() {
  std::function<void(RigidBody*)> rigidBodyStart = [this](RigidBody* rb) {
    rb->OnStart(*_physics);
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginClass<glm::vec2>("vec2")
      .addProperty("x", &glm::vec2::x)
      .addProperty("y", &glm::vec2::y)
      .endClass()

      .beginClass<b2Vec2>("Vector2")
      .addConstructor<void (*)(float, float)>()
      .addProperty("x", &b2Vec2::x)
      .addProperty("y", &b2Vec2::y)
      .addFunction("Normalize", &b2Vec2::Normalize)
      .addFunction("Length", &b2Vec2::Length)
      .addFunction(
          "__add",
          +[](b2Vec2* lhs, const b2Vec2& rhs) {
            return b2Vec2(lhs->x + rhs.x, lhs->y + rhs.y);
          })
      .addFunction(
          "__sub",
          +[](b2Vec2* lhs, const b2Vec2& rhs) {
            return b2Vec2(lhs->x - rhs.x, lhs->y - rhs.y);
          })
      .addFunction(
          "__mul",
          +[](b2Vec2* lhs, const float multiplier) {
            return b2Vec2(lhs->x * multiplier, lhs->y * multiplier);
          })
      .addStaticFunction("Distance", &b2Distance)
      .addStaticFunction(
          "Dot", static_cast<float (*)(const b2Vec2&, const b2Vec2&)>(&b2Dot))
      .endClass()

      .beginClass<ParticleSystem>("ParticleSystem")
      .addProperty("Particle", &ParticleSystem::ImageName)
      .addProperty("key", &ParticleSystem::key)
      .addProperty("type", &ParticleSystem::type)
      .addProperty("enabled", &ParticleSystem::enabled)
      .addProperty("actor", &ParticleSystem::actor)
      .addProperty("x", &ParticleSystem::SpawnCenterX)
      .addProperty("y", &ParticleSystem::SpawnCenterY)
      .addProperty("start_scale_min", &ParticleSystem::ScaleMin)
      .addProperty("start_scale_max", &ParticleSystem::ScaleMax)
      .addProperty("rotation_min", &ParticleSystem::RotationMin)
      .addProperty("rotation_max", &ParticleSystem::RotationMax)
      .addProperty("start_color_r", &ParticleSystem::start_color_r)
      .addProperty("start_color_g", &ParticleSystem::start_color_g)
      .addProperty("start_color_b", &ParticleSystem::start_color_b)
      .addProperty("start_color_a", &ParticleSystem::start_color_a)
      .addProperty("end_color_r", &ParticleSystem::end_color_r)
      .addProperty("end_color_g", &ParticleSystem::end_color_g)
      .addProperty("end_color_b", &ParticleSystem::end_color_b)
      .addProperty("end_color_a", &ParticleSystem::end_color_a)
      .addProperty("emit_radius_min", &ParticleSystem::MinSpawnRadius)
      .addProperty("emit_radius_max", &ParticleSystem::MaxSpawnRadius)
      .addProperty("emit_angle_min", &ParticleSystem::MinSpawnAngle)
      .addProperty("emit_angle_max", &ParticleSystem::MaxSpawnAngle)
      .addProperty("start_speed_min", &ParticleSystem::SpeedMin)
      .addProperty("start_speed_max", &ParticleSystem::SpeedMax)
      .addProperty("rotation_speed_min", &ParticleSystem::RotationSpeedMin)
      .addProperty("rotation_speed_max", &ParticleSystem::RotationSpeedMax)
      .addProperty("gravity_scale_x", &ParticleSystem::ParticleAccelerationX)
      .addProperty("gravity_scale_y", &ParticleSystem::ParticleAccelerationY)
      .addProperty("image", &ParticleSystem::ImageName)
      .addProperty("sorting_order", &ParticleSystem::SortingOrder)
      .addProperty("drag_factor", &ParticleSystem::DragFactor)
      .addProperty("angular_drag_factor", &ParticleSystem::RotationDragFactor)
      .addProperty("end_scale", &ParticleSystem::EndScale)
      .addProperty("frames_between_bursts", &ParticleSystem::FramesBetweenSpawn)
      .addProperty("burst_quantity", &ParticleSystem::ParticlesPerSpawn)
      .addProperty("duration_frames", &ParticleSystem::ParticleLifeTimeInFrames)
      .addFunction("OnStart", &ParticleSystem::OnStart)
      .addFunction("OnUpdate", &ParticleSystem::OnUpdate)
      .addFunction("Stop", &ParticleSystem::DisableSpawn)
      .addFunction("Play", &ParticleSystem::EnableSpawn)
      .addFunction("Burst", &ParticleSystem::SpawnParticles)
      .endClass()

      .beginClass<HitResult>("HitResult")
      .addProperty("actor", &HitResult::actor)
      .addProperty("point", &HitResult::point)
      .addProperty("is_trigger", &HitResult::is_trigger)
      .addProperty("normal", &HitResult::normal)
      .endClass()

      .beginClass<Collision>("Collision")
      .addProperty("other", &Collision::other)
      .addProperty("point", &Collision::point)
      .addProperty("relative_velocity", &Collision::relative_velocity)
      .addProperty("normal", &Collision::normal)
      .endClass()

      .beginClass<RigidBody>("Rigidbody")
      .addProperty("key", &RigidBody::key)
      .addProperty("type", &RigidBody::type)
      .addProperty("enabled", &RigidBody::enabled)
      .addProperty("actor", &RigidBody::actor)
      .addProperty("x", &RigidBody::x)
      .addProperty("y", &RigidBody::y)
      .addProperty("body_type", &RigidBody::body_type)
      .addProperty("precise", &RigidBody::precise)
      .addProperty("gravity_scale", &RigidBody::gravity_scale)
      .addProperty("density", &RigidBody::density)
      .addProperty("angular_friction", &RigidBody::angular_friction)
      .addProperty("rotation", &RigidBody::rotation)
      .addProperty("has_collider", &RigidBody::has_collider)
      .addProperty("has_trigger", &RigidBody::has_trigger)
      .addProperty("collider_type", &RigidBody::collider_type)
      .addProperty("width", &RigidBody::width)
      .addProperty("height", &RigidBody::height)
      .addProperty("radius", &RigidBody::radius)
      .addProperty("friction", &RigidBody::friction)
      .addProperty("bounciness", &RigidBody::bounciness)
      .addProperty("trigger_type", &RigidBody::trigger_type)
      .addProperty("trigger_width", &RigidBody::trigger_width)
      .addProperty("trigger_height", &RigidBody::trigger_height)
      .addProperty("trigger_radius", &RigidBody::trigger_radius)
      .addFunction(
          "GetPosition",
          +[](RigidBody* rb) {
            if (rb->body == nullptr) {
              return b2Vec2{rb->x, rb->y};
            }
            return rb->body->GetPosition();
          })
      .addFunction(
          "GetRotation",
          +[](RigidBody* rb) {
            if (rb->body == nullptr) {
              return rb->rotation;
            }
            return ConvertRadainsToDegrees(rb->body->GetAngle());
          })
      .addFunction(
          "GetVelocity",
          +[](RigidBody* rb) { return rb->body->GetLinearVelocity(); })
      .addFunction(
          "GetAngularVelocity",
          +[](RigidBody* rb) {
            return ConvertRadainsToDegrees(rb->body->GetAngularVelocity());
          })
      .addFunction(
          "GetGravityScale",
          +[](RigidBody* rb) {
            if (rb->body == nullptr) {
              return rb->gravity_scale;
            }
            return rb->body->GetGravityScale();
          })
      .addFunction(
          "GetUpDirection",
          +[](RigidBody* rb) {
            float radAngle = -rb->body->GetAngle() + (b2_pi / 2.0f);
            auto res = b2Vec2(glm::cos(radAngle), -glm::sin(radAngle));
            res.Normalize();
            return res;
          })
      .addFunction(
          "GetRightDirection",
          +[](RigidBody* rb) {
            float radAngle = -rb->body->GetAngle();
            return b2Vec2(glm::cos(radAngle), -glm::sin(radAngle));
          })
      .addFunction("OnStart", rigidBodyStart)
      .addFunction(
          "AddForce",
          +[](RigidBody* rb, b2Vec2& force) {
            rb->body->ApplyForceToCenter(force, true);
          })
      .addFunction(
          "SetVelocity",
          +[](RigidBody* rb, b2Vec2& velocity) {
            rb->body->SetLinearVelocity(velocity);
          })
      .addFunction(
          "SetPosition",
          +[](RigidBody* rb, b2Vec2& position) {
            if (rb->body == nullptr) {
              rb->x = position.x;
              rb->y = position.y;
            } else {
              rb->body->SetTransform(position, rb->body->GetAngle());
            }
          })
      .addFunction(
          "SetRotation",
          +[](RigidBody* rb, float degClockwise) {
            if (rb->body == nullptr) {
              rb->rotation = degClockwise;
            } else {
              rb->body->SetTransform(rb->body->GetPosition(),
                                     ConvertDegreesToRadians(degClockwise));
            }
          })
      .addFunction(
          "SetAngularVelocity",
          +[](RigidBody* rb, float degClockwise) {
            rb->body->SetAngularVelocity(ConvertDegreesToRadians(degClockwise));
          })
      .addFunction(
          "SetGravityScale",
          +[](RigidBody* rb, float gravityScale) {
            if (rb->body == nullptr) {
              rb->gravity_scale = gravityScale;
            } else {
              rb->body->SetGravityScale(gravityScale);
            }
          })
      .addFunction(
          "SetUpDirection",
          +[](RigidBody* rb, b2Vec2& direction) {
            direction.Normalize();
            rb->body->SetTransform(rb->body->GetPosition(),
                                   -glm::atan(-direction.y, direction.x) +
                                       (b2_pi / 2.0f));
          })
      .addFunction(
          "SetRightDirection",
          +[](RigidBody* rb, b2Vec2& direction) {
            direction.Normalize();
            rb->body->SetTransform(rb->body->GetPosition(),
                                   -glm::atan(-direction.y, direction.x));
          })
      .endClass();
}

void LuaScriptingBackend::InitializeDebugAPI() {
  std::function<void(const std::string&)> log =
      [](const std::string& message) { std::cout << message << std::endl; };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Debug")
      .addFunction("Log", log)
      .endNamespace();
}

void LuaScriptingBackend::InitializeActorClassAPI() {
  std::function<luabridge::LuaRef(Actor*, const std::string&)>
      getComponentByKey = [this](Actor* actor, const std::string& key) {
        if (actor->components.find(key) == actor->components.end() ||
            actor->pendingDeletion.count(key) > 0) {
          return luabridge::LuaRef(_luaState);
        }
        return Unwrap(actor->components.at(key));
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> getComponent =
      [this](Actor* actor, const std::string& templateType) {
        auto component = luabridge::LuaRef(_luaState);
        if (actor->_componentKeysByTemplateType.find(templateType) ==
            actor->_componentKeysByTemplateType.end()) {
          return component;
        }
        for (const auto& componentName :
             actor->_componentKeysByTemplateType.at(templateType)) {
          if (actor->pendingDeletion.count(componentName) == 0) {
            component = Unwrap(actor->components.at(componentName));
            break;
          }
        }
        return component;
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> getComponents =
      [this](Actor* actor, const std::string& templateType) {
        auto ref = luabridge::newTable(_luaState);
        if (actor->_componentKeysByTemplateType.find(templateType) ==
            actor->_componentKeysByTemplateType.end()) {
          return ref;
        }
        int i = 1;
        for (const auto& componentName :
             actor->_componentKeysByTemplateType.at(templateType)) {
          if (actor->pendingDeletion.count(componentName) == 0) {
            ref[i++] = Unwrap(actor->components.at(componentName));
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> createComponent =
      [this](Actor* actor, const std::string& templateType) {
        auto templateComponent = LoadComponentTemplate(templateType);
        std::string componentName =
            "r" + std::to_string(_scene->_componentCounter++);
        auto component =
            CreateComponentInstance(componentName, templateComponent);
        _scene->newComponents.emplace(actor->id, component);
        return Unwrap(component);
      };
  std::function<void(Actor*, luabridge::LuaRef)> removeComponent =
      [this](Actor* actor, luabridge::LuaRef component) {
        component["enabled"] = false;
        actor->pendingDeletion.emplace(component["key"].cast<std::string>());
        _scene->removedComponents.emplace(actor->id,
                                          component["key"].cast<std::string>());
      };

  luabridge::getGlobalNamespace(_luaState)
      .beginClass<Actor>("Actor")
      .addFunction(
          "GetName", +[](const Actor* actor) { return actor->name; })
      .addFunction(
          "GetID", +[](const Actor* actor) { return actor->id; })
      .addFunction("GetComponentByKey", getComponentByKey)
      .addFunction("GetComponent", getComponent)
      .addFunction("GetComponents", getComponents)
      .addFunction("AddComponent", createComponent)
      .addFunction("RemoveComponent", removeComponent)
      .endClass();
}

void LuaScriptingBackend::InitializeActorAPI() {
  std::function<luabridge::LuaRef(const std::string&)> FindActor =
      [this](const std::string& actorName) {
        auto ref = luabridge::LuaRef(_luaState);
        if (_scene->_actorsByName.find(actorName) ==
                _scene->_actorsByName.end() ||
            _scene->_actorsByName.at(actorName).empty()) {
          return ref;
        }
        for (auto& actorId : _scene->_actorsByName.at(actorName)) {
          if (_scene->deactivedActors.count(actorId) == 0) {
            ref = luabridge::LuaRef(_luaState, &_scene->actors[actorId]);
            break;
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(const std::string&)> FindAllActor =
      [this](const std::string& actorName) {
        auto ref = luabridge::newTable(_luaState);
        if (_scene->_actorsByName.find(actorName) ==
            _scene->_actorsByName.end()) {
          return ref;
        }
        int i = 1;
        for (const int actorId : _scene->_actorsByName.at(actorName)) {
          if (_scene->deactivedActors.count(actorId) == 0) {
            ref[i++] = luabridge::LuaRef(_luaState, &_scene->actors[actorId]);
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(const std::string&)> CreateActor =
      [this](const std::string& actorTemplate) {
        auto ref = _scene->CreateActor(actorTemplate);
        return Unwrap(ref);
      };
  std::function<void(Actor&)> RemoveActor = [this](Actor& actor) {
    for (auto& [componentName, component] : actor.components) {
      SetBool(component, "enabled", false);
    }
    _scene->actorChanges.emplace(actor.id, false);
    _scene->deactivedActors.emplace(actor.id);
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Actor")
      .addFunction("Find", FindActor)
      .addFunction("FindAll", FindAllActor)
      .addFunction("Instantiate", CreateActor)
      .addFunction("Destroy", RemoveActor)
      .endNamespace();
}

void LuaScriptingBackend::InitializeApplicationAPI() {
  std::function<void()> quit = []() { exit(0); };
  std::function<unsigned int()> getFrame = [this]() {
    return _scene->_frameNumber;
  };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Application")
      .addFunction("GetFrame", getFrame)
      .addFunction("Quit", quit)
      .addFunction(
          "Sleep",
          +[](int milliseconds) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(milliseconds));
          })
      .addFunction(
          "OpenURL",
          +[](const std::string& url) {
#ifdef _WIN32
            std::string command = "start" + url;
#elif defined(__APPLE__)
            std::string command = "open" + url;
#else
            std::string command = "xdg-open" + url;
#endif
            std::system(command.c_str());
          })
      .endNamespace();
}

void LuaScriptingBackend::InitializeInputAPI() {
  std::function<bool(const std::string&)> getKey =
      [this](const std::string& keycode) { return _input->GetKey(keycode); };
  std::function<bool(const std::string&)> getKeyDown =
      [this](const std::string& keycode) {
        return _input->GetKeyDown(keycode);
      };
  std::function<bool(const std::string&)> getKeyUp =
      [this](const std::string& keycode) {
        return _input->GetKeyUp(keycode);
      };
  std::function<bool(int)> getmousebutton = [this](int buttonNum) {
    return _input->GetMouseButton(buttonNum);
  };
  std::function<bool(int)> getmousebuttonDown = [this](int buttonNum) {
    return _input->GetMouseButtonDown(buttonNum);
  };
  std::function<bool(int)> getmousebuttonUp = [this](int buttonNum) {
    return _input->GetMouseButtonUp(buttonNum);
  };
  std::function<glm::vec2()> getmousePosition = [this]() {
    return _input->GetMousePosition();
  };
  std::function<float()> getmouseDelta = [this]() {
    return _input->GetMouseScrollDelta();
  };
  std::function<void()> hideCursor = [this]() { _input->HideCursor(); };
  std::function<void()> showCursor = [this]() { _input->ShowCursor(); };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Input")
      .addFunction("GetKey", getKey)
      .addFunction("GetKeyDown", getKeyDown)
      .addFunction("GetKeyUp", getKeyUp)
      .addFunction("GetMouseButton", getmousebutton)
      .addFunction("GetMouseButtonDown", getmousebuttonDown)
      .addFunction("GetMouseButtonUp", getmousebuttonUp)
      .addFunction("GetMousePosition", getmousePosition)
      .addFunction("GetMouseScrollDelta", getmouseDelta)
      .addFunction("HideCursor", hideCursor)
      .addFunction("ShowCursor", showCursor)
      .endNamespace();
}

void LuaScriptingBackend::InitializeTextAPI() {
  std::function<void(const std::string&, int, int, const std::string&,
                     unsigned int, int, int, int, int)>
      draw = [this](const std::string& text, int x, int y,
                    const std::string& fontName, unsigned int fontSize, int r,
                    int g, int b, int a) {
        _graphics->RenderText(fontName, fontSize, text, x, y,
                              Graphics::RGBA{r, g, b, a});
      };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Text")
      .addFunction("Draw", draw)
      .endNamespace();
}

void LuaScriptingBackend::InitializeAudioAPI() {
  std::function<void(int, const std::string&, bool)> play =
      [this](int channel, const std::string& audioName, bool loop) {
        _audio->PlayAudio(audioName, channel, (loop) ? -1 : 0);
      };
  std::function<void(int)> halt = [this](int channel) {
    _audio->StopAudio(channel);
  };
  std::function<void(int, int)> setVolume = [this](int channel, int volume) {
    _audio->SetVolume(channel, volume);
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Audio")
      .addFunction("Play", play)
      .addFunction("Halt", halt)
      .addFunction("SetVolume", setVolume)
      .endNamespace();
}

void LuaScriptingBackend::InitializeDrawAPI() {
  std::function<void(const std::string&, float, float)> drawUI =
      [this](const std::string& imageName, float x, float y) {
        _graphics->DrawSprite(Graphics::SpriteInfo{imageName,
                                                   {int(x), int(y)},
                                                   {1.0f, 1.0f},
                                                   {0, 0},
                                                   0.0f,
                                                   {255, 255, 255, 255},
                                                   {1, 0}});
      };
  std::function<void(const std::string&, float, float, float, float, float,
                     float, float)>
      drawUIEx = [this](const std::string& imageName, float x, float y,
                        float r, float g, float b, float a,
                        float sortingOrder) {
        _graphics->DrawSprite(Graphics::SpriteInfo{imageName,
                                                   {int(x), int(y)},
                                                   {1.0f, 1.0f},
                                                   {0, 0},
                                                   0.0f,
                                                   {r, g, b, a},
                                                   {1, sortingOrder}});
      };
  std::function<void(const std::string&, int, int)> drawSprite =
      [this](const std::string& imageName, float x, float y) {
        auto [spriteWidth, spriteHeight] =
            _graphics->GetSpriteDimension(imageName);
        std::pair<float, float> pivot = {spriteWidth / 2, spriteHeight / 2};
        auto screenPosition = _scene->ConvertCordsToScreen(x, y);
        screenPosition.first -= pivot.first;
        screenPosition.second -= pivot.second;
        _graphics->DrawSprite(Graphics::SpriteInfo{imageName,
                                                   screenPosition,
                                                   {1.0f, 1.0f},
                                                   pivot,
                                                   0.0f,
                                                   {255, 255, 255, 255},
                                                   {0, 0}});
      };
  std::function<void(const std::string&, float, float, float, float, float,
                     float, float, float, float, float, float, float)>
      drawSpriteEx = [this](const std::string& imageName, float x, float y,
                            float rotation, float scaleX, float scaleY,
                            float pivotX, float pivotY, float r, float g,
                            float b, float a, float sortingOrder) {
        auto [spriteWidth, spriteHeight] =
            _graphics->GetSpriteDimension(imageName);
        std::pair<float, float> pivot = {scaleX * spriteWidth * pivotX,
                                         scaleY * spriteHeight * pivotY};
        auto screenPosition = _scene->ConvertCordsToScreen(x, y);
        screenPosition.first -= pivot.first;
        screenPosition.second -= pivot.second;
        _graphics->DrawSprite(
            Graphics::SpriteInfo{imageName,
                                 screenPosition,
                                 {scaleX, scaleY},
                                 pivot,
                                 static_cast<float>(int(rotation)),
                                 {r, g, b, a},
                                 {0, sortingOrder}});
      };
  std::function<void(float, float, float, float, float, float)> drawPixel =
      [this](float x, float y, float r, float g, float b, float a) {
        _graphics->DrawPoint(x, y, {r, g, b, a});
      };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Image")
      .addFunction("DrawUI", drawUI)
      .addFunction("DrawUIEx", drawUIEx)
      .addFunction("Draw", drawSprite)
      .addFunction("DrawEx", drawSpriteEx)
      .addFunction("DrawPixel", drawPixel)
      .endNamespace();
}

void LuaScriptingBackend::InitializeCameraAPI() {
  std::function<void(float, float)> setPos = [this](float x, float y) {
    _scene->_cameraPos = {x, y};
  };
  std::function<float()> getPosX = [this]() {
    return _scene->_cameraPos.first;
  };
  std::function<float()> getPosY = [this]() {
    return _scene->_cameraPos.second;
  };
  std::function<void(float)> setZoom = [this](float zoomFactor) {
    _scene->_cameraZoom = zoomFactor;
    _graphics->SetRenderScale(_scene->_cameraZoom);
  };
  std::function<float()> getZoom = [this]() { return _scene->_cameraZoom; };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Camera")
      .addFunction("SetPosition", setPos)
      .addFunction("GetPositionX", getPosX)
      .addFunction("GetPositionY", getPosY)
      .addFunction("SetZoom", setZoom)
      .addFunction("GetZoom", getZoom)
      .endNamespace();
}

void LuaScriptingBackend::InitializeSceneAPI() {
  std::function<void(const std::string&)> load =
      [this](const std::string& sceneName) { _scene->_nextScene = sceneName; };
  std::function<std::string()> getCurrent = [this]() {
    return _scene->_currentScene;
  };
  std::function<void(Actor*)> dontDestroy = [](Actor* actor) {
    actor->dontDestroy = true;
  };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Scene")
      .addFunction("Load", load)
      .addFunction("GetCurrent", getCurrent)
      .addFunction("DontDestroy", dontDestroy)
      .endNamespace();
}

void LuaScriptingBackend::InitializePhysicsAPI() {
  lua_State* L = _luaState;
  std::function<luabridge::LuaRef(b2Vec2, b2Vec2, float)> raycast =
      [L](b2Vec2 pos, b2Vec2 dir, float dist) {
        auto ref = luabridge::LuaRef(L);
        dir.Normalize();
        dir *= dist;
        static_raycast_listener->Reset();
        static_physics_world->RayCast(static_raycast_listener, pos, pos + dir);
        auto& hitResults = static_raycast_listener->GetAllRayCast();
        if (!hitResults.empty()) {
          ref = luabridge::LuaRef(L, *hitResults.begin());
        }
        return ref;
      };
  std::function<luabridge::LuaRef(b2Vec2, b2Vec2, float)> raycastAll =
      [L](b2Vec2 pos, b2Vec2 dir, float dist) {
        auto ref = luabridge::newTable(L);
        dir.Normalize();
        dir *= dist;
        static_raycast_listener->Reset();
        static_physics_world->RayCast(static_raycast_listener, pos, pos + dir);
        auto& hitResults = static_raycast_listener->GetAllRayCast();
        if (!hitResults.empty()) {
          int i = 1;
          for (auto& hitResult : hitResults) {
            ref[i++] = luabridge::LuaRef(L, &hitResult);
          }
        }
        return ref;
      };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Physics")
      .addFunction("Raycast", raycast)
      .addFunction("RaycastAll", raycastAll)
      .endNamespace();
}

void LuaScriptingBackend::InitializeEventsAPI() {
  std::function<void(const std::string&, luabridge::LuaRef)> publish =
      [this](const std::string& eventType, luabridge::LuaRef eventObject) {
        if (_subscriptions.find(eventType) == _subscriptions.end()) {
          return;
        }
        auto& callbacks = _subscriptions.at(eventType);
        for (auto& [component, func] : callbacks) {
          try {
            func(component, eventObject);
          } catch (const luabridge::LuaException& e) {
            ReportError(component["actor"].cast<Actor*>()->name, e);
          }
        }
      };

  std::function<void(const std::string&, luabridge::LuaRef, luabridge::LuaRef)>
      subscribe = [this](const std::string& eventType,
                         luabridge::LuaRef component, luabridge::LuaRef func) {
        _newSubscriptions.emplace(eventType, component, func);
      };

  std::function<void(const std::string&, luabridge::LuaRef, luabridge::LuaRef)>
      unsubscribe = [this](const std::string& eventType,
                           luabridge::LuaRef component,
                           luabridge::LuaRef func) {
        _removeSubscriptions.emplace(eventType, component, func);
      };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Event")
      .addFunction("Publish", publish)
      .addFunction("Subscribe", subscribe)
      .addFunction("Unsubscribe", unsubscribe)
      .endNamespace();
}

void LuaScriptingBackend::InitializeNetworkAPI() {
  if (!_network) {
    return;
  }
  std::function<void(const std::vector<uint8_t>&)> networkCallback =
      [this](const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(_networkLock);
        _networkMessageQueue.push(data);
      };
  _network->SetHandlePacketCallback(networkCallback);

  std::function<b2Vec2()> receive = [this]() {
    std::vector<uint8_t> msg;
    b2Vec2 pos{};
    {
      std::lock_guard<std::mutex> lock(_networkLock);
      if (_networkMessageQueue.empty()) {
        return pos;
      }
      msg = _networkMessageQueue.front();
      _networkMessageQueue.pop();
    }
    std::cout << "Got message\n";
    for (auto& v : msg) {
      std::cout << int(v) << ",";
    }
    std::cout << "\n";
    int x = 0;
    int y = 0;
    std::memcpy(&x, msg.data(), sizeof(float));
    std::memcpy(&y, msg.data() + sizeof(float), sizeof(float));
    x = NetworkToHost(x);
    y = NetworkToHost(y);
    std::cout << "received: " << x << "," << y << std::endl;

    pos.y = y;
    pos.x = x;
    return pos;
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Network")
      .addFunction("Receive", receive)
      .endNamespace();
}
