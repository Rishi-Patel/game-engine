#include "SceneManager.h"

#include <LuaBridge/Optional.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

#include "AudioManager.h"
#include "Defines.h"
#include "GraphicsManager.h"
#include "Helper.h"
#include "InputManager.h"

static const std::filesystem::path SCENE_PATH =
    std::filesystem::path{"resources"} / std::filesystem::path{"scenes"};
static const std::filesystem::path COMPONENT_DIR =
    std::filesystem::path("resources") /
    std::filesystem::path("component_types");
static const std::filesystem::path TEMPLATE_PATH =
    std::filesystem::path{"resources"} /
    std::filesystem::path{"actor_templates"};

namespace luabridge {

namespace detail {

// Add to of FuncTraits.h (within luabridge)
// Place just below the struct "struct Caller<ReturnType, 9>"
// As a result of doing so, luabridge can now support additional parameters in
// lua functions.

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
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
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
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
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
              tvl.tl.tl.tl.tl.tl.tl.tl.tl.hd, tvl.tl.tl.tl.tl.tl.tl.tl.tl.tl.hd,
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
}
}  // namespace luabridge

constexpr auto PIXELS_PER_METER = 100;

RaycastListener* static_raycast_listener = nullptr;
lua_State* static_lua_state = nullptr;
b2World* static_physics_world = nullptr;
Graphics::GraphicsManager* static_graphics_manager = nullptr;
std::pair<float, float>* static_camera_pos = nullptr;
float* static_camera_zoom = nullptr;

static std::pair<float, float> StaticConvertCordsToScreen(float x, float y) {
  auto [screenWidth, screenHeight] =
      static_graphics_manager->GetScreenDimension();
  return {x * PIXELS_PER_METER +
              (screenWidth * 0.5f) * (1.0f / *static_camera_zoom),
          y * PIXELS_PER_METER +
              (screenHeight * 0.5f) * (1.0f / *static_camera_zoom)};
}

static float ConvertDegreesToRadians(float angleInDegrees) {
  static constexpr float RADIANS_PER_DEGREE = b2_pi / 180.0f;
  return angleInDegrees * RADIANS_PER_DEGREE;
}
static float ConvertRadainsToDegrees(float angleInRadians) {
  static constexpr float DEGREES_PER_RADIAN = 180.0f / b2_pi;
  return angleInRadians * DEGREES_PER_RADIAN;
}

static luabridge::LuaRef GetSingleRayCastCollision(b2Vec2 pos, b2Vec2 dir,
                                                   float dist) {
  auto ref = luabridge::LuaRef(static_lua_state);
  dir.Normalize();
  dir *= dist;

  static_raycast_listener->Reset();
  static_physics_world->RayCast(static_raycast_listener, pos, pos + dir);
  auto& hitResults = static_raycast_listener->GetAllRayCast();
  if (hitResults.empty() == false) {
    ref = luabridge::LuaRef(static_lua_state, *hitResults.begin());
  }
  return ref;
}
static luabridge::LuaRef GetAllRayCastCollisions(b2Vec2 pos, b2Vec2 dir,
                                                 float dist) {
  auto ref = luabridge::newTable(static_lua_state);
  dir.Normalize();
  dir *= dist;

  static_raycast_listener->Reset();
  static_physics_world->RayCast(static_raycast_listener, pos, pos + dir);
  auto& hitResults = static_raycast_listener->GetAllRayCast();
  if (hitResults.empty() == false) {
    int i = 1;
    for (auto& hitResult : hitResults) {
      ref[i++] = luabridge::LuaRef(static_lua_state, &hitResult);
    }
  }
  return ref;
}

static void StaticDrawSprite(const std::string& imageName, float x, float y,
                             float rotation, float scaleX, float scaleY,
                             float pivotX, float pivotY, float r, float g,
                             float b, float a, float sortingOrder) {
  auto [spriteWidth, spriteHeight] =
      static_graphics_manager->GetSpriteDimension(imageName);
  std::pair<float, float> pivot = {scaleX * spriteWidth * pivotX,
                                   scaleY * spriteHeight * pivotY};
  // x -= static_camera_pos->first;
  // y -= static_camera_pos->second;
  auto screenPosition = StaticConvertCordsToScreen(x, y);
  screenPosition.first -= pivot.first;
  screenPosition.second -= pivot.second;
  static_graphics_manager->DrawSprite(
      Graphics::SpriteInfo{imageName,
                           screenPosition,
                           {scaleX, scaleY},
                           pivot,
                           static_cast<float>(int(rotation)),
                           {r, g, b, a},
                           {0, sortingOrder}});
};

static void ReportError(const std::string& actorName,
                        const luabridge::LuaException& e) {
  std::string error_message = e.what();
  std::replace(error_message.begin(), error_message.end(), '\\', '/');
  std::cout << "\033[31m" << actorName << " : " << error_message << "\033[0m"
            << std::endl;
}

struct Particle {
  float x{0.0f}, y{0.0f};
  float vx{0.0f}, vy{0.0f};
  float rotationSpeed{0.0f};
  float initialScale{1.0f};
  float rotation{0.0f};
  unsigned int startFrame{0};
  bool isActive{false};
};

struct ParticleSystem {
  std::string key{""};
  std::string type{""};
  bool enabled{true};
  Actor* actor{nullptr};

  std::string ImageName = "";
  float ScaleMin = 1.0f;
  float ScaleMax = 1.0f;

  float RotationMin = 0.0f;
  float RotationMax = 0.0f;

  float MinSpawnRadius = 0.0f;
  float MaxSpawnRadius = 0.5f;

  float MinSpawnAngle = 0.0f;
  float MaxSpawnAngle = 360.0f;

  float SpawnCenterX = 0.0f;
  float SpawnCenterY = 0.0f;

  float SpeedMin{0.0f};
  float SpeedMax{0.0f};

  float RotationSpeedMin{0.0f};
  float RotationSpeedMax{0.0f};

  float ParticleAccelerationX{0.0f};
  float ParticleAccelerationY{0.0f};

  float RotationDragFactor{1.0f};

  float DragFactor{1.0f};

  std::optional<float> EndScale{};

  int start_color_r = 255;
  int start_color_b = 255;
  int start_color_g = 255;
  int start_color_a = 255;

  std::optional<int> end_color_r{};
  std::optional<int> end_color_b{};
  std::optional<int> end_color_g{};
  std::optional<int> end_color_a{};

  int SortingOrder = 9999;

  std::optional<RandomEngine> AngleGenerator;
  std::optional<RandomEngine> RadiusGenerator;
  std::optional<RandomEngine> ScaleGenerator;
  std::optional<RandomEngine> RotationGenerator;
  std::optional<RandomEngine> SpeedGenerator;
  std::optional<RandomEngine> RotationSpeedGenerator;
  std::vector<Particle> Particles;

  std::queue<size_t> FreeList;

  int ParticlesPerSpawn = 1;
  int FramesBetweenSpawn = 1;
  unsigned int ParticleSystemFrameNumber = 0;
  int ParticleLifeTimeInFrames = 300;

  bool should_spawn_new_particles{true};

  void DisableSpawn() { should_spawn_new_particles = false; }
  void EnableSpawn() { should_spawn_new_particles = true; }

  void OnStart() {
    static_graphics_manager->LoadParticle(ImageName);

    AngleGenerator = RandomEngine(MinSpawnAngle, MaxSpawnAngle, 298);
    RadiusGenerator = RandomEngine(MinSpawnRadius, MaxSpawnRadius, 404);
    ScaleGenerator = RandomEngine(ScaleMin, ScaleMax, 494);
    RotationGenerator = RandomEngine(RotationMin, RotationMax, 440);
    SpeedGenerator = RandomEngine(SpeedMin, SpeedMax, 498);
    RotationSpeedGenerator =
        RandomEngine(RotationSpeedMin, RotationSpeedMax, 305);
  }

  void SpawnParticles() {
    for (int i = 0; i < std::max(1, ParticlesPerSpawn); i++) {
      float angle = ConvertDegreesToRadians(AngleGenerator->Sample());
      float cosAngle = glm::cos(angle);
      float sinAngle = glm::sin(angle);
      float radius = RadiusGenerator->Sample();
      float initalScale = ScaleGenerator->Sample();
      float rotation = RotationGenerator->Sample();
      float speed = SpeedGenerator->Sample();
      float rotationSpeed = RotationSpeedGenerator->Sample();
      Particle p{radius * cosAngle + SpawnCenterX,
                 radius * sinAngle + SpawnCenterY,
                 speed * cosAngle,
                 speed * sinAngle,
                 rotationSpeed,
                 initalScale,
                 rotation,
                 ParticleSystemFrameNumber,
                 true};
      if (FreeList.empty()) {
        Particles.emplace_back(std::move(p));
      } else {
        std::swap(p, Particles[FreeList.front()]);
        FreeList.pop();
      }
    }
  }

  void OnUpdate() {
    // Add new particles
    if (should_spawn_new_particles &&
        ParticleSystemFrameNumber % std::max(1, FramesBetweenSpawn) == 0) {
      SpawnParticles();
    }

    // Remove expired particles
    int i = 0;
    for (auto& [x, y, vx, vy, rotationSpeed, initalScale, rotation,
                particleStartFrame, isActive] : Particles) {
      if (!isActive) {
        i++;
        continue;
      }

      int framesParticleHasBeenAlive =
          ParticleSystemFrameNumber - particleStartFrame;
      if (framesParticleHasBeenAlive >= std::max(1, ParticleLifeTimeInFrames)) {
        isActive = false;
        FreeList.push(i);
      }
      i++;
    }

    for (auto& [x, y, vx, vy, rotationSpeed, initalScale, rotation,
                particleStartFrame, isActive] : Particles) {
      if (!isActive) {
        continue;
      }
      int framesParticleHasBeenAlive =
          ParticleSystemFrameNumber - particleStartFrame;

      vx += ParticleAccelerationX;
      vy += ParticleAccelerationY;
      vx *= DragFactor;
      vy *= DragFactor;
      rotationSpeed *= RotationDragFactor;
      x += vx;
      y += vy;
      rotation += rotationSpeed;
      float lifetime_percentage =
          static_cast<float>(framesParticleHasBeenAlive) /
          ParticleLifeTimeInFrames;
      float interp_scale =
          EndScale.has_value()
              ? glm::mix(initalScale, EndScale.value(), lifetime_percentage)
              : initalScale;
      int color_r =
          end_color_r.has_value()
              ? glm::mix(start_color_r, end_color_r.value_or(start_color_r),
                         lifetime_percentage)
              : start_color_r;
      int color_g =
          end_color_g.has_value()
              ? glm::mix(start_color_g, end_color_g.value_or(start_color_g),
                         lifetime_percentage)
              : start_color_g;
      int color_b =
          end_color_b.has_value()
              ? glm::mix(start_color_b, end_color_b.value_or(start_color_b),
                         lifetime_percentage)
              : start_color_b;
      int color_a =
          end_color_a.has_value()
              ? glm::mix(start_color_a, end_color_a.value_or(start_color_a),
                         lifetime_percentage)
              : start_color_a;

      StaticDrawSprite(ImageName, x, y, rotation, interp_scale, interp_scale,
                       0.5f, 0.5f, color_r, color_g, color_b, color_a,
                       SortingOrder);
    }
    ParticleSystemFrameNumber++;
  }
};

struct Collision {
  Actor* other{nullptr};
  b2Vec2 point{0.0f, 0.0f};
  b2Vec2 relative_velocity{0.0f, 0.0f};
  b2Vec2 normal{0.0f, 0.0f};
};

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
  std::string functionName = "OnCollisionEnter";
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
    auto component = actorA->components.at(componentName);
    if (component["enabled"].cast<bool>() == false) {
      continue;
    }
    try {
      component[functionName](component, collision);
    } catch (const luabridge::LuaException& e) {
      ReportError(actorA->name, e);
    }
  }

  collision.other = actorA;
  for (const auto& componentName : actorBComponentsToUpdate) {
    auto component = actorB->components.at(componentName);
    if (component["enabled"].cast<bool>() == false) {
      continue;
    }
    try {
      component[functionName](component, collision);
    } catch (const luabridge::LuaException& e) {
      ReportError(actorB->name, e);
    }
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
  std::string functionName = "OnCollisionExit";
  bool isTrigger =
      (contact->GetFixtureA()->GetFilterData().categoryBits == 0x0002);
  if (isTrigger) {
    actorAComponentsToUpdate = actorA->onTriggerEndComponents;
    actorBComponentsToUpdate = actorB->onTriggerEndComponents;
    functionName = "OnTriggerExit";
  }

  for (const auto& componentName : actorAComponentsToUpdate) {
    auto component = actorA->components.at(componentName);
    if (component["enabled"].cast<bool>() == false) {
      continue;
    }
    try {
      component[functionName](component, collision);
    } catch (const luabridge::LuaException& e) {
      ReportError(actorA->name, e);
    }
  }

  collision.other = actorA;
  for (const auto& componentName : actorBComponentsToUpdate) {
    auto component = actorB->components.at(componentName);
    if (component["enabled"].cast<bool>() == false) {
      continue;
    }
    try {
      component[functionName](component, collision);
    } catch (const luabridge::LuaException& e) {
      ReportError(actorB->name, e);
    }
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

struct RigidBody {
  std::string key{""};
  std::string type{""};
  bool enabled{true};
  Actor* actor{nullptr};

  float x{0.0f};
  float y{0.0f};
  std::string body_type{"dynamic"};
  bool precise{true};
  float gravity_scale{1.0f};
  float density{1.0f};
  float angular_friction{0.3f};
  float rotation{0.0f};
  bool has_collider{true};
  bool has_trigger{true};

  std::string collider_type{"box"};
  float width{1.0f};
  float height{1.0f};
  float radius{0.5f};
  float friction{0.3f};
  float bounciness{0.3f};

  std::string trigger_type{"box"};
  float trigger_width{1.0f};
  float trigger_height{1.0f};
  float trigger_radius{1.0f};

  b2Body* body{nullptr};

  void OnStart(b2World& world) {
    b2BodyDef def;
    if (body_type == "dynamic") {
      def.type = b2_dynamicBody;
    } else if (body_type == "kinematic") {
      def.type = b2_kinematicBody;
    } else {
      def.type = b2_staticBody;
    }
    def.position = {x, y};
    def.bullet = precise;
    def.gravityScale = gravity_scale;
    def.angularDamping = angular_friction;
    def.angle = ConvertDegreesToRadians(rotation);
    def.awake = true;
    body = world.CreateBody(&def);

    if (!has_trigger && !has_collider) {
      b2FixtureDef fixture;
      b2PolygonShape my_shape;
      fixture.filter.maskBits = 0x0000;
      my_shape.SetAsBox(0.5f * width, 0.5f * height);
      fixture.density = density;
      fixture.shape = &my_shape;
      fixture.isSensor = true;
      body->CreateFixture(&fixture);
    }

    if (has_collider) {
      b2FixtureDef fixture;
      fixture.filter.maskBits = 0x0001;
      fixture.filter.categoryBits = 0x0001;
      fixture.density = density;
      fixture.isSensor = false;
      fixture.friction = friction;
      fixture.restitution = bounciness;
      fixture.userData.pointer = reinterpret_cast<uintptr_t>(actor);
      if (collider_type == "box") {
        b2PolygonShape my_shape;
        my_shape.SetAsBox(0.5f * width, 0.5f * height);
        fixture.shape = &my_shape;
        body->CreateFixture(&fixture);
      } else {
        b2CircleShape my_shape;
        my_shape.m_radius = radius;
        fixture.shape = &my_shape;
        body->CreateFixture(&fixture);
      }
    }

    if (has_trigger) {
      b2FixtureDef fixture;
      fixture.filter.maskBits = 0x0002;
      fixture.filter.categoryBits = 0x0002;
      fixture.density = density;
      fixture.isSensor = true;
      fixture.friction = friction;
      fixture.restitution = bounciness;
      fixture.userData.pointer = reinterpret_cast<uintptr_t>(actor);
      if (trigger_type == "box") {
        b2PolygonShape my_shape;
        my_shape.SetAsBox(0.5f * trigger_width, 0.5f * trigger_height);
        fixture.shape = &my_shape;
        body->CreateFixture(&fixture);
      } else {
        b2CircleShape my_shape;
        my_shape.m_radius = trigger_radius;
        fixture.shape = &my_shape;
        body->CreateFixture(&fixture);
      }
    }
  }
};

luabridge::LuaRef SceneManager::CreateActor(const std::string& actorTemplate) {
  auto* actor = &actors[_actorCounter];
  actor->id = _actorCounter++;
  auto* templateActor = GetTemplateActor(actorTemplate);
  LoadActorFromTemplate(*actor, *templateActor);
  _actorsByName[actor->name].emplace(actor->id);
  for (auto& [componentName, component] : actor->components) {
    newComponents.emplace(actor->id, component);
  }
  actorChanges.emplace(actor->id, true);

  return luabridge::LuaRef(_luaState, actor);
}

luabridge::LuaRef SceneManager::GetComponent(Actor* actor,
                                             const std::string& templateType) {
  auto component = luabridge::LuaRef(_luaState);
  if (actor->_componentKeysByTemplateType.find(templateType) ==
      actor->_componentKeysByTemplateType.end()) {
    return component;
  }
  for (const auto& componentName :
       actor->_componentKeysByTemplateType.at(templateType)) {
    if (actor->pendingDeletion.count(componentName) == 0) {
      component = actor->components.at(componentName);
      break;
    }
  }
  return component;
};

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

static void AddComponentProperty(luabridge::LuaRef& component,
                                 const std::string& propertyName,
                                 const rapidjson::Value& value) {
  switch (value.GetType()) {
    case rapidjson::Type::kFalseType:
    case rapidjson::Type::kTrueType:
      component[propertyName] = value.GetBool();
      break;
    case rapidjson::Type::kNumberType: {
      if (value.IsInt()) {
        component[propertyName] = value.GetInt();
      } else {
        component[propertyName] = value.GetFloat();
      }
    } break;
    case rapidjson::Type::kStringType:
      component[propertyName] = value.GetString();
      break;
      // Unimplemented
    case rapidjson::Type::kNullType:
    case rapidjson::Type::kArrayType:
    case rapidjson::Type::kObjectType:
      std::cout << "Unimplemented component property type: " << value.GetType()
                << std::endl;
      exit(0);
  }
}

void SceneManager::AttachComponentToActor(Actor& actor,
                                          luabridge::LuaRef& component) {
  std::string key = component["key"].cast<std::string>();
  auto [itr, _] = actor.components.emplace(key, component);
  actor._componentKeysByTemplateType[component["type"]].emplace(key);
  component["actor"] = &actor;
  component["enabled"] = true;

  auto onStart = component["OnStart"];
  if (onStart.isFunction()) {
    actor.onStartComponents.emplace(key);
  }
  auto onUpdate = component["OnUpdate"];
  if (onUpdate.isFunction()) {
    actor.onUpdateComponents.emplace(key, itr);
  }
  auto onLateUpdate = component["OnLateUpdate"];
  if (onLateUpdate.isFunction()) {
    actor.onLateUpdateComponents.emplace(key);
  }
  auto onCollisionBegin = component["OnCollisionEnter"];
  if (onCollisionBegin.isFunction()) {
    actor.onContactBeginComponents.emplace(key);
  }
  auto onCollisionEnd = component["OnCollisionExit"];
  if (onCollisionEnd.isFunction()) {
    actor.onContactEndComponents.emplace(key);
  }
  auto onTriggerBegin = component["OnTriggerEnter"];
  if (onTriggerBegin.isFunction()) {
    actor.onTriggerBeginComponents.emplace(key);
  }
  auto onTriggerEnd = component["OnTriggerExit"];
  if (onTriggerEnd.isFunction()) {
    actor.onTriggerEndComponents.emplace(key);
  }
  auto onDestroy = component["OnDestroy"];
  if (onDestroy.isFunction()) {
    actor.onDestroyComponents.emplace(key);
  }
}

void SceneManager::DetachComponentFromActor(Actor& actor,
                                            const std::string& componentName) {
  auto& component = actor.components.at(componentName);
  std::string componentType = component["type"];
  actor.pendingDeletion.erase(componentName);
  actor.onStartComponents.erase(componentName);
  actor.onLateUpdateComponents.erase(componentName);
  actor.onUpdateComponents.erase(componentName);
  actor.onContactBeginComponents.erase(componentName);
  actor.onContactEndComponents.erase(componentName);
  actor.onTriggerBeginComponents.erase(componentName);
  actor.onTriggerEndComponents.erase(componentName);
  if (component["OnDestroy"].isFunction()) {
    try {
      component["OnDestroy"](component);
    } catch (const luabridge::LuaException& e) {
      ReportError(actor.name, e);
    }
  }
  if (componentType == "Rigidbody") {
    _physicsWorld->DestroyBody(component.cast<RigidBody*>()->body);
    delete component.cast<RigidBody*>();
  } else if (componentType == "ParticleSystem") {
    delete component.cast<ParticleSystem*>();
  }
  actor.onDestroyComponents.erase(componentName);
  actor.components.erase(componentName);
  actor._componentKeysByTemplateType[componentType].erase(componentName);
  if (actor._componentKeysByTemplateType[componentType].empty()) {
    actor._componentKeysByTemplateType.erase(componentType);
  }
}

void SceneManager::Log(const std::string& message) {
  std::cout << message << std::endl;
}

luabridge::LuaRef SceneManager::CreateComponentInstance(
    const std::string& componentInstanceName,
    const luabridge::LuaRef& templateComponent) {
  luabridge::LuaRef componentInstance = luabridge::newTable(_luaState);
  EstablishInheritance(componentInstance, templateComponent);
  componentInstance["key"] = componentInstanceName;
  return componentInstance;
}

void SceneManager::EstablishInheritance(
    luabridge::LuaRef& instance, const luabridge::LuaRef& template_component) {
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

luabridge::LuaRef SceneManager::RunFile(const std::filesystem::path& filename) {
  if (luaL_dofile(_luaState, filename.string().c_str()) != LUA_OK) {
    std::cout << "problem with lua file " << filename.stem().string().c_str();
    exit(0);
  }
  return luabridge::getGlobal(_luaState, filename.stem().string().c_str());
}

void SceneManager::InitializePhysics() {
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
        auto templateComponent =
            LoadComponentTemplate(componentItr->value["type"].GetString());
        luabridge::LuaRef componentInstance = CreateComponentInstance(
            componentItr->name.GetString(), templateComponent);
        AttachComponentToActor(actor, componentInstance);
      }
      for (auto propertyItr = componentItr->value.MemberBegin();
           propertyItr != componentItr->value.MemberEnd(); propertyItr++) {
        AddComponentProperty(
            actor.components.at(componentItr->name.GetString()),
            propertyItr->name.GetString(), propertyItr->value);
      }
    }
  }
}

luabridge::LuaRef SceneManager::LoadComponentTemplate(
    const std::string& componentName) {
  if (_componentTemplates.find(componentName) != _componentTemplates.end()) {
    return _componentTemplates.at(componentName);
  }

  if (componentName == "Rigidbody") {
    InitializePhysics();
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

  return _componentTemplates.at(componentName);
}

void SceneManager::LoadActorFromTemplate(Actor& actor, Actor& templateActor) {
  actor.name = templateActor.name;
  for (auto& [componentName, templateActorComponent] :
       templateActor.components) {
    auto actorComponent =
        CreateComponentInstance(componentName, templateActorComponent);
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
                           Input::InputManager* inputManager)
    : _currentScene(""),
      _nextScene("basic"),
      _graphicsManager(graphicsManager),
      _audioManager(audioManager),
      _inputManager(inputManager),
      _componentCounter(0),
      _actorCounter(0),
      _cameraPos(0.0f, 0.0f),
      _cameraZoom(1.0f),
      _frameNumber(0) {
  InitializeAPI();
  static_raycast_listener = &_raycastListener;
  static_lua_state = _luaState;
  static_graphics_manager = _graphicsManager;
  static_camera_pos = &_cameraPos;
  static_camera_zoom = &_cameraZoom;
}

bool SceneManager::IsOnScreen(Actor& actor, const glm::ivec4& screenCoords) {
  return true;
}

void SceneManager::RunOnUpdate() {
  for (auto& [_, actor] : onUpdateActors) {
    for (auto& [_, itr] : actor->onUpdateComponents) {
      auto component = itr->second;
      if (component["enabled"].cast<bool>() == false) {
        continue;
      }
      try {
        component["OnUpdate"](component);
      } catch (const luabridge::LuaException& e) {
        ReportError(actor->name, e);
      }
    }
  }
}

void SceneManager::HandleActorChanges() {
  while (actorChanges.empty() == false) {
    auto [actorId, isNewActor] = actorChanges.front();
    actorChanges.pop();
    if (isNewActor) {
      if (actors[actorId].onUpdateComponents.empty() == false) {
        onUpdateActors.emplace(actorId, &actors[actorId]);
      }
      if (actors[actorId].onLateUpdateComponents.empty() == false) {
        onLateUpdateActors.insert(actorId);
      }
    } else {
      if (actors.find(actorId) == actors.end()) {
        continue;
      }
      for (auto& componentName : actors[actorId].onDestroyComponents) {
        auto component = actors[actorId].components.at(componentName);
        try {
          component["OnDestroy"](component);
        } catch (const luabridge::LuaException& e) {
          ReportError(actors[actorId].name, e);
        }
      }
      if (actors[actorId]._componentKeysByTemplateType.find("Rigidbody") !=
          actors[actorId]._componentKeysByTemplateType.end()) {
        for (auto& componentName :
             actors[actorId]._componentKeysByTemplateType.at("Rigidbody")) {
          auto component = actors[actorId].components.at(componentName);
          _physicsWorld->DestroyBody(component.cast<RigidBody*>()->body);
          delete component.cast<RigidBody*>();
        }
      }
      if (actors[actorId]._componentKeysByTemplateType.find("ParticleSystem") !=
          actors[actorId]._componentKeysByTemplateType.end()) {
        for (auto& componentName :
             actors[actorId]._componentKeysByTemplateType.at(
                 "ParticleSystem")) {
          auto component = actors[actorId].components.at(componentName);
          delete component.cast<ParticleSystem*>();
        }
      }

      onUpdateActors.erase(actorId);
      onLateUpdateActors.erase(actorId);
      _actorsByName[actors[actorId].name].erase(actorId);
      actors.erase(actorId);
      deactivedActors.erase(actorId);
    }
  }
}

void SceneManager::RunOnLateUpdate() {
  for (auto actorId : onLateUpdateActors) {
    auto& actor = actors[actorId];
    for (auto& componentName : actor.onLateUpdateComponents) {
      auto component = actor.components.at(componentName);
      if (component["enabled"].cast<bool>() == false) {
        continue;
      }
      try {
        component["OnLateUpdate"](component);
      } catch (const luabridge::LuaException& e) {
        ReportError(actor.name, e);
      }
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
    if (component["actor"].isNil()) {
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
    if (component["enabled"].cast<bool>() == false ||
        actor.onStartComponents.count(component["key"]) == 0) {
      continue;
    }
    try {
      component["OnStart"](component);
    } catch (const luabridge::LuaException& e) {
      ReportError(actor.name, e);
    }
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

  while (newSubscriptions.empty() == false) {
    auto [eventType, component, func] = newSubscriptions.front();
    newSubscriptions.pop();
    subscriptions[eventType].emplace_back(component, func);
  }
  while (removeSubscriptions.empty() == false) {
    auto [eventType, component, func] = removeSubscriptions.front();
    removeSubscriptions.pop();
    auto& callbacks = subscriptions.at(eventType);
    int i = 0;
    for (auto& [_component, _func] : callbacks) {
      //if (_component == component && _func == func) {
      //  callbacks.erase(callbacks.begin() + i);
      //  break;
      //}
      i++;
    }
    if (callbacks.empty()) {
      subscriptions.erase(eventType);
    }
  }

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
        for (auto& componentName : itr->second.onDestroyComponents) {
          auto component = itr->second.components.at(componentName);
          try {
            component["OnDestroy"](component);
          } catch (const luabridge::LuaException& e) {
            ReportError(itr->second.name, e);
          }
        }
        if (itr->second._componentKeysByTemplateType.find("Rigidbody") !=
            itr->second._componentKeysByTemplateType.end()) {
          for (auto& componentName :
               itr->second._componentKeysByTemplateType.at("Rigidbody")) {
            auto component = itr->second.components.at(componentName);
            _physicsWorld->DestroyBody(component.cast<RigidBody*>()->body);
            delete component.cast<RigidBody*>();
          }
        }
        if (itr->second._componentKeysByTemplateType.find("ParticleSystem") !=
            itr->second._componentKeysByTemplateType.end()) {
          for (auto& componentName :
               itr->second._componentKeysByTemplateType.at("ParticleSystem")) {
            auto component = itr->second.components.at(componentName);
            delete component.cast<ParticleSystem*>();
          }
        }

        _actorsByName[itr->second.name].erase(itr->second.id);
        onUpdateActors.erase(itr->second.id);
        onLateUpdateActors.erase(itr->second.id);
        deactivedActors.erase(itr->second.id);
        if (_actorsByName[itr->second.name].empty()) {
          _actorsByName.erase(itr->second.name);
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

void SceneManager::InitializeAPI() {
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
}
void SceneManager::InitializeClassAPI() {
  std::function<void(RigidBody*)> rigidBodyStart = [&](RigidBody* rb) {
    rb->OnStart(*_physicsWorld.get());
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
            rb->body->SetTransform(
                rb->body->GetPosition(),
                -glm::atan(-direction.y, direction.x) + (b2_pi / 2.0f));
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
void SceneManager::InitializeDebugAPI() {
  std::function<void(const std::string&)> log =
      [&](const std::string& message) { Log(message); };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Debug")
      .addFunction("Log", log)
      .endNamespace();
}
void SceneManager::InitializeActorClassAPI() {
  std::function<luabridge::LuaRef(Actor*, const std::string&)>
      getComponentByKey = [&](Actor* actor, const std::string& key) {
        if (actor->components.find(key) == actor->components.end() ||
            actor->pendingDeletion.count(key) > 0) {
          return luabridge::LuaRef(_luaState);
        }
        return actor->components.at(key);
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> getComponent =
      [&](Actor* actor, const std::string& templateType) {
        auto component = luabridge::LuaRef(_luaState);
        if (actor->_componentKeysByTemplateType.find(templateType) ==
            actor->_componentKeysByTemplateType.end()) {
          return component;
        }
        for (const auto& componentName :
             actor->_componentKeysByTemplateType.at(templateType)) {
          if (actor->pendingDeletion.count(componentName) == 0) {
            component = actor->components.at(componentName);
            break;
          }
        }
        return component;
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> getComponents =
      [&](Actor* actor, const std::string& templateType) {
        auto ref = luabridge::newTable(_luaState);
        if (actor->_componentKeysByTemplateType.find(templateType) ==
            actor->_componentKeysByTemplateType.end()) {
          return ref;
        }
        int i = 1;
        for (const auto& componentName :
             actor->_componentKeysByTemplateType.at(templateType)) {
          if (actor->pendingDeletion.count(componentName) == 0) {
            ref[i++] = actor->components.at(componentName);
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(Actor*, const std::string&)> createComponent =
      [&](Actor* actor, const std::string& templateType) {
        auto templateComponent = LoadComponentTemplate(templateType);
        std::string componentName = "r" + std::to_string(_componentCounter++);
        auto component =
            CreateComponentInstance(componentName, templateComponent);
        newComponents.emplace(actor->id, component);
        return component;
      };
  std::function<void(Actor*, luabridge::LuaRef)> removeComponent =
      [&](Actor* actor, luabridge::LuaRef component) {
        component["enabled"] = false;
        actor->pendingDeletion.emplace(component["key"].cast<std::string>());
        removedComponents.emplace(actor->id, component["key"]);
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
void SceneManager::InitializeActorAPI() {
  std::function<luabridge::LuaRef(const std::string&)> FindActor =
      [&](const std::string& actorName) {
        auto ref = luabridge::LuaRef(_luaState);
        if (_actorsByName.find(actorName) == _actorsByName.end() ||
            _actorsByName.at(actorName).empty()) {
          return ref;
        }
        for (auto& actorId : _actorsByName.at(actorName)) {
          if (deactivedActors.count(actorId) == 0) {
            ref = luabridge::LuaRef(_luaState, &actors[actorId]);
            break;
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(const std::string&)> FindAllActor =
      [&](const std::string& actorName) {
        auto ref = luabridge::newTable(_luaState);
        if (_actorsByName.find(actorName) == _actorsByName.end()) {
          return ref;
        }
        int i = 1;
        for (const int actorId : _actorsByName.at(actorName)) {
          if (deactivedActors.count(actorId) == 0) {
            ref[i++] = luabridge::LuaRef(_luaState, &actors[actorId]);
          }
        }
        return ref;
      };
  std::function<luabridge::LuaRef(const std::string&)> CreateActor =
      [&](const std::string& actorTemplate) {
        auto* actor = &actors[_actorCounter];
        actor->id = _actorCounter++;
        auto* templateActor = GetTemplateActor(actorTemplate);
        LoadActorFromTemplate(*actor, *templateActor);
        _actorsByName[actor->name].emplace(actor->id);
        for (auto& [componentName, component] : actor->components) {
          newComponents.emplace(actor->id, component);
        }
        actorChanges.emplace(actor->id, true);

        return luabridge::LuaRef(_luaState, actor);
      };
  std::function<void(Actor&)> RemoveActor = [&](Actor& actor) {
    for (auto& [componentName, component] : actor.components) {
      component["enabled"] = false;
    }
    actorChanges.emplace(actor.id, false);
    deactivedActors.emplace(actor.id);
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Actor")
      .addFunction("Find", FindActor)
      .addFunction("FindAll", FindAllActor)
      .addFunction("Instantiate", CreateActor)
      .addFunction("Destroy", RemoveActor)
      .endNamespace();
}
void SceneManager::InitializeApplicationAPI() {
  std::function<void()> quit = [&]() { exit(0); };
  std::function<unsigned int()> getFrame = [&]() { return _frameNumber; };
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
void SceneManager::InitializeInputAPI() {
  std::function<bool(const std::string&)> getKey =
      [&](const std::string& keycode) {
        return _inputManager->GetKey(keycode);
      };
  std::function<bool(const std::string&)> getKeyDown =
      [&](const std::string& keycode) {
        return _inputManager->GetKeyDown(keycode);
      };
  std::function<bool(const std::string&)> getKeyUp =
      [&](const std::string& keycode) {
        return _inputManager->GetKeyUp(keycode);
      };
  std::function<bool(int)> getmousebutton = [&](int buttonNum) {
    return _inputManager->GetMouseButton(buttonNum);
  };
  std::function<bool(int)> getmousebuttonDown = [&](int buttonNum) {
    return _inputManager->GetMouseButtonDown(buttonNum);
  };
  std::function<bool(int)> getmousebuttonUp = [&](int buttonNum) {
    return _inputManager->GetMouseButtonUp(buttonNum);
  };
  std::function<glm::vec2()> getmousePosition = [&]() {
    return _inputManager->GetMousePosition();
  };
  std::function<float()> getmouseDelta = [&]() {
    return _inputManager->GetMouseScrollDelta();
  };
  std::function<void()> hideCursor = [&]() { _inputManager->HideCursor(); };
  std::function<void()> showCursor = [&]() { _inputManager->ShowCursor(); };

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
void SceneManager::InitializeTextAPI() {
  std::function<void(const std::string&, int, int, const std::string&,
                     unsigned int, int, int, int, int)>
      draw = [&](const std::string& text, int x, int y,
                 const std::string& fontName, unsigned int fontSize, int r,
                 int g, int b, int a) {
        _graphicsManager->DrawText(fontName, fontSize, text, x, y,
                                   Graphics::RGBA{r, g, b, a});
      };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Text")
      .addFunction("Draw", draw)
      .endNamespace();
}
void SceneManager::InitializeAudioAPI() {
  std::function<void(int, const std::string&, bool)> play =
      [&](int channel, const std::string& audioName, bool loop) {
        _audioManager->PlayAudio(audioName, channel, (loop) ? -1 : 0);
      };
  std::function<void(int)> halt = [&](int channel) {
    _audioManager->StopAudio(channel);
  };
  std::function<void(int, int)> setVolume = [&](int channel, int volume) {
    _audioManager->SetVolume(channel, volume);
  };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Audio")
      .addFunction("Play", play)
      .addFunction("Halt", halt)
      .addFunction("SetVolume", setVolume)
      .endNamespace();
}
void SceneManager::InitializeDrawAPI() {
  std::function<void(const std::string&, float, float)> drawUI =
      [&](const std::string& imageName, float x, float y) {
        _graphicsManager->DrawSprite(Graphics::SpriteInfo{imageName,
                                                          {int(x), int(y)},
                                                          {1.0f, 1.0f},
                                                          {0, 0},
                                                          0.0f,
                                                          {255, 255, 255, 255},
                                                          {1, 0}});
      };
  std::function<void(const std::string&, float, float, float, float, float,
                     float, float)>
      drawUIEx = [&](const std::string& imageName, float x, float y, float r,
                     float g, float b, float a, float sortingOrder) {
        _graphicsManager->DrawSprite(Graphics::SpriteInfo{imageName,
                                                          {int(x), int(y)},
                                                          {1.0f, 1.0f},
                                                          {0, 0},
                                                          0.0f,
                                                          {r, g, b, a},
                                                          {1, sortingOrder}});
      };
  std::function<void(const std::string&, int, int)> drawSprite =
      [&](const std::string& imageName, float x, float y) {
        auto [spriteWidth, spriteHeight] =
            _graphicsManager->GetSpriteDimension(imageName);
        std::pair<float, float> pivot = {spriteWidth / 2, spriteHeight / 2};
        // x -= _cameraPos.first;
        // y -= _cameraPos.second;
        auto screenPosition = ConvertCordsToScreen(x, y);
        screenPosition.first -= pivot.first;
        screenPosition.second -= pivot.second;
        _graphicsManager->DrawSprite(Graphics::SpriteInfo{imageName,
                                                          screenPosition,
                                                          {1.0f, 1.0f},
                                                          pivot,
                                                          0.0f,
                                                          {255, 255, 255, 255},
                                                          {0, 0}});
      };
  std::function<void(const std::string&, float, float, float, float, float,
                     float, float, float, float, float, float, float)>
      drawSpriteEx = [&](const std::string& imageName, float x, float y,
                         float rotation, float scaleX, float scaleY,
                         float pivotX, float pivotY, float r, float g, float b,
                         float a, float sortingOrder) {
        auto [spriteWidth, spriteHeight] =
            _graphicsManager->GetSpriteDimension(imageName);
        std::pair<float, float> pivot = {scaleX * spriteWidth * pivotX,
                                         scaleY * spriteHeight * pivotY};
        // x -= _cameraPos.first;
        // y -= _cameraPos.second;
        auto screenPosition = ConvertCordsToScreen(x, y);
        screenPosition.first -= pivot.first;
        screenPosition.second -= pivot.second;
        _graphicsManager->DrawSprite(
            Graphics::SpriteInfo{imageName,
                                 screenPosition,
                                 {scaleX, scaleY},
                                 pivot,
                                 static_cast<float>(int(rotation)),
                                 {r, g, b, a},
                                 {0, sortingOrder}});
      };
  std::function<void(float, float, float, float, float, float)> drawPixel =
      [&](float x, float y, float r, float g, float b, float a) {
        _graphicsManager->DrawPoint(x, y, {r, g, b, a});
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

void SceneManager::InitializeCameraAPI() {
  std::function<void(float, float)> setPos = [&](float x, float y) {
    _cameraPos = {x, y};
  };
  std::function<float()> getPosX = [&]() { return _cameraPos.first; };
  std::function<float()> getPosY = [&]() { return _cameraPos.second; };
  std::function<void(float)> setZoom = [&](float zoomFactor) {
    _cameraZoom = zoomFactor;
    _graphicsManager->SetRenderScale(_cameraZoom);
  };
  std::function<float()> getZoom = [&]() { return _cameraZoom; };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Camera")
      .addFunction("SetPosition", setPos)
      .addFunction("GetPositionX", getPosX)
      .addFunction("GetPositionY", getPosY)
      .addFunction("SetZoom", setZoom)
      .addFunction("GetZoom", getZoom)
      .endNamespace();
}

void SceneManager::InitializeSceneAPI() {
  std::function<void(const std::string&)> load =
      [&](const std::string& sceneName) { _nextScene = sceneName; };
  std::function<std::string()> getCurrent = [&]() { return _currentScene; };
  std::function<void(Actor*)> dontDestroy = [&](Actor* actor) {
    actor->dontDestroy = true;
  };
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Scene")
      .addFunction("Load", load)
      .addFunction("GetCurrent", getCurrent)
      .addFunction("DontDestroy", dontDestroy)
      .endNamespace();
}

void SceneManager::InitializePhysicsAPI() {
  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Physics")
      .addFunction("Raycast", &GetSingleRayCastCollision)
      .addFunction("RaycastAll", &GetAllRayCastCollisions)
      .endNamespace();
}

void SceneManager::InitializeEventsAPI() {
  std::function<void(const std::string&, luabridge::LuaRef)> publish =
      [&](const std::string& eventType, luabridge::LuaRef eventObject) {
        if (subscriptions.find(eventType) == subscriptions.end()) {
          return;
        }
        auto& callbacks = subscriptions.at(eventType);
        for (auto& [component, func] : callbacks) {
          try {
            func(component, eventObject);
          } catch (const luabridge::LuaException& e) {
            ReportError(component["actor"].cast<Actor*>()->name, e);
          }
        }
      };

  std::function<void(const std::string&, luabridge::LuaRef, luabridge::LuaRef)>
      subscribe = [&](const std::string& eventType, luabridge::LuaRef component,
                      luabridge::LuaRef func) {
        newSubscriptions.emplace(eventType, component, func);
      };

  std::function<void(const std::string&, luabridge::LuaRef, luabridge::LuaRef)>
      unsubscribe = [&](const std::string& eventType,
                        luabridge::LuaRef component, luabridge::LuaRef func) {
        removeSubscriptions.emplace(eventType, component, func);
      };

  luabridge::getGlobalNamespace(_luaState)
      .beginNamespace("Event")
      .addFunction("Publish", publish)
      .addFunction("Subscribe", subscribe)
      .addFunction("Unsubscribe", unsubscribe)
      .endNamespace();
}