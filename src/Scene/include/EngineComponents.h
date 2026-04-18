#pragma once

#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "GraphicsManager.h"
#include "Helper.h"
#include "box2d/box2d.h"
#include "glm/glm.hpp"

struct Actor;

extern Graphics::GraphicsManager* static_graphics_manager;
extern float* static_camera_zoom;

inline constexpr int PIXELS_PER_METER = 100;

inline float ConvertDegreesToRadians(float angleInDegrees) {
  static constexpr float RADIANS_PER_DEGREE = b2_pi / 180.0f;
  return angleInDegrees * RADIANS_PER_DEGREE;
}

inline float ConvertRadainsToDegrees(float angleInRadians) {
  static constexpr float DEGREES_PER_RADIAN = 180.0f / b2_pi;
  return angleInRadians * DEGREES_PER_RADIAN;
}

inline std::pair<float, float> StaticConvertCordsToScreen(float x, float y) {
  auto [screenWidth, screenHeight] =
      static_graphics_manager->GetScreenDimension();
  return {x * PIXELS_PER_METER +
              (screenWidth * 0.5f) * (1.0f / *static_camera_zoom),
          y * PIXELS_PER_METER +
              (screenHeight * 0.5f) * (1.0f / *static_camera_zoom)};
}

struct Collision {
  Actor* other{nullptr};
  b2Vec2 point{0.0f, 0.0f};
  b2Vec2 relative_velocity{0.0f, 0.0f};
  b2Vec2 normal{0.0f, 0.0f};
};

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

  std::pair<int, int> cachedSpriteDimension{0, 0};

  void OnStart() {
    static_graphics_manager->LoadParticle(ImageName);
    cachedSpriteDimension =
        static_graphics_manager->GetSpriteDimension(ImageName);

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
    if (should_spawn_new_particles &&
        ParticleSystemFrameNumber % std::max(1, FramesBetweenSpawn) == 0) {
      SpawnParticles();
    }

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
        i++;
        continue;
      }

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

      auto [spriteWidth, spriteHeight] = cachedSpriteDimension;
      std::pair<float, float> pivot = {interp_scale * spriteWidth * 0.5f,
                                       interp_scale * spriteHeight * 0.5f};
      auto screenPosition = StaticConvertCordsToScreen(x, y);
      screenPosition.first -= pivot.first;
      screenPosition.second -= pivot.second;
      static_graphics_manager->DrawSprite(Graphics::SpriteInfo{
          ImageName,
          screenPosition,
          {interp_scale, interp_scale},
          pivot,
          static_cast<float>(int(rotation)),
          {static_cast<float>(color_r), static_cast<float>(color_g),
           static_cast<float>(color_b), static_cast<float>(color_a)},
          {0, static_cast<float>(SortingOrder)}});
      i++;
    }
    ParticleSystemFrameNumber++;
  }
};

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
