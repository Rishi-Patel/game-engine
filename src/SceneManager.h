#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <unordered_map>
#include <queue>
#include <vector>
#include <set>
#include <unordered_set>

#include "glm/glm.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/hash.hpp"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "lua.hpp"
#include "LuaBridge/LuaBridge.h"

#include "box2d/box2d.h"


namespace Graphics
{
    class GraphicsManager;
}

namespace Input {
    class InputManager;
}

class AudioManager;

enum class GameMode;
class AudioManager;

class ContactListener : public b2ContactListener {
    void BeginContact(b2Contact* contact) final;
    void EndContact(b2Contact* contact) final;
};

struct Actor
{
    size_t id = 0;
    std::string name = "";
    bool dontDestroy = false;
    std::map<std::string, luabridge::LuaRef> components;
    std::unordered_map<std::string, std::set<std::string>> _componentKeysByTemplateType;
    std::map<std::string, std::map<std::string, luabridge::LuaRef>::iterator> onUpdateComponents;
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
    float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) final;
    const std::vector<HitResult>& GetAllRayCast() {
        std::sort(hitResults.begin(), hitResults.end());
        return hitResults;
    }
    void Reset() {
        hitResults.clear();
    }
private:
    std::vector<HitResult> hitResults;
};

class SceneManager
{
public:
    SceneManager(Graphics::GraphicsManager* graphicsManager, AudioManager* audioManager, Input::InputManager* inputManager);

    void UpdateSceneActors();
    void SetScene(const std::string& sceneName);

    luabridge::LuaRef CreateActor(const std::string& actorTemplate);
    luabridge::LuaRef GetComponent(Actor* actor, const std::string& templateType);
private:
    Graphics::GraphicsManager* _graphicsManager;
    AudioManager* _audioManager;
    Input::InputManager* _inputManager;

    std::unordered_map<int, Actor> actors;

    std::unordered_map<std::string, luabridge::LuaRef> _componentTemplates;
    std::unordered_map<std::string, Actor> templateActors;
    std::unordered_map<std::string, std::set<int>> _actorsByName;
    std::queue<std::pair<int, luabridge::LuaRef>> newComponents;
    std::queue<std::pair<int, luabridge::LuaRef>> newComponentsBuffer; // used this to process newComponents added the last frame (swap with newComponents)
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

    std::unordered_map<std::string, std::vector<std::pair<luabridge::LuaRef, luabridge::LuaRef>>> subscriptions;
    std::queue<std::tuple<std::string, luabridge::LuaRef, luabridge::LuaRef>> newSubscriptions;
    std::queue<std::tuple<std::string, luabridge::LuaRef, luabridge::LuaRef>> removeSubscriptions;

    std::pair<float, float> _cameraPos;
    float _cameraZoom;

    std::optional<std::string> _nextScene;
    std::string _currentScene;
    unsigned int _frameNumber;

    bool IsOnScreen(Actor& actor, const glm::ivec4& screenCoords);

    luabridge::LuaRef LoadComponentTemplate(const std::string& componentPath);
    void LoadActorFromConfig(Actor& actor, const rapidjson::Value& actorConfig);
    void LoadTemplateFromConfig(Actor& actor, const rapidjson::Document& templateConfig);
    Actor* GetTemplateActor(const std::string& templateActorName);
    void LoadActorFromTemplate(Actor& actor, Actor& templateActor);
    void SetParameter(Actor& actor, const std::string& configName, const rapidjson::Value& value);
    std::pair<float, float> ConvertCordsToScreen(float x, float y);

    void RunOnUpdate();
    void HandleActorChanges();
    void RunOnLateUpdate();
    void AddComponents();
    void RemoveComponents();


    void InitializeAPI();
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

    void InitializePhysics();

    lua_State* _luaState;
    luabridge::LuaRef RunFile(const std::filesystem::path& filename);
    void EstablishInheritance(luabridge::LuaRef& instance, const luabridge::LuaRef& template_component);
    void Log(const std::string& message);
    void AttachComponentToActor(Actor& actor, luabridge::LuaRef& component);
    void DetachComponentFromActor(Actor& actor, const std::string& componentName);
    luabridge::LuaRef CreateComponentInstance(const std::string& componentInstanceName, const luabridge::LuaRef& templateComponent);
};

extern SceneManager* scene;
extern std::pair<float, float>* static_camera_pos;