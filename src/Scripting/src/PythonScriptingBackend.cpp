#include "PythonScriptingBackend.h"

#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <thread>

#include "AudioManager.h"
#include "Defines.h"
#include "GraphicsManager.h"
#include "InputManager.h"
#include "NetworkManager.h"
#include "SceneManager.h"

namespace py = pybind11;

// Defined in SceneManager.cpp; used by the Physics.Raycast lambdas so they can
// hit the same raycast listener / world as the rest of the engine.
extern RaycastListener* static_raycast_listener;
extern b2World* static_physics_world;

// The Actor type is bound in a file-scope PYBIND11_EMBEDDED_MODULE, so its
// method lambdas can't capture `this`. These globals let those lambdas reach
// the backend/scene. Set in Initialize, cleared in the destructor.
static PythonScriptingBackend* s_instance = nullptr;
static SceneManager* s_scene = nullptr;

static const std::filesystem::path COMPONENT_DIR =
    std::filesystem::path("resources") /
    std::filesystem::path("component_types");

static void ReportError(const std::string& actorName,
                        const py::error_already_set& e) {
  std::cout << "\033[31m" << actorName << " : " << e.what() << "\033[0m"
            << std::endl;
}

static bool PyEqual(const py::object& a, const py::object& b) {
  return PyObject_RichCompareBool(a.ptr(), b.ptr(), Py_EQ) == 1;
}

// ============================================================================
// Native type bindings. These are registered once on first import of the
// "engine_native" module.
// ============================================================================

PYBIND11_EMBEDDED_MODULE(engine_native, m) {
  py::class_<glm::vec2>(m, "vec2")
      .def_readwrite("x", &glm::vec2::x)
      .def_readwrite("y", &glm::vec2::y);

  py::class_<b2Vec2>(m, "Vector2")
      .def(py::init<float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f)
      .def_readwrite("x", &b2Vec2::x)
      .def_readwrite("y", &b2Vec2::y)
      .def("Normalize", &b2Vec2::Normalize)
      .def("Length", &b2Vec2::Length)
      .def("__add__",
           [](const b2Vec2& a, const b2Vec2& b) {
             return b2Vec2{a.x + b.x, a.y + b.y};
           })
      .def("__sub__",
           [](const b2Vec2& a, const b2Vec2& b) {
             return b2Vec2{a.x - b.x, a.y - b.y};
           })
      .def("__mul__",
           [](const b2Vec2& a, float s) { return b2Vec2{a.x * s, a.y * s}; })
      .def_static("Distance", &b2Distance)
      .def_static(
          "Dot",
          static_cast<float (*)(const b2Vec2&, const b2Vec2&)>(&b2Dot));

  py::class_<HitResult>(m, "HitResult")
      .def_readwrite("actor", &HitResult::actor)
      .def_readwrite("point", &HitResult::point)
      .def_readwrite("is_trigger", &HitResult::is_trigger)
      .def_readwrite("normal", &HitResult::normal);

  py::class_<Collision>(m, "Collision")
      .def_readwrite("other", &Collision::other)
      .def_readwrite("point", &Collision::point)
      .def_readwrite("relative_velocity", &Collision::relative_velocity)
      .def_readwrite("normal", &Collision::normal);

  py::class_<Actor>(m, "Actor")
      .def("GetName", [](const Actor* a) { return a->name; })
      .def("GetID", [](const Actor* a) { return a->id; })
      .def("GetComponentByKey",
           [](Actor* actor, const std::string& key) -> py::object {
             if (actor->components.find(key) == actor->components.end() ||
                 actor->pendingDeletion.count(key) > 0) {
               return py::none();
             }
             return PythonScriptingBackend::Unwrap(actor->components.at(key));
           })
      .def("GetComponent",
           [](Actor* actor, const std::string& templateType) -> py::object {
             auto it =
                 actor->_componentKeysByTemplateType.find(templateType);
             if (it == actor->_componentKeysByTemplateType.end()) {
               return py::none();
             }
             for (const auto& componentName : it->second) {
               if (actor->pendingDeletion.count(componentName) == 0) {
                 return PythonScriptingBackend::Unwrap(
                     actor->components.at(componentName));
               }
             }
             return py::none();
           })
      .def("GetComponents",
           [](Actor* actor, const std::string& templateType) {
             py::list result;
             auto it =
                 actor->_componentKeysByTemplateType.find(templateType);
             if (it == actor->_componentKeysByTemplateType.end()) {
               return result;
             }
             for (const auto& componentName : it->second) {
               if (actor->pendingDeletion.count(componentName) == 0) {
                 result.append(PythonScriptingBackend::Unwrap(
                     actor->components.at(componentName)));
               }
             }
             return result;
           })
      .def("AddComponent",
           [](Actor* actor, const std::string& templateType) {
             return s_instance->ActorAddComponent(actor, templateType);
           })
      .def("RemoveComponent", [](Actor* actor, py::object component) {
        s_instance->ActorRemoveComponent(actor, component);
      });

  py::class_<ParticleSystem>(m, "ParticleSystem")
      .def(py::init<>())
      .def_readwrite("Particle", &ParticleSystem::ImageName)
      .def_readwrite("key", &ParticleSystem::key)
      .def_readwrite("type", &ParticleSystem::type)
      .def_readwrite("enabled", &ParticleSystem::enabled)
      .def_readwrite("actor", &ParticleSystem::actor)
      .def_readwrite("x", &ParticleSystem::SpawnCenterX)
      .def_readwrite("y", &ParticleSystem::SpawnCenterY)
      .def_readwrite("start_scale_min", &ParticleSystem::ScaleMin)
      .def_readwrite("start_scale_max", &ParticleSystem::ScaleMax)
      .def_readwrite("rotation_min", &ParticleSystem::RotationMin)
      .def_readwrite("rotation_max", &ParticleSystem::RotationMax)
      .def_readwrite("start_color_r", &ParticleSystem::start_color_r)
      .def_readwrite("start_color_g", &ParticleSystem::start_color_g)
      .def_readwrite("start_color_b", &ParticleSystem::start_color_b)
      .def_readwrite("start_color_a", &ParticleSystem::start_color_a)
      .def_readwrite("end_color_r", &ParticleSystem::end_color_r)
      .def_readwrite("end_color_g", &ParticleSystem::end_color_g)
      .def_readwrite("end_color_b", &ParticleSystem::end_color_b)
      .def_readwrite("end_color_a", &ParticleSystem::end_color_a)
      .def_readwrite("emit_radius_min", &ParticleSystem::MinSpawnRadius)
      .def_readwrite("emit_radius_max", &ParticleSystem::MaxSpawnRadius)
      .def_readwrite("emit_angle_min", &ParticleSystem::MinSpawnAngle)
      .def_readwrite("emit_angle_max", &ParticleSystem::MaxSpawnAngle)
      .def_readwrite("start_speed_min", &ParticleSystem::SpeedMin)
      .def_readwrite("start_speed_max", &ParticleSystem::SpeedMax)
      .def_readwrite("rotation_speed_min",
                     &ParticleSystem::RotationSpeedMin)
      .def_readwrite("rotation_speed_max",
                     &ParticleSystem::RotationSpeedMax)
      .def_readwrite("gravity_scale_x",
                     &ParticleSystem::ParticleAccelerationX)
      .def_readwrite("gravity_scale_y",
                     &ParticleSystem::ParticleAccelerationY)
      .def_readwrite("image", &ParticleSystem::ImageName)
      .def_readwrite("sorting_order", &ParticleSystem::SortingOrder)
      .def_readwrite("drag_factor", &ParticleSystem::DragFactor)
      .def_readwrite("angular_drag_factor",
                     &ParticleSystem::RotationDragFactor)
      .def_readwrite("end_scale", &ParticleSystem::EndScale)
      .def_readwrite("frames_between_bursts",
                     &ParticleSystem::FramesBetweenSpawn)
      .def_readwrite("burst_quantity", &ParticleSystem::ParticlesPerSpawn)
      .def_readwrite("duration_frames",
                     &ParticleSystem::ParticleLifeTimeInFrames)
      .def("OnStart", &ParticleSystem::OnStart)
      .def("OnUpdate", &ParticleSystem::OnUpdate)
      .def("Stop", &ParticleSystem::DisableSpawn)
      .def("Play", &ParticleSystem::EnableSpawn)
      .def("Burst", &ParticleSystem::SpawnParticles);

  py::class_<RigidBody>(m, "Rigidbody")
      .def(py::init<>())
      .def_readwrite("key", &RigidBody::key)
      .def_readwrite("type", &RigidBody::type)
      .def_readwrite("enabled", &RigidBody::enabled)
      .def_readwrite("actor", &RigidBody::actor)
      .def_readwrite("x", &RigidBody::x)
      .def_readwrite("y", &RigidBody::y)
      .def_readwrite("body_type", &RigidBody::body_type)
      .def_readwrite("precise", &RigidBody::precise)
      .def_readwrite("gravity_scale", &RigidBody::gravity_scale)
      .def_readwrite("density", &RigidBody::density)
      .def_readwrite("angular_friction", &RigidBody::angular_friction)
      .def_readwrite("rotation", &RigidBody::rotation)
      .def_readwrite("has_collider", &RigidBody::has_collider)
      .def_readwrite("has_trigger", &RigidBody::has_trigger)
      .def_readwrite("collider_type", &RigidBody::collider_type)
      .def_readwrite("width", &RigidBody::width)
      .def_readwrite("height", &RigidBody::height)
      .def_readwrite("radius", &RigidBody::radius)
      .def_readwrite("friction", &RigidBody::friction)
      .def_readwrite("bounciness", &RigidBody::bounciness)
      .def_readwrite("trigger_type", &RigidBody::trigger_type)
      .def_readwrite("trigger_width", &RigidBody::trigger_width)
      .def_readwrite("trigger_height", &RigidBody::trigger_height)
      .def_readwrite("trigger_radius", &RigidBody::trigger_radius)
      .def("GetPosition",
           [](RigidBody* rb) {
             if (rb->body == nullptr) return b2Vec2{rb->x, rb->y};
             return rb->body->GetPosition();
           })
      .def("GetRotation",
           [](RigidBody* rb) {
             if (rb->body == nullptr) return rb->rotation;
             return ConvertRadainsToDegrees(rb->body->GetAngle());
           })
      .def("GetVelocity",
           [](RigidBody* rb) { return rb->body->GetLinearVelocity(); })
      .def("GetAngularVelocity",
           [](RigidBody* rb) {
             return ConvertRadainsToDegrees(rb->body->GetAngularVelocity());
           })
      .def("GetGravityScale",
           [](RigidBody* rb) {
             if (rb->body == nullptr) return rb->gravity_scale;
             return rb->body->GetGravityScale();
           })
      .def("GetUpDirection",
           [](RigidBody* rb) {
             float radAngle = -rb->body->GetAngle() + (b2_pi / 2.0f);
             auto res = b2Vec2(glm::cos(radAngle), -glm::sin(radAngle));
             res.Normalize();
             return res;
           })
      .def("GetRightDirection",
           [](RigidBody* rb) {
             float radAngle = -rb->body->GetAngle();
             return b2Vec2(glm::cos(radAngle), -glm::sin(radAngle));
           })
      .def("OnStart",
           [](RigidBody* rb) {
             if (s_instance) rb->OnStart(*static_physics_world);
           })
      .def("AddForce",
           [](RigidBody* rb, const b2Vec2& force) {
             rb->body->ApplyForceToCenter(force, true);
           })
      .def("SetVelocity",
           [](RigidBody* rb, const b2Vec2& velocity) {
             rb->body->SetLinearVelocity(velocity);
           })
      .def("SetPosition",
           [](RigidBody* rb, const b2Vec2& position) {
             if (rb->body == nullptr) {
               rb->x = position.x;
               rb->y = position.y;
             } else {
               rb->body->SetTransform(position, rb->body->GetAngle());
             }
           })
      .def("SetRotation",
           [](RigidBody* rb, float degClockwise) {
             if (rb->body == nullptr) {
               rb->rotation = degClockwise;
             } else {
               rb->body->SetTransform(
                   rb->body->GetPosition(),
                   ConvertDegreesToRadians(degClockwise));
             }
           })
      .def("SetAngularVelocity",
           [](RigidBody* rb, float degClockwise) {
             rb->body->SetAngularVelocity(
                 ConvertDegreesToRadians(degClockwise));
           })
      .def("SetGravityScale",
           [](RigidBody* rb, float gravityScale) {
             if (rb->body == nullptr) {
               rb->gravity_scale = gravityScale;
             } else {
               rb->body->SetGravityScale(gravityScale);
             }
           })
      .def("SetUpDirection",
           [](RigidBody* rb, b2Vec2 direction) {
             direction.Normalize();
             rb->body->SetTransform(
                 rb->body->GetPosition(),
                 -glm::atan(-direction.y, direction.x) + (b2_pi / 2.0f));
           })
      .def("SetRightDirection", [](RigidBody* rb, b2Vec2 direction) {
        direction.Normalize();
        rb->body->SetTransform(rb->body->GetPosition(),
                               -glm::atan(-direction.y, direction.x));
      });
}

// ============================================================================

py::object& PythonScriptingBackend::Unwrap(ScriptRef& ref) {
  return *std::any_cast<py::object>(&ref);
}
const py::object& PythonScriptingBackend::Unwrap(const ScriptRef& ref) {
  return *std::any_cast<py::object>(&ref);
}

PythonScriptingBackend::PythonScriptingBackend()
    : _scene(nullptr),
      _graphics(nullptr),
      _audio(nullptr),
      _input(nullptr),
      _network(nullptr),
      _physics(nullptr) {
  _interp = std::make_unique<py::scoped_interpreter>();
}

PythonScriptingBackend::~PythonScriptingBackend() {
  // Python objects must be destroyed before the interpreter shuts down.
  _componentTemplates.clear();
  _subscriptions.clear();
  while (!_newSubscriptions.empty()) _newSubscriptions.pop();
  while (!_removeSubscriptions.empty()) _removeSubscriptions.pop();
  _engine = py::module_();
  s_instance = nullptr;
  s_scene = nullptr;
}

void PythonScriptingBackend::Initialize(SceneManager* scene,
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
  s_instance = this;
  s_scene = scene;

  auto componentDir = std::filesystem::absolute(COMPONENT_DIR);
  py::module_::import("sys").attr("path").attr("insert")(
      0, componentDir.string());

  // Force the native module to register its types.
  py::module_::import("engine_native");

  BuildEngineModule();
  InstallBuiltins();
}

void PythonScriptingBackend::ProcessFrameEndQueues() {
  while (!_newSubscriptions.empty()) {
    auto [eventType, component, func] = _newSubscriptions.front();
    _newSubscriptions.pop();
    _subscriptions[eventType].emplace_back(component, func);
  }
  while (!_removeSubscriptions.empty()) {
    auto [eventType, component, func] = _removeSubscriptions.front();
    _removeSubscriptions.pop();
    auto subIt = _subscriptions.find(eventType);
    if (subIt == _subscriptions.end()) continue;
    auto& callbacks = subIt->second;
    for (auto it = callbacks.begin(); it != callbacks.end(); ++it) {
      if (PyEqual(it->first, component) && PyEqual(it->second, func)) {
        callbacks.erase(it);
        break;
      }
    }
    if (callbacks.empty()) _subscriptions.erase(subIt);
  }
}

py::object PythonScriptingBackend::LoadPythonClass(
    const std::string& componentName) {
  py::module_ mod = py::module_::import(componentName.c_str());
  return mod.attr(componentName.c_str());
}

void PythonScriptingBackend::EstablishInheritance(
    py::object& instance, const py::object& template_component) {
  std::string type_str;
  if (py::hasattr(template_component, "type")) {
    auto t = template_component.attr("type");
    if (!t.is_none()) type_str = py::cast<std::string>(t);
  }
  if (type_str == "Rigidbody") {
    auto* parent = py::cast<RigidBody*>(template_component);
    instance = py::cast(new RigidBody(*parent),
                        py::return_value_policy::take_ownership);
  } else if (type_str == "ParticleSystem") {
    auto* parent = py::cast<ParticleSystem*>(template_component);
    instance = py::cast(new ParticleSystem(*parent),
                        py::return_value_policy::take_ownership);
  } else if (PyType_Check(template_component.ptr())) {
    instance = template_component();
  } else {
    // Template is an already-built instance (e.g. a template actor's
    // component): construct a fresh instance of the same class and carry
    // over any per-template attribute overrides stored on __dict__.
    py::object cls = template_component.attr("__class__");
    instance = cls();
    if (py::hasattr(template_component, "__dict__")) {
      py::dict overrides = template_component.attr("__dict__");
      for (auto item : overrides) {
        instance.attr(item.first) = py::reinterpret_borrow<py::object>(item.second);
      }
    }
  }
}

ScriptRef PythonScriptingBackend::LoadComponentTemplate(
    const std::string& componentName) {
  auto existing = _componentTemplates.find(componentName);
  if (existing != _componentTemplates.end()) {
    return ScriptRef(existing->second);
  }
  py::object templ;
  if (componentName == "Rigidbody") {
    auto* rb = new RigidBody();
    templ = py::cast(rb, py::return_value_policy::take_ownership);
    templ.attr("type") = py::str(componentName);
  } else if (componentName == "ParticleSystem") {
    auto* ps = new ParticleSystem();
    templ = py::cast(ps, py::return_value_policy::take_ownership);
    templ.attr("type") = py::str(componentName);
  } else {
    const auto componentPath =
        COMPONENT_DIR / (componentName + std::string(".py"));
    if (!std::filesystem::exists(componentPath)) {
      std::cout << "error: failed to locate component " + componentName;
      exit(0);
    }
    templ = LoadPythonClass(componentName);
    // Install a class-level `type` attribute so instances inherit it.
    templ.attr("type") = py::str(componentName);
  }
  _componentTemplates.emplace(componentName, templ);
  return ScriptRef(templ);
}

ScriptRef PythonScriptingBackend::CreateComponentInstance(
    const std::string& componentInstanceName, const ScriptRef& templateRef) {
  py::object instance;
  EstablishInheritance(instance, Unwrap(templateRef));
  instance.attr("key") = py::str(componentInstanceName);
  return ScriptRef(instance);
}

void PythonScriptingBackend::DestroyNativeComponent(
    ScriptRef& /*ref*/, const std::string& /*componentType*/) {
  // pybind11 manages the C++ object lifetime via the py::object refcount; the
  // ScriptRef destructor (via std::any) will decrement it for us.
}

ScriptRef PythonScriptingBackend::WrapActor(Actor* actor) {
  return ScriptRef(py::cast(actor, py::return_value_policy::reference).cast<py::object>());
}

bool PythonScriptingBackend::IsRefValid(const ScriptRef& ref) {
  if (!ref.has_value()) return false;
  auto* pref = std::any_cast<py::object>(&ref);
  return pref && !pref->is_none();
}

bool PythonScriptingBackend::IsMethod(const ScriptRef& ref,
                                      const std::string& key) {
  const auto& obj = Unwrap(ref);
  if (!py::hasattr(obj, key.c_str())) return false;
  auto attr = obj.attr(key.c_str());
  return PyCallable_Check(attr.ptr()) != 0;
}

bool PythonScriptingBackend::HasActorProperty(const ScriptRef& ref,
                                              const std::string& key) {
  const auto& obj = Unwrap(ref);
  if (!py::hasattr(obj, key.c_str())) return false;
  return !obj.attr(key.c_str()).is_none();
}

bool PythonScriptingBackend::GetBool(const ScriptRef& ref,
                                     const std::string& key) {
  return py::cast<bool>(Unwrap(ref).attr(key.c_str()));
}
int PythonScriptingBackend::GetInt(const ScriptRef& ref,
                                   const std::string& key) {
  return py::cast<int>(Unwrap(ref).attr(key.c_str()));
}
float PythonScriptingBackend::GetFloat(const ScriptRef& ref,
                                       const std::string& key) {
  return py::cast<float>(Unwrap(ref).attr(key.c_str()));
}
std::string PythonScriptingBackend::GetString(const ScriptRef& ref,
                                              const std::string& key) {
  return py::cast<std::string>(Unwrap(ref).attr(key.c_str()));
}
Actor* PythonScriptingBackend::GetActor(const ScriptRef& ref,
                                        const std::string& key) {
  auto attr = Unwrap(ref).attr(key.c_str());
  if (attr.is_none()) return nullptr;
  return py::cast<Actor*>(attr);
}

void PythonScriptingBackend::SetBool(ScriptRef& ref, const std::string& key,
                                     bool v) {
  Unwrap(ref).attr(key.c_str()) = py::bool_(v);
}
void PythonScriptingBackend::SetInt(ScriptRef& ref, const std::string& key,
                                    int v) {
  Unwrap(ref).attr(key.c_str()) = py::int_(v);
}
void PythonScriptingBackend::SetFloat(ScriptRef& ref, const std::string& key,
                                      float v) {
  Unwrap(ref).attr(key.c_str()) = py::float_(v);
}
void PythonScriptingBackend::SetString(ScriptRef& ref, const std::string& key,
                                       const std::string& v) {
  Unwrap(ref).attr(key.c_str()) = py::str(v);
}
void PythonScriptingBackend::SetActor(ScriptRef& ref, const std::string& key,
                                      Actor* actor) {
  Unwrap(ref).attr(key.c_str()) =
      py::cast(actor, py::return_value_policy::reference);
}

void PythonScriptingBackend::CallSelfMethod(ScriptRef& ref,
                                            const std::string& method,
                                            std::string_view actorName) {
  try {
    Unwrap(ref).attr(method.c_str())();
  } catch (const py::error_already_set& e) {
    ReportError(std::string(actorName), e);
  }
}

void PythonScriptingBackend::CallCollisionMethod(ScriptRef& ref,
                                                 const std::string& method,
                                                 const Collision& collision,
                                                 std::string_view actorName) {
  try {
    Unwrap(ref).attr(method.c_str())(collision);
  } catch (const py::error_already_set& e) {
    ReportError(std::string(actorName), e);
  }
}

py::object PythonScriptingBackend::ActorAddComponent(
    Actor* actor, const std::string& templateType) {
  auto templateComponent = LoadComponentTemplate(templateType);
  std::string componentName =
      "r" + std::to_string(_scene->_componentCounter++);
  auto component = CreateComponentInstance(componentName, templateComponent);
  _scene->newComponents.emplace(actor->id, component);
  return Unwrap(component);
}

void PythonScriptingBackend::ActorRemoveComponent(Actor* actor,
                                                  py::object component) {
  component.attr("enabled") = py::bool_(false);
  std::string key = py::cast<std::string>(component.attr("key"));
  actor->pendingDeletion.emplace(key);
  _scene->removedComponents.emplace(actor->id, key);
}

RigidBody* PythonScriptingBackend::AsRigidBody(const ScriptRef& ref) {
  return py::cast<RigidBody*>(Unwrap(ref));
}
ParticleSystem* PythonScriptingBackend::AsParticleSystem(
    const ScriptRef& ref) {
  return py::cast<ParticleSystem*>(Unwrap(ref));
}

// ============================================================================
// API registration
// ============================================================================

void PythonScriptingBackend::BuildEngineModule() {
  py::module_ types = py::module_::import("types");
  py::object SN = types.attr("SimpleNamespace");

  // --- Debug ---
  py::object Debug = SN();
  Debug.attr("Log") = py::cpp_function(
      [](const std::string& msg) { std::cout << msg << std::endl; });

  // --- Application ---
  py::object App = SN();
  App.attr("GetFrame") =
      py::cpp_function([this]() { return _scene->_frameNumber; });
  App.attr("Quit") = py::cpp_function([]() { exit(0); });
  App.attr("Sleep") = py::cpp_function([](int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  });
  App.attr("OpenURL") = py::cpp_function([](const std::string& url) {
#ifdef _WIN32
    std::string command = "start" + url;
#elif defined(__APPLE__)
    std::string command = "open" + url;
#else
    std::string command = "xdg-open" + url;
#endif
    std::system(command.c_str());
  });

  // --- Input ---
  py::object In = SN();
  In.attr("GetKey") = py::cpp_function(
      [this](const std::string& k) { return _input->GetKey(k); });
  In.attr("GetKeyDown") = py::cpp_function(
      [this](const std::string& k) { return _input->GetKeyDown(k); });
  In.attr("GetKeyUp") = py::cpp_function(
      [this](const std::string& k) { return _input->GetKeyUp(k); });
  In.attr("GetMouseButton") =
      py::cpp_function([this](int b) { return _input->GetMouseButton(b); });
  In.attr("GetMouseButtonDown") = py::cpp_function(
      [this](int b) { return _input->GetMouseButtonDown(b); });
  In.attr("GetMouseButtonUp") =
      py::cpp_function([this](int b) { return _input->GetMouseButtonUp(b); });
  In.attr("GetMousePosition") =
      py::cpp_function([this]() { return _input->GetMousePosition(); });
  In.attr("GetMouseScrollDelta") =
      py::cpp_function([this]() { return _input->GetMouseScrollDelta(); });
  In.attr("HideCursor") =
      py::cpp_function([this]() { _input->HideCursor(); });
  In.attr("ShowCursor") =
      py::cpp_function([this]() { _input->ShowCursor(); });

  // --- Text ---
  py::object Text = SN();
  Text.attr("Draw") = py::cpp_function(
      [this](const std::string& text, int x, int y, const std::string& font,
             unsigned int size, int r, int g, int b, int a) {
        _graphics->RenderText(font, size, text, x, y,
                              Graphics::RGBA{r, g, b, a});
      });

  // --- Audio ---
  py::object Audio = SN();
  Audio.attr("Play") = py::cpp_function(
      [this](int channel, const std::string& name, bool loop) {
        _audio->PlayAudio(name, channel, loop ? -1 : 0);
      });
  Audio.attr("Halt") =
      py::cpp_function([this](int channel) { _audio->StopAudio(channel); });
  Audio.attr("SetVolume") = py::cpp_function(
      [this](int c, int v) { _audio->SetVolume(c, v); });

  // --- Image ---
  py::object Image = SN();
  Image.attr("DrawUI") = py::cpp_function(
      [this](const std::string& imageName, float x, float y) {
        _graphics->DrawSprite(Graphics::SpriteInfo{
            imageName,
            {int(x), int(y)},
            {1.0f, 1.0f},
            {0, 0},
            0.0f,
            {255, 255, 255, 255},
            {1, 0}});
      });
  Image.attr("DrawUIEx") = py::cpp_function(
      [this](const std::string& imageName, float x, float y, float r, float g,
             float b, float a, float sortingOrder) {
        _graphics->DrawSprite(Graphics::SpriteInfo{imageName,
                                                   {int(x), int(y)},
                                                   {1.0f, 1.0f},
                                                   {0, 0},
                                                   0.0f,
                                                   {r, g, b, a},
                                                   {1, sortingOrder}});
      });
  Image.attr("Draw") = py::cpp_function(
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
      });
  Image.attr("DrawEx") = py::cpp_function(
      [this](const std::string& imageName, float x, float y, float rotation,
             float scaleX, float scaleY, float pivotX, float pivotY, float r,
             float g, float b, float a, float sortingOrder) {
        auto [spriteWidth, spriteHeight] =
            _graphics->GetSpriteDimension(imageName);
        std::pair<float, float> pivot = {scaleX * spriteWidth * pivotX,
                                         scaleY * spriteHeight * pivotY};
        auto screenPosition = _scene->ConvertCordsToScreen(x, y);
        screenPosition.first -= pivot.first;
        screenPosition.second -= pivot.second;
        _graphics->DrawSprite(Graphics::SpriteInfo{
            imageName,
            screenPosition,
            {scaleX, scaleY},
            pivot,
            static_cast<float>(int(rotation)),
            {r, g, b, a},
            {0, sortingOrder}});
      });
  Image.attr("DrawPixel") = py::cpp_function(
      [this](float x, float y, float r, float g, float b, float a) {
        _graphics->DrawPoint(int(x), int(y), {r, g, b, a});
      });

  // --- Camera ---
  py::object Camera = SN();
  Camera.attr("SetPosition") = py::cpp_function([this](float x, float y) {
    _scene->_cameraPos = {x, y};
  });
  Camera.attr("GetPositionX") =
      py::cpp_function([this]() { return _scene->_cameraPos.first; });
  Camera.attr("GetPositionY") =
      py::cpp_function([this]() { return _scene->_cameraPos.second; });
  Camera.attr("SetZoom") = py::cpp_function([this](float zoom) {
    _scene->_cameraZoom = zoom;
    _graphics->SetRenderScale(_scene->_cameraZoom);
  });
  Camera.attr("GetZoom") =
      py::cpp_function([this]() { return _scene->_cameraZoom; });

  // --- Scene ---
  py::object Scene = SN();
  Scene.attr("Load") = py::cpp_function(
      [this](const std::string& name) { _scene->_nextScene = name; });
  Scene.attr("GetCurrent") =
      py::cpp_function([this]() { return _scene->_currentScene; });
  Scene.attr("DontDestroy") =
      py::cpp_function([](Actor* a) { a->dontDestroy = true; });

  // --- Physics ---
  py::object Physics = SN();
  Physics.attr("Raycast") = py::cpp_function(
      [](b2Vec2 pos, b2Vec2 dir, float dist) -> py::object {
        dir.Normalize();
        dir *= dist;
        static_raycast_listener->Reset();
        static_physics_world->RayCast(static_raycast_listener, pos,
                                      pos + dir);
        auto& hits = static_raycast_listener->GetAllRayCast();
        if (hits.empty()) return py::none();
        return py::cast(hits.front());
      });
  Physics.attr("RaycastAll") = py::cpp_function(
      [](b2Vec2 pos, b2Vec2 dir, float dist) {
        py::list result;
        dir.Normalize();
        dir *= dist;
        static_raycast_listener->Reset();
        static_physics_world->RayCast(static_raycast_listener, pos,
                                      pos + dir);
        auto& hits = static_raycast_listener->GetAllRayCast();
        for (auto& h : hits) result.append(h);
        return result;
      });

  // --- Event ---
  py::object Event = SN();
  Event.attr("Publish") = py::cpp_function(
      [this](const std::string& eventType, py::object eventObject) {
        auto it = _subscriptions.find(eventType);
        if (it == _subscriptions.end()) return;
        for (auto& [component, func] : it->second) {
          try {
            func(component, eventObject);
          } catch (const py::error_already_set& e) {
            std::string actorName;
            if (py::hasattr(component, "actor") &&
                !component.attr("actor").is_none()) {
              actorName = py::cast<Actor*>(component.attr("actor"))->name;
            }
            ReportError(actorName, e);
          }
        }
      });
  Event.attr("Subscribe") = py::cpp_function(
      [this](const std::string& eventType, py::object component,
             py::object func) {
        _newSubscriptions.emplace(eventType, component, func);
      });
  Event.attr("Unsubscribe") = py::cpp_function(
      [this](const std::string& eventType, py::object component,
             py::object func) {
        _removeSubscriptions.emplace(eventType, component, func);
      });

  // --- Network ---
  py::object Network = SN();
  if (_network) {
    std::function<void(const std::vector<uint8_t>&)> cb =
        [this](const std::vector<uint8_t>& data) {
          std::lock_guard<std::mutex> lock(_networkLock);
          _networkMessageQueue.push(data);
        };
    _network->SetHandlePacketCallback(cb);

    Network.attr("Receive") = py::cpp_function([this]() {
      std::vector<uint8_t> msg;
      b2Vec2 pos{};
      {
        std::lock_guard<std::mutex> lock(_networkLock);
        if (_networkMessageQueue.empty()) return pos;
        msg = _networkMessageQueue.front();
        _networkMessageQueue.pop();
      }
      int x = 0;
      int y = 0;
      std::memcpy(&x, msg.data(), sizeof(float));
      std::memcpy(&y, msg.data() + sizeof(float), sizeof(float));
      x = NetworkToHost(x);
      y = NetworkToHost(y);
      pos.x = x;
      pos.y = y;
      return pos;
    });
  }

  // --- Actor (module-level namespace, separate from the bound Actor class) ---
  py::module_ native = py::module_::import("engine_native");
  py::object ActorClass = native.attr("Actor");
  py::object staticmethod = py::module_::import("builtins").attr("staticmethod");

  ActorClass.attr("Find") = staticmethod(
      py::cpp_function([this](const std::string& name) -> py::object {
        auto it = _scene->_actorsByName.find(name);
        if (it == _scene->_actorsByName.end() || it->second.empty()) {
          return py::none();
        }
        for (auto& actorId : it->second) {
          if (_scene->deactivedActors.count(actorId) == 0) {
            return py::cast(&_scene->actors[actorId],
                            py::return_value_policy::reference);
          }
        }
        return py::none();
      }));
  ActorClass.attr("FindAll") = staticmethod(
      py::cpp_function([this](const std::string& name) {
        py::list result;
        auto it = _scene->_actorsByName.find(name);
        if (it == _scene->_actorsByName.end()) return result;
        for (auto& actorId : it->second) {
          if (_scene->deactivedActors.count(actorId) == 0) {
            result.append(py::cast(&_scene->actors[actorId],
                                   py::return_value_policy::reference));
          }
        }
        return result;
      }));
  ActorClass.attr("Instantiate") = staticmethod(
      py::cpp_function([this](const std::string& templateName) {
        auto ref = _scene->CreateActor(templateName);
        return Unwrap(ref);
      }));
  ActorClass.attr("Destroy") = staticmethod(
      py::cpp_function([this](Actor* actor) {
        for (auto& [_, component] : actor->components) {
          SetBool(component, "enabled", false);
        }
        _scene->actorChanges.emplace(actor->id, false);
        _scene->deactivedActors.emplace(actor->id);
      }));

  // Stash everything on the engine module.
  _engine = types.attr("ModuleType")("engine").cast<py::module_>();
  _engine.attr("Debug") = Debug;
  _engine.attr("Application") = App;
  _engine.attr("Input") = In;
  _engine.attr("Text") = Text;
  _engine.attr("Audio") = Audio;
  _engine.attr("Image") = Image;
  _engine.attr("Camera") = Camera;
  _engine.attr("Scene") = Scene;
  _engine.attr("Physics") = Physics;
  _engine.attr("Event") = Event;
  _engine.attr("Network") = Network;
  _engine.attr("Actor") = ActorClass;
  _engine.attr("Vector2") = native.attr("Vector2");
  _engine.attr("vec2") = native.attr("vec2");
  _engine.attr("Collision") = native.attr("Collision");
  _engine.attr("HitResult") = native.attr("HitResult");
  _engine.attr("Rigidbody") = native.attr("Rigidbody");
  _engine.attr("ParticleSystem") = native.attr("ParticleSystem");

  py::module_::import("sys").attr("modules")["engine"] = _engine;
}

void PythonScriptingBackend::InstallBuiltins() {
  py::module_ builtins = py::module_::import("builtins");
  for (const char* name :
       {"Debug", "Application", "Input", "Text", "Audio", "Image", "Camera",
        "Scene", "Physics", "Event", "Network", "Actor", "Vector2", "vec2",
        "Collision", "HitResult", "Rigidbody", "ParticleSystem"}) {
    builtins.attr(name) = _engine.attr(name);
  }
}
