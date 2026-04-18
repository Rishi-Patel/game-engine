#pragma once

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

#include "InputEnumerators.h"
#include "glm/glm.hpp"

namespace Input
{
class InputManager
{
  public:
    InputManager();
    size_t AddContext();
    void SetContext(size_t newContext);
    void AddActionToEvent(size_t context, Event event, Action &&action);
    void AddActionToEvent(size_t context, Event event, const Action& action);
    void CheckForEvents();
    bool GetKey(const std::string& key);
    bool GetKeyDown(const std::string& key);
    bool GetKeyUp(const std::string& key);
    glm::vec2 GetMousePosition();
    bool GetMouseButton(int button);
    bool GetMouseButtonDown(int button);
    bool GetMouseButtonUp(int button);
    float GetMouseScrollDelta();
    void HideCursor();
    void ShowCursor();

  private:
    enum class InputState {UP, DOWN};
    size_t _currentContext;
    std::unordered_set<size_t> _eventsJustChanged;
    std::vector<std::unordered_map<Event, std::vector<Action>>> _eventToActions;
    std::vector<InputState> _states;
    glm::vec2 _mousePosition;
    float _mouseScrollDelta;
    
};

} // namespace Input
