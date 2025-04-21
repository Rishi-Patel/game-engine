#include "InputManager.h"

#include <stdexcept>

#include "Helper.h"
#include "SDL2/SDL.h"

using Input::Action;
using Input::Event;
using Input::InputManager;

constexpr size_t MOUSE_BUTTON_LEFT = SDL_Scancode::SDL_NUM_SCANCODES;
constexpr size_t MOUSE_BUTTON_MIDDLE = SDL_Scancode::SDL_NUM_SCANCODES + 1;
constexpr size_t MOUSE_BUTTON_RIGHT = SDL_Scancode::SDL_NUM_SCANCODES + 2;

static size_t TranslateSDLMouseButton(uint8_t sdl_button) {
    // SDL_BUTTON codes for Left, Middle, Right are 1, 2, 3 respectively. This function
    // maps to my constexpr definitions for mouse buttons
    return SDL_Scancode::SDL_NUM_SCANCODES - 1 + sdl_button;
}

static const std::unordered_map<int, size_t> __mousebuttoncode_to_scancode = {
    {1, MOUSE_BUTTON_LEFT},
    {2, MOUSE_BUTTON_MIDDLE},
    {3, MOUSE_BUTTON_RIGHT}
};

static const std::unordered_map<std::string, SDL_Scancode> __keycode_to_scancode = {
    // Directional (arrow) Keys
    {"up", SDL_SCANCODE_UP},
    {"down", SDL_SCANCODE_DOWN},
    {"right", SDL_SCANCODE_RIGHT},
    {"left", SDL_SCANCODE_LEFT},

    // Misc Keys
    {"escape", SDL_SCANCODE_ESCAPE},

    // Modifier Keys
    {"lshift", SDL_SCANCODE_LSHIFT},
    {"rshift", SDL_SCANCODE_RSHIFT},
    {"lctrl", SDL_SCANCODE_LCTRL},
    {"rctrl", SDL_SCANCODE_RCTRL},
    {"lalt", SDL_SCANCODE_LALT},
    {"ralt", SDL_SCANCODE_RALT},

    // Editing Keys
    {"tab", SDL_SCANCODE_TAB},
    {"return", SDL_SCANCODE_RETURN},
    {"enter", SDL_SCANCODE_RETURN},
    {"backspace", SDL_SCANCODE_BACKSPACE},
    {"delete", SDL_SCANCODE_DELETE},
    {"insert", SDL_SCANCODE_INSERT},

    // Character Keys
    {"space", SDL_SCANCODE_SPACE},
    {"a", SDL_SCANCODE_A},
    {"b", SDL_SCANCODE_B},
    {"c", SDL_SCANCODE_C},
    {"d", SDL_SCANCODE_D},
    {"e", SDL_SCANCODE_E},
    {"f", SDL_SCANCODE_F},
    {"g", SDL_SCANCODE_G},
    {"h", SDL_SCANCODE_H},
    {"i", SDL_SCANCODE_I},
    {"j", SDL_SCANCODE_J},
    {"k", SDL_SCANCODE_K},
    {"l", SDL_SCANCODE_L},
    {"m", SDL_SCANCODE_M},
    {"n", SDL_SCANCODE_N},
    {"o", SDL_SCANCODE_O},
    {"p", SDL_SCANCODE_P},
    {"q", SDL_SCANCODE_Q},
    {"r", SDL_SCANCODE_R},
    {"s", SDL_SCANCODE_S},
    {"t", SDL_SCANCODE_T},
    {"u", SDL_SCANCODE_U},
    {"v", SDL_SCANCODE_V},
    {"w", SDL_SCANCODE_W},
    {"x", SDL_SCANCODE_X},
    {"y", SDL_SCANCODE_Y},
    {"z", SDL_SCANCODE_Z},
    {"0", SDL_SCANCODE_0},
    {"1", SDL_SCANCODE_1},
    {"2", SDL_SCANCODE_2},
    {"3", SDL_SCANCODE_3},
    {"4", SDL_SCANCODE_4},
    {"5", SDL_SCANCODE_5},
    {"6", SDL_SCANCODE_6},
    {"7", SDL_SCANCODE_7},
    {"8", SDL_SCANCODE_8},
    {"9", SDL_SCANCODE_9},
    {"/", SDL_SCANCODE_SLASH},
    {";", SDL_SCANCODE_SEMICOLON},
    {"=", SDL_SCANCODE_EQUALS},
    {"-", SDL_SCANCODE_MINUS},
    {".", SDL_SCANCODE_PERIOD},
    {",", SDL_SCANCODE_COMMA},
    {"[", SDL_SCANCODE_LEFTBRACKET},
    {"]", SDL_SCANCODE_RIGHTBRACKET},
    {"\\", SDL_SCANCODE_BACKSLASH},
    {"'", SDL_SCANCODE_APOSTROPHE}
};

Event TranslateSDLKeyToEvent(const SDL_Scancode key)
{
    switch (key)
    {
    case SDL_Scancode::SDL_SCANCODE_SPACE:
        return Event::Space;
    case SDL_Scancode::SDL_SCANCODE_RETURN:
        return Event::Enter;
    case SDL_Scancode::SDL_SCANCODE_LEFT:
        return Event::LeftArrow;
    case SDL_Scancode::SDL_SCANCODE_UP:
        return Event::UpArrow;
    case SDL_Scancode::SDL_SCANCODE_DOWN:
        return Event::DownArrow;
    case SDL_Scancode::SDL_SCANCODE_RIGHT:
        return Event::RightArrow;
    case SDL_Scancode::SDL_SCANCODE_W:
        return Event::W;
    case SDL_Scancode::SDL_SCANCODE_A:
        return Event::A;
    case SDL_Scancode::SDL_SCANCODE_S:
        return Event::S;
    case SDL_Scancode::SDL_SCANCODE_D:
        return Event::D;

    default:
        return Event::Unknown;
    }
}

InputManager::InputManager() : _currentContext(0), _eventToActions(1, std::unordered_map<Event, std::vector<Action>>()), _states(static_cast<size_t>(SDL_Scancode::SDL_NUM_SCANCODES + 3), InputState::UP), _mousePosition({0,0})
{
    if (SDL_WasInit(SDL_INIT_EVENTS) == false && SDL_Init(SDL_INIT_EVENTS) < 0)
    {

        throw std::runtime_error("SDL_EVENTS could not be initialized! SDL_Error: " + std::string(SDL_GetError()));
    }
}

size_t InputManager::AddContext()
{
    _eventToActions.emplace_back(std::unordered_map<Event, std::vector<Action>>());
    return _eventToActions.size() - 1;
}

void InputManager::SetContext(size_t newContext)
{
    if (newContext >= _eventToActions.size())
    {
        // ToDo: Log "Context does not exist"
        return;
    }
    _currentContext = newContext;
}

void InputManager::AddActionToEvent(const size_t context, const Event event, Action &&action)
{
    if (context >= _eventToActions.size())
    {
        // ToDo: Log "Context does not exist"
        return;
    }
    _eventToActions[context][event].emplace_back(std::forward<Action>(action));
}

void InputManager::AddActionToEvent(const size_t context, const Event event, const Action& action)
{
    if (context >= _eventToActions.size())
    {
        // ToDo: Log "Context does not exist"
        return;
    }
    _eventToActions[context][event].emplace_back(action);
}

void InputManager::CheckForEvents()
{
    _mouseScrollDelta = 0.0f;
    _eventsJustChanged.clear();
    SDL_Event e;
    while (Helper::SDL_PollEvent(&e))
    {
        if (e.type == SDL_QUIT)
        {
            if (_eventToActions[0].find(Event::Exit) != _eventToActions[0].end())
            {
                _eventToActions[0][Event::Exit].front().OnKeyDown();
            }
            continue;
        }

        if (e.type == SDL_KEYUP || e.type == SDL_KEYDOWN)
        {
            InputState newInputState = (e.type == SDL_KEYUP) ? InputState::UP : InputState::DOWN;
            bool isNewState = newInputState != _states[e.key.keysym.scancode];
            if (isNewState) {
                _eventsJustChanged.emplace(e.key.keysym.scancode);
            }
            _states[e.key.keysym.scancode] = newInputState;
            Event event = TranslateSDLKeyToEvent(e.key.keysym.scancode);
            if (_eventToActions[_currentContext].find(event) != _eventToActions[_currentContext].end() && isNewState)
            {
                if (e.type == SDL_KEYUP)
                {
                    for (auto &action : _eventToActions[_currentContext][event])
                    {
                        action.OnKeyUp();
                    }
                }
                else
                {
                    for (auto &action : _eventToActions[_currentContext][event])
                    {
                        action.OnKeyDown();
                    }
                }
            }
        }
        if (e.type == SDL_MOUSEBUTTONDOWN)
        {
            InputState newInputState = InputState::DOWN;
            bool isNewState = newInputState != _states[TranslateSDLMouseButton(e.button.button)];
            if (isNewState) {
                _eventsJustChanged.emplace(TranslateSDLMouseButton(e.button.button));
            }
            _states[TranslateSDLMouseButton(e.button.button)] = newInputState;
            Event event = Event::MouseClick;
            if (_eventToActions[_currentContext].find(event) != _eventToActions[_currentContext].end() && isNewState)
            {
                for (auto &action : _eventToActions[_currentContext][event])
                {
                    action.OnKeyDown();
                }
            }
        }
        if (e.type == SDL_MOUSEBUTTONUP)
        {
            InputState newInputState = InputState::UP;
            bool isNewState = newInputState != _states[TranslateSDLMouseButton(e.button.button)];
            if (isNewState) {
                _eventsJustChanged.emplace(TranslateSDLMouseButton(e.button.button));
            }
            _states[TranslateSDLMouseButton(e.button.button)] = newInputState;
            Event event = Event::MouseClick;
            if (_eventToActions[_currentContext].find(event) != _eventToActions[_currentContext].end() && isNewState)
            {
                for (auto &action : _eventToActions[_currentContext][event])
                {
                    action.OnKeyUp();
                }
            }
        }
        if (e.type == SDL_MOUSEMOTION) {
            _mousePosition = { e.motion.x, e.motion.y };
        }
        if (e.type == SDL_MOUSEWHEEL) {
            _mouseScrollDelta = e.wheel.preciseY;
        }
    }
}

bool InputManager::GetKey(const std::string& key) {
    if (__keycode_to_scancode.find(key) == __keycode_to_scancode.end()) {
        return false;
    }
    return _states[__keycode_to_scancode.at(key)] == InputState::DOWN;
}

bool InputManager::GetKeyDown(const std::string& key) {
    if (__keycode_to_scancode.find(key) == __keycode_to_scancode.end()) {
        return false;
    }
    return _states[__keycode_to_scancode.at(key)] == InputState::DOWN && _eventsJustChanged.count(__keycode_to_scancode.at(key)) > 0;
}

bool InputManager::GetKeyUp(const std::string& key) {
    if (__keycode_to_scancode.find(key) == __keycode_to_scancode.end()) {
        return false;
    }
    return _states[__keycode_to_scancode.at(key)] == InputState::UP && _eventsJustChanged.count(__keycode_to_scancode.at(key)) > 0;
}

glm::vec2 InputManager::GetMousePosition() {
    return _mousePosition;
}
bool InputManager::GetMouseButton(int button) {
    if (__mousebuttoncode_to_scancode.find(button) == __mousebuttoncode_to_scancode.end()) {
        return false;
    }
    return _states[TranslateSDLMouseButton(button)] == InputState::DOWN;
}
bool InputManager::GetMouseButtonDown(int button) {
    if (__mousebuttoncode_to_scancode.find(button) == __mousebuttoncode_to_scancode.end()) {
        return false;
    }
    return _states[TranslateSDLMouseButton(button)] == InputState::DOWN && _eventsJustChanged.count(TranslateSDLMouseButton(button)) > 0;
}
bool InputManager::GetMouseButtonUp(int button) {
    if (__mousebuttoncode_to_scancode.find(button) == __mousebuttoncode_to_scancode.end()) {
        return false;
    }
    return _states[TranslateSDLMouseButton(button)] == InputState::UP && _eventsJustChanged.count(TranslateSDLMouseButton(button)) > 0;
}

float InputManager::GetMouseScrollDelta() {
    return _mouseScrollDelta;
}

void InputManager::HideCursor() {
    SDL_ShowCursor(SDL_DISABLE);
}
void InputManager::ShowCursor() {
    SDL_ShowCursor(SDL_ENABLE);
}