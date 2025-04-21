#pragma once

#include <array>
#include <functional>

namespace Input
{

static constexpr auto kDescriptionSize = 15;

using OnKeyDownCallback = std::function<void()>;
using OnKeyUpCallback = std::function<void()>;

// ANSI Standard layout
enum class Event
{
    Unknown = 0,

    Exit,

    Escape,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F0,
    F10,
    F11,
    F12,

    Tilde,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,
    Num0,
    Subtract,
    Add,
    Backspace,

    Tab,
    Q,
    W,
    E,
    R,
    T,
    Y,
    U,
    I,
    O,
    P,
    LeftCurlBrace,
    RightCurlBrace,
    Backslash,

    CapsLock,
    A,
    S,
    D,
    F,
    G,
    H,
    J,
    K,
    L,
    Colon,
    Quote,
    Enter,
    LeftShift,
    Z,
    X,
    C,
    V,
    B,
    N,
    M,
    Comma,
    Period,
    Frontslash,
    RightShift,

    LeftCtrl,
    LeftWin,
    LeftAlt,
    Space,
    RightAlt,
    RightWin,
    Menu,
    RightCtrl,

    UpArrow,
    DownArrow,
    LeftArrow,
    RightArrow,

    MouseClick
};

struct Action
{
    std::array<char, kDescriptionSize + 1> Description;
    OnKeyDownCallback OnKeyDown;
    OnKeyUpCallback OnKeyUp;
};

} // namespace Input