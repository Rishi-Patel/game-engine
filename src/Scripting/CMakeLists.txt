add_library(Scripting STATIC
    src/LuaScriptingBackend.cpp
    src/PythonScriptingBackend.cpp
)

target_include_directories(Scripting PUBLIC include)
target_include_directories(Scripting SYSTEM PUBLIC "${CMAKE_BINARY_DIR}/_deps/lua-src/src")

target_link_libraries(Scripting
    PUBLIC
        Scene
        lua
        LuaBridge
        pybind11::embed
    PRIVATE
        Audio
        Graphics
        Input
        Network
)
