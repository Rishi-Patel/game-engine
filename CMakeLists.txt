cmake_minimum_required (VERSION 3.8)
set (CMAKE_CXX_STANDARD 23)

project ("GameEngine" C CXX)
set(BUILD_SHARED_LIBS OFF)

include(FetchContent)
include(ExternalProject)

FetchContent_Declare(SDL2
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-2.28.5
)
FetchContent_MakeAvailable(SDL2)

FetchContent_Declare(SDL2IMAGE
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_image.git
    GIT_TAG release-2.6.3 
)
SET(SDL2IMAGE_AVIF OFF)
set(SDL2IMAGE_INSTALL OFF)
set(BUILD_SHARED_LIBS FALSE)
FetchContent_MakeAvailable(SDL2IMAGE)

FetchContent_Declare(SDL2_ttf
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_ttf.git
    GIT_TAG release-2.20.2
)
FetchContent_MakeAvailable(SDL2_ttf)

FetchContent_Declare(SDL2_mixer
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_mixer.git
    GIT_TAG release-2.6.3
)
SET(SDL2MIXER_OPUS OFF)
SET(SDL2MIXER_FLAC OFF)
SET(SDL2MIXER_MOD OFF)
SET(SDL2MIXER_MIDI_FLUIDSYNTH OFF)
FetchContent_MakeAvailable(SDL2_mixer)

FetchContent_Declare(box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v2.4.1
)
FetchContent_MakeAvailable(box2d)

FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 0.9.9.8
)
FetchContent_MakeAvailable(glm)

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
ExternalProject_Add(glew
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/_deps/glew"
    URL https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz
    BUILD_COMMAND ""
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
)
file(GLOB glew_src
     "${CMAKE_CURRENT_BINARY_DIR}/_deps/glew/src/glew/src/glew.c"
)

ExternalProject_Add(lua 
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua"
    URL https://www.lua.org/ftp/lua-5.4.7.tar.gz
    BUILD_COMMAND ""
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
)
file(GLOB lua_src
     "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua/src/lua/src/*.h"
     "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua/src/lua/src/*.c"
)
list(REMOVE_ITEM lua_src "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua/src/lua/src/luac.c" "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua/src/lua/src/lua.c")

FetchContent_Declare(LuaBridge
    GIT_REPOSITORY https://github.com/vinniefalco/LuaBridge.git
    GIT_TAG 2.7
)
FetchContent_MakeAvailable(LuaBridge)

FetchContent_Declare(RapidJSON 
    GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
    GIT_TAG v1.1.0
)
FetchContent_MakeAvailable(RapidJSON)


file(GLOB src
     "src/*.h"
     "src/*.cpp"
)

add_executable (game_engine ${glew_src} ${lua_src} ${src})
add_dependencies(game_engine lua)

target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua/src/lua/src")
target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/rapidjson-src/include")
target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/glew/src/glew/include")

target_link_libraries(game_engine PRIVATE SDL2::SDL2main)
target_link_libraries(game_engine PRIVATE SDL2::SDL2-static)
target_link_libraries(game_engine PRIVATE SDL2_ttf::SDL2_ttf-static)
target_link_libraries(game_engine PRIVATE SDL2_image::SDL2_image-static)
target_link_libraries(game_engine PRIVATE SDL2_mixer::SDL2_mixer-static)
target_link_libraries(game_engine PRIVATE box2d)
target_link_libraries(game_engine PRIVATE LuaBridge)
target_link_libraries(game_engine PRIVATE glm::glm)
target_link_libraries(game_engine PRIVATE Freetype::Freetype)
target_link_libraries(game_engine PRIVATE ${OPENGL_LIBRARY}) # filled by "find_package(OpenGL REQUIRED)" 