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

FetchContent_Declare(glew
    URL https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz
    SOURCE_SUBDIR "build/cmake"
)
FetchContent_MakeAvailable(glew)

FetchContent_Declare(lua
    URL https://www.lua.org/ftp/lua-5.4.7.tar.gz
)
FetchContent_MakeAvailable(lua)
file(GLOB lua_src
     "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/*.h"
     "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/*.c"
)
list(REMOVE_ITEM lua_src "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/luac.c" "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/lua.c")
add_library(lua STATIC ${lua_src})


FetchContent_Declare(LuaBridge
    GIT_REPOSITORY https://github.com/vinniefalco/LuaBridge.git
    GIT_TAG 2.7
)
FetchContent_MakeAvailable(LuaBridge)

FetchContent_Declare(RapidJSON 
    GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
    GIT_TAG v1.1.0
)
option(RAPIDJSON_BUILD_DOC "" OFF)
option(RAPIDJSON_BUILD_EXAMPLES "" OFF)
option(RAPIDJSON_BUILD_TESTS "" OFF)
FetchContent_MakeAvailable(RapidJSON)

FetchContent_Declare (
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
# available options : TRACY_ENABLE , TRACY_ON_DEMAND , TRACY_NO_BROADCAST , TRACY_NO_CODE_TRANSFER , ...
option(TRACY_ENABLE "" ON)
option(TRACY_ON_DEMAND "" ON)
FetchContent_MakeAvailable(tracy)

FetchContent_Declare(
  Boost
  GIT_REPOSITORY https://github.com/boostorg/boost.git
  GIT_TAG boost-1.88.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(Boost)
if(MSVC)
    add_compile_options(/EHsc)
endif(MSVC)

if(NOT TARGET freetype)
    FetchContent_Declare(
      freetype
      GIT_REPOSITORY https://gitlab.freedesktop.org/freetype/freetype
      GIT_TAG VER-2-13-3
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(freetype)
endif()

FetchContent_Declare(Quill
    GIT_REPOSITORY https://github.com/odygrd/quill.git
    GIT_TAG v10.0.0
)
FetchContent_MakeAvailable(Quill)

file(GLOB src
     "src/*.cpp"
)

list(REMOVE_ITEM src "${CMAKE_CURRENT_SOURCE_DIR}/src/Client.cpp")

add_executable (game_engine ${src})
add_custom_command(TARGET game_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources" "$<TARGET_FILE_DIR:game_engine>/resources"
    COMMENT "Copying resources directory to build directory"
)

target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src")
target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/glew-src/include")
target_include_directories(game_engine PUBLIC "${CMAKE_CURRENT_BINARY_DIR}/_deps/rapidjson-src/include")

target_link_libraries(game_engine PRIVATE SDL2::SDL2main)
target_link_libraries(game_engine PRIVATE SDL2::SDL2-static)
target_link_libraries(game_engine PRIVATE SDL2_ttf::SDL2_ttf-static)
target_link_libraries(game_engine PRIVATE SDL2_image::SDL2_image-static)
target_link_libraries(game_engine PRIVATE SDL2_mixer::SDL2_mixer-static)
target_link_libraries(game_engine PRIVATE box2d)
target_link_libraries(game_engine PRIVATE lua)
target_link_libraries(game_engine PRIVATE LuaBridge)
target_link_libraries(game_engine PRIVATE glm::glm)
target_link_libraries(game_engine PRIVATE freetype)
target_link_libraries(game_engine PRIVATE ${OPENGL_LIBRARY}) # filled by "find_package(OpenGL REQUIRED)" 
target_link_libraries(game_engine PRIVATE Tracy::TracyClient) 
target_link_libraries(game_engine PRIVATE Boost::asio)
target_link_libraries(game_engine PRIVATE quill::quill)
target_link_libraries(game_engine PRIVATE glew_s)

add_executable(client "src/Client.cpp" "src/NetworkMessage.cpp")
target_link_libraries(client PRIVATE Boost::asio)