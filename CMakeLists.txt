cmake_minimum_required(VERSION 3.21)
set (CMAKE_CXX_STANDARD 23)


project ("GameEngine" C CXX)
set(BUILD_SHARED_LIBS OFF)

# Required for glew
find_package(OpenGL REQUIRED COMPONENTS OpenGL)

include(FetchContent)
include(ExternalProject)

# FetchContent all dependencies
add_subdirectory(cmake)


######### Game Engine #########

file(GLOB src 
"src/*.cpp")
list(REMOVE_ITEM src "${CMAKE_CURRENT_SOURCE_DIR}/src/Client.cpp")

add_executable (game_engine ${src})
if (MSVC)
    target_compile_options(game_engine PRIVATE /W4 /EHsc)
else()
    target_compile_options(game_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_custom_command(TARGET game_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources" "$<TARGET_FILE_DIR:game_engine>/resources"
    COMMENT "Copying resources directory to build directory"
)

target_include_directories(game_engine SYSTEM PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src")
target_include_directories(game_engine SYSTEM PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/_deps/glew-src/include")
target_include_directories(game_engine SYSTEM PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/_deps/rapidjson-src/include")

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
target_link_libraries(game_engine PRIVATE OpenGL::OpenGL)
target_link_libraries(game_engine PRIVATE Boost::asio)
target_link_libraries(game_engine PRIVATE quill::quill)
target_link_libraries(game_engine PRIVATE glew_s)

target_link_libraries(game_engine PRIVATE Tracy::TracyClient)

######### Network Client #########

add_executable(client "src/Client.cpp" "src/NetworkMessage.cpp" "src/NetworkManager.cpp")
target_link_libraries(client PRIVATE quill::quill Boost::asio)