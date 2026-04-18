add_library(Scene STATIC
    src/SceneManager.cpp
)

target_include_directories(Scene PUBLIC include)
target_include_directories(Scene SYSTEM PUBLIC "${CMAKE_BINARY_DIR}/_deps/rapidjson-src/include")

target_link_libraries(Scene
    PUBLIC
        Core
        Graphics
        glm::glm
        box2d
    PRIVATE
        Audio
        Input
        Network
)
