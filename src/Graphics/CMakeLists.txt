add_library(Graphics STATIC
    src/GraphicsManager.cpp
    src/Shader.cpp
    src/SpriteRenderer.cpp
    src/Texture.cpp
)

target_include_directories(Graphics PUBLIC include)
target_include_directories(Graphics SYSTEM PUBLIC "${CMAKE_BINARY_DIR}/_deps/glew-src/include")

target_link_libraries(Graphics
    PUBLIC
        Core
        glew_s
        OpenGL::OpenGL
        glm::glm
    PRIVATE
        SDL2_ttf::SDL2_ttf-static
        freetype
        Tracy::TracyClient
)
