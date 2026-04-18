add_library(Input STATIC
    src/InputManager.cpp
)

target_include_directories(Input PUBLIC include)

target_link_libraries(Input
    PUBLIC
        Core
        glm::glm
)
