add_library(Audio STATIC
    src/AudioManager.cpp
)

target_include_directories(Audio PUBLIC include)

target_link_libraries(Audio
    PUBLIC
        Core
        SDL2_mixer::SDL2_mixer-static
)
