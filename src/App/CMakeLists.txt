add_executable(game_engine
    src/main.cpp
)

if (MSVC)
    target_compile_options(game_engine PRIVATE /W4 /EHsc)
else()
    target_compile_options(game_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(game_engine
    PRIVATE
        SDL2::SDL2main
        Audio
        Graphics
        Input
        Network
        Resource
        Scene
        Scripting
)

add_custom_command(TARGET game_engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources" "$<TARGET_FILE_DIR:game_engine>/resources"
    COMMENT "Copying resources directory to build directory"
)

add_executable(client
    src/Client.cpp
)

target_link_libraries(client
    PRIVATE
        Network
)
