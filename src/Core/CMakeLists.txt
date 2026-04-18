add_library(Core INTERFACE)

target_include_directories(Core INTERFACE include)

target_link_libraries(Core INTERFACE
    SDL2::SDL2-static
    SDL2_image::SDL2_image-static
    quill::quill
)
