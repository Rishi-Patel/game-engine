FetchContent_Declare(SDL2IMAGE
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_image.git
    GIT_TAG release-2.6.3 
)
SET(SDL2IMAGE_AVIF OFF)
set(SDL2IMAGE_INSTALL OFF)
set(BUILD_SHARED_LIBS FALSE)
FetchContent_MakeAvailable(SDL2IMAGE)