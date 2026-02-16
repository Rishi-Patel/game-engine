FetchContent_Declare(
    freetype
    GIT_REPOSITORY https://gitlab.freedesktop.org/freetype/freetype
    GIT_TAG VER-2-13-3
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(freetype)

# Unfortunately SLD2_ttf uses the old way to get FreeType so cant use FetchContent.
# Instead we must provide FREETYPE_LIBRARY and FREETYPE_INCLUDE_DIRS variables for it to find FreeType.

# Figure out of this is shared or static
if(BUILD_SHARED_LIBS)
    set(FREETYPE_LIB_SUFFIX "${CMAKE_SHARED_LIBRARY_SUFFIX}")
else()
    set(FREETYPE_LIB_SUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")
endif()

# handle debug/release names (freetype adds a 'd' to the library name)
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(FREETYPE_DEBUG_SUFFIX "d") 
else()
    set(FREETYPE_DEBUG_SUFFIX "")
endif()

set(FREETYPE_LIBRARY ${freetype_BINARY_DIR}/libfreetype${FREETYPE_DEBUG_SUFFIX}${FREETYPE_LIB_SUFFIX})
set(FREETYPE_INCLUDE_DIRS ${freetype_SOURCE_DIR}/include)

FetchContent_Declare(SDL2_ttf
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_ttf.git
    GIT_TAG release-2.20.2
)
FetchContent_MakeAvailable(SDL2_ttf)
