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

# Engine components and executables
add_subdirectory(src)
