FetchContent_Declare(glew
    URL https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz
    SOURCE_SUBDIR "build/cmake"
)
FetchContent_MakeAvailable(glew)