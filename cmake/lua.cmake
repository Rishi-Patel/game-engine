FetchContent_Declare(lua
    URL https://www.lua.org/ftp/lua-5.4.7.tar.gz
)
FetchContent_MakeAvailable(lua)
file(GLOB lua_src
     "${CMAKE_CURRENT_BINARY_DIR}/../_deps/lua-src/src/*.h"
     "${CMAKE_CURRENT_BINARY_DIR}/../_deps/lua-src/src/*.c"
)
list(REMOVE_ITEM lua_src "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/luac.c" "${CMAKE_CURRENT_BINARY_DIR}/_deps/lua-src/src/lua.c")
add_library(lua STATIC ${lua_src})


FetchContent_Declare(LuaBridge
    GIT_REPOSITORY https://github.com/vinniefalco/LuaBridge.git
    GIT_TAG 2.7
)
FetchContent_MakeAvailable(LuaBridge)