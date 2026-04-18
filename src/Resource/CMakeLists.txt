add_library(Resource STATIC
    src/ResourceManager.cpp
)

target_include_directories(Resource PUBLIC include)
target_include_directories(Resource SYSTEM PRIVATE "${CMAKE_BINARY_DIR}/_deps/rapidjson-src/include")

target_link_libraries(Resource
    PUBLIC
        Core
)
