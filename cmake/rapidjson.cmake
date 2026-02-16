FetchContent_Declare(RapidJSON 
    GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
    GIT_TAG v1.1.0
)
option(RAPIDJSON_BUILD_DOC "" OFF)
option(RAPIDJSON_BUILD_EXAMPLES "" OFF)
option(RAPIDJSON_BUILD_TESTS "" OFF)
FetchContent_MakeAvailable(RapidJSON)