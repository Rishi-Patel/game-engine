add_library(Network STATIC
    src/NetworkManager.cpp
    src/NetworkMessage.cpp
)

target_include_directories(Network PUBLIC include)

target_link_libraries(Network
    PUBLIC
        quill::quill
        Boost::asio
)
