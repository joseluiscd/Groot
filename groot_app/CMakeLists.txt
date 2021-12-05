
file(GLOB SOURCES 
    include/groot_app/*.hpp
    include/groot_app/*.h
    src/*.cpp
)

add_library(groot_app
    STATIC
    ${SOURCES}
)

target_link_libraries(
    groot_app
    PUBLIC

    groot_graph
    groot_base

    gfx
    Async++
    EnTT::EnTT
)

target_include_directories(groot_app
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(groot_app_test main_test.cpp)
target_link_libraries(groot_app_test groot_app)

add_executable(groot main.cpp)
target_link_libraries(groot groot_app)