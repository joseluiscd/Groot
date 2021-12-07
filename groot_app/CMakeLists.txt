
file(GLOB SOURCES 
    include/groot_app/*.hpp
    include/groot_app/*.h
    src/*.cpp
)

add_library(groot_app
    SHARED
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
target_compile_definitions(groot_app PUBLIC GROOT_DLL GROOT_EXPORTS_DLL)
set_target_properties(groot_app PROPERTIES CXX_VISIBILITY_PRESET hidden)

add_executable(groot_app_test main_test.cpp)
target_link_libraries(groot_app_test groot_app)

add_executable(groot main.cpp)
target_link_libraries(groot groot_app)
set_target_properties(groot PROPERTIES CXX_VISIBILITY_PRESET hidden)