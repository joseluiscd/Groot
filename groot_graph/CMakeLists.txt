
file(GLOB SOURCES 
    include/groot_graph/*.hpp
    include/groot_graph/*.h
    src/*.cpp
)

add_library(groot_graph
    SHARED
    ${SOURCES}
)

target_link_libraries(
    groot_graph
    groot_base
)
target_compile_definitions(groot_graph
    PUBLIC GROOT_DLL
    PRIVATE GROOT_EXPORTS_DLL
)

add_executable(groot_graph_test
    tests.cpp
)
target_link_libraries(groot_graph_test groot_graph)

target_include_directories(groot_graph
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
set_target_properties(groot_graph PROPERTIES CXX_VISIBILITY_PRESET hidden)