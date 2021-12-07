
file(GLOB SOURCES 
    include/groot_graph/*.hpp
    include/groot_graph/*.h
    src/*.cpp
)

file(GLOB HEADERS
    include/groot_graph/*.hpp
    include/groot_graph/*.h
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(groot_graph PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(groot_graph PROPERTIES PUBLIC_HEADER "${HEADERS}")

install(TARGETS
    groot_graph
    EXPORT GrootTargets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/groot_graph)
