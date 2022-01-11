
file(GLOB SOURCES 
    include/groot_app/*.hpp
    include/groot_app/*.h
    src/*.cpp
)

file(GLOB HEADERS
    include/groot_app/*.hpp
    include/groot_app/*.h
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_definitions(groot_app PUBLIC GROOT_APP_DLL PRIVATE GROOT_APP_DLL_EXPORTS)
set_target_properties(groot_app PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(groot_app PROPERTIES PUBLIC_HEADER "${HEADERS}")

add_executable(groot_app_test main_test.cpp)
target_link_libraries(groot_app_test groot_app)

add_executable(groot main.cpp)
target_link_libraries(groot groot_app)
set_target_properties(groot PROPERTIES CXX_VISIBILITY_PRESET hidden)

install(TARGETS
    groot_app
    groot

    EXPORT GrootTargets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/groot_app)
