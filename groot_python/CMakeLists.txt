file(GLOB SOURCES 
    *.cpp
    *.hpp
)

add_library(pygroot_lib
    OBJECT

    ${SOURCES}
)

target_link_libraries(pygroot_lib
    PUBLIC
    groot_base
    groot_graph
    groot_app
   
    pybind11::pybind11
)
target_include_directories(pygroot_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(pygroot_lib PROPERTIES CXX_VISIBILITY_PRESET hidden)


Python3_add_library(pygroot 
    MODULE
)
target_link_libraries(pygroot PUBLIC pygroot_lib)


file(GLOB TEST_SOURCE tests/*.cpp)
add_executable(pygroot_test ${TEST_SOURCE})
target_link_libraries(pygroot_test PUBLIC pygroot_lib doctest::doctest pybind11::embed)

add_executable(groot_python3 interpreter/interpreter.cpp)
target_link_libraries(groot_python3 PUBLIC pygroot_lib pybind11::embed)
