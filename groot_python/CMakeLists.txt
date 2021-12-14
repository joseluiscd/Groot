file(GLOB SOURCES 
    *.cpp
    *.hpp
)

add_library(pygroot_lib
    OBJECT

    ${SOURCES}
)
target_compile_definitions(pygroot_lib
    PUBLIC
    GROOT_DLL

    PRIVATE
    GROOT_EXPORTS_DLL
)
target_link_libraries(pygroot_lib
    PUBLIC
    groot_base
    groot_graph
    groot_app
    
    Boost::python39
    Boost::numpy39
    Python3::Module
)
target_include_directories(pygroot_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(pygroot_lib PROPERTIES CXX_VISIBILITY_PRESET hidden)


Python3_add_library(pygroot 
    MODULE
)
target_link_libraries(pygroot PUBLIC pygroot_lib)


file(GLOB TEST_SOURCE tests/*.cpp)
add_executable(pygroot_test ${TEST_SOURCE})
target_link_libraries(pygroot_test PUBLIC pygroot_lib doctest::doctest Python3::Python)

add_executable(groot_python3 interpreter/interpreter.cpp)
target_link_libraries(groot_python3 PUBLIC pygroot_lib Python3::Python)
