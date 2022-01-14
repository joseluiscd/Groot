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
target_include_directories(pygroot_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(pygroot_lib PROPERTIES CXX_VISIBILITY_PRESET hidden)


Python3_add_library(pygroot 
    MODULE
)
target_link_libraries(pygroot PUBLIC pygroot_lib)

add_executable(groot_python3 interpreter/interpreter.cpp)
target_link_libraries(groot_python3 PUBLIC pygroot_lib pybind11::embed)

install(TARGETS
	groot_python3
    pygroot
    pygroot_lib

    EXPORT GrootTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
    )


