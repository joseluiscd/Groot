file(GLOB SOURCES 
    *.cpp
    *.hpp
)

Python3_add_library(pygroot
    MODULE

    ${SOURCES}
)
target_compile_definitions(pygroot
    PUBLIC
    GROOT_DLL

    PRIVATE
    GROOT_EXPORTS_DLL
)
target_link_libraries(pygroot
    PUBLIC
    groot_base
    groot_graph
    groot_app
    
    Boost::python39
    Boost::numpy39
)
set_target_properties(pygroot PROPERTIES CXX_VISIBILITY_PRESET hidden)