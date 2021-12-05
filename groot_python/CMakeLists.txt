file(GLOB SOURCES 
    *.cpp
    *.hpp
)

Python3_add_library(pygroot
    MODULE

    ${SOURCES}
)

target_link_libraries(pygroot
    PUBLIC
    groot_base
    groot_graph
    groot_app
    
    Boost::python39
    Boost::numpy39
)