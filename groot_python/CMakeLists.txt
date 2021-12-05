
Python3_add_library(pygroot
    MODULE

    python.hpp
    python.cpp

    python_imgui.hpp
    python_imgui.cpp

    python_task.hpp
    python_task.cpp

    python_graph.hpp
    python_graph.cpp
)

target_link_libraries(pygroot
    PUBLIC
    groot_base
    groot_graph
    groot_app
    
    Boost::python39
    Boost::numpy39
)