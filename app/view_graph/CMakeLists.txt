
add_executable(view_graph
    imfilebrowser.h
    create_graph.hpp
    create_graph.cpp
    graph_view.hpp
    graph_view.cpp
    main.cpp
)
target_link_libraries(view_graph groot gfx)