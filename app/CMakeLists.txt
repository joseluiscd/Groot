
add_executable(groot_exe
    application.hpp
    application.cpp
    
    create_graph.hpp
    create_graph.cpp
    graph_cluster.hpp
    graph_cluster.cpp
    graph_view.hpp
    graph_view.cpp
    open_graph.hpp
    open_graph.cpp
    main.cpp
)
target_link_libraries(groot_exe groot gfx jet-live)