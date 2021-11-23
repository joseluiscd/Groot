
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

add_library(groot
    SHARED

    app_log.hpp
    app_log.cpp

    application.hpp
    application.cpp

    task.hpp
    task.cpp

    command_gui.hpp
    command_gui.cpp
    
    resources.hpp
    components.hpp
    components.cpp

    entity_editor.hpp

    cloud_io.hpp
    cloud_io.cpp

    graph_io.hpp
    graph_io.cpp

    serde.hpp

    create_graph.hpp
    create_graph.cpp
    graph_cluster.hpp
    graph_cluster.cpp

    open_workspace.hpp
    open_workspace.cpp
    save_workspace.hpp
    save_workspace.cpp

    render.hpp
    render.cpp

    python.hpp
    python.cpp

    python_imgui.hpp
    python_imgui.cpp

    cloud_system.hpp
    cloud_system.cpp

    cylinder_marching.hpp
    cylinder_marching.cpp
    
    cylinder_connect.hpp
    cylinder_connect.cpp

    graph_resample.hpp
    graph_resample.cpp

    viewer_system.cpp
    viewer_system.hpp

    graph_viewer_system.cpp
    graph_viewer_system.hpp

    testing.cpp
)

target_link_libraries(groot
    groot_base
    gfx
    EnTT::EnTT
    Python3::Python
    Async++
)

add_executable(groot_test main_test.cpp)
target_link_libraries(groot_test groot)

add_executable(groot_run main.cpp)
target_link_libraries(groot_run groot)