
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

add_library(groot
    SHARED

    application.hpp
    application.cpp
    
    resources.hpp
    components.hpp
    components.cpp

    entity_editor.hpp

    import_ply.hpp
    import_ply.cpp

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

    cloud_system.hpp
    cloud_system.cpp

    cylinder_marching.hpp
    cylinder_marching.cpp
    
    viewer_system.cpp
    viewer_system.hpp

    graph_viewer_system.cpp
    graph_viewer_system.hpp

    testing.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/3rdparty/zep/include ${LUA_INCLUDE_DIR})
target_link_libraries(groot
    groot_base
    gfx
    EnTT::EnTT
    Python3::Python
)

if (CMAKE_BUILD_TYPE MATCHES Debug)
target_link_libraries(groot jet-live)
target_compile_definitions(groot PUBLIC HOT_CODE_RELOAD)
endif()

add_executable(groot_exe main.cpp)
target_link_libraries(groot_exe groot)