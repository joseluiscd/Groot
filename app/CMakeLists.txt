
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
    save_graph.hpp
    save_graph.cpp
    
    editor.hpp
    editor_imgui.hpp
    editor.cpp

    lua.hpp
    lua.cpp
    lua_cmd.hpp
    lua_cmd.cpp
    
    main.cpp
)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/zep/include ${LUA_INCLUDE_DIR})
target_link_libraries(groot_exe groot gfx jet-live Zep::Zep ${LUA_LIBRARIES})