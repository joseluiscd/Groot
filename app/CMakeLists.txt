
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
#set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

add_executable(groot_exe
    application.hpp
    application.cpp
    
    create_graph.hpp
    create_graph.cpp
    graph_cluster.hpp
    graph_cluster.cpp
    #graph_view.hpp
    #graph_view.cpp

    open_graph.hpp
    open_graph.cpp
    save_graph.hpp
    save_graph.cpp

    editor.hpp
    editor_imgui.hpp
    editor.cpp

    #particle_sim.hpp
    #particle_sim.cpp

    lua.hpp
    lua.cpp
    
    viewer_system.cpp
    viewer_system.hpp

    graph_viewer_system.cpp
    graph_viewer_system.hpp
    
    main.cpp
)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/zep/include ${LUA_INCLUDE_DIR})
target_link_libraries(groot_exe
    groot
    gfx
    #jet-live
    Zep::Zep
    EnTT::EnTT
    ${LUA_LIBRARIES})
