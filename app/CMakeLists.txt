
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

    render.hpp
    render.cpp

    viewer_system.hpp
    viewer_system.cpp

    cloud_system.hpp
    cloud_system.cpp

    cloud_io_system.hpp
    cloud_io_system.cpp

    testing.cpp
)

target_link_libraries(groot
    groot_base
    Python3::Python
    bait
)

add_executable(groot_exe main.cpp)
target_link_libraries(groot_exe groot)