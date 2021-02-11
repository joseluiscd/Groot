cmake_minimum_required (VERSION 3.7)
project (Groot 
    LANGUAGES CXX
    VERSION 0.0.1
    )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    )


find_package(CGAL 4.14 REQUIRED)
find_package(PCL 1.11 REQUIRED COMPONENTS common io kdtree)
find_package(LEMON REQUIRED) # 1.3
find_package(glm REQUIRED)
find_package(spdlog 1.8 REQUIRED)

add_subdirectory(gfx)

file(GLOB SOURCES 
    include/groot/*.hpp
    include/groot/*.h
    src/*.cpp
)

add_library(groot
    ${SOURCES}
)

#target_link_directories(groot ${PCL_LIBRARY_DIRS})
#target_include_directories(groot ${PCL_INCLUDE_DIRS})
target_link_libraries(groot 
    ${PCL_LIBRARIES} 
    ${LEMON_LIBRARY}
    glm
    CGAL::CGAL 
    spdlog::spdlog
)
target_include_directories(groot
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/util
)

add_subdirectory(app)