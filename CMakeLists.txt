cmake_minimum_required (VERSION 3.7)
project (Groot 
    LANGUAGES CXX
    VERSION 0.0.1
    )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    )


find_package(Boost 1.69 REQUIRED COMPONENTS graph serialization python39 numpy39)
find_package(CGAL 5 REQUIRED)
find_package(TBB REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog 1.5 REQUIRED)
find_package(Eigen3 REQUIRED 3.3)
find_package(Python3 REQUIRED COMPONENTS Development)

include(CGAL_TBB_support)
include(CGAL_Eigen_support)

set(GFX_BUILD_EXAMPLES OFF)
set(BUILD_DEMOS OFF CACHE BOOL "enable Zep demos")
set(BUILD_EXTENSIONS OFF CACHE BOOL "enable Zep extensions")
include(CMakePackageConfigHelpers)

add_subdirectory(3rdparty/glfw)
add_subdirectory(3rdparty/gfx)
add_subdirectory(3rdparty/doctest)
add_subdirectory(3rdparty/entt)

file(GLOB SOURCES 
    include/groot/*.hpp
    include/groot/*.h
    src/*.cpp
)

add_library(groot_base
    OBJECT
    ${SOURCES}
)

target_link_libraries(groot_base
    CGAL::CGAL
    CGAL::TBB_support
    CGAL::Eigen_support
    spdlog::spdlog
    Boost::boost
    Boost::serialization
    Boost::python39
    Boost::numpy39
    doctest
    gfx
)

target_include_directories(groot_base
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/util
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eventpp/include
)

add_subdirectory(app)
