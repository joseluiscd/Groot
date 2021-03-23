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

set(Boost_USE_STATIC_LIBS ON)

find_package(CGAL 5 REQUIRED)
find_package(TBB REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog 1.8 REQUIRED)
find_package(Boost 1.73 REQUIRED COMPONENTS graph serialization)
find_package(Lua 5.4 REQUIRED)
find_package(Eigen3 REQUIRED 3.3)

include(CGAL_TBB_support)
include(CGAL_Eigen_support)

set(GFX_BUILD_EXAMPLES OFF)
set(BUILD_DEMOS OFF CACHE BOOL "enable Zep demos")
set(BUILD_EXTENSIONS OFF CACHE BOOL "enable Zep extensions")
include(CMakePackageConfigHelpers)

add_subdirectory(3rdparty/gfx)
add_subdirectory(3rdparty/zep)
add_subdirectory(3rdparty/doctest)

#include(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jet-live/cmake/jet_live_setup.cmake) # setup needed compiler and linker flags, include this file in your root CMakeLists.txt
#set(JET_LIVE_BUILD_EXAMPLE OFF)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jet-live)

file(GLOB SOURCES 
    include/groot/*.hpp
    include/groot/*.h
    src/*.cpp
)

add_library(groot
    OBJECT
    ${SOURCES}
)

target_link_libraries(groot
    glm
    CGAL::CGAL
    CGAL::TBB_support
    CGAL::Eigen_support
    spdlog::spdlog
    Boost::boost
    Boost::serialization
    doctest
)

target_include_directories(groot
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/util
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/eventpp/include
)

add_subdirectory(app)

add_executable(test_exe testing.cpp)
target_link_libraries(test_exe groot doctest_with_main)
