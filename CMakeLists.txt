cmake_minimum_required (VERSION 3.7)
project (Groot 
    LANGUAGES CXX
    VERSION 0.0.1
    )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_INLINES_HIDDEN ON)

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    )

include(GNUInstallDirs)

find_package(glfw3 3.3 REQUIRED)
find_package(Boost 1.75 REQUIRED COMPONENTS graph python39 numpy39)
find_package(TBB REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog 1.5 REQUIRED)
find_package(Eigen3 REQUIRED 3.3)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(CGAL 5.3 REQUIRED)


set(GFX_BUILD_EXAMPLES OFF CACHE BOOL "")
set(JUST_INSTALL_CEREAL ON CACHE BOOL "")
set(CEREAL_INSTALL ON CACHE BOOL "")
set(DOCTEST_WITH_MAIN_IN_STATIC_LIB  OFF CACHE BOOL "")
include(CMakePackageConfigHelpers)

add_subdirectory(3rdparty/gfx)
add_subdirectory(3rdparty/cereal)
add_subdirectory(3rdparty/doctest)
add_subdirectory(3rdparty/entt)
add_subdirectory(3rdparty/asyncplusplus)


include(CGAL_TBB_support)
include(CGAL_Eigen_support)

file(GLOB SOURCES 
    include/groot/*.hpp
    include/groot/*.h
    src/*.cpp
)
file(GLOB PUBLIC_HEADERS
    include/groot/*.hpp
    include/groot/*.h
)

add_library(groot_base
    SHARED
    ${SOURCES}
)

target_link_libraries(groot_base
    CGAL::CGAL
    CGAL::TBB_support
    CGAL::Eigen_support
    spdlog::spdlog
    Boost::boost
    cereal::cereal
    doctest
)

target_include_directories(groot_base
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_definitions(groot_base PUBLIC GROOT_DLL PRIVATE GROOT_EXPORTS_DLL)
set_target_properties(groot_base PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(groot_base PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")

add_subdirectory(groot_graph)
add_subdirectory(groot_app)
add_subdirectory(groot_python)


install(TARGETS
    cereal
    doctest
    groot_base

    EXPORT GrootTargets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/groot)

export(EXPORT GrootTargets)
install(EXPORT GrootTargets DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
