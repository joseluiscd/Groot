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
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(USE_MOLD_LINKER OFF CACHE BOOL "Use mold linker in Linux")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic -Werror=pessimizing-move -Werror=return-type")

    if (USE_MOLD_LINKER)
        add_link_options("-fuse-ld=mold")
    endif()
endif()

set(CMAKE_MODULE_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
    )

include(GNUInstallDirs)

find_package(glfw3 3.3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(Boost 1.75 REQUIRED COMPONENTS graph)
find_package(TBB REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog 1.5 REQUIRED)
find_package(Eigen3 REQUIRED 3.3)
find_package(CGAL 5.3 REQUIRED)


set(GFX_BUILD_EXAMPLES OFF CACHE BOOL "")
set(JUST_INSTALL_CEREAL ON CACHE BOOL "")
set(CEREAL_INSTALL ON CACHE BOOL "")
set(DOCTEST_WITH_MAIN_IN_STATIC_LIB  OFF CACHE BOOL "")
set(SUITESPARSE OFF CACHE BOOL "")
set(MINIGLOG ON CACHE BOOL "")
include(CMakePackageConfigHelpers)

add_subdirectory(3rdparty/gfx)
add_subdirectory(3rdparty/cereal)
add_subdirectory(3rdparty/doctest)
add_subdirectory(3rdparty/entt)
add_subdirectory(3rdparty/asyncplusplus)
add_subdirectory(3rdparty/pybind11)
add_subdirectory(3rdparty/ceres-solver)


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
target_compile_definitions(groot_base PUBLIC GROOT_DLL PRIVATE GROOT_DLL_EXPORTS)
set_target_properties(groot_base PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(groot_base PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")

add_subdirectory(groot_graph)
add_subdirectory(groot_app)
add_subdirectory(groot_python)


install(TARGETS
    cereal
    doctest
    groot_base
    ceres

    EXPORT GrootTargets
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/groot)

export(EXPORT GrootTargets)
install(EXPORT GrootTargets DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
