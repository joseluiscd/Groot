project(Lua VERSION 5.4 LANGUAGES C)
file(GLOB LUA_SOURCES *.c *.h *.hpp)

add_library(lua STATIC ${LUA_SOURCES})
target_include_directories(lua PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})