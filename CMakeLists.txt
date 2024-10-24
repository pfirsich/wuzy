cmake_minimum_required(VERSION 3.0.0)

project(wuzy)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

include(cmake/wall.cmake)
include(cmake/no-rtti.cmake)
include(cmake/no-exceptions.cmake)

if(WUZY_ENABLE_ASAN)
  include(cmake/asan.cmake)
endif()
  
add_library(wuzy src/wuzy.cpp)
target_include_directories(wuzy PUBLIC include/)
set_wall(wuzy)
set_no_exceptions(wuzy)
set_no_rtti(wuzy)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  option(WUZY_BUILD_EXAMPLES "Build Examples" ON)

  if(WUZY_BUILD_EXAMPLES)
    add_subdirectory(deps/glwrap)
    add_subdirectory(examples)
  endif()

  #option(WUZY_BUILD_TESTS "Build tests" ON)
  #if(WUZY_BUILD_TESTS)
  #  add_subdirectory(tests)
  #endif()
endif()