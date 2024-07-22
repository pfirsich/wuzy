cmake_minimum_required(VERSION 3.0.0)

project(wuzy)

# For conan
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
# list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

include(cmake/wall.cmake)

if(WUZY_ENABLE_ASAN)
  include(cmake/asan.cmake)
endif()

add_library(wuzy src/wuzy.cpp)
target_include_directories(wuzy PUBLIC include/)
set_wall(wuzy)

option(WUZY_BUILD_TESTS "Build the wuzy test executables" OFF)
if(WUZY_BUILD_TESTS)
  add_subdirectory(deps/glwrap)

  add_executable(wuzy-test "src/wuzy-test.cpp")
  target_link_libraries(wuzy-test wuzy)
  target_link_libraries(wuzy-test glwx)

  add_executable(wuzy-epa-debug "src/wuzy-epa-debug.cpp")
  target_link_libraries(wuzy-epa-debug wuzy)
  target_link_libraries(wuzy-epa-debug glwx)

  add_executable(wuzy-gjk-debug "src/wuzy-gjk-debug.cpp")
  target_link_libraries(wuzy-gjk-debug wuzy)
  target_link_libraries(wuzy-gjk-debug glwx)

  add_executable(wuzy-level "src/wuzy-level.cpp")
  target_link_libraries(wuzy-level wuzy)
  target_link_libraries(wuzy-level glwx)
endif()