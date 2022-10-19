cmake_minimum_required(VERSION 3.0.0)

project(exquisite)

# For conan
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
# list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

include(cmake/wall.cmake)

if(EXQUISITE_ENABLE_ASAN)
  include(cmake/asan.cmake)
endif()

set(SRC
  src/wuzy.cpp
)

add_library(wuzy ${SRC})
target_include_directories(wuzy PUBLIC include/)
set_wall(wuzy)

add_executable(wuzy-test "src/wuzy-test.cpp")
target_link_libraries(wuzy-test wuzy)
