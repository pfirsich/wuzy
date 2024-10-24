find_package(fmt REQUIRED)

add_executable(wuzy-test "wuzy-test.cpp")
target_link_libraries(wuzy-test wuzy glwx)

add_executable(wuzy-epa-debug "wuzy-epa-debug.cpp")
target_link_libraries(wuzy-epa-debug wuzy glwx fmt::fmt)

add_executable(wuzy-gjk-debug "wuzy-gjk-debug.cpp")
target_link_libraries(wuzy-gjk-debug wuzy glwx fmt::fmt)

add_executable(wuzy-level "wuzy-level.cpp")
target_link_libraries(wuzy-level wuzy glwx fmt::fmt)