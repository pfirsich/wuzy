add_executable(wuzy-tests
    test_capsule.cpp
    test_gjk_toi.cpp
)
target_link_libraries(wuzy-tests wuzy)
target_include_directories(wuzy-tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
