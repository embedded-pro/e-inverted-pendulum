add_subdirectory(interfaces)
add_subdirectory(implementations)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    add_subdirectory(test_doubles)
endif()
