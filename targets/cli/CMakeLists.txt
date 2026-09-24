add_executable(inverted_pendulum_bot.cli.main)

target_include_directories(inverted_pendulum_bot.cli.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.cli.main PUBLIC
    inverted_pendulum_bot.platform_impl
    inverted_pendulum_bot.cli
    inverted_pendulum_bot.motion_actuation.implementations
    inverted_pendulum_bot.wheel_odometry.implementations
    inverted_pendulum_bot.inertial_sensing.implementations
    inverted_pendulum_bot.attitude_estimation.implementations
    inverted_pendulum_bot.control_loop.implementations
)

target_sources(inverted_pendulum_bot.cli.main PRIVATE
    Main.cpp
)

# Embedded targets need startup code, a linker script, and firmware artifacts.
# The host build links as a normal executable.
if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(inverted_pendulum_bot.cli.main)
    halst_target_bringup(inverted_pendulum_bot.cli.main)
    emil_generate_artifacts(TARGET inverted_pendulum_bot.cli.main LST MAP BIN HEX)
endif()
