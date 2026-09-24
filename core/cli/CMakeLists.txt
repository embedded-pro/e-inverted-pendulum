add_library(inverted_pendulum_bot.cli ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.cli PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.cli PUBLIC
    inverted_pendulum_bot.motion_actuation.interfaces
    inverted_pendulum_bot.wheel_odometry.interfaces
    inverted_pendulum_bot.inertial_sensing.interfaces
    inverted_pendulum_bot.attitude_estimation.interfaces
    inverted_pendulum_bot.control_loop.interfaces
    inverted_pendulum_bot.safety_supervisor.interfaces
    inverted_pendulum_bot.platform_abstraction
    services.util
    services.peripheral
    services.tracer
    infra.timer
    infra.util
)

target_sources(inverted_pendulum_bot.cli PRIVATE
    Cli.hpp
    Cli.cpp
    DecimalParser.hpp
    DecimalParser.cpp
)

add_subdirectory(test)
