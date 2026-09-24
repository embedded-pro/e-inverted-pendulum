add_library(inverted_pendulum_bot.telemetry.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.telemetry.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.telemetry.implementations PUBLIC
    inverted_pendulum_bot.telemetry.interfaces
    inverted_pendulum_bot.balance_control.interfaces
    inverted_pendulum_bot.control_loop.interfaces
    inverted_pendulum_bot.wheel_odometry.interfaces
)

target_sources(inverted_pendulum_bot.telemetry.implementations PRIVATE
    TelemetryRecorder.hpp
    TelemetryRecorder.cpp
)

add_subdirectory(test)
