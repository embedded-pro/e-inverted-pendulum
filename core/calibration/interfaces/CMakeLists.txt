add_library(inverted_pendulum_bot.calibration.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.calibration.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.calibration.interfaces INTERFACE
    inverted_pendulum_bot.platform_abstraction
    inverted_pendulum_bot.motion_actuation.interfaces
    inverted_pendulum_bot.wheel_odometry.interfaces
)

target_sources(inverted_pendulum_bot.calibration.interfaces PRIVATE
    Calibration.hpp
)
