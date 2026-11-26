add_library(inverted_pendulum_bot.calibration.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.calibration.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.calibration.implementations PUBLIC
    inverted_pendulum_bot.calibration.interfaces
    inverted_pendulum_bot.attitude_estimation.interfaces
    inverted_pendulum_bot.balance_control.interfaces
    inverted_pendulum_bot.inertial_sensing.interfaces
    inverted_pendulum_bot.persistence.implementations
    inverted_pendulum_bot.safety_supervisor.interfaces
    hal.interfaces
    infra.event
    services.util
)

target_sources(inverted_pendulum_bot.calibration.implementations PRIVATE
    CalibrationImpl.hpp
    CalibrationImpl.cpp
)

add_subdirectory(test)
