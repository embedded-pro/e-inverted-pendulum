add_library(inverted_pendulum_bot.motion_actuation.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.motion_actuation.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.motion_actuation.implementations PUBLIC
    inverted_pendulum_bot.motion_actuation.interfaces
    inverted_pendulum_bot.platform_abstraction
    infra.timer
    drivers.motor_controller
)

target_sources(inverted_pendulum_bot.motion_actuation.implementations PRIVATE
    BridgeMapping.hpp
    BridgeMapping.cpp
    Drv8711DriverConfiguration.hpp
    Drv8711DriverConfiguration.cpp
    MotionActuationImpl.hpp
    MotionActuationImpl.cpp
)

add_subdirectory(test)
