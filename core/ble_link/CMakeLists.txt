add_library(inverted_pendulum_bot.ble_link ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.ble_link PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.ble_link PUBLIC
    inverted_pendulum_bot.platform_abstraction
    inverted_pendulum_bot.safety_supervisor.interfaces
    inverted_pendulum_bot.balance_control.interfaces
    inverted_pendulum_bot.telemetry.interfaces
    services.ble
    infra.timer
)

target_sources(inverted_pendulum_bot.ble_link PRIVATE
    BleLink.hpp
    BleLink.cpp
    RobotControlService.hpp
    RobotControlService.cpp
    WireFormat.hpp
    WireFormat.cpp
)

add_subdirectory(test)
