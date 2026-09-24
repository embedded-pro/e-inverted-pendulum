add_library(inverted_pendulum_bot.platform_abstraction ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.platform_abstraction PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.platform_abstraction PUBLIC
    hal.interfaces
    infra.timer
    hal.synchronous_interfaces
    services.tracer
)

target_sources(inverted_pendulum_bot.platform_abstraction PRIVATE
    InertialFrame.hpp
    InertialFrame.cpp
    InertialSensor.hpp
    MotorDriver.hpp
    Platform.hpp
    WheelEncoders.hpp
)

add_subdirectory(test)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    add_subdirectory(test_doubles)
endif()
