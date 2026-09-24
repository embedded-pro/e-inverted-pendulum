add_library(inverted_pendulum_bot.platform_host ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.platform_host PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.platform_host PUBLIC
    inverted_pendulum_bot.platform_abstraction
    hal.generic
    infra.timer
    services.peripheral
    services.tracer
    infra.event
    infra.stream
)

target_sources(inverted_pendulum_bot.platform_host PRIVATE
    GpioStub.hpp
    InertialSensorStub.hpp
    Drv8711Emulator.hpp
    MotorDriverStub.hpp
    WheelEncoderStub.hpp
    WheelEncodersStub.hpp
    PlatformImpl.hpp
    PlatformImpl.cpp
)
