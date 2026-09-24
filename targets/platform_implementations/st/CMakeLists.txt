add_library(inverted_pendulum_bot.platform_st ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.platform_st PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.platform_st PUBLIC
    inverted_pendulum_bot.platform_abstraction
    hal_st.instantiations
    drivers.imu.mpu9250
    hal_st.synchronous_stm32fxxx
    infra.timer
    services.peripheral
    services.tracer
)

# Select the board-specific default clock configuration. Each supported ST board
# preset (see CMakePresets.json) sets TARGET_MCU, which maps to the matching
# hal_st clock header/init function.
set(INVERTED_PENDULUM_BOT_ST_SUPPORTED_MCUS stm32wb55)
if(NOT TARGET_MCU IN_LIST INVERTED_PENDULUM_BOT_ST_SUPPORTED_MCUS)
    message(FATAL_ERROR "Unsupported TARGET_MCU '${TARGET_MCU}' for inverted_pendulum_bot.platform_st. "
        "Supported values: ${INVERTED_PENDULUM_BOT_ST_SUPPORTED_MCUS}. Add a case in "
        "targets/platform_implementations/st/CMakeLists.txt for new boards.")
endif()

target_compile_definitions(inverted_pendulum_bot.platform_st PUBLIC
    $<$<STREQUAL:${TARGET_MCU},stm32wb55>:INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER="hal_st/stm32fxxx/DefaultClockNucleoWB55RG.hpp">
    $<$<STREQUAL:${TARGET_MCU},stm32wb55>:INVERTED_PENDULUM_BOT_ST_CLOCK_INIT=ConfigureDefaultClockNucleoWB55RG>
)

target_sources(inverted_pendulum_bot.platform_st PRIVATE
    InertialSensorStm.hpp
    InertialSensorStm.cpp
    MotorDriverStm.hpp
    MotorDriverStm.cpp
    WheelEncodersStm.hpp
    WheelEncodersStm.cpp
    PlatformImpl.hpp
    PlatformImpl.cpp
)
