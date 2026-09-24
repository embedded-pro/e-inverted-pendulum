add_library(inverted_pendulum_bot.balance_control.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.balance_control.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.balance_control.interfaces INTERFACE
    inverted_pendulum_bot.attitude_estimation.interfaces
    inverted_pendulum_bot.wheel_odometry.interfaces
    infra.util
)

target_sources(inverted_pendulum_bot.balance_control.interfaces PRIVATE
    BalanceControl.hpp
    ControlStrategy.hpp
)
