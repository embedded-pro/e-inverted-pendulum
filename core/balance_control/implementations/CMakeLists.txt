add_library(inverted_pendulum_bot.balance_control.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.balance_control.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.balance_control.implementations PUBLIC
    inverted_pendulum_bot.balance_control.interfaces
    inverted_pendulum_bot.control_loop.interfaces
    inverted_pendulum_bot.motion_actuation.interfaces
    inverted_pendulum_bot.wheel_odometry.interfaces
    numerical.controllers.implementations
    infra.timer
)

target_sources(inverted_pendulum_bot.balance_control.implementations PRIVATE
    BalanceControlImpl.hpp
    BalanceControlImpl.cpp
    CascadedPidStrategy.hpp
    CascadedPidStrategy.cpp
    ContinuousPid.hpp
    ContinuousPid.cpp
    LqrStrategy.hpp
    LqrStrategy.cpp
)

add_subdirectory(test)
