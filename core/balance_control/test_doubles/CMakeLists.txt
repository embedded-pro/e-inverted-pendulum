add_library(inverted_pendulum_bot.balance_control_mock INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.balance_control_mock INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
)

target_link_libraries(inverted_pendulum_bot.balance_control_mock INTERFACE
    inverted_pendulum_bot.balance_control.interfaces
    gmock
)

target_sources(inverted_pendulum_bot.balance_control_mock INTERFACE
    BalanceControlMock.hpp
)
