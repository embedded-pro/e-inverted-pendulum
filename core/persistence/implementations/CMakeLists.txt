add_library(inverted_pendulum_bot.persistence.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.persistence.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.persistence.implementations PUBLIC
    inverted_pendulum_bot.persistence.interfaces
    inverted_pendulum_bot.balance_control.interfaces
    hal.interfaces
    infra.timer
    infra.util
    services.util
)

target_sources(inverted_pendulum_bot.persistence.implementations PRIVATE
    PersistentBalanceControl.hpp
    PersistentBalanceControl.cpp
    TuningRecord.hpp
    TuningRecord.cpp
)

add_subdirectory(test)
