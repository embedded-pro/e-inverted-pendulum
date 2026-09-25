add_library(inverted_pendulum_bot.persistence ${EMIL_EXCLUDE_FROM_ALL})

protocol_buffer_echo_cpp(inverted_pendulum_bot.persistence Tuning.proto)

target_include_directories(inverted_pendulum_bot.persistence PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.persistence PUBLIC
    inverted_pendulum_bot.balance_control.interfaces
    inverted_pendulum_bot.safety_supervisor.interfaces
    services.util
    services.crypto
    hal.interfaces
    infra.timer
)

target_sources(inverted_pendulum_bot.persistence PRIVATE
    PersistingBalanceControl.hpp
    PersistingBalanceControl.cpp
    PersistentTuning.hpp
    PersistentTuning.cpp
)

add_subdirectory(test)
