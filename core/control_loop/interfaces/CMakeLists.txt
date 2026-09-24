add_library(inverted_pendulum_bot.control_loop.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.control_loop.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.control_loop.interfaces INTERFACE
    inverted_pendulum_bot.attitude_estimation.interfaces
    infra.timer
)

target_sources(inverted_pendulum_bot.control_loop.interfaces PRIVATE
    ControlLoop.hpp
    Stages.hpp
)
