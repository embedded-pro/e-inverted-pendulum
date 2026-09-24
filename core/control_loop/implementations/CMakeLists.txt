add_library(inverted_pendulum_bot.control_loop.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.control_loop.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.control_loop.implementations PUBLIC
    inverted_pendulum_bot.control_loop.interfaces
    inverted_pendulum_bot.attitude_estimation.interfaces
    inverted_pendulum_bot.inertial_sensing.interfaces
    infra.timer
    infra.util
)

target_sources(inverted_pendulum_bot.control_loop.implementations PRIVATE
    ControlLoopImpl.hpp
    ControlLoopImpl.cpp
)

add_subdirectory(test)
