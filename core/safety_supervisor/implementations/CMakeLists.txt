add_library(inverted_pendulum_bot.safety_supervisor.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.safety_supervisor.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.safety_supervisor.implementations PUBLIC
    inverted_pendulum_bot.safety_supervisor.interfaces
    inverted_pendulum_bot.control_loop.interfaces
    inverted_pendulum_bot.motion_actuation.interfaces
    inverted_pendulum_bot.inertial_sensing.interfaces
    infra.timer
)

target_sources(inverted_pendulum_bot.safety_supervisor.implementations PRIVATE
    SafetySupervisorImpl.hpp
    SafetySupervisorImpl.cpp
)

add_subdirectory(test)
