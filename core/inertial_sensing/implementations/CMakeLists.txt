add_library(inverted_pendulum_bot.inertial_sensing.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.inertial_sensing.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.inertial_sensing.implementations PUBLIC
    inverted_pendulum_bot.inertial_sensing.interfaces
    inverted_pendulum_bot.platform_abstraction
    infra.timer
)

target_sources(inverted_pendulum_bot.inertial_sensing.implementations PRIVATE
    InertialSensingImpl.hpp
    InertialSensingImpl.cpp
)

add_subdirectory(test)
