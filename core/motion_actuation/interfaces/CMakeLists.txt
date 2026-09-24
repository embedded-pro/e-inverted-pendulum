add_library(inverted_pendulum_bot.motion_actuation.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.motion_actuation.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.motion_actuation.interfaces INTERFACE
    infra.util
)

target_sources(inverted_pendulum_bot.motion_actuation.interfaces PRIVATE
    DriverConfiguration.hpp
    MotionActuation.hpp
)
