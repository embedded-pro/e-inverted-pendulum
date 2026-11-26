add_library(inverted_pendulum_bot.wheel_odometry.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.wheel_odometry.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.wheel_odometry.implementations PUBLIC
    inverted_pendulum_bot.wheel_odometry.interfaces
    inverted_pendulum_bot.platform_abstraction
    infra.timer
    robotics.mobile
)

target_sources(inverted_pendulum_bot.wheel_odometry.implementations PRIVATE
    WheelOdometryImpl.hpp
    WheelOdometryImpl.cpp
)

add_subdirectory(test)
