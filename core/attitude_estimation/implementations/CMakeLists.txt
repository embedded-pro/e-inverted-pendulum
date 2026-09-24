add_library(inverted_pendulum_bot.attitude_estimation.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(inverted_pendulum_bot.attitude_estimation.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(inverted_pendulum_bot.attitude_estimation.implementations PUBLIC
    inverted_pendulum_bot.attitude_estimation.interfaces
    numerical.filters.active
    infra.timer
)

target_sources(inverted_pendulum_bot.attitude_estimation.implementations PRIVATE
    AttitudeEstimationImpl.hpp
    AttitudeEstimationImpl.cpp
    ComplementaryPitchFilter.hpp
    ComplementaryPitchFilter.cpp
    KalmanPitchFilter.hpp
    KalmanPitchFilter.cpp
    PitchFilter.hpp
)

add_subdirectory(test)
