add_library(inverted_pendulum_bot.tool.lqr_design_lib STATIC)

target_include_directories(inverted_pendulum_bot.tool.lqr_design_lib PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(inverted_pendulum_bot.tool.lqr_design_lib PUBLIC
    inverted_pendulum_bot.balance_control.implementations
    robotics.mobile
    numerical.control_analysis
    numerical.controllers.implementations
    numerical.math
)

target_sources(inverted_pendulum_bot.tool.lqr_design_lib PRIVATE
    LqrDesign.hpp
    LqrDesign.cpp
)

add_executable(inverted_pendulum_bot.tool.lqr_design)

target_link_libraries(inverted_pendulum_bot.tool.lqr_design PRIVATE
    inverted_pendulum_bot.tool.lqr_design_lib
)

target_sources(inverted_pendulum_bot.tool.lqr_design PRIVATE
    Main.cpp
)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    add_subdirectory(test)
endif()
