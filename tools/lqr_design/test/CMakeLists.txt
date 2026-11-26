add_executable(inverted_pendulum_bot.tool.lqr_design_test)
emil_build_for(inverted_pendulum_bot.tool.lqr_design_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.tool.lqr_design_test)

target_link_libraries(inverted_pendulum_bot.tool.lqr_design_test PUBLIC
    gmock_main
    inverted_pendulum_bot.tool.lqr_design_lib
)

target_sources(inverted_pendulum_bot.tool.lqr_design_test PRIVATE
    TestLqrDesign.cpp
)
