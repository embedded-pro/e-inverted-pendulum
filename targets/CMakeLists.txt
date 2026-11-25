# Application entry points and platform-specific implementations.
#
# INVERTED_PENDULUM_BOT_TARGET_PLATFORM selects the active platform (see CMakePresets.json):
#   host preset -> "host"   embedded preset -> "st"
# Only the selected platform's implementation is compiled.
add_subdirectory(platform_implementations/${INVERTED_PENDULUM_BOT_TARGET_PLATFORM})

# Bind the active platform implementation behind the platform::Platform interface.
# A target's Main.cpp includes PLATFORM_IMPL_HEADER to construct the right
# PlatformImpl, while all application logic in core/ depends ONLY on the abstract
# interface (core/platform_abstraction/Platform.hpp). This is the seam that lets the
# same application run on the host and on every supported board.
add_library(inverted_pendulum_bot.platform_impl INTERFACE)

target_compile_definitions(inverted_pendulum_bot.platform_impl INTERFACE
    PLATFORM_IMPL_HEADER="targets/platform_implementations/${INVERTED_PENDULUM_BOT_TARGET_PLATFORM}/PlatformImpl.hpp"
)

target_link_libraries(inverted_pendulum_bot.platform_impl INTERFACE
    inverted_pendulum_bot.platform_${INVERTED_PENDULUM_BOT_TARGET_PLATFORM}
)

add_subdirectory(cli)
