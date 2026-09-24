cmake_minimum_required(VERSION 3.24)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(INVERTED_PENDULUM_BOT_STANDALONE On)
endif()

option(CMAKE_COMPILE_WARNING_AS_ERROR "Enable warnings-as-error" On)
option(INVERTED_PENDULUM_BOT_INCLUDE_DEFAULT_INIT "Include default initialization code; turn off when providing custom initialization" On)
option(INVERTED_PENDULUM_BOT_BUILD_TESTS "Enable build of the examples" Off)
option(INVERTED_PENDULUM_BOT_BUILD_SIMULATOR "Enable build of the Qt simulator applications" Off)
option(INVERTED_PENDULUM_BOT_BUILD_HIL_TESTS "Build hardware-in-the-loop tests" Off)
option(INVERTED_PENDULUM_BOT_EMBEDDED_BUILD "Building for an embedded target" Off)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    set(BUILD_TESTING On)
endif()

set(HALST_INCLUDE_DEFAULT_LINKER_SCRIPTS Off CACHE INTERNAL "")
set(HALST_BUILD_EXAMPLES Off CACHE INTERNAL "")

if(DEFINED EMIL_SEGGER_RTT_BUFFER_SIZE_UP)
    set(SEGGER_RTT_BUFFER_SIZE_UP ${EMIL_SEGGER_RTT_BUFFER_SIZE_UP} CACHE INTERNAL "")
endif()

add_definitions(-DEMIL_ENABLE_TRACING=1)

if(DEFINED INVERTED_PENDULUM_BOT_TARGET_BOARD)
    add_definitions(-DINVERTED_PENDULUM_BOT_TARGET_BOARD="${INVERTED_PENDULUM_BOT_TARGET_BOARD}")
else()
    add_definitions(-DINVERTED_PENDULUM_BOT_TARGET_BOARD="host")
endif()

project(inverted_pendulum_bot LANGUAGES C CXX ASM VERSION 0.0.1) # x-release-please-version

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

# Use -Og for Debug builds on GCC/Clang to enable optimizations while maintaining debuggability
# This improves real-time performance in Debug builds on resource-constrained targets
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_C_FLAGS_DEBUG "-Og -g" CACHE STRING "C Debug flags" FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-Og -g" CACHE STRING "C++ Debug flags" FORCE)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/MarkIncludesSystem.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerWarnings.cmake")

set_directory_properties(PROPERTY USE_FOLDERS ON)

set(INVERTED_PENDULUM_BOT_EXCLUDE_FROM_ALL "")

set(EMIL_INCLUDE_ECHO On CACHE BOOL "" FORCE)
set(EMIL_FETCH_ECHO_COMPILERS Off CACHE BOOL "" FORCE)

if (INVERTED_PENDULUM_BOT_EMBEDDED_BUILD OR CMAKE_CROSSCOMPILING)
    set(EMIL_BUILD_ECHO_COMPILERS Off CACHE BOOL "" FORCE)
else()
    set(EMIL_BUILD_ECHO_COMPILERS On CACHE BOOL "" FORCE)
endif()
set(EMIL_ENABLE_DOCKER_TOOLS Off CACHE BOOL "" FORCE)
set(EMIL_INCLUDE_MBEDTLS Off CACHE BOOL "" FORCE)

add_subdirectory(infra/embedded-infra-lib)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    set(CCR_FETCH_DEPS On CACHE BOOL "" FORCE)
    set(CCR_BUILD_TESTS Off CACHE BOOL "" FORCE)

    include(FetchContent)
    FetchContent_Declare(
        cucumber_cpp
        GIT_REPOSITORY https://github.com/philips-software/amp-cucumber-cpp-runner.git
        GIT_TAG        v4.0.0
    )
    FetchContent_MakeAvailable(cucumber_cpp)

    # cucumber_gherkin uses std::wstring_convert which is deprecated in C++17/GCC15.
    # Suppress -Werror=deprecated-declarations for this third-party target only.
    if (TARGET cucumber_cpp.library.api)
        target_compile_options(cucumber_cpp.library.api PRIVATE -Wno-deprecated-declarations)
    endif()
endif()

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    include(CTest)
    emil_enable_testing()
endif()

if (DEFINED TARGET_MCU_VENDOR)
    add_subdirectory(infra/hal/${TARGET_MCU_VENDOR})
    emil_exclude_directory_from_coverage(infra/hal/${TARGET_MCU_VENDOR})
endif()

add_subdirectory(infra/numerical-toolbox)

# Ordering is load-bearing. Both calls must sit after every submodule and FetchContent
# addition -- otherwise dependencies such as cucumber_cpp are missed -- and before
# add_subdirectory(core), or this project's own targets get marked SYSTEM and the warnings
# these flags exist to catch are silenced. See cmake/MarkIncludesSystem.cmake.
inverted_pendulum_bot_mark_includes_system("${CMAKE_CURRENT_SOURCE_DIR}")
inverted_pendulum_bot_enable_project_warnings()

add_subdirectory(core)
add_subdirectory(targets)

if (INVERTED_PENDULUM_BOT_BUILD_TESTS)
    add_subdirectory(integration_tests)
endif()

set(INVERTED_PENDULUM_BOT_CLANGFORMAT_DIRS core targets)

if (NOT INVERTED_PENDULUM_BOT_EMBEDDED_BUILD)
    add_subdirectory(tools)
    list(APPEND INVERTED_PENDULUM_BOT_CLANGFORMAT_DIRS tools)
endif()


emil_clangformat_directories(inverted_pendulum_bot DIRECTORIES ${INVERTED_PENDULUM_BOT_CLANGFORMAT_DIRS})
emil_folderize_all_targets()
emil_exclude_directory_from_coverage(infra/embedded-infra-lib)
emil_coverage_targets(DIRECTORIES .)

include(scripts/development_environment.cmake)

if (DEFINED TARGET_MCU)
    generate_development_environment_script(
        TARGET_MCU "${TARGET_MCU}"
        GDB_PORT "2334"
    )
endif()

set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_SOURCE_IGNORE_FILES ".vs/;.git/;build/")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION "${PROJECT_DESCRIPTION}")
set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_RPM_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_PACKAGE_HOMEPAGE_URL "${PROJECT_HOMEPAGE_URL}")
set(CPACK_PACKAGE_MAINTAINER "${CPACK_PACKAGE_VENDOR}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_VENDOR}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

include(CPack)
