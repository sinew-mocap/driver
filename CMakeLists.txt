# driver/ — the rebocap cluster (the ONLY place the rebocap dongle is known).
# Hexagon: core/ (rebocap frame codec) + ports/ (FrameSource, TrackerSink) +
# adapters/ (serial source, OSC-out, ftxui TUI).  Depends on the osc cluster for
# the neutral /sinew contract (build_tracker/build_accel).  No calibration, no
# Vulkan: streams the device quaternion out as raw /sinew OSC.

# ── Standalone-build entry (this repo cloned side-by-side in ~/) ──────────────
# When this cluster is the top-level CMake project, set up the toolchain.  Under
# the dev superbuild (a parent CMakeLists) CMAKE_SOURCE_DIR differs, so it no-ops.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.19)
    project(driver C CXX)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    option(SINEW_TESTS "Build the cluster's unit/property tests" OFF)
    if(SINEW_TESTS)
        enable_testing()
    endif()
endif()

include(FetchContent)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
  GIT_TAG        v5.0.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(ftxui)

add_executable(sinew_tui
    adapters/main.cpp
    adapters/osc_receiver.cpp
    adapters/sinew_serial_host.c
    core/sinew_protocol.c
    core/sinew_osc.c
    core/hwid_table.c
)

# Bake the default hwid/activate ini into the TUI so it ships sane defaults.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/adapters/hwid_table.ini" HWID_INI_CONTENT)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/adapters/activates.ini"  ACTIVATES_INI_CONTENT)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/adapters/default_ini.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/default_ini.h"
    @ONLY
)
target_include_directories(sinew_tui PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"            # generated default_ini.h
    "${CMAKE_CURRENT_SOURCE_DIR}/adapters"   # osc_receiver.hpp, okhsl.hpp, sinew_driver.h
    "${CMAKE_CURRENT_SOURCE_DIR}/core"       # sinew_protocol.h, hwid_table.h, nonic.h
    "${CMAKE_CURRENT_SOURCE_DIR}/ports"      # frame_source.h, tracker_sink.h
)
target_compile_definitions(sinew_tui PRIVATE SINEW_VERSION="${PROJECT_VERSION}")
target_link_libraries(sinew_tui PRIVATE
    ftxui::screen
    ftxui::dom
    ftxui::component
)

if(WIN32)
    target_link_libraries(sinew_tui PRIVATE ws2_32 setupapi)
else()
    target_link_libraries(sinew_tui PRIVATE m)
endif()

if(MSVC)
    target_compile_options(sinew_tui PRIVATE /W3 /O2)
    target_compile_definitions(sinew_tui PRIVATE NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(sinew_tui PRIVATE -Wall -O2)
endif()

if(MINGW)
    target_link_options(sinew_tui PRIVATE -static)
endif()

install(TARGETS sinew_tui RUNTIME DESTINATION . COMPONENT driver)

# ── Property tests (opt-in: -DSINEW_TESTS=ON) ────────────────────────────────
# RapidCheck (C++ QuickCheck) on the codec; mirrors the Lean Plausible properties.
if(SINEW_TESTS)
    if(NOT TARGET rapidcheck)
        include(FetchContent)
        FetchContent_Declare(rapidcheck
            GIT_REPOSITORY https://github.com/emil-e/rapidcheck
            GIT_TAG        b2d9ed2dddefc4b84318d664b4f221eb792d89c7)
        FetchContent_MakeAvailable(rapidcheck)
    endif()
    add_executable(driver_proptest
        tests/proptest.cpp
        core/sinew_protocol.c
        core/sinew_osc.c)
    target_include_directories(driver_proptest PRIVATE core)
    target_link_libraries(driver_proptest PRIVATE rapidcheck)
    add_test(NAME driver_proptest COMMAND driver_proptest)
endif()
