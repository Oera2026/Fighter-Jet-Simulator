cmake_minimum_required(VERSION 3.16)

project(FastJetSimulator
    VERSION 1.0.0
    DESCRIPTION "F-16 6-DoF flight dynamics model, FLCS and cockpit renderer"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# -march=native is deliberately NOT enabled by default: it produces binaries
# that fault on other machines. Opt in explicitly for a local tuned build.
option(FASTJET_NATIVE_ARCH "Optimise for the building machine's CPU" OFF)
option(FASTJET_BUILD_TESTS "Build the verification test suite" ON)
option(FASTJET_BUILD_VIEWER "Build the OpenGL cockpit viewer" ON)

# ---------------------------------------------------------------------------
# Core library (header-only)
# ---------------------------------------------------------------------------
add_library(fastjet INTERFACE)
add_library(FastJet::fastjet ALIAS fastjet)

target_include_directories(fastjet INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(fastjet INTERFACE cxx_std_20)
# Worker threads (fastjet/core/thread_pool.hpp) for loading and baking.
find_package(Threads REQUIRED)
target_link_libraries(fastjet INTERFACE Threads::Threads)
target_compile_definitions(fastjet INTERFACE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)

if(FASTJET_NATIVE_ARCH AND NOT MSVC)
    target_compile_options(fastjet INTERFACE -march=native)
endif()

# Shared warning set for first-party targets.
if(MSVC)
    set(FASTJET_WARNINGS /W4)
else()
    set(FASTJET_WARNINGS -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Cockpit viewer (needs SDL3 + OpenGL)
# ---------------------------------------------------------------------------
if(FASTJET_BUILD_VIEWER)
    find_package(OpenGL REQUIRED)

    find_package(SDL3 QUIET)
    if(NOT SDL3_FOUND)
        message(STATUS "Fetching SDL3 via FetchContent...")
        include(FetchContent)
        FetchContent_Declare(
            SDL3
            GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
            GIT_TAG release-3.2.0
        )
        set(SDL_TEST OFF CACHE BOOL "" FORCE)
        set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(SDL3)
    endif()

    add_executable(f16_sim_viewer src/f16_sim_viewer.cpp)
    target_link_libraries(f16_sim_viewer PRIVATE fastjet SDL3::SDL3 OpenGL::GL)
    target_compile_options(f16_sim_viewer PRIVATE ${FASTJET_WARNINGS})
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(FASTJET_BUILD_TESTS)
    enable_testing()

    # Pure physics tests: no graphics context required.
    set(FASTJET_CORE_TESTS
        test_atmosphere
        test_thread_pool
        test_ballistic
        test_gyroscopic
        test_performance
        test_interpolator
        test_aero_coefficients
        test_pitch_divergence
        test_actuators
        test_flcs_level_flight
        test_flcs_pullup
        test_flcs_review
        test_flcs_handling
        test_signal_conditioning
        test_force_curves
        test_avionics_controls
        test_config_persistence
        test_propulsion
        test_landing_gear
        test_compressibility
        test_integrated_flight
        test_throttle_control
        test_performance_envelope
        test_terrain_field
        test_model_glb
        test_aircraft_specs
        test_multi_aircraft_performance
        test_ofc_handover
        test_ofc_closed_loop
        test_user_settings
        test_menu_navigation
        test_aircraft_entity
        test_guns
        test_air_combat_geometry
        test_bfm_ai
        test_missiles
        test_evade_mode
        test_radar_scope
        test_flight_realism
        test_ejection
        test_stovl
        test_missile_warning
        test_air_combat)

    # Tests that need SDL3 and/or an OpenGL context.
    set(FASTJET_GRAPHICS_TESTS
        test_input_pipeline
        test_camera_rig
        test_hud_collimation
        test_hud_clipping
        test_flight_instruments
        test_render_pipeline
        test_terrain_render
        test_render_fidelity
        test_jet_cockpits)

    foreach(test_name IN LISTS FASTJET_CORE_TESTS)
        add_executable(${test_name} tests/${test_name}.cpp)
        target_link_libraries(${test_name} PRIVATE fastjet)
        target_compile_options(${test_name} PRIVATE ${FASTJET_WARNINGS})
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()

    if(SDL3_FOUND AND EPOXY_FOUND AND OpenGL_FOUND)
        foreach(test_name IN LISTS FASTJET_GRAPHICS_TESTS)
            add_executable(${test_name} tests/${test_name}.cpp)
            target_link_libraries(${test_name}
                PRIVATE fastjet PkgConfig::SDL3 PkgConfig::EPOXY OpenGL::GL)
            target_compile_options(${test_name} PRIVATE ${FASTJET_WARNINGS})
            add_test(NAME ${test_name} COMMAND ${test_name})
        endforeach()
    endif()
endif()

install(DIRECTORY include/fastjet DESTINATION include)
