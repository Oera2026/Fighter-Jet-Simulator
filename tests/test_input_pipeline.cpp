#include "../include/fastjet/input/input_manager.hpp"
#include "../include/fastjet/flcs/f16_flcs.hpp"
#include "../include/fastjet/aero/f16_aero_model.hpp"
#include "../include/fastjet/environment/atmosphere1976.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace fastjet::math;
using namespace fastjet::fdm;
using namespace fastjet::environment;
using namespace fastjet::aero;
using namespace fastjet::flcs;
using namespace fastjet::input;

void test_sdl3_driver_initialization() {
    std::cout << "[Test] Hardware Polling: SDL3 Driver Initialization... " << std::flush;

    SDL3InputDriver sdl_driver;
    const bool ok = sdl_driver.initialize();
    assert(ok);

    // Call poll() to verify sub-millisecond execution
    const auto t0 = std::chrono::high_resolution_clock::now();
    sdl_driver.poll();
    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    // Polling must execute well within sub-millisecond latency (< 1000 us)
    assert(dt_us < 1000);

    const int device_count = sdl_driver.get_device_count();
    std::cout << "PASSED (SDL3 initialized, detected " << device_count
              << " hardware devices, poll latency: " << dt_us << " us)\n";
}

void test_virtual_hotas_to_flcs_pipeline() {
    std::cout << "[Test] End-to-End Pipeline: HOTAS Hardware Polling -> FLCS Driving... " << std::flush;

    auto v_driver = std::make_shared<VirtualHOTASDriver>();
    InputManager input_mgr(v_driver);

    // Profile settings: pitch curvature c = 0.60
    input_mgr.config.pitch_cal.curvature = 0.60;
    input_mgr.config.pitch_cal.inner_deadband = 0.02;
    input_mgr.config.pitch_cal.outer_deadband = 0.01;

    F16FLCS flcs;
    const MassProperties mass = MassProperties::create_clean_f16();

    FlightState state;
    state.pos_ned = Vector3(0.0, 0.0, -5000.0);
    state.vel_b = Vector3(200.0, 0.0, 0.0);
    state.omega_b = Vector3::zero();
    state.q_att = Quaternion::identity();

    constexpr double DT = 0.005; // 200 Hz

    // 1. Center Stick & Idle Throttle
    v_driver->set_axis(input_mgr.config.pitch_axis, 0);       // Center pitch
    v_driver->set_axis(input_mgr.config.roll_axis, 0);        // Center roll
    v_driver->set_axis(input_mgr.config.yaw_axis, 0);         // Center rudder
    // Throttle idle: position 0.06 (within [DETENT_CUTOFF, DETENT_IDLE] range = [0.03, 0.08])
    // Taking calibration inversion into account:
    const double idle_fraction = input_mgr.config.throttle_cal.inverted ? (1.0 - 0.06) : 0.06;
    v_driver->set_axis(input_mgr.config.throttle_axis, -32768 + static_cast<int>(65535 * idle_fraction));

    PilotCommands pilot = input_mgr.update(DT);
    assert(std::abs(pilot.pitch_stick) < 1e-4);
    assert(std::abs(pilot.roll_stick) < 1e-4);
    assert(std::abs(pilot.rudder_pedal) < 1e-4);
    assert(input_mgr.throttle.state == ThrottleController::DetentState::IDLE);

    // 2. Apply Half Aft Stick (50% physical pull)
    // Raw value: 16384 -> normalized ~0.50
    v_driver->set_axis(input_mgr.config.pitch_axis, 16384);

    pilot = input_mgr.update(DT);
    // With c = 0.60, 50% pull yields: y = 0.40*(0.50) + 0.60*(0.50^3) = 0.20 + 0.075 = 0.275
    // After deadband correction, it should be approximately 0.27
    assert(pilot.pitch_stick > 0.20 && pilot.pitch_stick < 0.35);

    // 3. Apply Digital Trim Hat (Nose Up)
    v_driver->set_button(input_mgr.config.btn_trim_up, true);
    for (int i = 0; i < 20; ++i) {
        pilot = input_mgr.update(DT); // 20 steps = 0.1s
    }
    v_driver->set_button(input_mgr.config.btn_trim_up, false);

    // Trim hat added nose-up offset
    assert(input_mgr.trim_hat.trim_pitch > 0.002);

    // STOVL/HOOK button: one press is one edge, however long it is held.
    v_driver->set_button(input_mgr.config.btn_stovl_mode, true);
    input_mgr.update(DT);
    assert(input_mgr.stovl_button_pressed);
    input_mgr.update(DT);
    assert(!input_mgr.stovl_button_pressed);
    v_driver->set_button(input_mgr.config.btn_stovl_mode, false);
    input_mgr.update(DT);
    assert(!input_mgr.stovl_button_pressed);

    // 4. Drive FLCS with conditioned PilotCommands
    const AirData ad = Atmosphere1976::compute(state.altitude(), state.airspeed());
    ControlSurfaces cur_surf(flcs.stabilator.position, flcs.flaperon.position, flcs.rudder.position);
    AircraftForces aero = F16AeroModel::compute_forces_and_moments(state, cur_surf, ad.dynamic_pressure);

    const ControlSurfaces flcs_surfaces = flcs.update(DT, state, pilot, ad.dynamic_pressure, aero, mass);

    // Stabilator responded to stick pull
    assert(flcs_surfaces.delta_e != 0.0);

    // 5. Benchmark update throughput (100,000 updates)
    constexpr int BENCH_STEPS = 100000;
    const auto bench_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < BENCH_STEPS; ++i) {
        pilot = input_mgr.update(DT);
    }
    const auto bench_finish = std::chrono::high_resolution_clock::now();
    const double elapsed_us = std::chrono::duration<double, std::micro>(bench_finish - bench_start).count();
    const double latency_per_update_ns = (elapsed_us * 1000.0) / BENCH_STEPS;

    // Latency must be sub-microsecond (< 1,000 ns), far beating sub-millisecond requirement
    assert(latency_per_update_ns < 1000.0);

    std::cout << "PASSED (End-to-end FLCS driving verified, Latency: "
              << latency_per_update_ns << " ns per update)\n";
}

int main() {
    std::cout << "=== Controller Input Pipeline & SDL3 Driver Verification ===\n";
    test_sdl3_driver_initialization();
    test_virtual_hotas_to_flcs_pipeline();
    std::cout << "All Input Pipeline tests passed successfully!\n\n";
    return 0;
}
