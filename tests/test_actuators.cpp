#include "../include/fastjet/flcs/actuator.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace fastjet::flcs;

void test_stabilator_rate_and_deflection_limits() {
    std::cout << "[Test] Stabilator Actuator (tau=0.05s, rate=60 deg/s, limits=[-25, +25] deg)... " << std::flush;

    Actuator stab = Actuator::create_stabilator(0.0);
    constexpr double DT = 0.005; // 200 Hz

    // 1. Test rate limit during full step command from 0 to +25 deg
    double time = 0.0;
    double max_observed_rate = 0.0;

    while (stab.position < 24.99 && time < 1.0) {
        stab.step(25.0, DT);
        time += DT;
        if (stab.rate > max_observed_rate) {
            max_observed_rate = stab.rate;
        }
        // Strict rate limit check at every micro-step
        assert(stab.rate <= 60.0001);
    }

    // Minimum physical transit time for 25 deg at 60 deg/s is 25/60 = 0.4167s
    assert(time >= 25.0 / 60.0 - 0.01);
    assert(std::abs(max_observed_rate - 60.0) < 1e-3);

    // 2. Test position limit clamping
    // Command +40 deg (beyond +25 deg limit)
    for (int i = 0; i < 200; ++i) {
        stab.step(40.0, DT);
    }
    assert(std::abs(stab.position - 25.0) < 1e-6);
    assert(stab.is_pos_limited);

    // Command -40 deg (beyond -25 deg limit)
    for (int i = 0; i < 400; ++i) {
        stab.step(-40.0, DT);
    }
    assert(std::abs(stab.position - (-25.0)) < 1e-6);
    assert(stab.is_pos_limited);

    std::cout << "PASSED (Max rate=" << max_observed_rate
              << " deg/s, Transit time=" << time << " s, Clamped at +/-25 deg)\n";
}

void test_flaperon_and_rudder_actuators() {
    std::cout << "[Test] Roll surface (80 deg/s, [-20, +20]) & Rudder (120 deg/s, [-30, +30])... " << std::flush;

    constexpr double DT = 0.005;

    // Flaperon
    Actuator flap = Actuator::create_flaperon(0.0);
    double flap_max_rate = 0.0;
    for (int i = 0; i < 200; ++i) {
        flap.step(30.0, DT); // Command beyond +20 limit
        if (flap.rate > flap_max_rate) flap_max_rate = flap.rate;
        assert(flap.rate <= 80.0001);
    }
    assert(std::abs(flap.position - 20.0) < 1e-6);
    assert(std::abs(flap_max_rate - 80.0) < 1e-3);

    // Rudder
    Actuator rud = Actuator::create_rudder(0.0);
    double rud_max_rate = 0.0;
    for (int i = 0; i < 200; ++i) {
        rud.step(50.0, DT); // Command beyond +30 limit
        if (rud.rate > rud_max_rate) rud_max_rate = rud.rate;
        assert(rud.rate <= 120.0001);
    }
    assert(std::abs(rud.position - 30.0) < 1e-6);
    assert(std::abs(rud_max_rate - 120.0) < 1e-3);

    std::cout << "PASSED (Flaperon max rate=" << flap_max_rate
              << " deg/s, Rudder max rate=" << rud_max_rate << " deg/s)\n";
}

void test_first_order_lag_response() {
    std::cout << "[Test] First-Order Hydraulic Lag Exponential Response... " << std::flush;

    // Command a small 1-deg step where rate limit (60 deg/s) is not hit:
    // Initial rate = (1 - 0) / 0.05 = 20 deg/s < 60 deg/s
    Actuator stab = Actuator::create_stabilator(0.0);
    constexpr double DT = 0.001; // 1 ms

    // At t = tau = 0.05s, first-order step response reaches 1 - 1/e ~ 63.21%
    for (int i = 0; i < 50; ++i) {
        stab.step(1.0, DT);
    }

    const double expected_val = 1.0 * (1.0 - std::exp(-1.0)); // ~0.63212
    assert(std::abs(stab.position - expected_val) < 0.01);

    std::cout << "PASSED (Response at t=tau: " << stab.position
              << " deg, Expected: " << expected_val << " deg)\n";
}

int main() {
    std::cout << "=== Hydraulic Actuator Dynamics Verification ===\n";
    test_stabilator_rate_and_deflection_limits();
    test_flaperon_and_rudder_actuators();
    test_first_order_lag_response();
    std::cout << "All Actuator tests passed successfully!\n\n";
    return 0;
}
