#include "../include/fastjet/input/input_config.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace fastjet::input;

void test_json_serialization_and_file_persistence() {
    std::cout << "[Test] Configuration Persistence: JSON Save & Load Round-Trip... " << std::flush;

    ControllerProfile original;
    original.profile_name = "Thrustmaster Warthog F-16 Profile";

    // Custom pitch calibration
    original.pitch_device = 1;
    original.pitch_axis = 2;
    original.pitch_cal = AxisCalibration(-32000, 50, 32000, 0.04, 0.03, true, 0.65);

    // Custom roll calibration
    original.roll_device = 1;
    original.roll_axis = 0;
    original.roll_cal = AxisCalibration(-32768, 0, 32767, 0.02, 0.01, false, 0.45);

    // Custom throttle calibration
    original.throttle_device = 2;
    original.throttle_axis = 1;
    original.throttle_cal = AxisCalibration(0, 0, 65535, 0.01, 0.01, false, 0.00);

    // Custom button bindings
    original.btn_trim_up = 12;
    original.btn_trim_down = 14;
    original.btn_speedbrake_extend = 7;
    original.btn_speedbrake_retract = 8;
    original.btn_parking_brake = 21;
    original.btn_stovl_mode = 9;

    // 1. Serialize to JSON string
    const std::string json_str = original.to_json();
    assert(!json_str.empty());
    assert(json_str.find("\"Thrustmaster Warthog F-16 Profile\"") != std::string::npos);

    // 2. Save to file
    const std::string test_filepath = "test_controller_profile.json";
    const bool save_ok = original.save_to_file(test_filepath);
    assert(save_ok);

    // 3. Load from file into a fresh profile
    ControllerProfile loaded;
    const bool load_ok = loaded.load_from_file(test_filepath);
    assert(load_ok);

    // 4. Verify all fields match identically
    assert(loaded.profile_name == original.profile_name);
    assert(loaded.pitch_device == 1);
    assert(loaded.pitch_axis == 2);
    assert(loaded.pitch_cal.raw_min == -32000);
    assert(loaded.pitch_cal.raw_center == 50);
    assert(loaded.pitch_cal.raw_max == 32000);
    assert(std::abs(loaded.pitch_cal.inner_deadband - 0.04) < 1e-6);
    assert(std::abs(loaded.pitch_cal.outer_deadband - 0.03) < 1e-6);
    assert(loaded.pitch_cal.inverted == true);
    assert(std::abs(loaded.pitch_cal.curvature - 0.65) < 1e-6);

    assert(loaded.throttle_device == 2);
    assert(loaded.throttle_axis == 1);
    assert(loaded.throttle_cal.raw_max == 65535);

    assert(loaded.btn_trim_up == 12);
    assert(loaded.btn_trim_down == 14);
    assert(loaded.btn_speedbrake_extend == 7);
    assert(loaded.btn_speedbrake_retract == 8);
    assert(loaded.btn_parking_brake == 21);
    assert(loaded.btn_stovl_mode == 9);

    // A profile saved before the STOVL button existed keeps its default
    // instead of landing on button 0 (trim up).
    std::string legacy = json_str;
    const std::size_t comma = legacy.find(",\n    \"stovl_mode\"");
    assert(comma != std::string::npos);
    legacy.erase(comma, legacy.find('\n', comma + 2) - comma);
    ControllerProfile from_legacy;
    assert(from_legacy.from_json(legacy));
    assert(from_legacy.btn_parking_brake == 21);
    assert(from_legacy.btn_stovl_mode == ControllerProfile{}.btn_stovl_mode);

    // Clean up temporary test file
    std::remove(test_filepath.c_str());

    std::cout << "PASSED\n";
}

int main() {
    std::cout << "=== Controller Configuration Persistence Verification ===\n";
    test_json_serialization_and_file_persistence();
    std::cout << "All Configuration Persistence tests passed successfully!\n\n";
    return 0;
}
