#pragma once

#include "joystick_driver.hpp"
#include "input_config.hpp"
#include "avionics_controls.hpp"
#include "../flcs/pilot_commands.hpp"
#include <memory>

namespace fastjet {
namespace input {

/**
 * @brief Complete Flight Sim Controller Input Processing Subsystem.
 * Bridges low-level hardware polling to the F-16 FLCS and FDM.
 * Zero dynamic allocation during hot polling loop.
 */
class InputManager {
public:
    std::shared_ptr<IJoystickDriver> driver{nullptr};
    ControllerProfile config{};

    // Avionics Controls
    DigitalTrimHat trim_hat{};
    ThrottleController throttle{};
    SpeedbrakeController speedbrake{};
    WheelBrakes brakes{};

    // Conditioned analog outputs
    double pitch_in{0.0};      // [-1.0, +1.0]
    double roll_in{0.0};       // [-1.0, +1.0]
    double yaw_in{0.0};        // [-1.0, +1.0]
    double throttle_in{0.0};   // [0.0, 1.0]

    // STOVL/CTOL button: true on the update the button went down. The
    // host latches the mode; holding the button does not toggle it again.
    bool stovl_button_pressed{false};

    explicit InputManager(std::shared_ptr<IJoystickDriver> drv) noexcept
        : driver(std::move(drv)) {}

    /**
     * @brief Polls hardware and processes all axes, trims, and avionics.
     * Called at sub-millisecond rates (or synchronized at 200 Hz).
     *
     * @param dt Delta time in seconds
     * @return flcs::PilotCommands Resulting stick commands for the FLCS
     */
    flcs::PilotCommands update(double dt) noexcept {
        if (!driver) {
            return {0.0, 0.0, 0.0};
        }

        // 1. Poll low-level hardware
        driver->poll();

        // 2. Read and condition primary flight axes
        const int raw_pitch = driver->get_axis(config.pitch_device, config.pitch_axis);
        const int raw_roll  = driver->get_axis(config.roll_device,  config.roll_axis);
        const int raw_yaw   = driver->get_axis(config.yaw_device,   config.yaw_axis);
        const int raw_thr   = driver->get_axis(config.throttle_device, config.throttle_axis);

        pitch_in = SignalConditioner::process_bipolar(raw_pitch, config.pitch_cal);
        roll_in  = SignalConditioner::process_bipolar(raw_roll,  config.roll_cal);
        yaw_in   = SignalConditioner::process_bipolar(raw_yaw,   config.yaw_cal);
        throttle_in = SignalConditioner::process_unipolar(raw_thr, config.throttle_cal);

        // 3. Process Wheel Brakes
        const int raw_lb = driver->get_axis(config.left_brake_device,  config.left_brake_axis);
        const int raw_rb = driver->get_axis(config.right_brake_device, config.right_brake_axis);
        brakes.left_brake  = SignalConditioner::process_unipolar(raw_lb, config.left_brake_cal);
        brakes.right_brake = SignalConditioner::process_unipolar(raw_rb, config.right_brake_cal);

        // 4. Process Digital Trim Hat
        double trim_p_dir = 0.0;
        double trim_r_dir = 0.0;
        if (driver->get_button(config.pitch_device, config.btn_trim_up))   trim_p_dir += 1.0;
        if (driver->get_button(config.pitch_device, config.btn_trim_down)) trim_p_dir -= 1.0;
        if (driver->get_button(config.roll_device,  config.btn_trim_left)) trim_r_dir -= 1.0;
        if (driver->get_button(config.roll_device,  config.btn_trim_right))trim_r_dir += 1.0;
        trim_hat.step(trim_p_dir, trim_r_dir, dt);

        // 5. Process Speedbrake HOTAS Switch
        SpeedbrakeController::SwitchPosition sb_sw = SpeedbrakeController::SwitchPosition::OFF;
        if (driver->get_button(config.throttle_device, config.btn_speedbrake_extend)) {
            sb_sw = SpeedbrakeController::SwitchPosition::EXTEND;
        } else if (driver->get_button(config.throttle_device, config.btn_speedbrake_retract)) {
            sb_sw = SpeedbrakeController::SwitchPosition::RETRACT;
        }
        speedbrake.update(sb_sw, dt);

        // 6. Process Parking Brake Toggle
        if (driver->get_button(config.throttle_device, config.btn_parking_brake)) {
            brakes.parking_brake = !brakes.parking_brake;
        }

        // 7. STOVL/CTOL Mode Button (on the throttle, rising edge only)
        const bool stovl_btn = driver->get_button(config.throttle_device, config.btn_stovl_mode);
        stovl_button_pressed = stovl_btn && !stovl_button_held_;
        stovl_button_held_ = stovl_btn;

        // 8. Update Engine Throttle Detents and Thrust
        throttle.update(throttle_in);

        // 9. Synthesize PilotCommands with Digital Trim Hat
        const double final_pitch = std::clamp(pitch_in + trim_hat.trim_pitch, -1.0, 1.0);
        const double final_roll  = std::clamp(roll_in  + trim_hat.trim_roll,  -1.0, 1.0);
        const double final_yaw   = yaw_in;

        return {final_pitch, final_roll, final_yaw};
    }

private:
    bool stovl_button_held_{false};
};

} // namespace input
} // namespace fastjet
