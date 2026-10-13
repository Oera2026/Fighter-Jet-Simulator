#pragma once

#include "signal_conditioner.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace fastjet {
namespace input {

/**
 * @brief Complete controller mapping and calibration profile for F-16 HOTAS.
 */
struct ControllerProfile {
    std::string profile_name{"F-16 HOTAS Default"};

    // Axis Mappings: (device_id, axis_index)
    int pitch_device{0};
    int pitch_axis{1}; // Y-axis
    AxisCalibration pitch_cal{-32768, 0, 32767, 0.03, 0.02, false, 0.60};

    int roll_device{0};
    int roll_axis{0}; // X-axis
    AxisCalibration roll_cal{-32768, 0, 32767, 0.03, 0.02, false, 0.50};

    int yaw_device{0};
    int yaw_axis{2}; // Twist or rudder pedals
    AxisCalibration yaw_cal{-32768, 0, 32767, 0.04, 0.02, false, 0.40};

    int throttle_device{0};
    int throttle_axis{3}; // Throttle slider
    AxisCalibration throttle_cal{-32768, 0, 32767, 0.02, 0.01, true, 0.00}; // Inverted by default so forward=100% thrust

    int left_brake_device{0};
    int left_brake_axis{4};
    AxisCalibration left_brake_cal{0, 0, 32767, 0.05, 0.02, false, 0.00};

    int right_brake_device{0};
    int right_brake_axis{5};
    AxisCalibration right_brake_cal{0, 0, 32767, 0.05, 0.02, false, 0.00};

    // Button Bindings (button index)
    int btn_trim_up{0};
    int btn_trim_down{1};
    int btn_trim_left{2};
    int btn_trim_right{3};
    int btn_speedbrake_extend{4};
    int btn_speedbrake_retract{5};
    int btn_parking_brake{6};
    int btn_stovl_mode{7}; ///< F-35B STOVL/CTOL conversion button (each press toggles)

    /**
     * @brief Serializes profile to formatted JSON string.
     */
    [[nodiscard]] std::string to_json() const {
        std::ostringstream ss;
        ss << "{\n";
        ss << "  \"profile_name\": \"" << profile_name << "\",\n";

        auto write_axis = [&ss](const std::string& name, int dev, int ax, const AxisCalibration& c, bool trailing_comma = true) {
            ss << "  \"" << name << "\": {\n";
            ss << "    \"device\": " << dev << ",\n";
            ss << "    \"axis\": " << ax << ",\n";
            ss << "    \"raw_min\": " << c.raw_min << ",\n";
            ss << "    \"raw_center\": " << c.raw_center << ",\n";
            ss << "    \"raw_max\": " << c.raw_max << ",\n";
            ss << "    \"inner_deadband\": " << c.inner_deadband << ",\n";
            ss << "    \"outer_deadband\": " << c.outer_deadband << ",\n";
            ss << "    \"inverted\": " << (c.inverted ? "true" : "false") << ",\n";
            ss << "    \"curvature\": " << c.curvature << "\n";
            ss << "  }" << (trailing_comma ? "," : "") << "\n";
        };

        write_axis("pitch", pitch_device, pitch_axis, pitch_cal);
        write_axis("roll", roll_device, roll_axis, roll_cal);
        write_axis("yaw", yaw_device, yaw_axis, yaw_cal);
        write_axis("throttle", throttle_device, throttle_axis, throttle_cal);
        write_axis("left_brake", left_brake_device, left_brake_axis, left_brake_cal);
        write_axis("right_brake", right_brake_device, right_brake_axis, right_brake_cal);

        ss << "  \"buttons\": {\n";
        ss << "    \"trim_up\": " << btn_trim_up << ",\n";
        ss << "    \"trim_down\": " << btn_trim_down << ",\n";
        ss << "    \"trim_left\": " << btn_trim_left << ",\n";
        ss << "    \"trim_right\": " << btn_trim_right << ",\n";
        ss << "    \"speedbrake_extend\": " << btn_speedbrake_extend << ",\n";
        ss << "    \"speedbrake_retract\": " << btn_speedbrake_retract << ",\n";
        ss << "    \"parking_brake\": " << btn_parking_brake << ",\n";
        ss << "    \"stovl_mode\": " << btn_stovl_mode << "\n";
        ss << "  }\n";
        ss << "}\n";

        return ss.str();
    }

    /**
     * @brief Parses JSON string to populate profile.
     */
    bool from_json(const std::string& json_str) {
        auto extract_string = [&json_str](const std::string& key) -> std::string {
            const std::size_t pos = json_str.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            const std::size_t colon = json_str.find(':', pos);
            if (colon == std::string::npos) return "";
            const std::size_t q1 = json_str.find('"', colon);
            if (q1 == std::string::npos) return "";
            const std::size_t q2 = json_str.find('"', q1 + 1);
            if (q2 == std::string::npos) return "";
            return json_str.substr(q1 + 1, q2 - q1 - 1);
        };

        const std::string parsed_name = extract_string("profile_name");
        if (!parsed_name.empty()) {
            profile_name = parsed_name;
        }

        auto extract_from_block = [](const std::string& block_str, const std::string& key) -> std::string {
            const std::size_t pos = block_str.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            const std::size_t colon = block_str.find(':', pos);
            if (colon == std::string::npos) return "";
            std::size_t end = block_str.find_first_of(",\n}\r", colon);
            std::string val = block_str.substr(colon + 1, (end != std::string::npos) ? (end - colon - 1) : std::string::npos);
            // Trim whitespace and quotes
            const auto first = val.find_first_not_of(" \t\"");
            const auto last = val.find_last_not_of(" \t\"");
            return (first != std::string::npos && last != std::string::npos) ? val.substr(first, last - first + 1) : "";
        };

        auto parse_axis = [&](const std::string& name, int& dev, int& ax, AxisCalibration& c) {
            const std::size_t block_pos = json_str.find("\"" + name + "\"");
            if (block_pos == std::string::npos) return;
            const std::size_t brace_open = json_str.find('{', block_pos);
            if (brace_open == std::string::npos) return;
            const std::size_t brace_close = json_str.find('}', brace_open);
            if (brace_close == std::string::npos) return;
            const std::string block_str = json_str.substr(brace_open, brace_close - brace_open + 1);

            std::string s_dev = extract_from_block(block_str, "device");
            std::string s_ax  = extract_from_block(block_str, "axis");
            std::string s_min = extract_from_block(block_str, "raw_min");
            std::string s_cen = extract_from_block(block_str, "raw_center");
            std::string s_max = extract_from_block(block_str, "raw_max");
            std::string s_idb = extract_from_block(block_str, "inner_deadband");
            std::string s_odb = extract_from_block(block_str, "outer_deadband");
            std::string s_inv = extract_from_block(block_str, "inverted");
            std::string s_crv = extract_from_block(block_str, "curvature");

            if (!s_dev.empty()) dev = std::stoi(s_dev);
            if (!s_ax.empty())  ax  = std::stoi(s_ax);
            if (!s_min.empty()) c.raw_min = std::stoi(s_min);
            if (!s_cen.empty()) c.raw_center = std::stoi(s_cen);
            if (!s_max.empty()) c.raw_max = std::stoi(s_max);
            if (!s_idb.empty()) c.inner_deadband = std::stod(s_idb);
            if (!s_odb.empty()) c.outer_deadband = std::stod(s_odb);
            if (!s_inv.empty()) c.inverted = (s_inv == "true");
            if (!s_crv.empty()) c.curvature = std::stod(s_crv);
        };

        parse_axis("pitch", pitch_device, pitch_axis, pitch_cal);
        parse_axis("roll", roll_device, roll_axis, roll_cal);
        parse_axis("yaw", yaw_device, yaw_axis, yaw_cal);
        parse_axis("throttle", throttle_device, throttle_axis, throttle_cal);
        parse_axis("left_brake", left_brake_device, left_brake_axis, left_brake_cal);
        parse_axis("right_brake", right_brake_device, right_brake_axis, right_brake_cal);

        // Parse buttons block
        const std::size_t btn_block_pos = json_str.find("\"buttons\"");
        if (btn_block_pos != std::string::npos) {
            const std::size_t b_open = json_str.find('{', btn_block_pos);
            const std::size_t b_close = json_str.find('}', b_open);
            if (b_open != std::string::npos && b_close != std::string::npos) {
                const std::string b_str = json_str.substr(b_open, b_close - b_open + 1);
                auto get_btn = [&](const std::string& key, int fallback = 0) -> int {
                    std::string s = extract_from_block(b_str, key);
                    return s.empty() ? fallback : std::stoi(s);
                };
                btn_trim_up = get_btn("trim_up");
                btn_trim_down = get_btn("trim_down");
                btn_trim_left = get_btn("trim_left");
                btn_trim_right = get_btn("trim_right");
                btn_speedbrake_extend = get_btn("speedbrake_extend");
                btn_speedbrake_retract = get_btn("speedbrake_retract");
                btn_parking_brake = get_btn("parking_brake");
                // Added later: a profile saved before it keeps the default
                // rather than landing on button 0 (trim up).
                btn_stovl_mode = get_btn("stovl_mode", btn_stovl_mode);
            }
        }

        return true;
    }

    bool save_to_file(const std::string& filepath) const {
        std::ofstream ofs(filepath);
        if (!ofs.is_open()) return false;
        ofs << to_json();
        return ofs.good();
    }

    bool load_from_file(const std::string& filepath) {
        std::ifstream ifs(filepath);
        if (!ifs.is_open()) return false;
        std::stringstream ss;
        ss << ifs.rdbuf();
        return from_json(ss.str());
    }
};

} // namespace input
} // namespace fastjet
