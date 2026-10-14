#pragma once

#include "fastjet/graphics/instrument_canvas.hpp"
#include "fastjet/graphics/cockpit_telemetry.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

namespace fastjet::graphics {

/// @brief Cockpit dashboard flight instruments rendered to an off-screen FBO texture
class FlightInstruments : public InstrumentCanvas {
public:
    static constexpr int TEX_WIDTH = 2048;
    static constexpr int TEX_HEIGHT = 2048;  ///< With the centre display row

    FlightInstruments() = default;

    ~FlightInstruments() {
        destroy();
    }

    /// @param centre_page Also draw the centre display. Without it the atlas
    /// holds only the MFD row, at half the height: the same pixels per
    /// display, with half the clearing and mipmapping each frame.
    bool init(bool centre_page = true) {
        centre_page_ = centre_page;
        return init_canvas(TEX_WIDTH, centre_page ? TEX_HEIGHT : TEX_HEIGHT / 2);
    }

    /// @brief Adds or drops the centre display, rebuilding the atlas at the
    /// matching size (cockpits differ in how many displays they carry).
    bool set_centre_page(bool centre_page) {
        if (initialized_ && centre_page == centre_page_) return true;
        destroy_canvas();
        return init(centre_page);
    }

    /// @brief Atlas V range of the MFD row (the two MFD pages).
    [[nodiscard]] float mfd_row_v0() const noexcept { return centre_page_ ? 0.5f : 0.0f; }

    void destroy() noexcept {
        destroy_canvas();
    }

    /// @brief Left MFD page: AoA indexer, ADI ball, Mach tape, plus Landing Gear & Surface status panel
    void draw_left_mfd(const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        // Background: MFD dark screen bezel
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, Color4{0.04f, 0.05f, 0.06f, 1.0f});

        const double alpha_deg = state.alpha() * (180.0 / 3.14159265);
        const double pitch_rad = state.pitch();
        const double roll_rad  = state.roll();

        // =========================================================================
        // 1. Three-Light Angle-of-Attack (AoA) Indexer (Left side: x in [-0.90, -0.65])
        // =========================================================================
        {
            const float ix = -0.78f;
            const Color4 off_col{0.12f, 0.12f, 0.12f, 1.0f};

            // Outer housing box
            add_rect(ix, 0.15f, 0.22f, 0.60f, Color4{0.08f, 0.09f, 0.10f, 1.0f});

            // Upper chevron (pointing down: \/)
            const bool high_aoa = (alpha_deg > 14.0);
            const Color4 top_col = high_aoa ? Color4::amber() : off_col;
            add_tri(ix, 0.30f, ix - 0.08f, 0.40f, ix + 0.08f, 0.40f, top_col);

            // Center donut
            const bool on_speed = (alpha_deg >= 12.0 && alpha_deg <= 14.0);
            const Color4 mid_col = on_speed ? Color4::hud_green() : off_col;
            add_circle(ix, 0.15f, 0.055f, mid_col, 20);
            add_circle(ix, 0.15f, 0.025f, Color4{0.08f, 0.09f, 0.10f, 1.0f}, 16);

            // Lower chevron (pointing up: /\)
            const bool low_aoa = (alpha_deg < 12.0);
            const Color4 bot_col = low_aoa ? Color4::hud_green() : off_col;
            add_tri(ix, 0.00f, ix - 0.08f, -0.10f, ix + 0.08f, -0.10f, bot_col);
        }

        // =========================================================================
        // 2. Attitude Director Indicator (ADI) Ball (Center: x in [-0.45, +0.45])
        // =========================================================================
        {
            const float adi_x = 0.0f;
            const float adi_y = 0.15f;
            const float adi_r = 0.44f;

            // ADI Outer Bezel ring
            add_circle(adi_x, adi_y, adi_r + 0.03f, Color4{0.18f, 0.19f, 0.20f, 1.0f}, 32);

            const float pitch_offset = static_cast<float>(pitch_rad / (3.14159265 * 0.5)) * adi_r * 0.9f;
            const float cos_r = static_cast<float>(std::cos(-roll_rad));
            const float sin_r = static_cast<float>(std::sin(-roll_rad));

            constexpr int ADI_SEGS = 36;
            for (int i = 0; i < ADI_SEGS; ++i) {
                const float a0 = i * (2.0f * 3.14159265f / ADI_SEGS);
                const float a1 = (i + 1) * (2.0f * 3.14159265f / ADI_SEGS);

                const float lx0 = adi_r * std::cos(a0);
                const float ly0 = adi_r * std::sin(a0);
                const float lx1 = adi_r * std::cos(a1);
                const float ly1 = adi_r * std::sin(a1);

                const float mid_y = (ly0 + ly1) * 0.5f - pitch_offset;
                const Color4 hemisphere_col = (mid_y >= 0.0f) ? Color4::sky_blue() : Color4::earth_brown();

                const float rx0 = adi_x + (lx0 * cos_r - ly0 * sin_r);
                const float ry0 = adi_y + (lx0 * sin_r + ly0 * cos_r);
                const float rx1 = adi_x + (lx1 * cos_r - ly1 * sin_r);
                const float ry1 = adi_y + (lx1 * sin_r + ly1 * cos_r);

                add_tri(adi_x, adi_y, rx0, ry0, rx1, ry1, hemisphere_col);
            }

            // White Horizon line across the ADI
            const float hx0 = -adi_r * cos_r - pitch_offset * (-sin_r);
            const float hy0 = -adi_r * sin_r + pitch_offset * (cos_r);
            const float hx1 =  adi_r * cos_r - pitch_offset * (-sin_r);
            const float hy1 =  adi_r * sin_r + pitch_offset * (cos_r);
            add_line(adi_x + hx0, adi_y + hy0, adi_x + hx1, adi_y + hy1, Color4::white());

            // Miniature Aircraft Symbol
            const Color4 symbol_col{1.0f, 0.85f, 0.0f, 1.0f};
            add_circle(adi_x, adi_y, 0.02f, symbol_col, 12);
            add_rect(adi_x - 0.12f, adi_y, 0.12f, 0.02f, symbol_col);
            add_rect(adi_x - 0.06f, adi_y - 0.03f, 0.02f, 0.04f, symbol_col);
            add_rect(adi_x + 0.12f, adi_y, 0.12f, 0.02f, symbol_col);
            add_rect(adi_x + 0.06f, adi_y - 0.03f, 0.02f, 0.04f, symbol_col);
        }

        // =========================================================================
        // 3. Mach Number Tape (Right side: x in [+0.60, +0.90])
        // =========================================================================
        {
            const float mach_x = 0.75f;
            const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
            const float mach = static_cast<float>(air.mach_number);

            add_rect(mach_x, 0.15f, 0.24f, 0.65f, Color4{0.08f, 0.09f, 0.10f, 1.0f});
            draw_string("M", mach_x - 0.04f, 0.40f, 0.08f, Color4::hud_green());

            char m_str[8];
            std::snprintf(m_str, sizeof(m_str), "%1.2f", std::clamp(mach, 0.0f, 9.99f));
            draw_string(m_str, mach_x - 0.09f, 0.28f, 0.07f, Color4::white());

            const float bar_bottom = -0.12f;
            const float bar_max_h  = 0.35f;
            const float fill_frac  = std::clamp(mach / 2.5f, 0.0f, 1.0f);
            const float bar_h      = bar_max_h * fill_frac;

            add_rect(mach_x, bar_bottom + bar_max_h * 0.5f, 0.08f, bar_max_h, Color4{0.15f, 0.16f, 0.18f, 1.0f});
            if (bar_h > 0.005f) {
                const Color4 fill_col = (mach >= 1.0f) ? Color4::amber() : Color4::hud_green();
                add_rect(mach_x, bar_bottom + bar_h * 0.5f, 0.07f, bar_h, fill_col);
            }
        }

        // =========================================================================
        // 4. Landing Gear, Speedbrakes & Wheel Brakes Panel (Lower Quadrant)
        // =========================================================================
        {
            const float py = -0.65f;
            add_rect(0.0f, py, 1.94f, 0.55f, Color4{0.06f, 0.07f, 0.09f, 1.0f});
            add_line(-0.97f, py + 0.275f, 0.97f, py + 0.275f, Color4{0.25f, 0.28f, 0.30f, 1.0f});

            // 4a. Three-Green Landing Gear Indicator Box
            const float gx = -0.62f;
            add_rect(gx, py, 0.58f, 0.46f, Color4{0.03f, 0.03f, 0.04f, 1.0f});
            draw_string("GEAR", gx - 0.12f, py + 0.16f, 0.065f, Color4{0.6f, 0.7f, 0.8f, 1.0f});

            Color4 nose_col = Color4{0.18f, 0.20f, 0.22f, 1.0f};
            Color4 left_col = nose_col;
            Color4 right_col = nose_col;

            if (tel.gear_collapsed) {
                nose_col = left_col = right_col = Color4::red();
            } else if (tel.gear_deployed && tel.gear_transit_pos >= 0.98) {
                nose_col = left_col = right_col = Color4::hud_green();
            } else if (tel.gear_deployed && tel.gear_transit_pos < 0.98) {
                nose_col = left_col = right_col = Color4::amber();
            }

            // Nose gear lamp
            add_circle(gx, py + 0.04f, 0.045f, nose_col, 16);
            draw_string("N", gx - 0.02f, py + 0.04f, 0.050f, Color4::black());

            // Left main gear lamp
            add_circle(gx - 0.14f, py - 0.07f, 0.045f, left_col, 16);
            draw_string("L", gx - 0.16f, py - 0.07f, 0.050f, Color4::black());

            // Right main gear lamp
            add_circle(gx + 0.14f, py - 0.07f, 0.045f, right_col, 16);
            draw_string("R", gx + 0.12f, py - 0.07f, 0.050f, Color4::black());

            if (tel.gear_collapsed) {
                draw_string("COLLAPSE", gx - 0.22f, py - 0.17f, 0.055f, Color4::red());
            } else if (!tel.gear_deployed) {
                draw_string("UP / LOCKED", gx - 0.25f, py - 0.17f, 0.055f, Color4{0.5f, 0.5f, 0.5f, 1.0f});
            } else {
                draw_string("DOWN / LOCKED", gx - 0.26f, py - 0.17f, 0.055f, Color4::hud_green());
            }

            // 4b. Speedbrake Status Box (Center)
            const float sx = 0.0f;
            add_rect(sx, py, 0.58f, 0.46f, Color4{0.03f, 0.03f, 0.04f, 1.0f});
            draw_string("SPDBRK", sx - 0.18f, py + 0.16f, 0.065f, Color4{0.6f, 0.7f, 0.8f, 1.0f});

            const bool sb_out = (tel.speedbrake_pos > 0.02);
            const Color4 sb_col = sb_out ? Color4::amber() : Color4::hud_green();
            char sb_buf[16];
            std::snprintf(sb_buf, sizeof(sb_buf), "%3.0f%%", tel.speedbrake_pos * 100.0);
            draw_string(sb_out ? "EXT" : "RET", sx - 0.18f, py + 0.03f, 0.070f, sb_col);
            draw_string(sb_buf, sx + 0.02f, py + 0.03f, 0.065f, sb_col);

            add_rect(sx, py - 0.10f, 0.46f, 0.055f, Color4{0.15f, 0.16f, 0.18f, 1.0f});
            if (tel.speedbrake_pos > 0.02) {
                const float bar_w = 0.46f * static_cast<float>(tel.speedbrake_pos);
                add_rect(sx - 0.23f + bar_w * 0.5f, py - 0.10f, bar_w, 0.045f, Color4::amber());
            }

            // 4c. Wheel Brakes & Caution/Warning Box (Right side)
            const float bx = 0.62f;
            add_rect(bx, py, 0.58f, 0.46f, Color4{0.03f, 0.03f, 0.04f, 1.0f});
            draw_string("SYSTEMS", bx - 0.18f, py + 0.16f, 0.065f, Color4{0.6f, 0.7f, 0.8f, 1.0f});

            const bool braking = (tel.brake_left > 0.05 || tel.brake_right > 0.05);
            add_rect(bx, py + 0.03f, 0.48f, 0.08f, braking ? Color4::amber() : Color4{0.15f, 0.16f, 0.18f, 1.0f});
            draw_string("WHEEL BRAKE", bx - 0.22f, py + 0.03f, 0.055f, braking ? Color4::black() : Color4{0.5f, 0.5f, 0.5f, 1.0f});

            if (tel.is_crashed) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::red());
                draw_string("IMPACT / CRASH", bx - 0.22f, py - 0.10f, 0.055f, Color4::white());
            } else if (tel.structure_failed) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::red());
                draw_string("STRUCTURE FAILED", bx - 0.23f, py - 0.10f, 0.050f, Color4::white());
            } else if (tel.ofc_tumble_active) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("ANTI-SPIN RECOVER", bx - 0.23f, py - 0.10f, 0.048f, Color4::black());
            } else if (tel.ofc_gcas_active) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("AUTO-GCAS PULL", bx - 0.22f, py - 0.10f, 0.050f, Color4::black());
            } else if (tel.ofc_gloc_active) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("GLOC AUTO-REC", bx - 0.22f, py - 0.10f, 0.050f, Color4::black());
            } else if (tel.over_g_alert) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("OVER-G WARNING", bx - 0.22f, py - 0.10f, 0.055f, Color4::black());
            } else if (tel.airframe_yielded) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::red());
                draw_string("WING OVERSTRESS", bx - 0.22f, py - 0.10f, 0.050f, Color4::white());
            } else if (tel.high_aoa_alert) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("HIGH AOA / STALL", bx - 0.23f, py - 0.10f, 0.050f, Color4::black());
            } else if (tel.bingo_fuel_alert) {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4::amber());
                draw_string("LOW FUEL BINGO", bx - 0.22f, py - 0.10f, 0.050f, Color4::black());
            } else {
                add_rect(bx, py - 0.10f, 0.48f, 0.08f, Color4{0.08f, 0.18f, 0.10f, 1.0f});
                draw_string("FLCS NORMAL", bx - 0.20f, py - 0.10f, 0.055f, Color4::hud_green());
            }
        }
    }

    /// @brief Right MFD page: Authentic F-16 F110 Engine & Fuel Management Display (ENG/FUEL)
    void draw_right_mfd([[maybe_unused]] const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        // Background: MFD dark screen bezel
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, Color4{0.03f, 0.04f, 0.05f, 1.0f});

        // 1. Header & Title Bar with Detent Indicator
        // The powerplant is named from the active airframe: this page is shared
        // by all five aircraft, only one of which flies behind an F110.
        add_rect(0.0f, 0.88f, 1.94f, 0.16f, Color4{0.06f, 0.08f, 0.11f, 1.0f});
        char eng_hdr[48];
        std::snprintf(eng_hdr, sizeof(eng_hdr), "%.*s ENG/FUEL",
                      static_cast<int>(aircraft::engine_designation(tel.aircraft_type).size()),
                      aircraft::engine_designation(tel.aircraft_type).data());
        Color4 detent_col = Color4::hud_green();
        if (std::string(tel.detent_str) == "AFTERBURNER") detent_col = Color4::amber();
        else if (std::string(tel.detent_str) == "CUTOFF") detent_col = Color4::red();
        else if (std::string(tel.detent_str) == "IDLE") detent_col = Color4::sky_blue();

        char det_buf[32];
        std::snprintf(det_buf, sizeof(det_buf), "[ %s ]", tel.detent_str);
        // Header left, detent right-aligned to the same margin; the header
        // shrinks if a long engine name would run into the detent.
        constexpr float kAdv = 0.85f; // draw_string cell advance per unit size
        const float det_size = 0.070f;
        const float det_x = 0.92f - static_cast<float>(std::strlen(det_buf)) * det_size * kAdv;
        const float hdr_room = det_x - 0.05f - (-0.92f);
        const float hdr_size = std::min(0.070f, hdr_room / (static_cast<float>(std::strlen(eng_hdr)) * kAdv));
        draw_string(eng_hdr, -0.92f, 0.88f, hdr_size, Color4::hud_green());
        draw_string(det_buf, det_x, 0.88f, det_size, detent_col);

        // 2. Engine Primary Parameters (Left column)
        const float col1_x = -0.48f;

        // Core RPM %
        {
            const float y = 0.65f;
            draw_string("RPM %", col1_x - 0.42f, y, 0.065f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%5.1f", tel.engine_rpm_pct);
            const Color4 rpm_col = (tel.engine_rpm_pct > 100.0) ? Color4::amber() : Color4::hud_green();
            draw_string(buf, col1_x + 0.10f, y, 0.075f, rpm_col);

            add_rect(col1_x, y - 0.08f, 0.84f, 0.04f, Color4{0.12f, 0.13f, 0.15f, 1.0f});
            const float rpm_frac = std::clamp(static_cast<float>(tel.engine_rpm_pct / 110.0), 0.0f, 1.0f);
            if (rpm_frac > 0.01f) {
                add_rect(col1_x - 0.42f + (0.84f * rpm_frac) * 0.5f, y - 0.08f, 0.84f * rpm_frac, 0.035f, rpm_col);
            }
        }

        // Fan Turbine Inlet Temp (FTIT C)
        {
            const float y = 0.40f;
            draw_string("FTIT C", col1_x - 0.42f, y, 0.065f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%4.0f", tel.engine_ftit_deg_c);
            const Color4 ftit_col = (tel.engine_ftit_deg_c > 920.0) ? Color4::amber() : Color4::hud_green();
            draw_string(buf, col1_x + 0.12f, y, 0.075f, ftit_col);

            add_rect(col1_x, y - 0.08f, 0.84f, 0.04f, Color4{0.12f, 0.13f, 0.15f, 1.0f});
            const float ftit_frac = std::clamp(static_cast<float>(tel.engine_ftit_deg_c / 1100.0), 0.0f, 1.0f);
            if (ftit_frac > 0.01f) {
                add_rect(col1_x - 0.42f + (0.84f * ftit_frac) * 0.5f, y - 0.08f, 0.84f * ftit_frac, 0.035f, ftit_col);
            }
        }

        // Net Thrust & Exhaust Nozzle Position
        {
            const float y = 0.16f;
            draw_string("THRUST", col1_x - 0.42f, y, 0.065f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            char buf[20];
            std::snprintf(buf, sizeof(buf), "%4.1f kN", tel.net_thrust_n / 1000.0);
            draw_string(buf, col1_x + 0.02f, y, 0.070f, Color4::hud_green());

            draw_string("NOZZLE", col1_x - 0.42f, y - 0.12f, 0.065f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            std::snprintf(buf, sizeof(buf), "%3.0f %%", tel.nozzle_pos_pct);
            draw_string(buf, col1_x + 0.08f, y - 0.12f, 0.070f, Color4::hud_green());
        }

        // 3. Secondary Parameters & Pressures (Right column)
        const float col2_x = 0.52f;
        {
            const float y = 0.65f;
            draw_string("OIL PRESS", col2_x - 0.38f, y, 0.060f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            char buf[20];
            std::snprintf(buf, sizeof(buf), "%2.0f PSI", tel.oil_pressure_psi);
            draw_string(buf, col2_x + 0.10f, y, 0.065f, Color4::hud_green());

            draw_string("HYD SYS A", col2_x - 0.38f, y - 0.12f, 0.060f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            std::snprintf(buf, sizeof(buf), "%4.0f", tel.hyd_press_a_psi);
            draw_string(buf, col2_x + 0.10f, y - 0.12f, 0.065f, Color4::hud_green());

            draw_string("HYD SYS B", col2_x - 0.38f, y - 0.24f, 0.060f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            std::snprintf(buf, sizeof(buf), "%4.0f", tel.hyd_press_b_psi);
            draw_string(buf, col2_x + 0.10f, y - 0.24f, 0.065f, Color4::hud_green());

            draw_string("THROTTLE", col2_x - 0.38f, y - 0.38f, 0.060f, Color4{0.7f, 0.75f, 0.8f, 1.0f});
            std::snprintf(buf, sizeof(buf), "%3.0f %%", tel.throttle_input * 100.0);
            draw_string(buf, col2_x + 0.10f, y - 0.38f, 0.065f, Color4::hud_green());
            add_rect(col2_x, y - 0.46f, 0.76f, 0.035f, Color4{0.12f, 0.13f, 0.15f, 1.0f});
            const float th_frac = std::clamp(static_cast<float>(tel.throttle_input), 0.0f, 1.0f);
            if (th_frac > 0.01f) {
                add_rect(col2_x - 0.38f + (0.76f * th_frac) * 0.5f, y - 0.46f, 0.76f * th_frac, 0.030f, detent_col);
            }
        }

        // 4. Fuel Subsystem Section (Lower half)
        {
            const float fy = -0.32f;
            add_line(-0.95f, fy + 0.20f, 0.95f, fy + 0.20f, Color4{0.20f, 0.22f, 0.25f, 1.0f});

            draw_string("INTERNAL FUEL SYSTEM", -0.90f, fy + 0.10f, 0.065f, Color4{0.6f, 0.7f, 0.8f, 1.0f});

            char buf[32];
            std::snprintf(buf, sizeof(buf), "TOTAL: %4.0f KG", tel.fuel_remaining_kg);
            const Color4 fuel_col = (tel.fuel_remaining_kg < 800.0) ? Color4::amber() : Color4::hud_green();
            draw_string(buf, -0.90f, fy - 0.04f, 0.075f, fuel_col);

            std::snprintf(buf, sizeof(buf), "FLOW: %4.0f KG/H", tel.fuel_flow_kg_hr);
            draw_string(buf, 0.08f, fy - 0.04f, 0.070f, Color4::hud_green());

            // Fuel quantity bar
            add_rect(0.0f, fy - 0.16f, 1.80f, 0.06f, Color4{0.12f, 0.13f, 0.15f, 1.0f});
            const float f_frac = std::clamp(static_cast<float>(tel.fuel_fraction), 0.0f, 1.0f);
            if (f_frac > 0.01f) {
                add_rect(-0.90f + (1.80f * f_frac) * 0.5f, fy - 0.16f, 1.80f * f_frac, 0.05f, fuel_col);
            }

            // Inceptor control mode indicator box
            add_rect(0.0f, -0.74f, 1.84f, 0.16f, Color4{0.06f, 0.07f, 0.09f, 1.0f});
            draw_string("CONTROL INCEPTOR:", -0.88f, -0.72f, 0.055f, Color4{0.5f, 0.6f, 0.7f, 1.0f});
            char ctrl_buf[48];
            std::snprintf(ctrl_buf, sizeof(ctrl_buf), "%s", tel.input_name);
            // The value starts after the label (17 cells of 0.85 x 0.055).
            draw_string(ctrl_buf, -0.88f + 17.0f * 0.055f * 0.85f + 0.03f, -0.72f, 0.060f,
                        tel.is_hardware_hotas ? Color4::hud_green() : Color4::sky_blue());

            draw_string("[R] RESET SIM   [T] TRIM ZERO   [I] INV THROTTLE   [B] SPDBRK", -0.88f, -0.80f, 0.032f, Color4{0.45f, 0.50f, 0.55f, 1.0f});
        }
    }

    /// @brief Centre display (CPD): horizontal situation, compass rose, and DED scratchpad
    void draw_center_cpd(const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, Color4{0.03f, 0.04f, 0.05f, 1.0f});

        const Color4 hsd_line{0.15f, 0.75f, 0.35f, 1.0f};
        const Color4 dim_line{0.10f, 0.40f, 0.20f, 1.0f};
        const Color4 label{0.75f, 0.80f, 0.75f, 1.0f};

        const float heading = static_cast<float>(state.yaw());

        // Compass rose
        const float rose_r = 0.55f;
        const float rose_y = -0.15f;
        add_circle_outline(0.0f, rose_y, rose_r, hsd_line, 48);
        add_circle_outline(0.0f, rose_y, rose_r * 0.66f, dim_line, 40);
        add_circle_outline(0.0f, rose_y, rose_r * 0.33f, dim_line, 32);

        for (int deg = 0; deg < 360; deg += 10) {
            const float rel = static_cast<float>(deg) * 3.14159265f / 180.0f - heading;
            const float ux = std::sin(rel);
            const float uy = std::cos(rel);
            const bool major = (deg % 30 == 0);
            const float t_in = major ? 0.86f : 0.93f;
            add_line(ux * rose_r * t_in, rose_y + uy * rose_r * t_in,
                     ux * rose_r, rose_y + uy * rose_r,
                     major ? hsd_line : dim_line);
        }

        struct { const char* s; int deg; } cardinals[] = {
            {"N", 0}, {"E", 90}, {"S", 180}, {"W", 270}
        };
        for (const auto& c : cardinals) {
            const float rel = static_cast<float>(c.deg) * 3.14159265f / 180.0f - heading;
            const float lx = std::sin(rel) * rose_r * 0.76f;
            const float ly = std::cos(rel) * rose_r * 0.76f + rose_y;
            draw_string(c.s, lx - 0.03f, ly, 0.085f, label);
        }

        // Ownship symbol
        const Color4 own{1.0f, 0.85f, 0.0f, 1.0f};
        add_tri(0.0f, rose_y + 0.07f, -0.05f, rose_y - 0.05f, 0.05f, rose_y - 0.05f, own);

        // Steerpoint route
        {
            const float leg_bearing = 0.6f;
            const float rel = leg_bearing - heading;
            const float ex = std::sin(rel) * rose_r * 0.9f;
            const float ey = std::cos(rel) * rose_r * 0.9f + rose_y;
            add_line(0.0f, rose_y, ex, ey, Color4{0.95f, 0.55f, 0.10f, 1.0f});
            add_circle_outline(ex, ey, 0.045f, Color4{0.95f, 0.55f, 0.10f, 1.0f}, 12);
        }

        // Data Entry Display (DED) Scratchpad at the top
        {
            const float ded_y = 0.68f;
            add_rect(0.0f, ded_y, 1.84f, 0.32f, Color4{0.04f, 0.10f, 0.05f, 1.0f});
            // Outline (add_rect fills: four edges instead).
            add_line(-0.92f, ded_y - 0.16f, 0.92f, ded_y - 0.16f, Color4::hud_green());
            add_line(-0.92f, ded_y + 0.16f, 0.92f, ded_y + 0.16f, Color4::hud_green());
            add_line(-0.92f, ded_y - 0.16f, -0.92f, ded_y + 0.16f, Color4::hud_green());
            add_line(0.92f, ded_y - 0.16f, 0.92f, ded_y + 0.16f, Color4::hud_green());

            draw_string("DED: NAV/TRK   STPT: 01", -0.84f, ded_y + 0.09f, 0.060f, Color4::hud_green());

            char b_buf[48];
            std::snprintf(b_buf, sizeof(b_buf), "BINGO: 800 KG   QTY: %4.0f KG", tel.fuel_remaining_kg);
            draw_string(b_buf, -0.84f, ded_y - 0.01f, 0.060f, (tel.fuel_remaining_kg < 800.0) ? Color4::amber() : Color4::hud_green());

            draw_string("WIND: 045/10 KT   BARO: 29.92", -0.84f, ded_y - 0.11f, 0.055f, Color4::hud_green());
        }

        // Ground speed and altitude blocks along the bottom edge
        {
            char buf[24];
            const double gs_kt = state.airspeed() * 1.94384;
            std::snprintf(buf, sizeof(buf), "%3.0f", std::clamp(gs_kt, 0.0, 999.0));
            draw_string("GS", -0.92f, -0.86f, 0.07f, label);
            draw_string(buf, -0.74f, -0.86f, 0.085f, Color4::hud_green());

            const double alt_ft = state.altitude() * 3.28084;
            std::snprintf(buf, sizeof(buf), "%5.0f", std::clamp(alt_ft, -999.0, 99999.0));
            draw_string("ALT", 0.34f, -0.86f, 0.07f, label);
            draw_string(buf, 0.60f, -0.86f, 0.085f, Color4::hud_green());
        }
    }

    /// @brief Left MFD, FCR page: the APG-68 B-scope in CRM (RWS / STT) and
    /// ACM (auto-acquisition search / STT).
    ///
    /// Laid out like the F-16's colour MFD: OSB legends round the edge (mode
    /// top left, range scale with its bump arrows and the azimuth/bar settings
    /// down the left, page selectors along the bottom), the scope in the
    /// middle with range on the vertical axis and azimuth across it, the
    /// artificial horizon, antenna azimuth and elevation carets, and the
    /// acquisition cursor with the altitude block the scan covers at its
    /// range. RWS shows bricks only; STT adds the target's velocity vector,
    /// its altitude and the aspect / heading / speed / closure data line.
    /// ACM search shows no bricks, only the azimuth and elevation its scan
    /// pattern covers; the first jet it finds becomes an STT track. Over all
    /// of it lies the datalink picture: every other jet in red, radar
    /// contact or not, pinned to the scope edge when it is off the scope.
    void draw_fcr(const RadarTelemetry& r) {
        const Color4 glass{0.008f, 0.012f, 0.010f, 1.0f};
        const Color4 ink{0.30f, 1.0f, 0.38f, 1.0f};
        const Color4 dim{0.10f, 0.44f, 0.15f, 1.0f};
        const Color4 brick_col[3] = {{0.82f, 1.0f, 0.84f, 1.0f}, {0.46f, 0.70f, 0.50f, 1.0f},
                                     {0.26f, 0.42f, 0.29f, 1.0f}};
        constexpr float TS = 0.072f;   // legend text size
        // Scope: azimuth across [-SX, SX], range up from Y0 to Y1.
        constexpr float SX = 0.72f, Y0 = -0.72f, Y1 = 0.70f;
        constexpr float YM = 0.5f * (Y0 + Y1);
        // Soft key positions: the F-16C's MFD, or the Typhoon MHDD's
        // five keys a side (the same scope as its ATTACK format).
        static constexpr float kOsbF16[5] = {-0.64f, -0.32f, 0.0f, 0.32f, 0.64f};
        static constexpr float kOsbMhdd[5] = {-0.80f, -0.40f, 0.0f, 0.40f, 0.80f};
        const float* OSB = typhoon_ ? kOsbMhdd : kOsbF16; // top and bottom rows
        const float* OSBS = kOsbF16;                       // side columns (both displays)

        add_rect(0.0f, 0.0f, 2.0f, 2.0f, glass);
        auto seg = [&](float x0, float y0, float x1, float y1, const Color4& c, float w = 0.009f) {
            add_bar(x0, y0, x1, y1, w, c);
        };
        auto az_x = [&](double az_deg) {
            return static_cast<float>(std::clamp(az_deg / r.az_limit_deg, -1.0, 1.0)) * SX;
        };
        auto range_y = [&](double nm) {
            return Y0 + static_cast<float>(std::clamp(nm / r.range_scale_nm, 0.0, 1.0)) * (Y1 - Y0);
        };
        auto in_scope = [&](double nm) { return nm >= 0.0 && nm <= r.range_scale_nm; };
        auto boxed = [&](const char* s, float x, float y, const Color4& c) {
            const float w = text_width(s, TS) + 0.035f;
            draw_text(s, x, y, TS, c, 0);
            const float b = y - 0.022f, t = y + 0.068f;
            seg(x - 0.5f * w, b, x + 0.5f * w, b, c, 0.007f);
            seg(x - 0.5f * w, t, x + 0.5f * w, t, c, 0.007f);
            seg(x - 0.5f * w, b, x - 0.5f * w, t, c, 0.007f);
            seg(x + 0.5f * w, b, x + 0.5f * w, t, c, 0.007f);
        };

        // OSB legends: mode row along the top, page selectors along the bottom.
        const bool stt = r.mode == 2;
        const bool acm_search = r.mode == 3;
        static constexpr const char* kAcmScan[3] = {"20", "60", "BORE"};
        draw_text(r.acm ? "ACM" : typhoon_ ? "AA" : "CRM", OSB[0], 0.875f, TS, ink, 0);
        draw_text(stt ? "STT" : r.acm ? kAcmScan[std::clamp(r.acm_scan, 0, 2)] : "RWS", OSB[1], 0.875f, TS, ink, 0);
        draw_text(typhoon_ ? "MPRF" : "NORM", OSB[2], 0.875f, TS, ink, 0);
        draw_text(typhoon_ ? "IFF" : "OVRD", OSB[3], 0.875f, TS, ink, 0);
        draw_text(typhoon_ ? "DATA" : "CNTL", OSB[4], 0.875f, TS, ink, 0);
        if (typhoon_) {
            // Format selection along the bottom, the ATTACK format boxed.
            draw_text("PA", OSB[0], -0.935f, TS, ink, 0);
            draw_text("DASS", OSB[1], -0.935f, TS, ink, 0);
            boxed("ATTK", OSB[2], -0.935f, ink);
            draw_text("WPN", OSB[3], -0.935f, TS, ink, 0);
            draw_text("SYS", OSB[4], -0.935f, TS, ink, 0);
        } else {
            draw_text("DCLT", OSB[0], -0.935f, TS, ink, 0);
            draw_text("SWAP", OSB[1], -0.935f, TS, ink, 0);
            boxed("FCR", OSB[2], -0.935f, ink);
            draw_text("FLCS", OSB[3], -0.935f, TS, ink, 0);
            draw_text("SMS", OSB[4], -0.935f, TS, ink, 0);
        }

        if (!r.fitted || r.mode == 0) {
            draw_text(r.fitted ? "RDR OFF" : "NO RADAR", 0.0f, -0.02f, 0.10f, ink, 0);
            return;
        }

        // Range scale and its bump arrows (OSB 20/19), azimuth and bars (OSB 18/17).
        {
            const float x = -0.905f;
            add_tri(x, OSBS[4] + 0.035f, x - 0.035f, OSBS[4] - 0.025f, x + 0.035f, OSBS[4] - 0.025f, ink);
            add_tri(x, OSBS[3] - 0.035f, x - 0.035f, OSBS[3] + 0.025f, x + 0.035f, OSBS[3] + 0.025f, ink);
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(r.range_scale_nm + 0.5));
            draw_text(buf, x, 0.5f * (OSBS[3] + OSBS[4]) - 0.025f, TS, ink, 0);
            if (!r.acm) {
                std::snprintf(buf, sizeof(buf), "A%d", static_cast<int>(r.scan_az_deg / 10.0 + 0.5));
                draw_text(buf, -0.965f, OSBS[2] - 0.025f, TS, ink);
                std::snprintf(buf, sizeof(buf), "%dB", r.bars);
                draw_text(buf, -0.965f, OSBS[1] - 0.025f, TS, ink);
            }
        }

        // Range tics at quarter scale on both edges, azimuth tics every 30 deg.
        for (int q = 1; q <= 3; ++q) {
            const float y = Y0 + (Y1 - Y0) * 0.25f * static_cast<float>(q);
            const float len = q == 2 ? 0.06f : 0.035f;
            seg(-SX, y, -SX + len, y, dim);
            seg(SX - len, y, SX, y, dim);
        }
        for (int a = -60; a <= 60; a += 30) {
            const float x = az_x(a);
            seg(x, Y1 - 0.04f, x, Y1, dim);
            seg(x, Y0, x, Y0 + 0.04f, dim);
        }

        // Artificial horizon, with the usual centre gap, clipped to the scope.
        {
            // Bank right and the horizon rolls the other way: its right end rises.
            const float roll = static_cast<float>(r.roll_deg * kPi / 180.0);
            const float drop = static_cast<float>(std::clamp(r.pitch_deg, -60.0, 60.0)) * 0.0115f;
            const float c = std::cos(roll), s = std::sin(roll);
            const float cx = drop * s, cy = YM - drop * c;
            auto clipped = [&](float a0, float a1) {
                float x0 = cx + a0 * c, y0 = cy + a0 * s, x1 = cx + a1 * c, y1 = cy + a1 * s;
                if (clip_to(x0, y0, x1, y1, -SX, Y0, SX, Y1)) seg(x0, y0, x1, y1, ink, 0.008f);
            };
            clipped(-0.58f, -0.14f);
            clipped(0.14f, 0.58f);
        }

        // Antenna carets: azimuth under the scope, elevation left of it (+/-60 deg scale).
        {
            const float x = az_x(r.ant_az_deg);
            seg(x, Y0 - 0.02f, x, Y0 - 0.075f, ink);
            seg(x - 0.025f, Y0 - 0.075f, x + 0.025f, Y0 - 0.075f, ink);
            const float ex = -SX - 0.03f;
            for (int e = -40; e <= 40; e += 20) {
                const float y = YM + static_cast<float>(e) / 60.0f * 0.5f * (Y1 - Y0);
                seg(ex - (e == 0 ? 0.03f : 0.015f), y, ex, y, dim, 0.007f);
            }
            const float y = YM + static_cast<float>(std::clamp(r.ant_el_deg, -60.0, 60.0) / 60.0) * 0.5f * (Y1 - Y0);
            seg(ex - 0.055f, y, ex - 0.005f, y, ink);
            seg(ex - 0.055f, y - 0.025f, ex - 0.055f, y + 0.025f, ink);
        }

        // Search: dashed lines at the edges of a scan narrower than the scope,
        // and in ACM the elevation band (from the nose) on the elevation scale.
        if (!stt && r.scan_az_deg < r.az_limit_deg - 0.5) {
            for (int side = -1; side <= 1; side += 2) {
                const float x = az_x(side * r.scan_az_deg);
                for (float y = Y0; y < Y1; y += 0.08f) seg(x, y, x, std::min(y + 0.04f, Y1), dim, 0.007f);
            }
        }
        // Datalink picture: a red hollow diamond for each hostile, with its
        // track and altitude, whatever the radar is doing. One off the side of
        // the scope (behind the wing line, say) is an arrowhead pinned to that
        // side at its range; one beyond the range scale, to the top edge.
        {
            const Color4 hostile{1.0f, 0.30f, 0.24f, 1.0f};
            char alt[8];
            for (int i = 0; i < r.track_count; ++i) {
                const auto& t = r.tracks[i];
                const float y = range_y(t.range_nm);
                if (std::abs(t.az_deg) > r.az_limit_deg) {
                    const float s = t.az_deg > 0.0f ? 1.0f : -1.0f;
                    const float x = s * (SX - 0.06f);
                    add_tri(x + s * 0.055f, y, x - s * 0.03f, y + 0.05f, x - s * 0.03f, y - 0.05f, hostile);
                    continue;
                }
                const float x = az_x(t.az_deg);
                if (t.range_nm > r.range_scale_nm) {
                    add_tri(x, Y1 + 0.01f, x - 0.05f, Y1 - 0.075f, x + 0.05f, Y1 - 0.075f, hostile);
                    continue;
                }
                constexpr float D = 0.046f;
                seg(x, y + D, x + D, y, hostile, 0.007f);
                seg(x + D, y, x, y - D, hostile, 0.007f);
                seg(x, y - D, x - D, y, hostile, 0.007f);
                seg(x - D, y, x, y + D, hostile, 0.007f);
                if (t.locked && stt) continue; // The radar symbol carries its track and altitude
                const float a = t.rel_heading_deg * kPi / 180.0f;
                seg(x + std::sin(a) * D, y + std::cos(a) * D, x + std::sin(a) * 0.13f, y + std::cos(a) * 0.13f,
                    hostile, 0.007f);
                std::snprintf(alt, sizeof(alt), "%02d", static_cast<int>(std::lround(std::clamp(t.alt_kft, 0.0f, 99.0f))));
                const bool right = std::sin(a) < 0.0f;
                draw_text(alt, right ? x + 0.05f : x - 0.05f, y - 0.09f, 0.058f, hostile, right ? -1 : 1);
            }
        }

        if (acm_search) {
            const float ex = -SX - 0.03f;
            auto el_y = [&](double deg) {
                return YM + static_cast<float>(std::clamp(deg, -60.0, 60.0) / 60.0) * 0.5f * (Y1 - Y0);
            };
            seg(ex - 0.03f, el_y(r.scan_el_lo_deg), ex - 0.03f, el_y(r.scan_el_hi_deg), dim, 0.016f);
            return;
        }

        char buf[16];
        if (!stt) {
            // Search bricks, dimmer with age.
            for (int i = 0; i < r.hit_count; ++i) {
                const auto& h = r.hits[i];
                if (!in_scope(h.range_nm)) continue;
                add_rect(az_x(h.az_deg), range_y(h.range_nm), 0.052f, 0.030f,
                         brick_col[std::clamp(h.age, 0, 2)]);
            }
            // Acquisition cursor and the altitude block covered at its range.
            const float cx = az_x(r.cursor_az_deg), cy = range_y(r.cursor_range_nm);
            seg(cx - 0.042f, cy - 0.04f, cx - 0.042f, cy + 0.04f, ink, 0.008f);
            seg(cx + 0.042f, cy - 0.04f, cx + 0.042f, cy + 0.04f, ink, 0.008f);
            const float tx = cx > 0.45f ? cx - 0.07f : cx + 0.07f;
            const int align = cx > 0.45f ? 1 : -1;
            std::snprintf(buf, sizeof(buf), "%02d", std::clamp(r.cov_top_kft, -99, 99));
            draw_text(buf, tx, cy + 0.008f, 0.058f, ink, align);
            std::snprintf(buf, sizeof(buf), "%02d", std::clamp(r.cov_bottom_kft, -99, 99));
            draw_text(buf, tx, cy - 0.058f, 0.058f, ink, align);
            return;
        }

        // STT data line: aspect, target track, ground speed, closure.
        std::snprintf(buf, sizeof(buf), "%02d%c", r.aspect_tens, r.aspect_side);
        draw_text(buf, -0.60f, 0.755f, TS, ink, 0);
        std::snprintf(buf, sizeof(buf), "%03d", static_cast<int>(std::lround(r.tgt_heading_deg)) % 360);
        draw_text(buf, -0.20f, 0.755f, TS, ink, 0);
        std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(r.tgt_gs_kt)));
        draw_text(buf, 0.20f, 0.755f, TS, ink, 0);
        std::snprintf(buf, sizeof(buf), "%+d", static_cast<int>(std::lround(r.closure_kt)));
        draw_text(buf, 0.60f, 0.755f, TS, ink, 0);

        // The tracked target: filled square, velocity vector, altitude below.
        // On memory it flashes.
        if (in_scope(r.tgt_range_nm) && (!r.track_memory || r.blink)) {
            const float x = az_x(r.tgt_az_deg), y = range_y(r.tgt_range_nm);
            add_rect(x, y, 0.050f, 0.050f, ink);
            const float a = static_cast<float>(r.tgt_rel_heading_deg * kPi / 180.0);
            const float len = 0.035f + 0.14f * static_cast<float>(std::clamp(r.tgt_gs_kt / 600.0, 0.0, 1.5));
            seg(x, y, x + std::sin(a) * len, y + std::cos(a) * len, ink);
            // Altitude beside the symbol, on the side the vector does not take.
            std::snprintf(buf, sizeof(buf), "%02d", static_cast<int>(std::lround(std::clamp(r.tgt_alt_kft, 0.0, 99.0))));
            const bool right = std::sin(a) < 0.0f;
            draw_text(buf, right ? x + 0.045f : x - 0.045f, y - 0.085f, 0.060f, ink, right ? -1 : 1);
        }
    }

    /// @brief Right MFD in a fight: the 360 deg sweep display. Heading-up,
    /// ownship in the middle, range rings at quarter scale, clock ticks
    /// round the rim and the FCR's cone over the nose. The beam turns
    /// clockwise with a fading trail; each jet is a red blip where the beam
    /// last painted it, bright when fresh and dimming until the next pass,
    /// with its track and altitude. One beyond the scale sits on the rim.
    /// The bottom line calls the nearest one by clock position, and the
    /// bottom corners keep the fuel and throttle in view, since this page
    /// takes the ENG/FUEL page's place in a fight.
    void draw_sweep(const SweepTelemetry& s, const AvionicsTelemetry& tel) {
        const Color4 glass{0.008f, 0.012f, 0.010f, 1.0f};
        const Color4 ink{0.30f, 1.0f, 0.38f, 1.0f};
        const Color4 dim{0.10f, 0.44f, 0.15f, 1.0f};
        constexpr float TS = 0.072f;
        constexpr float CX = 0.0f, CY = -0.04f, R = 0.78f;
        auto faded = [](Color4 c, float a) { c.a *= std::clamp(a, 0.0f, 1.0f); return c; };
        auto at = [&](float bearing_deg, float rho, float& x, float& y) {
            const float a = bearing_deg * kPi / 180.0f;
            x = CX + rho * std::sin(a);
            y = CY + rho * std::cos(a);
        };

        add_rect(0.0f, 0.0f, 2.0f, 2.0f, glass);
        add_circle(CX, CY, R, Color4{0.02f, 0.07f, 0.035f, 1.0f}, 72);

        char buf[48];
        draw_text("SWEEP", -0.92f, 0.875f, TS, ink, -1);
        std::snprintf(buf, sizeof(buf), "%d NM", static_cast<int>(s.range_scale_nm + 0.5f));
        draw_text(buf, 0.92f, 0.875f, TS, ink, 1);

        // Range rings, clock ticks, and the cone the FCR covers.
        for (int q = 1; q <= 4; ++q) {
            const float r = R * 0.25f * static_cast<float>(q);
            const float w = q == 4 ? 0.006f : 0.0035f;
            add_ring(CX, CY, r - w, r + w, q == 4 ? ink : dim, 72);
        }
        {
            std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(s.range_scale_nm) * 0.5);
            float x, y;
            at(135.0f, R * 0.5f, x, y);
            draw_text(buf, x + 0.02f, y - 0.02f, 0.052f, dim, -1);
        }
        for (int d = 0; d < 360; d += 10) {
            const bool hour = d % 30 == 0;
            float x0, y0, x1, y1;
            at(static_cast<float>(d), R - (hour ? 0.07f : 0.035f), x0, y0);
            at(static_cast<float>(d), R, x1, y1);
            add_bar(x0, y0, x1, y1, hour ? 0.010f : 0.006f, hour ? ink : dim);
        }
        for (int side = -1; side <= 1; side += 2) {
            for (float f = 0.08f; f < 0.97f; f += 0.08f) {
                float x0, y0, x1, y1;
                at(side * s.fcr_az_limit_deg, R * f, x0, y0);
                at(side * s.fcr_az_limit_deg, R * std::min(f + 0.04f, 1.0f), x1, y1);
                add_bar(x0, y0, x1, y1, 0.005f, dim);
            }
        }

        // The beam and its afterglow.
        constexpr int kTrail = 16;
        constexpr float kTrailDeg = 3.0f;
        for (int k = 0; k < kTrail; ++k) {
            const float f = 1.0f - static_cast<float>(k) / kTrail;
            add_ring_sector(CX, CY, 0.0f, R, s.sweep_deg - kTrailDeg * static_cast<float>(k + 1),
                            s.sweep_deg - kTrailDeg * static_cast<float>(k), faded(ink, 0.30f * f * f), 2);
        }
        {
            float x, y;
            at(s.sweep_deg, R, x, y);
            add_bar(CX, CY, x, y, 0.010f, ink);
        }

        // Blips.
        const Color4 hostile{1.0f, 0.30f, 0.24f, 1.0f};
        int nearest = -1;
        for (int i = 0; i < s.blip_count; ++i) {
            const auto& b = s.blips[i];
            if (nearest < 0 || b.range_nm < s.blips[nearest].range_nm) nearest = i;
            const float glow = 0.22f + 0.78f * std::exp(-2.5f * b.age);
            const Color4 c = faded(hostile, glow);
            float x, y;
            if (b.range_nm > s.range_scale_nm) {
                // Off the scale: an arrowhead on the rim, pointing out.
                float tx, ty, lx, ly, rx, ry;
                at(b.bearing_deg, R + 0.02f, tx, ty);
                at(b.bearing_deg - 4.0f, R - 0.07f, lx, ly);
                at(b.bearing_deg + 4.0f, R - 0.07f, rx, ry);
                add_tri(tx, ty, lx, ly, rx, ry, c);
                continue;
            }
            at(b.bearing_deg, R * b.range_nm / s.range_scale_nm, x, y);
            if (b.age < 0.15f) add_circle(x, y, 0.10f, faded(hostile, 0.35f * (1.0f - b.age / 0.15f)), 20);
            add_circle(x, y, 0.042f, c, 16);
            const float a = b.rel_heading_deg * kPi / 180.0f;
            add_bar(x + std::sin(a) * 0.042f, y + std::cos(a) * 0.042f, x + std::sin(a) * 0.14f, y + std::cos(a) * 0.14f,
                    0.010f, c);
            std::snprintf(buf, sizeof(buf), "%02d", static_cast<int>(std::lround(std::clamp(b.alt_kft, 0.0f, 99.0f))));
            const bool right = std::sin(a) < 0.0f;
            draw_text(buf, right ? x + 0.06f : x - 0.06f, y - 0.10f, 0.062f, c, right ? -1 : 1);
        }

        if (tel.combat.maws_threat) draw_maws_cue(tel.combat, CX, CY, R, s.range_scale_nm, tel.radar.blink);

        // Ownship: a small jet in the middle, nose up.
        add_tri(CX, CY + 0.055f, CX - 0.018f, CY - 0.01f, CX + 0.018f, CY - 0.01f, Color4::white());
        add_bar(CX - 0.05f, CY - 0.005f, CX + 0.05f, CY - 0.005f, 0.014f, Color4::white());
        add_bar(CX - 0.022f, CY - 0.045f, CX + 0.022f, CY - 0.045f, 0.010f, Color4::white());
        add_bar(CX, CY + 0.02f, CX, CY - 0.05f, 0.012f, Color4::white());

        // Fuel (kg, amber at bingo) and throttle (%, amber in reheat), in the
        // bottom corners clear of the scope.
        {
            constexpr float CS = 0.062f;
            std::snprintf(buf, sizeof(buf), "FUEL %d", static_cast<int>(std::lround(std::clamp(tel.fuel_remaining_kg, 0.0, 99999.0))));
            draw_text(buf, -0.92f, -0.80f, CS, tel.bingo_fuel_alert ? Color4::amber() : ink, -1);
            const bool reheat = std::string(tel.detent_str) == "AFTERBURNER";
            std::snprintf(buf, sizeof(buf), "%s %d", reheat ? "AB" : "THR",
                          static_cast<int>(std::lround(std::clamp(tel.throttle_input, 0.0, 1.0) * 100.0)));
            draw_text(buf, 0.92f, -0.80f, CS, reheat ? Color4::amber() : ink, 1);
        }

        // Call the nearest one by the clock.
        if (nearest >= 0) {
            const auto& b = s.blips[nearest];
            int clock = static_cast<int>(std::lround(b.bearing_deg / 30.0f)) % 12;
            if (clock <= 0) clock += 12;
            std::snprintf(buf, sizeof(buf), "BANDIT  %d O'CLOCK  %.1f NM  %02dK", clock, static_cast<double>(b.range_nm),
                          static_cast<int>(std::lround(std::clamp(b.alt_kft, 0.0f, 99.0f))));
            draw_text(buf, 0.0f, -0.935f, TS, hostile, 0);
        } else {
            draw_text("NO CONTACTS", 0.0f, -0.935f, TS, dim, 0);
        }
    }

    /// @brief The MAWS threat on the sweep display (scope centre @p cx, @p cy,
    /// radius @p r): a flashing red arrowhead on the rim at the most urgent
    /// missile's bearing, pointing in, with its line of approach toward the
    /// ownship. A pulse-doppler MAW measures range, so its missile also gets a
    /// ring at that range. The top line names it: clock, range (if measured),
    /// time to go and how many more are inbound.
    void draw_maws_cue(const CombatTelemetry& c, float cx, float cy, float r, float range_scale_nm, bool blink) {
        const Color4 red{1.0f, 0.12f, 0.10f, blink ? 1.0f : 0.45f};
        const auto at = [&](float bearing_deg, float rho, float& x, float& y) {
            const float a = bearing_deg * kPi / 180.0f;
            x = cx + rho * std::sin(a);
            y = cy + rho * std::cos(a);
        };
        const float b = static_cast<float>(c.maws_bearing_deg);
        float tx, ty, lx, ly, rx, ry, ix, iy;
        at(b, r * 0.80f, tx, ty);
        at(b - 6.0f, r * 0.98f, lx, ly);
        at(b + 6.0f, r * 0.98f, rx, ry);
        add_tri(tx, ty, lx, ly, rx, ry, red);
        at(b, r * 0.30f, ix, iy);
        add_bar(tx, ty, ix, iy, 0.008f, red);
        if (c.maws_range_valid && range_scale_nm > 0.0f) {
            const float nm = static_cast<float>(c.maws_range_m / 1852.0);
            float mx, my;
            at(b, r * std::min(nm / range_scale_nm, 1.0f), mx, my);
            add_circle_outline(mx, my, 0.045f, red, 16);
        }

        char buf[48];
        char more[8] = "";
        if (c.maws_threat_count > 1) std::snprintf(more, sizeof(more), "  +%d", std::min(c.maws_threat_count - 1, 9));
        const int ttg = static_cast<int>(std::lround(std::clamp(c.maws_time_to_go_s, 0.0, 99.0)));
        if (c.maws_range_valid) {
            std::snprintf(buf, sizeof(buf), "MISSILE %d  %.1f NM  %d S%s", std::clamp(c.maws_clock, 1, 12),
                          c.maws_range_m / 1852.0, ttg, more);
        } else {
            std::snprintf(buf, sizeof(buf), "MISSILE %d  %d S%s", std::clamp(c.maws_clock, 1, 12), ttg, more);
        }
        // Between the SWEEP legend and the range scale.
        float ts = 0.072f;
        while (text_width(buf, ts) > 1.2f && ts > 0.04f) ts *= 0.9f;
        draw_text(buf, 0.0f, 0.875f, ts, Color4{1.0f, 0.12f, 0.10f, 1.0f}, 0);
    }

    // =========================================================================
    // Eurofighter Typhoon MHDD formats
    // =========================================================================
    // Five soft keys along each edge of the MHDD: 1-5 down the left, 6-10
    // across the top, 11-15 down the right, 16-20 across the bottom. The
    // bottom row selects the format, the current one boxed.

    static constexpr float kMhddKey[5] = {-0.80f, -0.40f, 0.0f, 0.40f, 0.80f};  ///< Top and bottom rows
    static constexpr float kMhddSide[5] = {-0.64f, -0.32f, 0.0f, 0.32f, 0.64f}; ///< Side columns, clear of the corners
    static constexpr float kMhddTs = 0.068f;

    void boxed_text(const char* s, float x, float y, float size, const Color4& c) {
        const float w = text_width(s, size) + 0.035f;
        draw_text(s, x, y, size, c, 0);
        const float b = y - 0.022f, t = y + size * 0.95f;
        add_bar(x - 0.5f * w, b, x + 0.5f * w, b, 0.007f, c);
        add_bar(x - 0.5f * w, t, x + 0.5f * w, t, 0.007f, c);
        add_bar(x - 0.5f * w, b, x - 0.5f * w, t, 0.007f, c);
        add_bar(x + 0.5f * w, b, x + 0.5f * w, t, 0.007f, c);
    }

    /// Soft key legends; null or empty leaves a key blank. Left and right
    /// run top to bottom.
    void mhdd_keys(const char* const top[5], const char* const left[5], const char* const right[5], int boxed_format,
                   const Color4& ink) {
        static constexpr const char* kFormats[5] = {"FUEL", "ENG", "PA", "HYD", "ATTK"};
        for (int i = 0; i < 5; ++i) {
            const float p = kMhddKey[i], side_y = kMhddSide[4 - i] - 0.025f;
            if (top[i] && *top[i]) draw_text(top[i], p, 0.875f, kMhddTs, ink, 0);
            if (left[i] && *left[i]) draw_text(left[i], -0.965f, side_y, kMhddTs, ink, -1);
            if (right[i] && *right[i]) draw_text(right[i], 0.965f, side_y, kMhddTs, ink, 1);
            if (i == boxed_format) boxed_text(kFormats[i], p, -0.935f, kMhddTs, ink);
            else draw_text(kFormats[i], p, -0.935f, kMhddTs, ink, 0);
        }
    }

    /// @brief Pilot Awareness format in its HSI form: heading-up, the
    /// ownship low on the display under a 120 degree compass arc, half-range
    /// ring, the route to the active waypoint (the home field), bearing and
    /// track pointers, heading box, and the waypoint, speed and height data.
    void draw_typhoon_pa(const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        const Color4 glass{0.004f, 0.006f, 0.010f, 1.0f};
        const Color4 ink{0.55f, 1.0f, 0.62f, 1.0f};
        const Color4 white{0.92f, 0.94f, 0.95f, 1.0f};
        const Color4 cyan{0.30f, 0.90f, 1.0f, 1.0f};
        const Color4 dim{0.18f, 0.40f, 0.45f, 1.0f};
        const Color4 route{1.0f, 0.35f, 0.95f, 1.0f};
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, glass);

        constexpr float OY = -0.52f, RA = 0.98f;
        const double hdg = std::fmod(state.yaw() * 180.0 / 3.14159265358979 + 360.0, 360.0);
        auto at = [&](double bearing_deg, float rho, float& x, float& y) {
            const float a = static_cast<float>((bearing_deg - hdg) * 3.14159265358979 / 180.0);
            x = rho * std::sin(a);
            y = OY + rho * std::cos(a);
        };
        auto rel = [&](double bearing_deg) { return std::remainder(bearing_deg - hdg, 360.0); };

        // Compass arc with 5 deg ticks and labels every 30 deg.
        add_ring_sector(0.0f, OY, RA - 0.006f, RA + 0.006f, -60.0f, 60.0f, cyan, 48);
        for (int b = 0; b < 360; b += 5) {
            if (std::abs(rel(b)) > 60.0) continue;
            const float len = b % 30 == 0 ? 0.07f : b % 10 == 0 ? 0.045f : 0.025f;
            float x0, y0, x1, y1;
            at(b, RA, x0, y0);
            at(b, RA - len, x1, y1);
            add_bar(x0, y0, x1, y1, 0.008f, cyan);
            if (b % 30 == 0 && std::abs(rel(b)) < 52.0) {
                char lab[4];
                if (b % 90 == 0) std::snprintf(lab, sizeof(lab), "%c", "NESW"[b / 90]);
                else std::snprintf(lab, sizeof(lab), "%d", b / 10);
                float tx, ty;
                at(b, RA - 0.13f, tx, ty);
                draw_text(lab, tx, ty - 0.025f, 0.066f, white, 0);
            }
        }
        // Half-range arc, dashed, and the lubber line.
        for (float a = -60.0f; a < 60.0f; a += 6.0f) {
            add_ring_sector(0.0f, OY, RA * 0.5f - 0.004f, RA * 0.5f + 0.004f, a, a + 3.0f, dim, 2);
        }
        for (float y = OY + 0.10f; y < OY + RA - 0.10f; y += 0.08f) add_bar(0.0f, y, 0.0f, y + 0.04f, 0.006f, dim);

        // Heading box over the arc.
        {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%03d", static_cast<int>(std::lround(hdg)) % 360);
            add_rect(0.0f, OY + RA + 0.075f, 0.20f, 0.10f, glass);
            boxed_text(buf, 0.0f, OY + RA + 0.05f, 0.070f, white);
        }

        // Active waypoint: the home field's runway midpoint.
        const double dn = 1500.0 - state.pos_ned.x, de = -state.pos_ned.y;
        const double dist_nm = std::sqrt(dn * dn + de * de) / 1852.0;
        const double brg = std::fmod(std::atan2(de, dn) * 180.0 / 3.14159265358979 + 360.0, 360.0);
        static constexpr float kScales[5] = {10.0f, 20.0f, 40.0f, 80.0f, 160.0f};
        float scale = kScales[4];
        for (float s : kScales) {
            if (dist_nm <= s * 0.9f) { scale = s; break; }
        }
        {
            const float rho = static_cast<float>(std::min(dist_nm / scale, 1.0)) * RA;
            float wx, wy;
            at(brg, rho, wx, wy);
            const bool on_arc = std::abs(rel(brg)) <= 60.0 || rho < RA * 0.5f;
            if (on_arc) {
                add_bar(0.0f, OY, wx, wy, 0.010f, route);
                add_ring(wx, wy, 0.030f, 0.042f, route, 20);
                draw_text("01", wx + 0.06f, wy - 0.02f, 0.058f, route, -1);
            }
            // Bearing pointer on the arc (clamped to its ends).
            const double rb = std::clamp(rel(brg), -60.0, 60.0);
            float bx, by, lx, ly, rx, ry;
            at(hdg + rb, RA + 0.012f, bx, by);
            at(hdg + rb - 3.0, RA + 0.06f, lx, ly);
            at(hdg + rb + 3.0, RA + 0.06f, rx, ry);
            add_tri(bx, by, lx, ly, rx, ry, route);
        }
        // Track pointer: a small diamond on the arc.
        {
            const math::Vector3 v = state.velocity_ned();
            if (v.x * v.x + v.y * v.y > 25.0) {
                const double trk = std::atan2(v.y, v.x) * 180.0 / 3.14159265358979;
                const double rt = std::clamp(rel(trk), -60.0, 60.0);
                float x, y;
                at(hdg + rt, RA - 0.035f, x, y);
                add_convex({x, y + 0.025f, x + 0.018f, y, x, y - 0.025f, x - 0.018f, y}, white);
            }
        }
        // Ownship.
        add_bar(0.0f, OY - 0.06f, 0.0f, OY + 0.07f, 0.016f, white);
        add_bar(-0.065f, OY + 0.015f, 0.065f, OY + 0.015f, 0.016f, white);
        add_bar(-0.030f, OY - 0.055f, 0.030f, OY - 0.055f, 0.012f, white);

        // Waypoint data (top left), speed and height (top right), scale.
        char buf[32];
        draw_text("WP 01  HOME", -0.70f, 0.64f, 0.060f, route, -1);
        std::snprintf(buf, sizeof(buf), "%03d/%.1f", static_cast<int>(std::lround(brg)) % 360, dist_nm);
        draw_text(buf, -0.70f, 0.55f, 0.060f, white, -1);
        const double gs_kt = std::sqrt(state.velocity_ned().x * state.velocity_ned().x +
                                       state.velocity_ned().y * state.velocity_ned().y) * 1.94384;
        if (gs_kt > 30.0) {
            const int ttg = static_cast<int>(dist_nm / gs_kt * 3600.0);
            std::snprintf(buf, sizeof(buf), "TTG %02d:%02d", std::min(ttg / 60, 99), ttg % 60);
            draw_text(buf, -0.70f, 0.46f, 0.060f, white, -1);
        }
        std::snprintf(buf, sizeof(buf), "GS %d", static_cast<int>(std::lround(std::clamp(gs_kt, 0.0, 999.0))));
        draw_text(buf, 0.70f, 0.64f, 0.060f, white, 1);
        std::snprintf(buf, sizeof(buf), "ALT %d", static_cast<int>(std::lround(std::clamp(state.altitude() * 3.28084, -999.0, 99999.0))));
        draw_text(buf, 0.70f, 0.55f, 0.060f, white, 1);
        std::snprintf(buf, sizeof(buf), "FUEL %d", static_cast<int>(std::lround(std::clamp(tel.fuel_remaining_kg, 0.0, 99999.0))));
        draw_text(buf, 0.70f, 0.46f, 0.060f, tel.bingo_fuel_alert ? Color4::amber() : white, 1);
        std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(scale));
        draw_text(buf, -0.70f, -0.68f, 0.060f, cyan, -1);
        draw_text("A/C  MAG", 0.70f, -0.68f, 0.060f, cyan, 1);

        static constexpr const char* kTop[5] = {"MAG", "A/C", "PA", "COMP", "TAC"};
        static constexpr const char* kLeft[5] = {"RNG+", "RNG-", "", "AIR", "MAP"};
        static constexpr const char* kRight[5] = {"WPT", "FREQ", "", "STOR", "HDHUD"};
        mhdd_keys(kTop, kLeft, kRight, 2, ink);
    }

    /// @brief FUEL format: the tank groups in plan view, blue infill to
    /// their contents, a pump in each fuselage group, and the totals, flow
    /// and time to empty.
    void draw_typhoon_fuel(const AvionicsTelemetry& tel) {
        const Color4 glass{0.004f, 0.006f, 0.010f, 1.0f};
        const Color4 ink{0.55f, 1.0f, 0.62f, 1.0f};
        const Color4 white{0.92f, 0.94f, 0.95f, 1.0f};
        const Color4 outline{0.62f, 0.66f, 0.70f, 1.0f};
        const Color4 fuel{0.10f, 0.42f, 1.0f, 1.0f};
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, glass);

        constexpr float CY = 0.10f;
        auto stroke = [&](const std::vector<float>& p, const Color4& c) {
            const size_t n = p.size() / 2;
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                add_bar(p[i * 2], p[i * 2 + 1] + CY, p[j * 2], p[j * 2 + 1] + CY, 0.008f, c);
            }
        };
        // Fill a convex tank below its fuel level (it drains from the top).
        auto fill = [&](const std::vector<float>& p, float frac) {
            float y_lo = 1e9f, y_hi = -1e9f;
            for (size_t i = 1; i < p.size(); i += 2) { y_lo = std::min(y_lo, p[i]); y_hi = std::max(y_hi, p[i]); }
            const float level = y_lo + (y_hi - y_lo) * std::clamp(frac, 0.0f, 1.0f);
            std::vector<float> out;
            const size_t n = p.size() / 2;
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const float x0 = p[i * 2], y0 = p[i * 2 + 1], x1 = p[j * 2], y1 = p[j * 2 + 1];
                const bool in0 = y0 <= level, in1 = y1 <= level;
                if (in0) { out.push_back(x0); out.push_back(y0 + CY); }
                if (in0 != in1) {
                    const float t = (level - y0) / (y1 - y0);
                    out.push_back(x0 + t * (x1 - x0));
                    out.push_back(level + CY);
                }
            }
            if (out.size() >= 6) add_convex(out, fuel);
        };

        // Airframe in plan view: fuselage, canards, cranked delta.
        stroke({0.0f, 0.82f, 0.09f, 0.55f, 0.11f, 0.20f, 0.62f, -0.40f, 0.62f, -0.46f, 0.12f, -0.46f, 0.08f, -0.62f,
                -0.08f, -0.62f, -0.12f, -0.46f, -0.62f, -0.46f, -0.62f, -0.40f, -0.11f, 0.20f, -0.09f, 0.55f},
               outline);
        for (float s : {-1.0f, 1.0f}) stroke({s * 0.10f, 0.50f, s * 0.32f, 0.36f, s * 0.32f, 0.32f, s * 0.10f, 0.36f}, outline);

        const float f = static_cast<float>(std::clamp(tel.fuel_fraction, 0.0, 1.0));
        const std::vector<float> tanks[5] = {
            {-0.065f, 0.42f, 0.065f, 0.42f, 0.065f, 0.18f, -0.065f, 0.18f},     // forward fuselage
            {-0.065f, 0.14f, 0.065f, 0.14f, 0.065f, -0.12f, -0.065f, -0.12f},   // centre
            {-0.065f, -0.16f, 0.065f, -0.16f, 0.065f, -0.40f, -0.065f, -0.40f}, // aft
            {-0.15f, 0.08f, -0.15f, -0.36f, -0.48f, -0.36f, -0.48f, -0.26f},    // left wing
            {0.15f, 0.08f, 0.48f, -0.26f, 0.48f, -0.36f, 0.15f, -0.36f},        // right wing
        };
        for (const auto& t : tanks) {
            fill(t, f);
            stroke(t, white);
        }
        // Booster pumps, white while running.
        for (float y : {0.30f, 0.01f, -0.28f}) {
            add_ring(0.0f, y + CY, 0.022f, 0.032f, tel.engine_rpm_pct > 20.0 ? white : Color4::red(), 16);
        }

        char buf[40];
        std::snprintf(buf, sizeof(buf), "TOTAL %d KG", static_cast<int>(std::lround(std::clamp(tel.fuel_remaining_kg, 0.0, 99999.0))));
        draw_text(buf, -0.72f, -0.62f, 0.070f, tel.bingo_fuel_alert ? Color4::amber() : white, -1);
        std::snprintf(buf, sizeof(buf), "FF %d KG/H", static_cast<int>(std::lround(std::clamp(tel.fuel_flow_kg_hr, 0.0, 99999.0))));
        draw_text(buf, 0.72f, -0.62f, 0.062f, white, 1);
        draw_text("BINGO 800", -0.72f, -0.74f, 0.062f, ink, -1);
        if (tel.fuel_flow_kg_hr > 1.0) {
            const int mins = static_cast<int>(tel.fuel_remaining_kg / tel.fuel_flow_kg_hr * 60.0);
            std::snprintf(buf, sizeof(buf), "TTE %d:%02d", std::min(mins / 60, 9), mins % 60);
            draw_text(buf, 0.72f, -0.74f, 0.062f, white, 1);
        }

        static constexpr const char* kTop[5] = {"XFER", "FWD", "AFT", "INTC", "REFU"};
        static constexpr const char* kLeft[5] = {"AUTO", "", "", "", ""};
        static constexpr const char* kRight[5] = {"TOT", "INT", "", "", ""};
        mhdd_keys(kTop, kLeft, kRight, 0, ink);
    }

    /// @brief ENG format: both EJ200s side by side, left engine left,
    /// parameter names down the middle: spool speed, turbine temperature,
    /// fuel flow, nozzle, oil; then the hydraulics, thrust and the
    /// throttle detent.
    void draw_typhoon_eng(const AvionicsTelemetry& tel) {
        const Color4 glass{0.004f, 0.006f, 0.010f, 1.0f};
        const Color4 ink{0.55f, 1.0f, 0.62f, 1.0f};
        const Color4 white{0.92f, 0.94f, 0.95f, 1.0f};
        const Color4 label{0.62f, 0.70f, 0.78f, 1.0f};
        const Color4 track{0.10f, 0.12f, 0.14f, 1.0f};
        add_rect(0.0f, 0.0f, 2.0f, 2.0f, glass);

        const char* detent = tel.detent_str;
        const std::string d(detent);
        // The throttle's dry range reads DRY, its top MAX DRY.
        const bool dry = d == "MIL POWER" || d == "MIL";
        const char* state = d == "AFTERBURNER" ? "REHEAT"
                            : dry              ? (tel.throttle_input >= 0.99 ? "MAX DRY" : "DRY")
                            : d == "CUTOFF"    ? "SHUT"
                                               : "IDLE";
        draw_text("EJ200", -0.70f, 0.72f, 0.066f, label, -1);
        draw_text(state, 0.70f, 0.72f, 0.066f, d == "AFTERBURNER" ? Color4::amber() : white, 1);
        draw_text("L", -0.42f, 0.60f, 0.070f, label, 0);
        draw_text("R", 0.42f, 0.60f, 0.070f, label, 0);

        struct Row {
            const char* name;
            double value;
            double full;
            double limit;
            const char* fmt;
        };
        const Row rows[5] = {
            {"NH %", tel.engine_rpm_pct, 110.0, 103.0, "%.1f"},
            {"TBT C", tel.engine_ftit_deg_c, 1100.0, 950.0, "%.0f"},
            {"FF KG/H", tel.fuel_flow_kg_hr * 0.5, 12000.0, 1e9, "%.0f"},
            {"NOZ %", tel.nozzle_pos_pct, 100.0, 1e9, "%.0f"},
            {"OIL BAR", tel.oil_pressure_psi * 0.0689476, 6.0, 1e9, "%.1f"},
        };
        for (int i = 0; i < 5; ++i) {
            const float y = 0.46f - 0.20f * static_cast<float>(i);
            draw_text(rows[i].name, 0.0f, y - 0.02f, 0.066f, label, 0);
            const Color4 c = rows[i].value > rows[i].limit ? Color4::amber() : ink;
            char buf[16];
            std::snprintf(buf, sizeof(buf), rows[i].fmt, rows[i].value);
            const float frac = static_cast<float>(std::clamp(rows[i].value / rows[i].full, 0.0, 1.0));
            for (float side : {-1.0f, 1.0f}) {
                const float x = side * 0.42f;
                draw_text(buf, x, y + 0.01f, 0.070f, white, 0);
                add_rect(x, y - 0.06f, 0.42f, 0.030f, track);
                if (frac > 0.005f) add_rect(x - 0.21f + 0.21f * frac, y - 0.06f, 0.42f * frac, 0.026f, c);
            }
        }

        add_bar(-0.85f, -0.52f, 0.85f, -0.52f, 0.006f, track);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "HYD  L %.0f   R %.0f PSI", tel.hyd_press_a_psi, tel.hyd_press_b_psi);
        draw_text(buf, 0.0f, -0.63f, 0.068f,
                  tel.hyd_press_a_psi < 1000.0 || tel.hyd_press_b_psi < 1000.0 ? Color4::amber() : white, 0);
        std::snprintf(buf, sizeof(buf), "THRUST %.1f KN   THR %d%%", tel.net_thrust_n / 1000.0,
                      static_cast<int>(std::lround(std::clamp(tel.throttle_input, 0.0, 1.0) * 100.0)));
        draw_text(buf, 0.0f, -0.75f, 0.066f, white, 0);

        static constexpr const char* kTop[5] = {"", "DECU", "LANE A", "LANE B", ""};
        static constexpr const char* kLeft[5] = {"", "", "", "", ""};
        static constexpr const char* kRight[5] = {"", "", "", "", ""};
        mhdd_keys(kTop, kLeft, kRight, 1, ink);
    }

    // =========================================================================
    // F-35 Panoramic Cockpit Display formats
    // =========================================================================
    // The 20 x 8 in PCD is two 10 x 8 in halves, each the middle 80 % of an
    // MFD page's height (jet_cockpits.hpp PCD_LEFT / PCD_RIGHT). The left half
    // is one large portal (TSD, or the FCR when paged); the right half two
    // 5 x 8 in portals (ENG/FUEL and the STOVL LiftSystem page), or one large
    // one with the sweep display in a fight. Each portal carries its name on
    // a tab at its top; the glass has no bezel keys.

    static constexpr Color4 kPcdGlass{0.006f, 0.008f, 0.014f, 1.0f};
    static constexpr Color4 kPcdFrame{0.20f, 0.23f, 0.27f, 1.0f};
    static constexpr Color4 kPcdLabel{0.62f, 0.68f, 0.74f, 1.0f};
    static constexpr Color4 kPcdWhite{0.92f, 0.94f, 0.95f, 1.0f};
    static constexpr Color4 kPcdGreen{0.40f, 1.0f, 0.48f, 1.0f};
    static constexpr Color4 kPcdCyan{0.35f, 0.85f, 1.0f, 1.0f};
    static constexpr Color4 kPcdTrack{0.10f, 0.12f, 0.15f, 1.0f};

    /// A portal's frame and its name tab, the portal spanning [-w, w] x [-h, h].
    void pcd_portal(float w, float h, const char* name) {
        add_rect(0.0f, 0.0f, 2.0f * w, 2.0f * h, kPcdGlass);
        add_bar(-w, h - 0.004f, w, h - 0.004f, 0.008f, kPcdFrame);
        add_bar(-w, -h + 0.004f, w, -h + 0.004f, 0.008f, kPcdFrame);
        add_bar(-w + 0.004f, -h, -w + 0.004f, h, 0.008f, kPcdFrame);
        add_bar(w - 0.004f, -h, w - 0.004f, h, 0.008f, kPcdFrame);
        const float tw = text_width(name, 0.060f) + 0.08f;
        add_rect(-w + 0.02f + 0.5f * tw, h - 0.055f, tw, 0.09f, kPcdTrack);
        draw_text(name, -w + 0.06f, h - 0.077f, 0.060f, kPcdWhite, -1);
    }

    /// @brief TSD (tactical situation display), 10 x 8 in: heading-up plan
    /// view round the ownship, compass rose and half-range ring, the route
    /// to the home field, the datalink tracks, and speed, height, heading
    /// and fuel in the corners. Page area [-1, 1] x [-0.8, 0.8].
    void draw_f35_tsd(const fdm::FlightState& state, const AvionicsTelemetry& tel) {
        pcd_portal(1.0f, 0.8f, "TSD1");
        constexpr float OY = -0.06f, RA = 0.64f;
        const double hdg = std::fmod(state.yaw() * 180.0 / 3.14159265358979 + 360.0, 360.0);
        auto at = [&](double bearing_deg, float rho, float& x, float& y) {
            const float a = static_cast<float>((bearing_deg - hdg) * 3.14159265358979 / 180.0);
            x = rho * std::sin(a);
            y = OY + rho * std::cos(a);
        };
        add_ring(0.0f, OY, RA - 0.004f, RA + 0.004f, kPcdLabel, 72);
        for (float a = 0.0f; a < 360.0f; a += 8.0f) add_ring_sector(0.0f, OY, RA * 0.5f - 0.003f, RA * 0.5f + 0.003f, a, a + 4.0f, kPcdFrame, 2);
        for (int b = 0; b < 360; b += 10) {
            float x0, y0, x1, y1;
            at(b, RA, x0, y0);
            at(b, RA - (b % 30 == 0 ? 0.05f : 0.025f), x1, y1);
            add_bar(x0, y0, x1, y1, 0.007f, kPcdLabel);
            if (b % 30 == 0) {
                char lab[4];
                if (b % 90 == 0) std::snprintf(lab, sizeof(lab), "%c", "NESW"[b / 90]);
                else std::snprintf(lab, sizeof(lab), "%d", b / 10);
                at(b, RA - 0.10f, x0, y0);
                draw_text(lab, x0, y0 - 0.022f, 0.056f, kPcdWhite, 0);
            }
        }
        // Heading box over the rose.
        char buf[40];
        std::snprintf(buf, sizeof(buf), "%03d", static_cast<int>(std::lround(hdg)) % 360);
        add_rect(0.0f, OY + RA + 0.05f, 0.17f, 0.085f, kPcdTrack);
        draw_text(buf, 0.0f, OY + RA + 0.028f, 0.060f, kPcdWhite, 0);

        // Home field and the scale that keeps it on the display.
        const double dn = 1500.0 - state.pos_ned.x, de = -state.pos_ned.y;
        const double dist_nm = std::sqrt(dn * dn + de * de) / 1852.0;
        const double brg = std::fmod(std::atan2(de, dn) * 180.0 / 3.14159265358979 + 360.0, 360.0);
        static constexpr float kScales[6] = {5.0f, 10.0f, 20.0f, 40.0f, 80.0f, 160.0f};
        float scale = kScales[5];
        for (float sc : kScales) {
            if (dist_nm <= sc * 0.9f) { scale = sc; break; }
        }
        {
            float wx, wy;
            at(brg, static_cast<float>(std::min(dist_nm / scale, 1.0)) * RA, wx, wy);
            add_bar(0.0f, OY, wx, wy, 0.008f, kPcdCyan);
            add_ring(wx, wy, 0.024f, 0.034f, kPcdCyan, 20);
        }
        // Datalink tracks: red half-diamonds with their heading and height.
        const Color4 hostile{1.0f, 0.30f, 0.24f, 1.0f};
        for (int i = 0; i < tel.radar.track_count; ++i) {
            const auto& t = tel.radar.tracks[i];
            if (t.range_nm > scale) continue;
            float x, y;
            at(hdg + t.az_deg, t.range_nm / scale * RA, x, y);
            constexpr float Dm = 0.035f;
            add_bar(x - Dm, y, x, y + Dm, 0.007f, hostile);
            add_bar(x, y + Dm, x + Dm, y, 0.007f, hostile);
            add_bar(x - Dm, y, x + Dm, y, 0.007f, hostile);
            const float a = t.rel_heading_deg * kPi / 180.0f;
            add_bar(x, y + Dm, x + std::sin(a) * 0.10f, y + Dm + std::cos(a) * 0.10f, 0.006f, hostile);
            std::snprintf(buf, sizeof(buf), "%02d", static_cast<int>(std::lround(std::clamp(t.alt_kft, 0.0f, 99.0f))));
            draw_text(buf, x, y - 0.075f, 0.046f, hostile, 0);
        }
        // Ownship: a small F-35 planform.
        add_convex({0.0f, OY + 0.055f, 0.012f, OY + 0.02f, 0.012f, OY - 0.04f, -0.012f, OY - 0.04f, -0.012f, OY + 0.02f}, kPcdWhite);
        add_convex({0.0f, OY + 0.012f, 0.045f, OY - 0.020f, 0.045f, OY - 0.030f, -0.045f, OY - 0.030f, -0.045f, OY - 0.020f}, kPcdWhite);

        // Corners: ground speed and altitude, fuel and the home field.
        const math::Vector3 v = state.velocity_ned();
        const double gs_kt = std::sqrt(v.x * v.x + v.y * v.y) * 1.94384;
        std::snprintf(buf, sizeof(buf), "GS %d", static_cast<int>(std::lround(std::clamp(gs_kt, 0.0, 999.0))));
        draw_text(buf, -0.95f, 0.56f, 0.054f, kPcdWhite, -1);
        std::snprintf(buf, sizeof(buf), "ALT %d", static_cast<int>(std::lround(std::clamp(state.altitude() * 3.28084, -999.0, 99999.0))));
        draw_text(buf, -0.95f, 0.48f, 0.054f, kPcdWhite, -1);
        std::snprintf(buf, sizeof(buf), "FUEL %d", static_cast<int>(std::lround(std::clamp(tel.fuel_remaining_kg * 2.20462, 0.0, 99999.0))));
        draw_text(buf, 0.95f, 0.56f, 0.054f, tel.bingo_fuel_alert ? Color4::amber() : kPcdWhite, 1);
        std::snprintf(buf, sizeof(buf), "HOME %03d/%.1f", static_cast<int>(std::lround(brg)) % 360, dist_nm);
        draw_text(buf, 0.95f, 0.48f, 0.054f, kPcdCyan, 1);
        std::snprintf(buf, sizeof(buf), "%g NM", static_cast<double>(scale));
        draw_text(buf, -0.95f, -0.74f, 0.054f, kPcdLabel, -1);
        draw_text("HDG UP", 0.95f, -0.74f, 0.054f, kPcdLabel, 1);
    }

    /// @brief ENG/FUEL portal, 5 x 8 in: the F135's spool speed, turbine
    /// temperature, fuel flow, nozzle and oil as bars with their limits;
    /// thrust and throttle; then total fuel (lb), bingo and time to empty.
    /// Portal area [-1, 1] x [-1.6, 1.6].
    void draw_f35_eng(const AvionicsTelemetry& tel) {
        pcd_portal(1.0f, 1.6f, "ENG");
        char buf[40];
        const std::string d(tel.detent_str);
        draw_text("F135-PW-600", -0.88f, 1.30f, 0.070f, kPcdLabel, -1);
        draw_text(d == "AFTERBURNER" ? "AB" : d == "CUTOFF" ? "OFF" : d == "IDLE" ? "IDLE" : "MIL", 0.88f, 1.30f, 0.070f,
                  d == "AFTERBURNER" ? Color4::amber() : kPcdGreen, 1);
        struct Row {
            const char* name;
            double value, full, limit;
            const char* fmt;
        };
        const Row rows[5] = {
            {"N2 %", tel.engine_rpm_pct, 110.0, 104.0, "%.1f"},
            {"TMT C", tel.engine_ftit_deg_c, 1100.0, 960.0, "%.0f"},
            {"FF PPH", tel.fuel_flow_kg_hr * 2.20462, 30000.0, 1e9, "%.0f"},
            {"NOZ %", tel.nozzle_pos_pct, 100.0, 1e9, "%.0f"},
            {"OIL PSI", tel.oil_pressure_psi, 100.0, 1e9, "%.0f"},
        };
        for (int i = 0; i < 5; ++i) {
            const float y = 1.05f - 0.27f * static_cast<float>(i);
            draw_text(rows[i].name, -0.88f, y, 0.068f, kPcdLabel, -1);
            std::snprintf(buf, sizeof(buf), rows[i].fmt, rows[i].value);
            const bool over = rows[i].value > rows[i].limit;
            draw_text(buf, 0.88f, y, 0.072f, over ? Color4::amber() : kPcdWhite, 1);
            const float frac = static_cast<float>(std::clamp(rows[i].value / rows[i].full, 0.0, 1.0));
            add_rect(0.0f, y - 0.07f, 1.76f, 0.035f, kPcdTrack);
            if (frac > 0.005f) add_rect(-0.88f + 0.88f * frac, y - 0.07f, 1.76f * frac, 0.030f, over ? Color4::amber() : kPcdGreen);
        }
        std::snprintf(buf, sizeof(buf), "THRUST %.0f KN", tel.net_thrust_n / 1000.0);
        draw_text(buf, -0.88f, -0.32f, 0.066f, kPcdWhite, -1);
        std::snprintf(buf, sizeof(buf), "THR %d%%", static_cast<int>(std::lround(std::clamp(tel.throttle_input, 0.0, 1.0) * 100.0)));
        draw_text(buf, 0.88f, -0.32f, 0.066f, kPcdWhite, 1);

        // FUEL.
        add_bar(-0.92f, -0.45f, 0.92f, -0.45f, 0.006f, kPcdFrame);
        draw_text("FUEL", -0.88f, -0.60f, 0.070f, kPcdLabel, -1);
        const double lb = tel.fuel_remaining_kg * 2.20462;
        std::snprintf(buf, sizeof(buf), "%d LB", static_cast<int>(std::lround(std::clamp(lb, 0.0, 99999.0))));
        const Color4 fuel_col = tel.bingo_fuel_alert ? Color4::amber() : kPcdWhite;
        draw_text(buf, 0.88f, -0.60f, 0.080f, fuel_col, 1);
        const float f = static_cast<float>(std::clamp(tel.fuel_fraction, 0.0, 1.0));
        add_rect(0.0f, -0.74f, 1.76f, 0.07f, kPcdTrack);
        if (f > 0.005f) add_rect(-0.88f + 0.88f * f, -0.74f, 1.76f * f, 0.062f, Color4{0.10f, 0.42f, 1.0f, 1.0f});
        std::snprintf(buf, sizeof(buf), "BINGO %d", static_cast<int>(std::lround(800.0 * 2.20462 / 100.0) * 100));
        draw_text(buf, -0.88f, -0.92f, 0.062f, kPcdLabel, -1);
        if (tel.fuel_flow_kg_hr > 1.0) {
            const int mins = static_cast<int>(tel.fuel_remaining_kg / tel.fuel_flow_kg_hr * 60.0);
            std::snprintf(buf, sizeof(buf), "TTE %d:%02d", std::min(mins / 60, 9), mins % 60);
            draw_text(buf, 0.88f, -0.92f, 0.062f, kPcdWhite, 1);
        }
        std::snprintf(buf, sizeof(buf), "HYD %.0f/%.0f", tel.hyd_press_a_psi, tel.hyd_press_b_psi);
        draw_text(buf, -0.88f, -1.12f, 0.062f,
                  tel.hyd_press_a_psi < 1000.0 || tel.hyd_press_b_psi < 1000.0 ? Color4::amber() : kPcdWhite, -1);
        const char* gear = tel.gear_collapsed ? "GEAR FAIL" : !tel.gear_deployed ? "GEAR UP"
                                                 : tel.gear_transit_pos < 0.98 ? "GEAR TRANS" : "GEAR DN";
        draw_text(gear, -0.88f, -1.32f, 0.062f,
                  tel.gear_collapsed ? Color4::red() : tel.gear_deployed && tel.gear_transit_pos >= 0.98 ? kPcdGreen
                  : tel.gear_deployed ? Color4::amber() : kPcdLabel, -1);
    }

    /// @brief STOVL (LiftSystem) portal, 5 x 8 in: the mode and the control
    /// law's phase; the jet in plan view with each door coloured by its
    /// state (grey shut, amber moving, green open) and the clutch; the 3BSM
    /// in side view on its 0-95 degree arc and the vane box angle; one
    /// thrust bar per lift post; and the hover margin. Portal area
    /// [-1, 1] x [-1.6, 1.6].
    void draw_f35_stovl(const AvionicsTelemetry& tel) {
        using propulsion::StovlMode;
        const propulsion::StovlStatus& st = tel.stovl;
        pcd_portal(1.0f, 1.6f, "STOVL");
        char buf[40];
        const Color4 amber = Color4::amber();
        auto door_col = [&](double v) { return v >= 0.98 ? kPcdGreen : v > 0.02 ? amber : kPcdLabel; };

        // Mode and phase.
        const bool converting = st.mode == StovlMode::CONVERTING_TO_STOVL || st.mode == StovlMode::CONVERTING_TO_CTOL;
        if (converting) {
            std::snprintf(buf, sizeof(buf), "%s %d%%", propulsion::to_string(st.mode),
                          static_cast<int>(std::lround(std::clamp(st.conversion, 0.0, 1.0) * 100.0)));
        } else {
            std::snprintf(buf, sizeof(buf), "%s", propulsion::to_string(st.mode));
        }
        draw_text(buf, 0.0f, 1.30f, 0.080f, converting ? amber : st.mode == StovlMode::STOVL ? kPcdGreen : kPcdWhite, 0);
        static constexpr const char* kPhase[3] = {"WINGBORNE", "SEMI-JETBORNE", "JETBORNE"};
        draw_text(st.equipped ? kPhase[std::min<int>(st.control_phase, 2)] : "NOT FITTED", 0.0f, 1.18f, 0.060f, kPcdLabel, 0);

        // Plan view, nose up: fuselage, wings, tails, then the doors.
        {
            constexpr float CX = -0.35f, CY = 0.62f, K = 0.50f;
            auto stroke = [&](std::initializer_list<float> pts, const Color4& c) {
                const float* p = pts.begin();
                const size_t n = pts.size() / 2;
                for (size_t i = 0; i < n; ++i) {
                    const size_t j = (i + 1) % n;
                    add_bar(CX + K * p[i * 2], CY + K * p[i * 2 + 1], CX + K * p[j * 2], CY + K * p[j * 2 + 1], 0.010f, c);
                }
            };
            stroke({0.0f, 0.95f, 0.10f, 0.62f, 0.20f, 0.38f, 0.22f, 0.12f, 0.80f, -0.32f, 0.80f, -0.44f, 0.24f, -0.38f,
                    0.50f, -0.62f, 0.50f, -0.74f, 0.10f, -0.80f, -0.10f, -0.80f, -0.50f, -0.74f, -0.50f, -0.62f,
                    -0.24f, -0.38f, -0.80f, -0.44f, -0.80f, -0.32f, -0.22f, 0.12f, -0.20f, 0.38f, -0.10f, 0.62f},
                   kPcdLabel);
            auto door = [&](float x, float y, float w, float h, double v) {
                add_rect(CX + K * x, CY + K * y, K * w, K * h, door_col(v));
            };
            door(0.0f, 0.30f, 0.16f, 0.20f, st.lift_fan_door);   // LiftFan inlet door
            door(0.0f, 0.15f, 0.12f, 0.06f, st.aux_inlet_door);  // auxiliary inlet doors
            door(-0.33f, -0.08f, 0.08f, 0.05f, st.roll_post_door);
            door(0.33f, -0.08f, 0.08f, 0.05f, st.roll_post_door);
            // Lower doors, beside the fan: two slim bars.
            door(-0.12f, 0.30f, 0.025f, 0.18f, st.fan_nozzle_door);
            door(0.12f, 0.30f, 0.025f, 0.18f, st.fan_nozzle_door);
            // Door and clutch legend, right of the plan view.
            const float lx = 0.20f;
            struct Item { const char* name; double v; };
            const Item items[5] = {{"LF DOOR", st.lift_fan_door}, {"AUX", st.aux_inlet_door},
                                   {"LWR", st.fan_nozzle_door}, {"ROLL", st.roll_post_door}, {"CLUTCH", st.clutch}};
            for (int i = 0; i < 5; ++i) {
                const float y = 0.98f - 0.16f * static_cast<float>(i);
                draw_text(items[i].name, lx, y, 0.056f, kPcdLabel, -1);
                std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(std::clamp(items[i].v, 0.0, 1.0) * 100.0)));
                draw_text(buf, 0.92f, y, 0.060f, door_col(items[i].v), 1);
            }
        }

        // 3BSM in side view: its arc from aft (0) round to 95 deg down.
        {
            constexpr float CX = -0.40f, CY = -0.02f, R = 0.30f;
            add_ring_sector(CX, CY, R - 0.006f, R + 0.006f, 90.0f, 185.0f, kPcdLabel, 24);
            for (int d = 0; d <= 90; d += 30) {
                const float a = (90.0f + static_cast<float>(d)) * kPi / 180.0f;
                add_bar(CX + (R - 0.05f) * std::sin(a), CY + (R - 0.05f) * std::cos(a), CX + R * std::sin(a),
                        CY + R * std::cos(a), 0.008f, kPcdLabel);
            }
            const float a = (90.0f + static_cast<float>(std::clamp(st.nozzle_pitch_deg, 0.0, 95.0))) * kPi / 180.0f;
            add_bar(CX, CY, CX + (R - 0.02f) * std::sin(a), CY + (R - 0.02f) * std::cos(a), 0.022f, kPcdWhite);
            add_circle(CX, CY, 0.035f, kPcdWhite, 16);
            draw_text("3BSM", 0.20f, 0.12f, 0.058f, kPcdLabel, -1);
            std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(st.nozzle_pitch_deg)));
            draw_text(buf, 0.92f, 0.12f, 0.070f, kPcdWhite, 1);
            draw_text("YAW", 0.20f, -0.02f, 0.058f, kPcdLabel, -1);
            std::snprintf(buf, sizeof(buf), "%+d", static_cast<int>(std::lround(st.nozzle_yaw_deg)));
            draw_text(buf, 0.92f, -0.02f, 0.062f, kPcdWhite, 1);
            draw_text("VANE", 0.20f, -0.16f, 0.058f, kPcdLabel, -1);
            std::snprintf(buf, sizeof(buf), "%+d", static_cast<int>(std::lround(st.vane_box_deg)));
            draw_text(buf, 0.92f, -0.16f, 0.062f, kPcdWhite, 1);
        }

        // Lift-post thrust bars [kN], full scale 100 kN (roll posts 10 kN).
        {
            struct Post { const char* name; double n, full; };
            const Post posts[4] = {{"FAN", st.lift_fan_thrust_n, 100e3}, {"NOZ", st.nozzle_thrust_n, 100e3},
                                   {"RP L", st.roll_post_left_n, 10e3}, {"RP R", st.roll_post_right_n, 10e3}};
            const float y0 = -0.95f, H = 0.50f;
            for (int i = 0; i < 4; ++i) {
                const float x = -0.66f + 0.44f * static_cast<float>(i);
                add_rect(x, y0 + 0.5f * H, 0.14f, H, kPcdTrack);
                const float f = static_cast<float>(std::clamp(posts[i].n / posts[i].full, 0.0, 1.0));
                if (f > 0.005f) add_rect(x, y0 + 0.5f * H * f, 0.12f, H * f, kPcdCyan);
                draw_text(posts[i].name, x, y0 - 0.08f, 0.050f, kPcdLabel, 0);
                std::snprintf(buf, sizeof(buf), "%.0f", posts[i].n / 1000.0);
                draw_text(buf, x, y0 + H + 0.03f, 0.050f, kPcdWhite, 0);
            }
        }

        // Hover margin: maximum jet lift over weight.
        {
            const double m = (st.hover_thrust_margin - 1.0) * 100.0;
            std::snprintf(buf, sizeof(buf), "HOVER MARGIN %+d%%", static_cast<int>(std::lround(std::clamp(m, -99.0, 99.0))));
            // Cautionary colours only once the lift system is in use.
            const bool live = st.equipped && st.mode != StovlMode::CTOL;
            draw_text(buf, 0.0f, -1.30f, 0.064f, !live ? kPcdLabel : m < 0.0 ? Color4::red() : m < 2.0 ? amber : kPcdGreen, 0);
            std::snprintf(buf, sizeof(buf), "JET LIFT %.0f KN", st.jet_lift_n / 1000.0);
            draw_text(buf, 0.0f, -1.45f, 0.058f, kPcdWhite, 0);
        }
    }

    /// @brief The whole PCD: the left half's large portal, the right half's
    /// two portals (or one, in a fight). @p cy and @p hh place the MFD row
    /// in the atlas.
    void draw_pcd(const fdm::FlightState& state, const AvionicsTelemetry& tel, float cy, float hh) {
        // Left half: the TSD, or the FCR's square format at 80 % centred.
        set_panel_viewport(-0.5f, cy, 0.5f, hh);
        add_rect(0.0f, 0.0f, 2.0f, 1.6f, kPcdGlass);
        if (tel.radar.page_fcr) {
            set_panel_viewport(-0.5f, cy, 0.4f, 0.8f * hh);
            draw_fcr(tel.radar);
        } else {
            draw_f35_tsd(state, tel);
        }
        // Right half: ENG/FUEL and STOVL side by side, or the sweep.
        set_panel_viewport(0.5f, cy, 0.5f, hh);
        add_rect(0.0f, 0.0f, 2.0f, 1.6f, kPcdGlass);
        if (tel.sweep.active) {
            set_panel_viewport(0.5f, cy, 0.4f, 0.8f * hh);
            draw_sweep(tel.sweep, tel);
            return;
        }
        set_panel_viewport(0.25f, cy, 0.25f, 0.5f * hh);
        draw_f35_eng(tel);
        set_panel_viewport(0.75f, cy, 0.25f, 0.5f * hh);
        draw_f35_stovl(tel);
    }

    /// @brief Liang-Barsky: clip a segment to a rectangle; false if none is left.
    static bool clip_to(float& x0, float& y0, float& x1, float& y1, float xmin, float ymin, float xmax,
                        float ymax) noexcept {
        const float dx = x1 - x0, dy = y1 - y0;
        float t0 = 0.0f, t1 = 1.0f;
        const float p[4] = {-dx, dx, -dy, dy};
        const float q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};
        for (int i = 0; i < 4; ++i) {
            if (p[i] == 0.0f) {
                if (q[i] < 0.0f) return false;
                continue;
            }
            const float t = q[i] / p[i];
            if (p[i] < 0.0f) t0 = std::max(t0, t);
            else t1 = std::min(t1, t);
            if (t0 > t1) return false;
        }
        const float ox = x0, oy = y0;
        x0 = ox + t0 * dx; y0 = oy + t0 * dy;
        x1 = ox + t1 * dx; y1 = oy + t1 * dy;
        return true;
    }

    /// @brief Update every instrument display and render off-screen to the atlas texture
    void update_and_render(const fdm::FlightState& state, const AvionicsTelemetry& telemetry = AvionicsTelemetry{}) {
        if (!initialized_) return;

        begin_canvas();

        // The MFD row fills the top half of the full atlas, or all of the
        // half-height one.
        const float row_cy = centre_page_ ? 0.5f : 0.0f;
        const float row_hh = centre_page_ ? 0.5f : 1.0f;
        // The Typhoon's MHDDs carry its own formats: FUEL left, PA centre,
        // ENG right, until a fight puts the ATTACK format and the sweep up.
        typhoon_ = telemetry.aircraft_type == aircraft::AircraftType::EUROFIGHTER_TYPHOON;
        // The F-35's one wide display: no separate centre page.
        if (telemetry.aircraft_type == aircraft::AircraftType::F35B_LIGHTNING_II) {
            draw_pcd(state, telemetry, row_cy, row_hh);
            render_canvas(Color4{0.0f, 0.0f, 0.0f, 1.0f});
            return;
        }

        set_panel_viewport(-0.5f, row_cy, 0.5f, row_hh);
        if (telemetry.radar.page_fcr) draw_fcr(telemetry.radar);
        else if (typhoon_) draw_typhoon_fuel(telemetry);
        else draw_left_mfd(state, telemetry);

        set_panel_viewport(0.5f, row_cy, 0.5f, row_hh);
        if (telemetry.sweep.active) draw_sweep(telemetry.sweep, telemetry);
        else if (typhoon_) draw_typhoon_eng(telemetry);
        else draw_right_mfd(state, telemetry);

        if (centre_page_) {
            set_panel_viewport(0.0f, -0.5f, 0.5f, 0.5f);
            if (typhoon_) draw_typhoon_pa(state, telemetry);
            else draw_center_cpd(state, telemetry);
        }

        render_canvas(Color4{0.04f, 0.05f, 0.06f, 1.0f});
    }

private:
    bool centre_page_ = true;
    bool typhoon_ = false;
};

} // namespace fastjet::graphics
