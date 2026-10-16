#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/cockpit_telemetry.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/flcs/imu_sensor.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fastjet::graphics {

/// @brief Line vertex for HUD symbology
struct HUDVertex {
    float pos[3];   // Direction vector [x, y, z] or clip-space position
    float color[4]; // RGBA
};

/// @brief Optical-infinity collimated HUD renderer with stencil combiner glass clipping
class HUDCollimator {
private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    ShaderProgram shader_;
    bool initialized_ = false;

    // Peak G recorded
    float peak_g_ = 1.0f;

    // Temporary line vertex buffer
    std::vector<HUDVertex> lines_;
    size_t masked_vertex_count_ = 0;
    /// Helmet-mounted display (F-35): no combiner. The aircraft-referenced
    /// "virtual HUD" is lines_[0, masked_vertex_count_); after it come the
    /// helmet-fixed symbols, drawn wherever the pilot looks.
    bool hmd_ = false;

    void add_line(float x0, float y0, float z0, float x1, float y1, float z1, const Color4& col) {
        HUDVertex v0{{x0, y0, z0}, {col.r, col.g, col.b, col.a}};
        HUDVertex v1{{x1, y1, z1}, {col.r, col.g, col.b, col.a}};
        lines_.push_back(v0);
        lines_.push_back(v1);
    }

    void add_rect(float x, float y, float w, float h, float z, const Color4& col) {
        // Outline box
        add_line(x - w*0.5f, y - h*0.5f, z, x + w*0.5f, y - h*0.5f, z, col);
        add_line(x + w*0.5f, y - h*0.5f, z, x + w*0.5f, y + h*0.5f, z, col);
        add_line(x + w*0.5f, y + h*0.5f, z, x - w*0.5f, y + h*0.5f, z, col);
        add_line(x - w*0.5f, y + h*0.5f, z, x - w*0.5f, y - h*0.5f, z, col);
    }

    /// @brief High-legibility 16-segment vector stroke glyph renderer for MIL-STD-1787 HUD symbology
    /// Supports digits '0'-'9', uppercase letters 'A'-'Z', and symbols '.', '-', '+', ' '
    void draw_char(char c, float cx, float cy, float scale, float z, const Color4& col) {
        const float w = 0.65f * scale;
        const float h = 1.0f * scale;
        const float l = cx - w * 0.5f;
        const float r = cx + w * 0.5f;
        const float b = cy - h * 0.5f;
        const float m = cy;
        const float t = cy + h * 0.5f;

        // 16-segment primitive stroke lambdas:
        // Outer horizontal: top, middle-left, middle-right, bottom
        auto h_top  = [&]() { add_line(l, t, z, r, t, z, col); };
        auto h_midl = [&]() { add_line(l, m, z, cx, m, z, col); };
        auto h_midr = [&]() { add_line(cx, m, z, r, m, z, col); };
        auto h_mid  = [&]() { add_line(l, m, z, r, m, z, col); };
        auto h_bot  = [&]() { add_line(l, b, z, r, b, z, col); };

        // Outer vertical: top-left, top-right, bottom-left, bottom-right
        auto v_tl = [&]() { add_line(l, m, z, l, t, z, col); };
        auto v_tr = [&]() { add_line(r, m, z, r, t, z, col); };
        auto v_bl = [&]() { add_line(l, b, z, l, m, z, col); };
        auto v_br = [&]() { add_line(r, b, z, r, m, z, col); };
        auto v_l  = [&]() { add_line(l, b, z, l, t, z, col); };
        auto v_r  = [&]() { add_line(r, b, z, r, t, z, col); };

        // Center vertical: full center, bottom-center
        auto v_c  = [&]() { add_line(cx, b, z, cx, t, z, col); };
        auto v_bc = [&]() { add_line(cx, b, z, cx, m, z, col); };

        // Diagonals:
        // Top diagonals: top-left-to-center, top-right-to-center
        auto d_tlc = [&]() { add_line(l, t, z, cx, m, z, col); };
        auto d_trc = [&]() { add_line(r, t, z, cx, m, z, col); };
        // Bottom diagonals: bottom-left-to-center, bottom-right-to-center
        auto d_blc = [&]() { add_line(l, b, z, cx, m, z, col); };
        auto d_brc = [&]() { add_line(r, b, z, cx, m, z, col); };

        switch (c) {
            // Numbers 0-9 with distinct military diagonals and cutouts
            case '0':
                h_top(); h_bot(); v_l(); v_r();
                add_line(r, t, z, l, b, z, col); // Distinctive forward slash across 0
                break;
            case '1':
                // Clear serif on top and baseline on bottom for instant recognition
                add_line(cx - 0.25f * w, t - 0.25f * h, z, cx, t, z, col);
                v_c();
                add_line(cx - 0.35f * w, b, z, cx + 0.35f * w, b, z, col);
                break;
            case '2':
                h_top(); v_tr(); h_mid(); v_bl(); h_bot();
                break;
            case '3':
                h_top(); v_tr(); h_midr(); v_br(); h_bot();
                break;
            case '4':
                v_tl(); h_mid(); v_r();
                break;
            case '5':
                h_top(); v_tl(); h_mid(); v_br(); h_bot();
                break;
            case '6':
                h_top(); v_l(); h_mid(); v_br(); h_bot();
                break;
            case '7':
                h_top(); v_tr(); v_br();
                break;
            case '8':
                h_top(); h_mid(); h_bot(); v_l(); v_r();
                break;
            case '9':
                h_top(); v_tl(); h_mid(); v_r(); h_bot();
                break;

            // Letters for HUD readouts (CAS, ALT, G, etc.)
            case 'A':
                h_top(); v_l(); v_r(); h_mid();
                break;
            case 'B':
                v_l(); h_top(); h_mid(); h_bot();
                add_line(r, m + 0.1f * h, z, r, t - 0.1f * h, z, col);
                add_line(r, b + 0.1f * h, z, r, m - 0.1f * h, z, col);
                break;
            case 'C':
                h_top(); v_l(); h_bot();
                break;
            case 'D':
                v_l(); h_top(); h_bot();
                add_line(r, b + 0.15f * h, z, r, t - 0.15f * h, z, col);
                break;
            case 'E':
                h_top(); v_l(); h_midl(); h_bot();
                break;
            case 'F':
                h_top(); v_l(); h_midl();
                break;
            case 'G':
                h_top(); v_l(); h_bot(); v_br(); h_midr();
                break;
            case 'H':
                v_l(); v_r(); h_mid();
                break;
            case 'I':
                h_top(); h_bot(); v_c();
                break;
            case 'J':
                v_r(); h_bot(); v_bl();
                break;
            case 'K':
                v_l(); d_trc(); d_brc();
                break;
            case 'L':
                v_l(); h_bot();
                break;
            case 'M':
                v_l(); v_r(); d_tlc(); d_trc();
                break;
            case 'N':
                v_l(); v_r(); add_line(l, t, z, r, b, z, col);
                break;
            case 'O':
                h_top(); h_bot(); v_l(); v_r();
                break;
            case 'P':
                v_l(); h_top(); v_tr(); h_mid();
                break;
            case 'R':
                v_l(); h_top(); v_tr(); h_mid(); d_brc();
                break;
            case 'S':
                h_top(); v_tl(); h_mid(); v_br(); h_bot();
                break;
            case 'T':
                h_top(); v_c();
                break;
            case 'U':
                v_l(); v_r(); h_bot();
                break;
            case 'V':
                add_line(l, t, z, cx, b, z, col);
                add_line(r, t, z, cx, b, z, col);
                break;
            case 'W':
                v_l(); v_r(); d_blc(); d_brc();
                break;
            case 'X':
                add_line(l, t, z, r, b, z, col);
                add_line(l, b, z, r, t, z, col);
                break;
            case 'Y':
                d_tlc(); d_trc(); v_bc();
                break;
            case 'Z':
                h_top(); add_line(r, t, z, l, b, z, col); h_bot();
                break;

            // Symbols
            case '-':
                h_mid();
                break;
            case '+':
                h_mid(); v_c();
                break;
            case '.':
                // Solid high-visibility dot
                add_rect(cx, b + 0.1f * scale, 0.18f * scale, 0.18f * scale, z, col);
                break;
            case ':':
                add_rect(cx, m + 0.2f * scale, 0.16f * scale, 0.16f * scale, z, col);
                add_rect(cx, m - 0.2f * scale, 0.16f * scale, 0.16f * scale, z, col);
                break;
            case ' ':
            default:
                break;
        }
    }

    void draw_string(const char* str, float start_x, float y, float scale, float z, const Color4& col) {
        float x = start_x;
        const float spacing = scale * 0.82f;
        while (*str) {
            draw_char(*str, x, y, scale, z, col);
            x += spacing;
            ++str;
        }
    }

    /// @brief Draw string centered at (center_x, y)
    void draw_string_centered(const char* str, float center_x, float y, float scale, float z, const Color4& col) {
        if (!str || !*str) return;
        const size_t len = std::strlen(str);
        const float spacing = scale * 0.82f;
        const float start_x = center_x - static_cast<float>(len - 1) * spacing * 0.5f;
        draw_string(str, start_x, y, scale, z, col);
    }

    /// @brief Draw string centered at (center_x, y), shrunk if need be so it
    /// is no wider than @p max_width (a long line would run off the combiner).
    void draw_string_fit(const char* str, float center_x, float y, float scale, float max_width, float z,
                         const Color4& col) {
        if (!str || !*str) return;
        const float len = static_cast<float>(std::strlen(str));
        // Width: the advance between characters plus one glyph.
        const float width = ((len - 1.0f) * 0.82f + 0.7f) * scale;
        if (width > max_width) scale *= max_width / width;
        draw_string_centered(str, center_x, y, scale, z, col);
    }

public:
    /// @brief Heading tape elevation above the boresight [rad] for an airframe.
    [[nodiscard]] static constexpr float tape_elevation(aircraft::AircraftType type) noexcept {
        return type == aircraft::AircraftType::EUROFIGHTER_TYPHOON ? 0.185f : 0.150f;
    }

    HUDCollimator() = default;

    ~HUDCollimator() {
        destroy();
    }

    bool init() {
        if (initialized_) return true;

        const char* vert_shader = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec4 aColor;

            uniform mat4 uProjection;
            uniform mat4 uView;

            out vec4 vColor;

            void main() {
                vColor = aColor;
                // Pos is a direction ray at optical infinity (e.g. z = -1000m)
                gl_Position = uProjection * uView * vec4(aPos, 1.0);
            }
        )";

        const char* frag_shader = R"(
            #version 330 core
            in vec4 vColor;
            out vec4 FragColor;

            void main() {
                FragColor = vColor;
            }
        )";

        if (!shader_.init_from_source(vert_shader, frag_shader)) {
            return false;
        }

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(HUDVertex), (void*)offsetof(HUDVertex, pos));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(HUDVertex), (void*)offsetof(HUDVertex, color));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
        if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
        shader_.destroy();
        initialized_ = false;
    }

    /// @brief Off-boresight angle beyond which the helmet repeats the
    /// primary flight data in its own frame [rad]: by then the virtual HUD
    /// is leaving the 40 x 30 degree display.
    static constexpr float kHelmetDataAngle = 0.26f;
    /// Line-of-sight elevation below which the pilot is reading the panel [rad].
    static constexpr float kCockpitDeclutterEl = 0.21f;

    /// @brief True if the jet flies with a helmet display instead of a HUD.
    [[nodiscard]] static constexpr bool helmet_display(aircraft::AircraftType type) noexcept {
        return type == aircraft::AircraftType::F35B_LIGHTNING_II;
    }

    /// @brief Build all collimated HUD symbology
    /// Symbology points are generated as 3D direction rays projected onto a virtual sphere at optical infinity (D = 100.0m)
    void build_symbology(const fdm::FlightState& state, const flcs::IMUData& imu, bool is_crashed = false, const AvionicsTelemetry& tel = AvionicsTelemetry{}) {
        lines_.clear();
        hmd_ = helmet_display(tel.aircraft_type);

        const Color4 hud_col = is_crashed ? Color4::red() : Color4::hud_green();
        const float D = 100.0f; // Virtual infinity distance [m]

        if (std::abs(static_cast<float>(imu.Nz)) > peak_g_) {
            peak_g_ = std::abs(static_cast<float>(imu.Nz));
        }

        // Elevation of the heading tape [rad]. The Typhoon's wide-angle
        // combiner is taller than the others, so its tape sits higher and
        // the pitch ladder gets the room under it.
        const float tape_el = tape_elevation(tel.aircraft_type);

        // =========================================================================
        // 0a. Airframe Type Legend
        // Always present, so the active aircraft is answerable at a glance
        // rather than only at the moment it is switched. Just under the
        // heading tape's left end, clear of the fight status line below.
        // =========================================================================
        {
            const float type_x = -D * std::tan(0.125f);
            const float type_y =  D * std::tan(tape_el) - D * 0.012f;
            const auto short_name = aircraft::to_short_string(tel.aircraft_type);
            char type_buf[16];
            std::snprintf(type_buf, sizeof(type_buf), "%.*s",
                          static_cast<int>(short_name.size()), short_name.data());
            draw_string(type_buf, type_x, type_y, D * 0.008f, -D, hud_col);
        }

        // =========================================================================
        // 0b. Airframe Change Confirmation Banner
        // Held briefly after a type change. Carries hard numbers that differ
        // between airframes, so the pilot can see the switch reached the flight
        // model and not just the name plate.
        // =========================================================================
        if (tel.aircraft_switch_timer > 0.0 && !is_crashed) {
            // Fade out over the final second so the banner retires gracefully.
            const float alpha = std::min(1.0f, static_cast<float>(tel.aircraft_switch_timer));
            const Color4 banner_col{0.0f, 1.0f, 0.25f, alpha};
            const Color4 spec_col{1.0f, 0.75f, 0.0f, alpha};

            const float banner_y = D * std::tan(0.075f);

            const auto full_name = aircraft::to_string(tel.aircraft_type);
            char name_buf[64];
            std::snprintf(name_buf, sizeof(name_buf), "%.*s",
                          static_cast<int>(full_name.size()), full_name.data());

            const auto spec = aircraft::signature_spec(tel.aircraft_type);
            char spec_buf[64];
            std::snprintf(spec_buf, sizeof(spec_buf), "%.*s",
                          static_cast<int>(spec.size()), spec.data());

            draw_string_centered("AIRFRAME CONFIGURED", 0.0f, banner_y + D * 0.030f,
                                 D * 0.010f, -D, spec_col);
            draw_string_centered(name_buf, 0.0f, banner_y, D * 0.017f, -D, banner_col);
            draw_string_centered(spec_buf, 0.0f, banner_y - D * 0.032f,
                                 D * 0.009f, -D, spec_col);
        }

        // =========================================================================
        // 0. Crash Warning Banner
        // =========================================================================
        if (is_crashed) {
            const float banner_y = D * std::tan(0.040f);
            const float box_w = D * 0.22f;
            const float box_h = D * 0.05f;
            add_rect(0.0f, banner_y, box_w, box_h, -D, Color4::red());
            draw_string_centered("CRASH", 0.0f, banner_y, D * 0.020f, -D, Color4::red());
            draw_string_centered("PRESS R TO RESET", 0.0f, banner_y - D * 0.045f, D * 0.010f, -D, Color4::amber());
        }

        // =========================================================================
        // 1. Waterline Cross (Fuselage Reference Line)
        // Fixed to aircraft body frame (+X_b forward, +Y_b right, -Z_b up)
        // In camera view space, boresight is [0, 0, -D]
        // =========================================================================
        {
            const float wl_size = D * std::tan(0.012f); // ~0.7 deg cross
            // Horizontal winglets
            add_line(-wl_size * 2.0f, 0.0f, -D, -wl_size * 0.5f, 0.0f, -D, hud_col);
            add_line( wl_size * 0.5f, 0.0f, -D,  wl_size * 2.0f, 0.0f, -D, hud_col);
            // Center miniature dot/cross
            add_line(0.0f, -wl_size * 0.4f, -D, 0.0f, wl_size * 0.4f, -D, hud_col);
        }

        // =========================================================================
        // 2. Flight Path Marker (FPM) / Velocity Vector
        // Projected at true aircraft velocity vector angles:
        // Clamped to display boundary with "Ghost FPM" symbology if outside IFOV
        // =========================================================================
        {
            const double speed = state.airspeed();
            float fpm_x = 0.0f;
            float fpm_y = 0.0f;
            bool is_ghost = false;

            if (speed > 1.0) {
                const double alpha = state.alpha();
                const double beta  = state.beta();

                fpm_x = static_cast<float>(D * std::tan(beta));
                fpm_y = static_cast<float>(D * std::tan(-alpha));

                // Display boundary limits (Instantaneous FOV of combiner
                // glass; the helmet's 40 x 30 degree field is far larger)
                const float max_fpm_x = D * std::tan(hmd_ ? 0.30f : 0.11f); // ~6.3 deg half-width
                const float max_fpm_y = D * std::tan(hmd_ ? 0.20f : 0.11f); // ~6.3 deg up
                const float min_fpm_y = -D * std::tan(hmd_ ? 0.22f : 0.08f); // ~4.6 deg down

                if (std::abs(fpm_x) > max_fpm_x) {
                    fpm_x = std::clamp(fpm_x, -max_fpm_x, max_fpm_x);
                    is_ghost = true;
                }
                if (fpm_y > max_fpm_y || fpm_y < min_fpm_y) {
                    fpm_y = std::clamp(fpm_y, min_fpm_y, max_fpm_y);
                    is_ghost = true;
                }
            }

            const Color4 fpm_col = is_ghost ? Color4::amber() : hud_col;

            // Circle of FPM (12-segment polygon)
            const float fpm_r = D * std::tan(0.007f); // ~0.4 deg radius
            constexpr int FPM_SEGS = 12;
            for (int i = 0; i < FPM_SEGS; ++i) {
                // If ghost FPM, draw dashed circle (alternate segments)
                if (is_ghost && (i % 2 == 1)) continue;
                const float a0 = i * (2.0f * 3.14159265f / FPM_SEGS);
                const float a1 = (i + 1) * (2.0f * 3.14159265f / FPM_SEGS);
                add_line(fpm_x + fpm_r * std::cos(a0), fpm_y + fpm_r * std::sin(a0), -D,
                         fpm_x + fpm_r * std::cos(a1), fpm_y + fpm_r * std::sin(a1), -D, fpm_col);
            }
            // Left winglet
            add_line(fpm_x - fpm_r, fpm_y, -D, fpm_x - fpm_r * 2.5f, fpm_y, -D, fpm_col);
            // Right winglet
            add_line(fpm_x + fpm_r, fpm_y, -D, fpm_x + fpm_r * 2.5f, fpm_y, -D, fpm_col);
            // Top vertical fin
            add_line(fpm_x, fpm_y + fpm_r, -D, fpm_x, fpm_y + fpm_r * 2.2f, -D, fpm_col);

            // If ghosted, add miniature cross through center to indicate clamped status
            if (is_ghost) {
                const float x_size = fpm_r * 0.6f;
                add_line(fpm_x - x_size, fpm_y - x_size, -D, fpm_x + x_size, fpm_y + x_size, -D, fpm_col);
                add_line(fpm_x - x_size, fpm_y + x_size, -D, fpm_x + x_size, fpm_y - x_size, -D, fpm_col);
            }
        }

        // =========================================================================
        // 3. Conformal Pitch Ladder (Horizon & Full -90 to +90 deg Pitch Rungs)
        // Roll angle phi, Pitch angle theta
        // =========================================================================
        {
            const double theta = state.pitch(); // [rad]
            const double phi   = state.roll();  // [rad]

            const float cos_phi = static_cast<float>(std::cos(phi));
            const float sin_phi = static_cast<float>(std::sin(phi));

            auto rotate_point = [&](float lx, float ly) -> std::pair<float, float> {
                return {lx * cos_phi - ly * sin_phi, lx * sin_phi + ly * cos_phi};
            };

            // Rungs every 5 degrees from -90 deg (Nadir) to +90 deg (Zenith)
            for (int deg = -90; deg <= 90; deg += 5) {
                const float rung_theta = deg * (3.14159265f / 180.0f);
                // Relative pitch angle in view space
                const float d_theta = rung_theta - static_cast<float>(theta);

                // Only render rungs within combiner glass vertical FOV,
                // and below the heading tape so the two never cross.
                if (d_theta < (hmd_ ? -0.20f : -0.04f) || d_theta > tape_el - 0.014f) continue;

                const float ly = D * std::tan(d_theta);

                if (deg == 90) {
                    // Zenith Symbol (+90 deg straight up: 8-pointed star)
                    const float z_r = D * std::tan(0.012f);
                    for (int s = 0; s < 4; ++s) {
                        const float ang = s * (3.14159265f / 4.0f);
                        const float dx = z_r * std::cos(ang);
                        const float dy = z_r * std::sin(ang);
                        auto [x0, y0] = rotate_point(-dx, ly - dy);
                        auto [x1, y1] = rotate_point( dx, ly + dy);
                        add_line(x0, y0, -D, x1, y1, -D, hud_col);
                    }
                } else if (deg == -90) {
                    // Nadir Symbol (-90 deg straight down: circle with internal cross)
                    const float n_r = D * std::tan(0.012f);
                    constexpr int NSEGS = 12;
                    for (int s = 0; s < NSEGS; ++s) {
                        const float a0 = s * (2.0f * 3.14159265f / NSEGS);
                        const float a1 = (s + 1) * (2.0f * 3.14159265f / NSEGS);
                        auto [x0, y0] = rotate_point(n_r * std::cos(a0), ly + n_r * std::sin(a0));
                        auto [x1, y1] = rotate_point(n_r * std::cos(a1), ly + n_r * std::sin(a1));
                        add_line(x0, y0, -D, x1, y1, -D, hud_col);
                    }
                    auto [cx0, cy0] = rotate_point(-n_r, ly);
                    auto [cx1, cy1] = rotate_point( n_r, ly);
                    add_line(cx0, cy0, -D, cx1, cy1, -D, hud_col);
                    auto [cy0_p, cy1_p] = rotate_point(0.0f, ly - n_r);
                    auto [cy2_p, cy3_p] = rotate_point(0.0f, ly + n_r);
                    add_line(cy0_p, cy1_p, -D, cy2_p, cy3_p, -D, hud_col);
                } else if (deg == 0) {
                    // Horizon line: continuous solid line with gap in center
                    const float w_half = D * std::tan(0.08f);
                    const float gap    = D * std::tan(0.02f);

                    auto [x0, y0] = rotate_point(-w_half, ly);
                    auto [x1, y1] = rotate_point(-gap, ly);
                    add_line(x0, y0, -D, x1, y1, -D, hud_col);

                    auto [x2, y2] = rotate_point(gap, ly);
                    auto [x3, y3] = rotate_point(w_half, ly);
                    add_line(x2, y2, -D, x3, y3, -D, hud_col);
                } else {
                    // Pitch rung: left and right bars with downward pointing tails towards horizon.
                    // Narrow enough that the rung and its tag stay inside the
                    // airspeed / altitude boxes and the stacks beneath them.
                    const float w_half = D * std::tan(0.038f);
                    const float gap    = D * std::tan(0.020f);
                    const float tail   = (deg > 0 ? -1.0f : 1.0f) * D * std::tan(0.008f);

                    if (deg > 0) {
                        // Positive pitch: solid bar
                        auto [lx0, ly0] = rotate_point(-w_half, ly);
                        auto [lx1, ly1] = rotate_point(-gap, ly);
                        auto [lxt, lyt] = rotate_point(-w_half, ly + tail);
                        add_line(lx0, ly0, -D, lx1, ly1, -D, hud_col);
                        add_line(lx0, ly0, -D, lxt, lyt, -D, hud_col);

                        auto [rx0, ry0] = rotate_point(gap, ly);
                        auto [rx1, ry1] = rotate_point(w_half, ly);
                        auto [rxt, ryt] = rotate_point(w_half, ly + tail);
                        add_line(rx0, ry0, -D, rx1, ry1, -D, hud_col);
                        add_line(rx1, ry1, -D, rxt, ryt, -D, hud_col);
                    } else {
                        // Negative pitch: dashed bars (F-16 standard dashed negative rungs)
                        const float seg_len = (w_half - gap) / 3.0f;
                        for (int s = 0; s < 3; ++s) {
                            if (s % 2 == 1) continue; // gap
                            const float x_start = -w_half + s * seg_len;
                            const float x_end   = x_start + seg_len;
                            auto [lx0, ly0] = rotate_point(x_start, ly);
                            auto [lx1, ly1] = rotate_point(x_end, ly);
                            add_line(lx0, ly0, -D, lx1, ly1, -D, hud_col);

                            const float rx_start = gap + s * seg_len;
                            const float rx_end   = rx_start + seg_len;
                            auto [rx0, ry0] = rotate_point(rx_start, ly);
                            auto [rx1, ry1] = rotate_point(rx_end, ly);
                            add_line(rx0, ry0, -D, rx1, ry1, -D, hud_col);
                        }
                        // Angle tails
                        auto [lx0, ly0] = rotate_point(-w_half, ly);
                        auto [lxt, lyt] = rotate_point(-w_half, ly + tail);
                        add_line(lx0, ly0, -D, lxt, lyt, -D, hud_col);

                        auto [rx1, ry1] = rotate_point(w_half, ly);
                        auto [rxt, ryt] = rotate_point(w_half, ly + tail);
                        add_line(rx1, ry1, -D, rxt, ryt, -D, hud_col);
                    }

                    // Numeric tag (e.g. "10", "20")
                    char tag[8];
                    std::snprintf(tag, sizeof(tag), "%02d", std::abs(deg));
                    auto [tx, ty] = rotate_point(-w_half - D * 0.015f, ly - D * 0.005f);
                    draw_string(tag, tx, ty, D * 0.010f, -D, hud_col);
                }
            }
        }

        // =========================================================================
        // 4. Heading Tape (Top of HUD)
        // Positioned safely within combiner glass top aperture
        // =========================================================================
        {
            const float top_y = D * std::tan(tape_el); // ~8.6 degrees above boresight (Typhoon ~10.6)
            const float tape_w = D * std::tan(0.082f); // ~4.7 degrees half-width

            // Baseline
            add_line(-tape_w, top_y, -D, tape_w, top_y, -D, hud_col);
            // Center heading index caret (V)
            add_line(0.0f, top_y - D * 0.007f, -D, -D * 0.005f, top_y - D * 0.014f, -D, hud_col);
            add_line(0.0f, top_y - D * 0.007f, -D,  D * 0.005f, top_y - D * 0.014f, -D, hud_col);

            const double hdg_deg = std::fmod(state.yaw() * (180.0 / 3.14159265) + 360.0, 360.0);

            // Ticks every 5 degrees
            const int center_tick = static_cast<int>(std::round(hdg_deg / 5.0)) * 5;
            for (int t = center_tick - 15; t <= center_tick + 15; t += 5) {
                const float d_deg = static_cast<float>(t - hdg_deg);
                const float tx = d_deg * (tape_w / 12.0f); // 12 deg half-span

                if (std::abs(tx) <= tape_w) {
                    const float tick_len = (t % 10 == 0) ? D * 0.008f : D * 0.005f;
                    add_line(tx, top_y, -D, tx, top_y + tick_len, -D, hud_col);

                    if (t % 10 == 0) {
                        int display_hdg = (t % 360 + 360) % 360 / 10;
                        char h_str[4];
                        std::snprintf(h_str, sizeof(h_str), "%02d", display_hdg);
                        draw_string_centered(h_str, tx, top_y + tick_len + D * 0.005f, D * 0.008f, -D, hud_col);
                    }
                }
            }
        }

        // =========================================================================
        // 5. Calibrated Airspeed (CAS) Box & Mach Readout (Left side of HUD)
        // Positioned safely within combiner glass left aperture
        // =========================================================================
        {
            const float cas_x = -D * std::tan(0.082f); // ~4.7 deg left
            const float cas_y =  D * std::tan(0.082f); // ~4.7 deg up (comfortably inside glass)
            const float box_w =  D * 0.046f;
            const float box_h =  D * 0.022f;

            add_rect(cas_x, cas_y, box_w, box_h, -D, hud_col);

            const int kts = static_cast<int>(std::round(state.airspeed() * 1.94384));
            char cas_str[8];
            // Format 3 digits with leading zero for < 1000 kts, and full 4 digits for >= 1000 kts without capping at 999
            std::snprintf(cas_str, sizeof(cas_str), (kts < 1000) ? "%03d" : "%d", std::clamp(kts, 0, 9999));
            draw_string_centered(cas_str, cas_x, cas_y, D * 0.011f, -D, hud_col);

            // Mach readout directly beneath airspeed box (MIL-STD-1787 HUD symbology)
            // Cleanly placed at +3.3 deg elevation, preventing any collision with G-meter
            const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
            const float mach = static_cast<float>(air.mach_number);
            char mach_str[12];
            if (mach >= 1.0f) {
                std::snprintf(mach_str, sizeof(mach_str), "M %1.2f", std::clamp(mach, 0.0f, 9.99f));
            } else {
                std::snprintf(mach_str, sizeof(mach_str), "M .%02d", std::clamp(static_cast<int>(std::round(mach * 100.0f)), 0, 99));
            }
            draw_string_centered(mach_str, cas_x, cas_y - box_h * 0.5f - D * 0.013f, D * 0.009f, -D, hud_col);
        }

        // =========================================================================
        // 6. Barometric Altitude Box (Right side of HUD)
        // Positioned safely within combiner glass right aperture
        // =========================================================================
        {
            const float alt_x = D * std::tan(0.082f); // ~4.7 deg right
            const float alt_y = D * std::tan(0.082f); // ~4.7 deg up (level with CAS box)
            const float box_w = D * 0.052f;
            const float box_h = D * 0.022f;

            add_rect(alt_x, alt_y, box_w, box_h, -D, hud_col);

            const int alt_ft = static_cast<int>(std::round(state.altitude() * 3.28084));
            char alt_str[10];
            std::snprintf(alt_str, sizeof(alt_str), "%05d", std::clamp(alt_ft, 0, 99999));
            draw_string_centered(alt_str, alt_x, alt_y, D * 0.011f, -D, hud_col);
        }

        // =========================================================================
        // 7. G-Meter (Bottom Left) - cleanly below Mach with dedicated spacing
        // Positioned well above bottom bezel of combiner glass
        // =========================================================================
        {
            const float g_x = -D * std::tan(0.082f);
            const float g_y =  D * std::tan(0.038f); // ~2.2 deg up (well above bottom bezel at -1.6 deg)

            char g_str[12];
            std::snprintf(g_str, sizeof(g_str), "%2.1fG", std::abs(static_cast<float>(imu.Nz)));
            draw_string(g_str, g_x - D * 0.019f, g_y, D * 0.009f, -D, hud_col);

            // Peak G display beneath
            char max_g_str[12];
            std::snprintf(max_g_str, sizeof(max_g_str), "%2.1f", peak_g_);
            draw_string(max_g_str, g_x - D * 0.019f, g_y - D * 0.0125f, D * 0.008f, -D, Color4::amber());
        }

        // =========================================================================
        // 8. Flight Annunciator Flags (Speedbrakes, Gear Warning, Reheat)
        // =========================================================================
        {
            // Speedbrake deployed annunciator
            if (tel.speedbrake_pos > 0.05) {
                const float sb_x = -D * std::tan(0.082f) - D * 0.019f;
                const float sb_y =  D * std::tan(0.012f);
                draw_string("SPDBRK", sb_x, sb_y, D * 0.0085f, -D, Color4::amber());
            }

            // Low altitude gear warning (< 350m AGL, IAS < 140 m/s with gear up)
            if (!tel.gear_deployed && environment::GroundCollision::get_agl(state) < 350.0 && state.airspeed() < 140.0) {
                const float gear_x = 0.0f;
                const float gear_y = D * std::tan(0.040f);
                draw_string_centered("CHECK GEAR", gear_x, gear_y, D * 0.011f, -D, Color4::amber());
            }

        }

        // =========================================================================
        // 9. Throttle and Fuel (right side, under the altitude box)
        // Mirrors the Mach / G stack on the left. The throttle reads the lever
        // in percent (amber, with AB, in reheat); fuel is total internal in kg,
        // amber at bingo. Kept here so they stay in view whatever the MFDs show.
        // =========================================================================
        {
            const float x0 = D * std::tan(0.082f); // Centred under the altitude box
            const float y0 = D * std::tan(0.082f) - (D * 0.022f * 0.5f) - D * 0.013f;
            const float line = D * 0.011f;
            char buf[16];

            const bool reheat = std::string(tel.detent_str) == "AFTERBURNER";
            const int thr_pct = static_cast<int>(std::lround(std::clamp(tel.throttle_input, 0.0, 1.0) * 100.0));
            std::snprintf(buf, sizeof(buf), reheat ? "AB  %3d" : "THR %3d", thr_pct);
            draw_string_centered(buf, x0, y0, D * 0.009f, -D, reheat ? Color4::amber() : hud_col);

            const int fuel_kg = static_cast<int>(std::lround(std::clamp(tel.fuel_remaining_kg, 0.0, 99999.0)));
            std::snprintf(buf, sizeof(buf), "FUEL %d", fuel_kg);
            draw_string_centered(buf, x0, y0 - line, D * 0.008f, -D, tel.bingo_fuel_alert ? Color4::amber() : hud_col);
        }

        if (tel.combat.active) build_combat_symbology(tel.combat, hud_col, D);
        if (hmd_ && tel.stovl.equipped) build_stovl_symbology(state, tel, hud_col, D);

        // All symbology is masked cleanly within the combiner glass aperture
        masked_vertex_count_ = lines_.size();
        if (hmd_ && !is_crashed) build_helmet_fixed(state, imu, tel, hud_col, D);
        // The missile warning goes in the helmet's own frame too: the pilot may be looking at it.
        if (hmd_ && !is_crashed && tel.combat.active) build_maws(tel.combat, D * std::tan(0.17f), D * 0.011f, D);

        // Upload buffer to GPU
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, lines_.size() * sizeof(HUDVertex), lines_.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    /// @brief Dogfight symbology: target designator (or locator line when the
    /// bandit is off the HUD), director gun pipper with range ring, target
    /// readouts, weapon state, damage cautions and the fight outcome.
    void build_combat_symbology(const CombatTelemetry& c, const Color4& hud_col, float D) {
        const Color4 amber = Color4::amber();
        auto to_hud = [D](const double* d, float& x, float& y) {
            if (d[0] < 0.05) return false; // behind or abeam: cannot be drawn in the HUD plane
            x = static_cast<float>(D * d[1] / d[0]);
            y = static_cast<float>(-D * d[2] / d[0]);
            return true;
        };
        auto circle = [&](float cx, float cy, float r, float frac, const Color4& col) {
            const int seg = 32;
            const int n = std::max(1, static_cast<int>(seg * frac));
            for (int i = 0; i < n; ++i) {
                // Clockwise from 12 o'clock, like a range bar unwinding.
                const float a0 = 1.5707963f - 6.2831853f * static_cast<float>(i) / seg;
                const float a1 = 1.5707963f - 6.2831853f * static_cast<float>(i + 1) / seg;
                add_line(cx + r * std::cos(a0), cy + r * std::sin(a0), -D, cx + r * std::cos(a1), cy + r * std::sin(a1),
                         -D, col);
            }
        };
        const float hud_r = D * std::tan(0.10f); // usable symbol field radius

        // Target designator box, or a locator line pointing at the bandit.
        if (c.target_valid) {
            float tx = 0.0f, ty = 0.0f;
            const bool front = to_hud(c.target_dir_b, tx, ty);
            // The helmet's virtual HUD reaches as far as the seeker reticle
            // does: the box stays on the bandit wherever the diamond can.
            const float box_r = hmd_ ? hud_r * 1.4f : hud_r;
            if (front && std::hypot(tx, ty) < box_r) {
                const float h = D * 0.007f;
                add_rect(tx, ty, 2.0f * h, 2.0f * h, -D, hud_col);
                if (c.hit_cue_s > 0.0) draw_string_centered("HIT", tx, ty + h + D * 0.006f, D * 0.008f, -D, hud_col);
            } else {
                const double ay = c.target_dir_b[1];
                const double az = -c.target_dir_b[2];
                const double n = std::max(1e-6, std::hypot(ay, az));
                const float ux = static_cast<float>(ay / n);
                const float uy = static_cast<float>(az / n);
                const float r0 = D * 0.012f;
                const float r1 = D * 0.045f;
                add_line(ux * r0, uy * r0, -D, ux * r1, uy * r1, -D, hud_col);
                char ata[8];
                std::snprintf(ata, sizeof(ata), "%d", static_cast<int>(std::round(c.ata_deg)));
                draw_string_centered(ata, ux * (r1 + D * 0.010f), uy * (r1 + D * 0.010f), D * 0.008f, -D, hud_col);
            }
        }

        // Director gun pipper: put it on the bandit and the rounds meet him.
        // The ring unwinds with range; full ring = max gun range.
        if (c.pipper_valid) {
            float px = 0.0f, py = 0.0f;
            if (to_hud(c.pipper_dir_b, px, py) && std::hypot(px, py) < hud_r * 1.3f) {
                const float r = D * 0.0175f; // 35 mil reticle
                const Color4 col = c.in_gun_range ? hud_col : amber;
                circle(px, py, r, 1.0f, col);
                add_rect(px, py, D * 0.0012f, D * 0.0012f, -D, col);
                const float frac = static_cast<float>(std::clamp(c.target_range_m / c.gun_max_range_m, 0.0, 1.0));
                circle(px, py, r * 0.8f, frac, col);
            }
        }

        // Target readouts, right, under the fuel line: range (nm), closure
        // (kt), aspect, weapon status, chaff, and flares. High enough that the whole stack stays
        // on the glass above the combiner's foot.
        const float rx = D * std::tan(0.070f);
        float ry = D * std::tan(0.031f);
        const float line = D * 0.011f;
        char buf[32];
        if (c.target_valid) {
            std::snprintf(buf, sizeof(buf), "R %.2f", c.target_range_m / 1852.0);
            draw_string(buf, rx, ry, D * 0.008f, -D, hud_col);
            ry -= line;
            std::snprintf(buf, sizeof(buf), "VC %d", static_cast<int>(std::round(c.closure_mps * 1.94384)));
            draw_string(buf, rx, ry, D * 0.008f, -D, hud_col);
            ry -= line;
            std::snprintf(buf, sizeof(buf), "AA %d", static_cast<int>(std::round(c.aspect_deg)));
            draw_string(buf, rx, ry, D * 0.008f, -D, hud_col);
            ry -= line;
        }

        // Armament: the gun, or the selected missile store's legend and count
        // ("9X 2", "120D 4"). Its lock state is cued under the pipper with SHOOT.
        if (c.selected_weapon == 0) {
            std::snprintf(buf, sizeof(buf), "%s %d", c.gun_name, c.ammo);
            draw_string(buf, rx, ry, D * 0.008f, -D, c.ammo > 0 ? hud_col : amber);
            if (c.gun_firing) draw_string("FIRE", rx + D * 0.045f, ry, D * 0.008f, -D, hud_col);
        } else {
            std::snprintf(buf, sizeof(buf), "%s %d", c.weapon_name, c.missiles_remaining);
            draw_string(buf, rx, ry, D * 0.008f, -D, c.missiles_remaining > 0 ? hud_col : amber);
        }

        if (c.chaff >= 0) {
            ry -= line;
            std::snprintf(buf, sizeof(buf), "CHF %d", c.chaff);
            draw_string(buf, rx, ry, D * 0.008f, -D, c.chaff > 0 ? hud_col : amber);
        }
        if (c.flares >= 0) {
            ry -= line;
            std::snprintf(buf, sizeof(buf), "FLR %d", c.flares);
            draw_string(buf, rx, ry, D * 0.008f, -D, c.flares > 0 ? hud_col : amber);
        }

        // Dynamic Launch Zone (DLZ) range bracket for missile employment
        if (c.selected_weapon > 0 && c.target_valid && c.dlz_rmax_m > 500.0) {
            const float dlz_x = rx + D * 0.048f;
            const float dlz_top_y = D * std::tan(0.025f);
            const float dlz_h = D * 0.055f;
            const float dlz_bot_y = dlz_top_y - dlz_h;
            const float scale = dlz_h / static_cast<float>(c.dlz_rmax_m * 1.15);

            // Vertical range scale line
            add_line(dlz_x, dlz_bot_y, -D, dlz_x, dlz_top_y, -D, hud_col);

            // Rmax tick (top)
            const float rmax_y = dlz_bot_y + static_cast<float>(c.dlz_rmax_m) * scale;
            add_line(dlz_x, rmax_y, -D, dlz_x + D * 0.006f, rmax_y, -D, hud_col);

            // Rne bracket (no-escape range)
            const float rne_y = dlz_bot_y + static_cast<float>(c.dlz_rne_m) * scale;
            add_line(dlz_x, rne_y, -D, dlz_x + D * 0.005f, rne_y, -D, hud_col);
            add_line(dlz_x + D * 0.005f, rne_y, -D, dlz_x + D * 0.005f, rmax_y, -D, hud_col);

            // Rmin tick (bottom)
            const float rmin_y = dlz_bot_y + static_cast<float>(c.dlz_rmin_m) * scale;
            add_line(dlz_x, rmin_y, -D, dlz_x + D * 0.006f, rmin_y, -D, hud_col);

            // Target range carrot (triangle pointing at current range)
            const float tgt_y = std::clamp(dlz_bot_y + static_cast<float>(c.target_range_m) * scale, dlz_bot_y, dlz_top_y);
            add_line(dlz_x - D * 0.004f, tgt_y + D * 0.002f, -D, dlz_x, tgt_y, -D, hud_col);
            add_line(dlz_x - D * 0.004f, tgt_y - D * 0.002f, -D, dlz_x, tgt_y, -D, hud_col);
            add_line(dlz_x - D * 0.004f, tgt_y - D * 0.002f, -D, dlz_x - D * 0.004f, tgt_y + D * 0.002f, -D, hud_col);
        }

        // Missile cues, centred 2 deg above the boresight: a flashing SHOOT
        // when locked inside the launch zone, otherwise a flashing NO LOCK
        // (amber) after a pickle that found nothing to shoot at, or a steady
        // LOCK. Clear of the seeker circle and gun cross below and the
        // airspeed and altitude boxes above; below the boresight it sat on the
        // horizon line and the flight path marker in level flight.
        const float cue_y = D * std::tan(0.035f);
        if (c.shoot_cue) {
            if (c.blink) draw_string_centered("SHOOT", 0.0f, cue_y, D * 0.012f, -D, hud_col);
        } else if (c.selected_weapon > 0 && c.weapon_cue[0] && c.weapon_cue_s > 0.0) {
            if (c.blink) draw_string_centered(c.weapon_cue, 0.0f, cue_y, D * 0.010f, -D, amber);
        } else if (c.selected_weapon > 0 && c.missile_locked) {
            draw_string_centered("LOCK", 0.0f, cue_y, D * 0.009f, -D, hud_col);
        }

        // Heat seeker reticle (AIM-9X / ASRAAM / AIM-9M)
        if (c.ir_seeker_active) {
            float sx = 0.0f, sy = 0.0f;
            if (to_hud(c.ir_seeker_dir_b, sx, sy) && std::hypot(sx, sy) < hud_r * 1.4f) {
                const float sr = D * 0.012f;
                if (c.ir_seeker_locked) {
                    // Solid locked diamond
                    add_line(sx, sy + sr, -D, sx + sr, sy, -D, hud_col);
                    add_line(sx + sr, sy, -D, sx, sy - sr, -D, hud_col);
                    add_line(sx, sy - sr, -D, sx - sr, sy, -D, hud_col);
                    add_line(sx - sr, sy, -D, sx, sy + sr, -D, hud_col);
                } else {
                    // Uncaged heat-seeker circle with 4 crosshair ticks
                    circle(sx, sy, sr, 1.0f, hud_col);
                    add_line(sx, sy + sr, -D, sx, sy + sr + D * 0.004f, -D, hud_col);
                    add_line(sx, sy - sr, -D, sx, sy - sr - D * 0.004f, -D, hud_col);
                    add_line(sx + sr, sy, -D, sx + sr + D * 0.004f, sy, -D, hud_col);
                    add_line(sx - sr, sy, -D, sx - sr - D * 0.004f, sy, -D, hud_col);
                }
            }
        }
        if (c.rwr_active) build_rwr(c, hud_col, D);
        build_maws(c, D * 0.069f, D * 0.009f, D);

        // Energy readouts under the G-meter: turn rate [deg/s] and Ps [ft/s].
        const float lx = -D * std::tan(0.082f) - D * 0.019f;
        float ly = -D * std::tan(0.004f);
        std::snprintf(buf, sizeof(buf), "TR %.1f", c.turn_rate_dps);
        draw_string(buf, lx, ly, D * 0.008f, -D, hud_col);
        ly -= line;
        std::snprintf(buf, sizeof(buf), "PS %+d", static_cast<int>(std::round(c.ps_mps * 3.28084)));
        draw_string(buf, lx, ly, D * 0.008f, -D, hud_col);

        // Damage cautions, centre low.
        float cy = -D * std::tan(0.025f);
        if (c.engine_fire) { draw_string_centered("ENGINE FIRE", 0.0f, cy, D * 0.010f, -D, Color4::red()); cy -= line; }
        if (c.engine_damage) { draw_string_centered("ENG DAMAGE", 0.0f, cy, D * 0.009f, -D, amber); cy -= line; }
        if (c.control_damage) { draw_string_centered("FLT CONTROL", 0.0f, cy, D * 0.009f, -D, amber); cy -= line; }
        if (c.fuel_leak) { draw_string_centered("FUEL LEAK", 0.0f, cy, D * 0.009f, -D, amber); }

        // Fight status line and the outcome banner.
        // The lines are held to FIGHT_TEXT_W, inside the narrowest combiner (the F-16's).
        const float text_w = D * FIGHT_TEXT_W;
        if (c.status[0]) draw_string_fit(c.status, 0.0f, D * std::tan(0.112f), D * 0.007f, text_w, -D, hud_col);
        if (c.banner[0]) {
            const float by = D * std::tan(0.060f);
            draw_string_fit(c.banner, 0.0f, by, D * 0.015f, text_w, -D, amber);
            float dy = D * 0.022f;
            if (c.debrief[0]) draw_string_fit(c.debrief, 0.0f, by - dy, D * 0.008f, text_w, -D, amber);
            if (c.debrief2[0]) {
                dy += D * 0.013f;
                draw_string_fit(c.debrief2, 0.0f, by - dy, D * 0.008f, text_w, -D, amber);
            }
            draw_string_centered("F9 NEW FIGHT", 0.0f, by - dy - D * 0.014f, D * 0.008f, -D, amber);
        }
    }

    /// Widest a fight status or debrief line is drawn, as a share of the HUD's
    /// focal distance (about +-7.5 deg).
    static constexpr float FIGHT_TEXT_W = 0.26f;

    /// @brief Threat cue under the heading tape: LOCK (steady amber) while a
    /// radar holds you, LAUNCH (flashing red) while a guided missile is inbound,
    /// with the threat's clock bearing ("LAUNCH 6"). The azimuth picture itself is on
    /// the RWR scope on the instrument panel.
    void build_rwr(const CombatTelemetry& c, const Color4& /*hud_col*/, float D) {
        if (c.rwr_level < 2) return;
        const bool launch = c.rwr_level >= 3;
        if (launch && !c.blink) return;
        const double bearing = launch && c.rwr_missile_valid ? c.rwr_missile_bearing_deg : c.rwr_emitter_bearing_deg;
        int clock = static_cast<int>(std::lround(std::fmod(bearing + 360.0, 360.0) / 30.0)) % 12;
        if (clock == 0) clock = 12;
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%s %d", launch ? "LAUNCH" : "LOCK", clock);
        draw_string_centered(buf, 0.0f, D * 0.084f, D * 0.009f, -D, launch ? Color4::red() : Color4::amber());
    }

    /// @brief Missile approach warning: "MISSILE 6" in red, the clock position
    /// of the most urgent missile the MAWS sees closing, with "HI" or "LO"
    /// when it is more than 30 deg off the wing plane. Steady, so it reads
    /// apart from the RWR's flashing LAUNCH line above it, and centred on
    /// @p y: under that line on the HUD, high in the helmet's own frame.
    /// The longest form ("MISSILE 12 LO") stays clear of the airspeed and
    /// altitude boxes on either side.
    void build_maws(const CombatTelemetry& c, float y, float scale, float D) {
        if (!c.maws_threat) return;
        const char* level = c.maws_elevation_deg > 30.0 ? " HI" : c.maws_elevation_deg < -30.0 ? " LO" : "";
        char buf[24];
        std::snprintf(buf, sizeof(buf), "MISSILE %d%s", std::clamp(c.maws_clock, 1, 12), level);
        draw_string_centered(buf, 0.0f, y, scale, -D, Color4::red());
    }

    /// @brief Height of the F-35B's wheels below its CG [m]: the radar
    /// altimeter reads zero on the ground.
    static double wheel_height_m() noexcept {
        static const double h = [] {
            const auto g = aircraft::AircraftConfig::get(aircraft::AircraftType::F35B_LIGHTNING_II).gear;
            return g.main_l_pos_b.z + g.rest_length;
        }();
        return h;
    }

    /// @brief Calibrated airspeed [kt], taken as equivalent airspeed.
    static double kcas(const fdm::FlightState& state) noexcept {
        const auto air = environment::Atmosphere1976::compute(std::max(state.altitude(), 0.0), state.airspeed());
        return state.airspeed() * std::sqrt(air.density / 1.225) * 1.94384;
    }

    /// @brief Radar altitude [ft], or a negative value above its 5,000 ft range.
    static int radar_altitude_ft(const fdm::FlightState& state) noexcept {
        const double ft = std::max(0.0, environment::GroundCollision::get_agl(state) - wheel_height_m()) * 3.28084;
        if (ft > 5000.0) return -1;
        // Nearest 5 ft below 500 ft (the hover and landing), 10 ft above.
        const double step = ft < 500.0 ? 5.0 : 10.0;
        return static_cast<int>(std::lround(ft / step) * step);
    }

    /// @brief F-35B STOVL symbology on the virtual HUD, from the start of a
    /// conversion until the jet is back in CTOL: the lift system's mode and
    /// control-law phase, nozzle angle and hover thrust margin (left); the
    /// vertical speed scale with the law's commanded rate (right); the
    /// hover pad with the ground-speed and drift vector and the commanded
    /// ground speed (low centre, semi- and fully jet-borne); radar altitude
    /// over the altitude box; and the conversion-speed caution.
    void build_stovl_symbology(const fdm::FlightState& state, const AvionicsTelemetry& tel, const Color4& col, float D) {
        using propulsion::StovlMode;
        const propulsion::StovlStatus& st = tel.stovl;
        if (st.mode == StovlMode::CTOL && st.conversion < 0.01) return;
        const Color4 amber = Color4::amber();
        const Color4 red = Color4::red();
        auto T = [D](float a) { return D * std::tan(a); };
        const float line = D * 0.011f;
        char buf[32];

        // Mode block, outboard of the speed column.
        {
            const float x = -T(0.175f);
            float y = T(0.055f);
            if (st.mode == StovlMode::STOVL) {
                draw_string("STOVL", x, y, D * 0.009f, -D, col);
            } else {
                std::snprintf(buf, sizeof(buf), "CONV %d", static_cast<int>(std::lround(std::clamp(st.conversion, 0.0, 1.0) * 100.0)));
                draw_string(buf, x, y, D * 0.009f, -D, amber);
            }
            y -= line;
            static constexpr const char* kPhase[3] = {"WB", "SJB", "JB"}; // wing-, semi-jet-, jet-borne
            draw_string(kPhase[std::min<int>(st.control_phase, 2)], x, y, D * 0.008f, -D, col);
            y -= line;
            std::snprintf(buf, sizeof(buf), "NOZ %d", static_cast<int>(std::lround(st.nozzle_pitch_deg)));
            draw_string(buf, x, y, D * 0.008f, -D, col);
            y -= line;
            // Hover margin: maximum jet lift over weight, as a percentage
            // above (+) or below (-) what holds the jet up.
            const double margin = (st.hover_thrust_margin - 1.0) * 100.0;
            std::snprintf(buf, sizeof(buf), "HM %+d", static_cast<int>(std::lround(std::clamp(margin, -99.0, 99.0))));
            draw_string(buf, x, y, D * 0.008f, -D, margin < 0.0 ? red : margin < 2.0 ? amber : col);
        }

        // Vertical speed scale, +-1000 fpm, outboard right of the ladder:
        // the filled caret is the climb rate, the open one the commanded.
        {
            constexpr double kFullFpm = 1000.0;
            const float x = T(0.150f), yc = -T(0.010f), H = T(0.050f);
            add_line(x, yc - H, -D, x, yc + H, -D, col);
            for (int k = -2; k <= 2; ++k) {
                const float y = yc + H * 0.5f * static_cast<float>(k);
                const float len = k == 0 ? D * 0.008f : D * 0.004f;
                add_line(x, y, -D, x + len, y, -D, col);
            }
            const double vs_fpm = -state.velocity_ned().z * 196.8504;
            const float yv = yc + H * static_cast<float>(std::clamp(vs_fpm / kFullFpm, -1.0, 1.0));
            const float c = D * 0.004f;
            add_line(x - 0.4f * c, yv, -D, x - 2.4f * c, yv + c, -D, col);
            add_line(x - 2.4f * c, yv + c, -D, x - 2.4f * c, yv - c, -D, col);
            add_line(x - 2.4f * c, yv - c, -D, x - 0.4f * c, yv, -D, col);
            add_line(x - 2.2f * c, yv, -D, x - 0.6f * c, yv, -D, col);
            const float yk = yc + H * static_cast<float>(std::clamp(st.commanded_vs_mps * 196.8504 / kFullFpm, -1.0, 1.0));
            add_line(x + 2.6f * c, yk + c, -D, x + 0.6f * c, yk, -D, col);
            add_line(x + 0.6f * c, yk, -D, x + 2.6f * c, yk - c, -D, col);
            std::snprintf(buf, sizeof(buf), "%+d", static_cast<int>(std::lround(std::clamp(vs_fpm, -9990.0, 9990.0) / 10.0) * 10));
            draw_string_centered(buf, x, yc - H - D * 0.010f, D * 0.008f, -D, col);
        }

        // Hover pad: a plan view, heading up. The ring is 30 kt; the vector
        // from the pad is the jet's ground velocity, the chevron on the
        // fore-aft axis the ground speed the control law is flying to.
        if (st.control_phase >= 1) {
            constexpr double kRingKt = 30.0;
            const float cx = 0.0f, cy = -T(0.105f), R = T(0.040f);
            constexpr int kSeg = 32;
            for (int i = 0; i < kSeg; i += 2) {
                const float a0 = 6.2831853f * static_cast<float>(i) / kSeg, a1 = 6.2831853f * static_cast<float>(i + 1) / kSeg;
                add_line(cx + R * std::cos(a0), cy + R * std::sin(a0), -D, cx + R * std::cos(a1), cy + R * std::sin(a1), -D, col);
            }
            const float p = R * 0.16f;
            add_rect(cx, cy, 2.0f * p, 2.0f * p, -D, col);
            const double kt = 1.94384;
            const float gx = static_cast<float>(std::clamp(tel.ground_right_mps * kt / kRingKt, -1.2, 1.2)) * R;
            const float gy = static_cast<float>(std::clamp(tel.ground_fwd_mps * kt / kRingKt, -1.2, 1.2)) * R;
            if (std::hypot(gx, gy) > p) add_line(cx, cy, -D, cx + gx, cy + gy, -D, col);
            // The vector's head: a small diamond (the hover cue: hold it in the pad).
            const float h = R * 0.10f;
            add_line(cx + gx, cy + gy + h, -D, cx + gx + h, cy + gy, -D, col);
            add_line(cx + gx + h, cy + gy, -D, cx + gx, cy + gy - h, -D, col);
            add_line(cx + gx, cy + gy - h, -D, cx + gx - h, cy + gy, -D, col);
            add_line(cx + gx - h, cy + gy, -D, cx + gx, cy + gy + h, -D, col);
            const float yk = cy + static_cast<float>(std::clamp(st.commanded_gs_mps * kt / kRingKt, -1.2, 1.2)) * R;
            add_line(cx - R - D * 0.006f, yk + D * 0.003f, -D, cx - R - D * 0.002f, yk, -D, col);
            add_line(cx - R - D * 0.002f, yk, -D, cx - R - D * 0.006f, yk - D * 0.003f, -D, col);
            const double gs = tel.ground_fwd_mps * kt, drift = tel.ground_right_mps * kt;
            std::snprintf(buf, sizeof(buf), "GS %d", static_cast<int>(std::lround(std::clamp(gs, -99.0, 999.0))));
            draw_string(buf, cx + R + D * 0.008f, cy + D * 0.004f, D * 0.008f, -D, col);
            std::snprintf(buf, sizeof(buf), "DR %d%c", static_cast<int>(std::lround(std::min(std::abs(drift), 99.0))),
                          drift >= 0.0 ? 'R' : 'L');
            draw_string(buf, cx + R + D * 0.008f, cy - D * 0.008f, D * 0.008f, -D, col);
        }

        // Radar altitude over the altitude box.
        {
            const int ralt = radar_altitude_ft(state);
            if (ralt >= 0) {
                std::snprintf(buf, sizeof(buf), "R %d", ralt);
                draw_string_centered(buf, T(0.082f), T(0.082f) + D * 0.021f, D * 0.009f, -D, col);
            }
        }

        // Conversion speed window: above the LiftFan's limit with the doors
        // open (or opening) the fan and its doors are being over-sped.
        {
            static constexpr double kLimit =
                aircraft::AircraftConfig::get(aircraft::AircraftType::F35B_LIGHTNING_II).stovl.max_conversion_kcas;
            const double cas = kcas(state);
            if (st.mode != StovlMode::CTOL && cas > kLimit) {
                std::snprintf(buf, sizeof(buf), "CONV SPEED %d", static_cast<int>(std::lround(cas)));
                draw_string_centered(buf, 0.0f, T(0.052f), D * 0.010f, -D, cas > kLimit * 1.1 ? red : amber);
            }
        }
    }

    /// @brief Helmet-fixed primary flight data (F-35 HMD): airspeed and
    /// altitude boxes low in the display, heading at its top, G under the
    /// airspeed and, in STOVL, the radar altitude and lift-system mode. Shown
    /// only while the pilot looks away from the nose (render_hmd()).
    void build_helmet_fixed(const fdm::FlightState& state, const flcs::IMUData& imu, const AvionicsTelemetry& tel,
                            const Color4& col, float D) {
        auto T = [D](float a) { return D * std::tan(a); };
        char buf[16];
        const float xl = -T(0.25f), xr = T(0.25f), y = -T(0.17f);
        const float box_h = D * 0.022f;
        add_rect(xl, y, D * 0.046f, box_h, -D, col);
        std::snprintf(buf, sizeof(buf), "%03d", std::clamp(static_cast<int>(std::lround(kcas(state))), 0, 999));
        draw_string_centered(buf, xl, y, D * 0.011f, -D, col);
        add_rect(xr, y, D * 0.052f, box_h, -D, col);
        std::snprintf(buf, sizeof(buf), "%05d", std::clamp(static_cast<int>(std::lround(state.altitude() * 3.28084)), 0, 99999));
        draw_string_centered(buf, xr, y, D * 0.011f, -D, col);
        std::snprintf(buf, sizeof(buf), "%2.1fG", std::abs(static_cast<float>(imu.Nz)));
        draw_string_centered(buf, xl, y - box_h - D * 0.004f, D * 0.009f, -D, col);
        const int hdg = static_cast<int>(std::lround(std::fmod(state.yaw() * (180.0 / 3.14159265) + 360.0, 360.0))) % 360;
        std::snprintf(buf, sizeof(buf), "%03d", hdg);
        add_rect(0.0f, T(0.22f), D * 0.040f, box_h, -D, col);
        draw_string_centered(buf, 0.0f, T(0.22f), D * 0.011f, -D, col);
        const propulsion::StovlStatus& st = tel.stovl;
        if (st.equipped && (st.mode != propulsion::StovlMode::CTOL || st.conversion > 0.01)) {
            const int ralt = radar_altitude_ft(state);
            if (ralt >= 0) {
                std::snprintf(buf, sizeof(buf), "R %d", ralt);
                draw_string_centered(buf, xr, y - box_h - D * 0.004f, D * 0.009f, -D, col);
            }
            draw_string_centered(propulsion::to_string(st.mode), 0.0f, y, D * 0.009f, -D, col);
        }
    }

    /// @brief Helmet display (F-35): the virtual HUD and STOVL symbols stay
    /// fixed to the jet (@p head_view turns them with the pilot's head), and
    /// once the head is turned off the nose the helmet's own flight data
    /// appear in the display's frame. The caller limits the drawing to the
    /// display's field (scissor); there is no combiner to mask it.
    void render_hmd(const Mat4& projection, const Mat4& head_view) {
        if (!initialized_ || lines_.empty()) return;
        shader_.use();
        shader_.set_mat4("uProjection", projection);
        shader_.set_mat4("uView", head_view);
        glLineWidth(2.0f);
        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(masked_vertex_count_));
        // Boresight (0, 0, -1) seen from the head: its angle off the line of
        // sight. Looking down into the cockpit (at the PCD) the helmet's own
        // data declutter.
        const float off_boresight = std::acos(std::clamp(head_view(2, 2), -1.0f, 1.0f));
        const float los_el = std::asin(std::clamp(-head_view(2, 1), -1.0f, 1.0f));
        const float los_az = std::atan2(-head_view(2, 0), head_view(2, 2));
        const bool in_cockpit = los_el < -kCockpitDeclutterEl && std::abs(los_az) < 0.8f;
        if (off_boresight > kHelmetDataAngle && !in_cockpit && lines_.size() > masked_vertex_count_) {
            shader_.set_mat4("uView", Mat4::identity());
            glDrawArrays(GL_LINES, static_cast<GLint>(masked_vertex_count_),
                         static_cast<GLsizei>(lines_.size() - masked_vertex_count_));
        }
        glBindVertexArray(0);
        glLineWidth(1.0f);
    }

    /// @brief Render collimated symbology that is masked to the physical combiner glass
    void render_masked(const Mat4& projection, const Mat4& view = Mat4::identity()) {
        if (!initialized_ || masked_vertex_count_ == 0) return;

        shader_.use();
        shader_.set_mat4("uProjection", projection);
        shader_.set_mat4("uView", view);

        glLineWidth(2.0f);
        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(masked_vertex_count_));
        glBindVertexArray(0);
        glLineWidth(1.0f);
    }

    /// @brief Render all HUD symbology without aperture clipping (for
    /// external / chase view). A helmet's own helmet-fixed data stay in it.
    void render_unmasked(const Mat4& projection, const Mat4& view = Mat4::identity()) {
        if (!initialized_ || lines_.empty()) return;

        shader_.use();
        shader_.set_mat4("uProjection", projection);
        shader_.set_mat4("uView", view);

        glLineWidth(2.0f);
        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(masked_vertex_count_));
        glBindVertexArray(0);
        glLineWidth(1.0f);
    }

    /// @brief Render all symbology (convenience / backwards-compatible)
    void render(const Mat4& projection, const Mat4& view = Mat4::identity()) {
        if (!initialized_ || lines_.empty()) return;

        shader_.use();
        shader_.set_mat4("uProjection", projection);
        shader_.set_mat4("uView", view);

        glLineWidth(2.0f);
        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines_.size()));
        glBindVertexArray(0);
        glLineWidth(1.0f);
    }

    size_t vertex_count() const noexcept { return lines_.size(); }
    size_t masked_vertex_count() const noexcept { return masked_vertex_count_; }
    bool helmet_mounted() const noexcept { return hmd_; }
    bool is_initialized() const noexcept { return initialized_; }
};

} // namespace fastjet::graphics
