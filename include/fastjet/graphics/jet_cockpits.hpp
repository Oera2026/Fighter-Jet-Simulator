#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/graphics/cockpit_gauges.hpp"
#include "fastjet/graphics/cockpit_geometry.hpp"
#include "fastjet/graphics/jet_airframes.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace fastjet::graphics {

/// @brief One airframe's cockpit, in body axes around the shared design eye
/// point (X 1.80, Y 0, Z -0.65): the interior seen from the seat (tub,
/// panel, consoles, seat, canopy frames), the HUD combiner (the stencil
/// aperture for the HUD), the live displays and the instrument faces. The
/// skin, canopy glass, wings and fins around it come from the airframe
/// model (jet_airframes.hpp), drawn in cockpit alignment.
struct CockpitLayout {
    std::vector<Vertex3D> shell;      ///< Interior and canopy frames
    std::vector<Vertex3D> combiner;   ///< HUD combiner glass
    std::vector<ScreenQuad> screens;  ///< FlightInstruments atlas (centre page included)
    std::vector<ScreenQuad> gauges;   ///< CockpitGauges atlas cells
    /// No HUD: the symbology is on the pilot's helmet (F-35), so there is no
    /// combiner and nothing masks it.
    bool helmet_display = false;
};

namespace cockpit_build {

/// The design eye point the cockpit is built around (CameraRig::DEP_*).
inline constexpr float EYE_X = 1.80f;
inline constexpr float EYE_Z = -0.65f;

struct Rgb {
    float r, g, b;
};
using P3 = std::array<float, 3>;

/// FlightInstruments atlas cells with the centre page: {u0, u1, v0, v1}.
struct AtlasCell {
    float u0, u1, v0, v1;
};
inline constexpr AtlasCell LEFT_PAGE{0.0f, 0.5f, 0.5f, 1.0f};
inline constexpr AtlasCell RIGHT_PAGE{0.5f, 1.0f, 0.5f, 1.0f};
inline constexpr AtlasCell CENTRE_PAGE{0.25f, 0.75f, 0.0f, 0.5f};
/// The F-35's Panoramic Cockpit Display: two 10 x 8 in halves, each the
/// middle 80 % of the height of an MFD page (FlightInstruments draws them so).
inline constexpr AtlasCell PCD_LEFT{0.0f, 0.5f, 0.55f, 0.95f};
inline constexpr AtlasCell PCD_RIGHT{0.5f, 1.0f, 0.55f, 0.95f};

/// @brief A flat instrument panel raked back toward the pilot:
/// X = x_top at Z = z_top (under the glare shield) to X = x_bot at Z = z_bot.
struct Plane {
    float x_top, z_top, x_bot, z_bot;
    [[nodiscard]] float x(float z, float proud) const noexcept {
        const float t = (z - z_top) / (z_bot - z_top);
        return x_top + t * (x_bot - x_top) - proud;
    }
};

/**
 * @brief Triangle soup builder for the cockpit shell.
 *
 * Normals come from the winding but the shell shader turns every face
 * toward the eye, so winding does not matter.
 */
class Builder {
public:
    explicit Builder(std::vector<Vertex3D>& out) : out_(out) {}

    void tri(const P3& a, const P3& b, const P3& c, const Rgb& col) {
        const float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const float w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float n[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
        const float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len <= 1e-12f) return;
        n[0] /= len;
        n[1] /= len;
        n[2] /= len;
        for (const P3* p : {&a, &b, &c}) {
            out_.push_back(Vertex3D{{(*p)[0], (*p)[1], (*p)[2]}, {n[0], n[1], n[2]}, {0.0f, 0.0f}, {col.r, col.g, col.b, 1.0f}});
        }
    }

    void quad(const P3& a, const P3& b, const P3& c, const P3& d, const Rgb& col) {
        tri(a, b, c, col);
        tri(a, c, d, col);
    }

    /// @brief The same quad and its mirror image across the centreline (Y -> -Y).
    void quad_pair(const P3& a, const P3& b, const P3& c, const P3& d, const Rgb& col) {
        quad(a, b, c, d, col);
        quad(mirror(a), mirror(b), mirror(c), mirror(d), col);
    }

    [[nodiscard]] static P3 mirror(const P3& p) noexcept { return {p[0], -p[1], p[2]}; }

    /// @brief Axis-aligned box centred at (cx, cy, cz).
    void box(float cx, float cy, float cz, float hx, float hy, float hz, const Rgb& col) {
        const float x0 = cx - hx, x1 = cx + hx, y0 = cy - hy, y1 = cy + hy, z0 = cz - hz, z1 = cz + hz;
        const P3 p[8] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                         {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        quad(p[0], p[1], p[2], p[3], col);
        quad(p[4], p[7], p[6], p[5], col);
        quad(p[0], p[4], p[5], p[1], col);
        quad(p[1], p[5], p[6], p[2], col);
        quad(p[2], p[6], p[7], p[3], col);
        quad(p[3], p[7], p[4], p[0], col);
    }

    /// @brief A square-section bar from p0 to p1 (canopy frames, HUD posts).
    void beam(const P3& p0, const P3& p1, float half_w, float half_h, const Rgb& col) {
        float d[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
        const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len <= 1e-6f) return;
        d[0] /= len; d[1] /= len; d[2] /= len;
        // A reference axis that is not parallel to the bar.
        float r[3] = {0.0f, 0.0f, -1.0f};
        if (std::fabs(d[2]) > 0.9f) { r[0] = 1.0f; r[2] = 0.0f; }
        float s[3] = {d[1] * r[2] - d[2] * r[1], d[2] * r[0] - d[0] * r[2], d[0] * r[1] - d[1] * r[0]};
        const float sl = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
        s[0] /= sl; s[1] /= sl; s[2] /= sl;
        const float u[3] = {s[1] * d[2] - s[2] * d[1], s[2] * d[0] - s[0] * d[2], s[0] * d[1] - s[1] * d[0]};
        auto corner = [&](const P3& base, float a, float b) {
            return P3{base[0] + s[0] * a * half_w + u[0] * b * half_h, base[1] + s[1] * a * half_w + u[1] * b * half_h,
                      base[2] + s[2] * a * half_w + u[2] * b * half_h};
        };
        const float sx[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
        const float sy[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
        for (int i = 0; i < 4; ++i) {
            const int j = (i + 1) % 4;
            quad(corner(p0, sx[i], sy[i]), corner(p1, sx[i], sy[i]), corner(p1, sx[j], sy[j]), corner(p0, sx[j], sy[j]), col);
        }
    }

    /// @brief A round tube from p0 to p1 with closed ends (sticks, handles).
    void tube(const P3& p0, const P3& p1, float radius, int sides, const Rgb& col) {
        float d[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
        const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len <= 1e-6f) return;
        d[0] /= len; d[1] /= len; d[2] /= len;
        float r[3] = {0.0f, 0.0f, -1.0f};
        if (std::fabs(d[2]) > 0.9f) { r[0] = 1.0f; r[2] = 0.0f; }
        float s[3] = {d[1] * r[2] - d[2] * r[1], d[2] * r[0] - d[0] * r[2], d[0] * r[1] - d[1] * r[0]};
        const float sl = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
        s[0] /= sl; s[1] /= sl; s[2] /= sl;
        const float u[3] = {s[1] * d[2] - s[2] * d[1], s[2] * d[0] - s[0] * d[2], s[0] * d[1] - s[1] * d[0]};
        auto ring = [&](const P3& c, int i) {
            const float a = 6.2831853f * static_cast<float>(i) / static_cast<float>(sides);
            const float ca = std::cos(a) * radius, sa = std::sin(a) * radius;
            return P3{c[0] + s[0] * ca + u[0] * sa, c[1] + s[1] * ca + u[1] * sa, c[2] + s[2] * ca + u[2] * sa};
        };
        for (int i = 0; i < sides; ++i) {
            quad(ring(p0, i), ring(p1, i), ring(p1, i + 1), ring(p0, i + 1), col);
            tri(p0, ring(p0, i + 1), ring(p0, i), col);
            tri(p1, ring(p1, i), ring(p1, i + 1), col);
        }
    }

    /// @brief A raised plate on a panel plane: its face, plus edges so it
    /// reads as a separate box catching the light.
    void panel_box(const Plane& pl, float y_c, float z_c, float half_w, float half_h, float proud, const Rgb& col) {
        const float y0 = y_c - half_w, y1 = y_c + half_w;
        const float z0 = z_c - half_h, z1 = z_c + half_h;
        const P3 a{pl.x(z0, proud), y0, z0}, b{pl.x(z0, proud), y1, z0};
        const P3 c{pl.x(z1, proud), y1, z1}, d{pl.x(z1, proud), y0, z1};
        quad(a, b, c, d, col);
        const P3 a0{pl.x(z0, 0.0f), y0, z0}, b0{pl.x(z0, 0.0f), y1, z0};
        const P3 c0{pl.x(z1, 0.0f), y1, z1}, d0{pl.x(z1, 0.0f), y0, z1};
        quad(a0, b0, b, a, col);
        quad(d, c, c0, d0, col);
        quad(a0, a, d, d0, col);
        quad(b, b0, c0, c, col);
    }

    /// @brief A row of small keys along Y on the panel plane.
    void key_row(const Plane& pl, float y_from, float y_to, int count, float z_c, float key_w, float key_h,
                 const Rgb& col) {
        for (int i = 0; i < count; ++i) {
            const float t = count > 1 ? static_cast<float>(i) / static_cast<float>(count - 1) : 0.5f;
            panel_box(pl, y_from + t * (y_to - y_from), z_c, key_w, key_h, 0.008f, col);
        }
    }

    /// @brief A column of small keys along Z on the panel plane.
    void key_column(const Plane& pl, float y_c, float z_from, float z_to, int count, float key_w, float key_h,
                    const Rgb& col) {
        for (int i = 0; i < count; ++i) {
            const float t = count > 1 ? static_cast<float>(i) / static_cast<float>(count - 1) : 0.5f;
            panel_box(pl, y_c, z_from + t * (z_to - z_from), key_w, key_h, 0.008f, col);
        }
    }

private:
    std::vector<Vertex3D>& out_;
};

// ---------------------------------------------------------------------------
// Displays and instruments on a panel plane
// ---------------------------------------------------------------------------

[[nodiscard]] inline std::array<std::array<float, 3>, 4> panel_corners(const Plane& pl, float y_c, float z_c,
                                                                       float half_w, float half_h, float proud) {
    const float zt = z_c - half_h, zb = z_c + half_h; // Z grows downward: the top edge is zt
    const float xt = pl.x(zt, proud), xb = pl.x(zb, proud);
    return {{{xt, y_c - half_w, zt}, {xt, y_c + half_w, zt}, {xb, y_c + half_w, zb}, {xb, y_c - half_w, zb}}};
}

/// @brief A live display showing one atlas page.
[[nodiscard]] inline ScreenQuad panel_screen(const Plane& pl, float y_c, float z_c, float half_w, float half_h,
                                             const AtlasCell& cell) {
    ScreenQuad q;
    q.corners = panel_corners(pl, y_c, z_c, half_w, half_h, 0.007f);
    q.u0 = cell.u0;
    q.u1 = cell.u1;
    q.v0 = cell.v0;
    q.v1 = cell.v1;
    return q;
}

/// @brief A working instrument face (CockpitGauges cell) on the panel.
[[nodiscard]] inline ScreenQuad panel_gauge(const Plane& pl, Gauge g, float y_c, float z_c, float half_w, float half_h) {
    return CockpitGauges::screen(g, panel_corners(pl, y_c, z_c, half_w, half_h, 0.0065f));
}

/// @brief A display bezel with its option keys and the dark well the glass sits in.
inline void display_bezel(Builder& b, const Plane& pl, float y_c, float z_c, float half_w, float half_h,
                          const Rgb& bezel, const Rgb& well, const Rgb& key, int keys_x, int keys_y,
                          float side_span = 0.8f) {
    b.panel_box(pl, y_c, z_c, half_w + 0.026f, half_h + 0.026f, 0.004f, bezel);
    b.panel_box(pl, y_c, z_c, half_w + 0.003f, half_h + 0.003f, 0.0055f, well);
    const float kx = 0.009f, kz = 0.006f;
    if (keys_x > 0) {
        b.key_row(pl, y_c - half_w * 0.8f, y_c + half_w * 0.8f, keys_x, z_c - half_h - 0.015f, kx, kz, key);
        b.key_row(pl, y_c - half_w * 0.8f, y_c + half_w * 0.8f, keys_x, z_c + half_h + 0.015f, kx, kz, key);
    }
    if (keys_y > 0) {
        b.key_column(pl, y_c - half_w - 0.016f, z_c - half_h * side_span, z_c + half_h * side_span, keys_y, kz, kx, key);
        b.key_column(pl, y_c + half_w + 0.016f, z_c - half_h * side_span, z_c + half_h * side_span, keys_y, kz, kx, key);
    }
}

/// @brief Silk-screened legend text as it reads from the seat: a line of
/// word-length marks with gaps, not one solid bar. Calls
/// @p mark(centre, half_length) for each word between @p from and @p to.
template <class Mark>
inline void legend_words(float from, float to, Mark&& mark) {
    constexpr float kWords[5] = {0.22f, 0.12f, 0.30f, 0.18f, 0.26f}; // fractions of the line
    const float len = to - from;
    float pos = from;
    for (int i = 0; pos < to - 1e-4f; ++i) {
        const float w = std::min(kWords[i % 5] * len, to - pos);
        mark(pos + 0.5f * w, 0.5f * w);
        pos += w + 0.06f * len;
    }
}

/// @brief A legend line on a panel plane, centred at (y_c, z_c).
inline void panel_legend(Builder& b, const Plane& pl, float y_c, float z_c, float half_len, const Rgb& col) {
    legend_words(y_c - half_len, y_c + half_len,
                 [&](float c, float h) { b.panel_box(pl, c, z_c, h, 0.0016f, 0.0034f, col); });
}

/// @brief Round instrument: bezel ring (square plate) and its working face.
inline void dial(Builder& b, CockpitLayout& lay, const Plane& pl, Gauge g, float y_c, float z_c, float half,
                 const Rgb& bezel) {
    b.panel_box(pl, y_c, z_c, half + 0.006f, half + 0.006f, 0.003f, bezel);
    lay.gauges.push_back(panel_gauge(pl, g, y_c, z_c, half, half));
}

// ---------------------------------------------------------------------------
// The parts every cockpit shares, shaped per airframe
// ---------------------------------------------------------------------------

struct TubSpec {
    /// Panel face: its top edge 0.12 m below the eye and 0.70 m ahead of it,
    /// raked back ~17 degrees, like the F-16C's panel in the asset.
    Plane panel{2.50f, -0.53f, 2.40f, -0.20f};
    float half_w = 0.42f;        ///< Instrument panel half-width
    float console_z = -0.22f;    ///< Side console tops
    float floor_z = 0.26f;       ///< Cockpit floor (heel line ~0.9 m below the eye)
    float seat_recline = 0.30f;  ///< Seat back rake [rad] (F-16: 30 deg; most others ~17 deg)
    Rgb glare{0.014f, 0.014f, 0.016f};
    Rgb panel_col{0.035f, 0.038f, 0.042f};
    Rgb console{0.030f, 0.032f, 0.035f};
    Rgb rail{0.045f, 0.047f, 0.050f};
    Rgb dark{0.020f, 0.020f, 0.022f};
    Rgb key{0.058f, 0.060f, 0.063f};
    Rgb switch_metal{0.090f, 0.092f, 0.095f};
    Rgb knob{0.012f, 0.012f, 0.013f};
    Rgb legend{0.32f, 0.32f, 0.30f}; ///< Silk-screened panel legends
    Rgb guard_red{0.200f, 0.030f, 0.025f};
    Rgb seat{0.040f, 0.042f, 0.040f};
    Rgb cushion{0.055f, 0.050f, 0.040f};
    Rgb handle_yellow{0.42f, 0.30f, 0.02f};
    bool side_stick = false;
    int throttles = 2;
};

/// @brief Glare shield, panel face, consoles, walls, floor, seat, stick and throttles.
inline void tub(Builder& b, const TubSpec& s, const CanopyFit& fit) {
    const Plane& pl = s.panel;
    const float hw = s.half_w;
    const float sill = EYE_Z + fit.sill_dz;
    const float yi = fit.half_span - 0.07f;              // inner edge of the sill rails
    const float x_aft = EYE_X + fit.tub_aft_dx;          // aft bulkhead
    const float x_ws = EYE_X + fit.ws_dx;                // windscreen foot
    const float z_ws = EYE_Z + fit.coaming_dz;

    // Glare shield: a hood whose lip overhangs the panel's top edge, sloping
    // forward and down to the windscreen foot. The lip sets the view over
    // the nose (about 6 degrees down).
    const float x_lip = pl.x_top - 0.04f, z_lip = pl.z_top - 0.045f;
    const float hw_ws = canopy_station(fit, fit.ws_dx).span * 0.95f;
    b.quad({x_lip, -hw, z_lip}, {x_lip, hw, z_lip}, {x_ws, hw_ws, z_ws}, {x_ws, -hw_ws, z_ws}, s.glare);
    b.quad({x_lip, -hw, z_lip}, {x_lip, hw, z_lip}, {pl.x_top, hw, pl.z_top}, {pl.x_top, -hw, pl.z_top}, s.glare);
    // Hood cheeks down to the sills.
    b.quad_pair({x_lip, hw, z_lip}, {x_ws, hw_ws, z_ws}, {x_ws, yi + 0.05f, sill}, {pl.x_top, hw, sill}, s.glare);

    // Panel face, and the kick panel down to the floor under it.
    b.quad({pl.x_top, -hw, pl.z_top}, {pl.x_top, hw, pl.z_top}, {pl.x_bot, hw, pl.z_bot}, {pl.x_bot, -hw, pl.z_bot},
           s.panel_col);
    b.quad({pl.x_bot, -hw, pl.z_bot}, {pl.x_bot, hw, pl.z_bot}, {pl.x_bot + 0.22f, hw * 0.7f, s.floor_z},
           {pl.x_bot + 0.22f, -hw * 0.7f, s.floor_z}, s.dark);
    // Panel cheeks from the panel's sides out to the walls.
    b.quad_pair({pl.x_top, hw, pl.z_top}, {pl.x_top, yi, sill}, {pl.x_bot, yi, s.console_z}, {pl.x_bot, hw, pl.z_bot},
                s.panel_col);
    b.quad_pair({pl.x_top, hw, pl.z_top}, {pl.x_top, yi, sill}, {x_lip, yi + 0.02f, sill - 0.02f}, {x_lip, hw, z_lip},
                s.glare);

    // Canopy sill rails, then the walls down to the consoles.
    b.quad_pair({x_aft, yi, sill}, {x_ws, yi, sill}, {x_ws, yi + 0.13f, sill + 0.01f}, {x_aft, yi + 0.13f, sill + 0.01f},
                s.rail);
    b.quad_pair({x_aft, yi, sill}, {pl.x_top, yi, sill}, {pl.x_bot, yi, s.console_z}, {x_aft, yi, s.console_z}, s.console);

    // Side consoles, their inboard faces down to the floor, and the floor.
    // The console top slopes down aft by 3 cm (z grows downward).
    auto console_top = [&](float x) {
        return s.console_z + 0.02f - 0.04f * (pl.x_bot - x) / (pl.x_bot - x_aft);
    };
    for (float side : {-1.0f, 1.0f}) {
        const float y_in = side * 0.22f, y_out = side * yi;
        b.quad({pl.x_bot, y_in, s.console_z + 0.02f}, {pl.x_bot, y_out, s.console_z},
               {x_aft, y_out, s.console_z - 0.03f}, {x_aft, y_in, s.console_z - 0.01f}, s.console);
        b.quad({pl.x_bot, y_in, s.console_z + 0.02f}, {x_aft, y_in, s.console_z - 0.01f}, {x_aft, y_in, s.floor_z},
               {pl.x_bot + 0.22f, y_in, s.floor_z}, s.panel_col);
        // Switch panels, edge to edge across the console, each with its own
        // hardware: bat-handle toggles on threaded bushings, rotary knobs,
        // guarded switches, push buttons and silk-screened legend strips.
        // The throttle quadrant (left) and a side-stick (right) take the
        // place of the panels beside the pilot's hips.
        for (int i = 0; i < 5; ++i) {
            const float x = pl.x_bot - 0.10f - 0.15f * static_cast<float>(i);
            if (side < 0.0f && (i == 1 || i == 2)) continue;
            if (side > 0.0f && s.side_stick && i == 2) continue;
            const float z = console_top(x) - 0.004f;
            const float yc = side * 0.345f;
            b.box(x, yc, z, 0.068f, 0.112f, 0.004f, s.panel_col);
            const float face = z - 0.004f;
            auto toggle = [&](float tx, float ty, bool fwd) {
                b.tube({tx, ty, face}, {tx, ty, face - 0.006f}, 0.0055f, 6, s.switch_metal);
                b.tube({tx, ty, face - 0.006f}, {tx + (fwd ? 0.008f : -0.008f), ty, face - 0.021f}, 0.0022f, 6,
                       s.switch_metal);
            };
            auto knob = [&](float kx, float ky, float r) {
                b.tube({kx, ky, face}, {kx, ky, face - 0.013f}, r, 10, s.knob);
                b.box(kx + r * 0.6f, ky, face - 0.0133f, r * 0.5f, 0.0012f, 0.0006f, s.legend);
            };
            auto legend = [&](float lx, float ly, float half_len) {
                legend_words(ly - half_len, ly + half_len,
                             [&](float c, float h) { b.box(lx, c, face - 0.0006f, 0.0016f, h, 0.0006f, s.legend); });
            };
            switch ((i + (side > 0.0f ? 2 : 0)) % 4) {
                case 0: // three toggles and a selector knob
                    legend(x + 0.034f, yc - side * 0.020f, 0.060f);
                    for (int k = 0; k < 3; ++k) toggle(x + 0.012f, yc + side * (-0.075f + 0.040f * static_cast<float>(k)), (i + k) % 2 == 0);
                    knob(x - 0.025f, yc + side * 0.070f, 0.0095f);
                    legend(x - 0.045f, yc - side * 0.040f, 0.050f);
                    break;
                case 1: // two rotaries and a guarded switch
                    knob(x - 0.010f, yc - side * 0.060f, 0.012f);
                    knob(x - 0.010f, yc - side * 0.010f, 0.009f);
                    b.box(x + 0.005f, yc + side * 0.060f, face - 0.010f, 0.016f, 0.012f, 0.010f, s.guard_red);
                    legend(x + 0.040f, yc, 0.090f);
                    break;
                case 2: // a row of four toggles over a second row of two
                    legend(x + 0.040f, yc, 0.090f);
                    for (int k = 0; k < 4; ++k) toggle(x + 0.020f, yc + side * (-0.075f + 0.050f * static_cast<float>(k)), k % 2 == 1);
                    for (int k = 0; k < 2; ++k) toggle(x - 0.030f, yc + side * (-0.050f + 0.060f * static_cast<float>(k)), true);
                    break;
                default: // push buttons and a small knob
                    for (int r = 0; r < 2; ++r) {
                        for (int k = 0; k < 3; ++k) {
                            b.box(x + 0.022f - 0.030f * static_cast<float>(r), yc + side * (-0.070f + 0.035f * static_cast<float>(k)),
                                  face - 0.004f, 0.010f, 0.012f, 0.004f, s.key);
                        }
                    }
                    knob(x - 0.010f, yc + side * 0.070f, 0.010f);
                    legend(x + 0.045f, yc, 0.090f);
                    break;
            }
        }
    }
    b.quad({x_aft, -0.22f, s.floor_z}, {pl.x_bot + 0.22f, -0.22f, s.floor_z}, {pl.x_bot + 0.22f, 0.22f, s.floor_z},
           {x_aft, 0.22f, s.floor_z}, s.dark);
    // Aft bulkhead behind the seat.
    b.quad({x_aft, -yi, sill}, {x_aft, yi, sill}, {x_aft, yi, s.floor_z}, {x_aft, -yi, s.floor_z}, s.console);

    // Rudder pedals, standing 25 mm off the kick panel and lying along it.
    {
        auto kick_x = [&](float z) { return pl.x_bot + 0.22f * (z - pl.z_bot) / (s.floor_z - pl.z_bot); };
        const float z1 = s.floor_z - 0.24f, z2 = s.floor_z - 0.06f;
        for (float side : {-1.0f, 1.0f}) {
            b.beam({kick_x(z1) - 0.025f, side * 0.13f, z1}, {kick_x(z2) - 0.025f, side * 0.13f, z2}, 0.045f, 0.010f,
                   s.switch_metal);
        }
    }

    // Ejection seat: pan, reclined back, headbox, side rails and the
    // yellow-and-black firing handle between the knees. The seat sits
    // where the pilot's body puts it: the back surface 0.24 m behind the eye
    // at eye height (back of the head plus helmet and headrest pad), so the
    // hip falls just ahead of the eye line and the pan runs forward from it.
    const float lean = std::tan(s.seat_recline);
    const float pan_z = EYE_Z + 0.74f; // cushion top ~0.71 m under the eye: a seated 50th-percentile pilot, 17 deg rake
    const float pan_x0 = EYE_X - 0.24f + (pan_z - EYE_Z) * lean, pan_x1 = pan_x0 + 0.46f;
    b.box(0.5f * (pan_x0 + pan_x1), 0.0f, pan_z + 0.05f, 0.5f * (pan_x1 - pan_x0), 0.23f, 0.05f, s.seat);
    b.box(0.5f * (pan_x0 + pan_x1), 0.0f, pan_z - 0.01f, 0.5f * (pan_x1 - pan_x0) - 0.02f, 0.20f, 0.02f, s.cushion);
    const float back_top_z = EYE_Z - 0.05f;
    const float back_x_bot = pan_x0, back_x_top = pan_x0 - (pan_z - back_top_z) * lean;
    // Seat side rails, up beside the headbox, their tops clear of the glass.
    auto under_canopy = [&](float x, float y, float want_z) {
        const CanopyStation st = canopy_station(fit, x - EYE_X);
        const float c = std::sqrt(std::max(0.0f, 1.0f - (y / st.span) * (y / st.span)));
        return std::max(want_z, sill + (EYE_Z + st.crown_dz - sill) * c + 0.015f);
    };
    const float rail_top = under_canopy(back_x_top - 0.08f, 0.27f, back_top_z - 0.30f) + 0.03f;
    for (float side : {-1.0f, 1.0f}) {
        b.beam({back_x_bot - 0.03f, side * 0.24f, s.floor_z - 0.05f}, {back_x_top - 0.05f, side * 0.24f, rail_top},
               0.025f, 0.03f, s.seat);
    }
    b.quad({back_x_bot, -0.21f, pan_z}, {back_x_bot, 0.21f, pan_z}, {back_x_top, 0.21f, back_top_z},
           {back_x_top, -0.21f, back_top_z}, s.cushion);
    // Headrest pad, then the headbox (parachute container) behind it.
    b.box(back_x_top - 0.012f, 0.0f, back_top_z - 0.10f, 0.012f, 0.09f, 0.08f, s.cushion);
    {
        // The headbox stands up to just under the canopy: its top corners
        // keep 15 mm clear of the glass.
        const float hx = back_x_top - 0.084f, hy = 0.135f;
        const CanopyStation st = canopy_station(fit, hx - 0.06f - EYE_X);
        const float c = std::sqrt(std::max(0.0f, 1.0f - (hy / st.span) * (hy / st.span)));
        const float top = std::max(back_top_z - 0.38f, sill + (EYE_Z + st.crown_dz - sill) * c + 0.015f);
        const float bot = back_top_z + 0.02f;
        b.box(hx, 0.0f, 0.5f * (top + bot), 0.06f, hy, 0.5f * (bot - top), s.seat);
    }
    b.tube({pan_x1 + 0.01f, -0.07f, pan_z - 0.01f}, {pan_x1 + 0.01f, 0.07f, pan_z - 0.01f}, 0.012f, 8, s.handle_yellow);

    // Throttle quadrant, forward on the left console where the left hand
    // falls: a raised quadrant with its slots and one lever per engine, at
    // mid travel, each with a chunky HOTAS grip.
    const float qx0 = pan_x0 + 0.10f, qx1 = pan_x0 + 0.42f, qy = -0.300f;
    const float qz = console_top(0.5f * (qx0 + qx1)) - 0.012f;
    b.box(0.5f * (qx0 + qx1), qy, qz, 0.5f * (qx1 - qx0), 0.050f, 0.012f, s.console);
    for (int i = 0; i < s.throttles; ++i) {
        const float y = qy + (s.throttles > 1 ? -0.017f + 0.034f * static_cast<float>(i) : 0.0f);
        b.box(0.5f * (qx0 + qx1), y, qz - 0.0125f, 0.5f * (qx1 - qx0) - 0.02f, 0.004f, 0.0008f, s.dark); // slot
        const float lx = 0.5f * (qx0 + qx1) + 0.02f;
        b.beam({lx, y, qz - 0.012f}, {lx + 0.01f, y, qz - 0.030f}, 0.006f, 0.008f, s.switch_metal);
    }
    {
        // Grips: one tall handle per lever, canted inboard, its top crowded
        // with HOTAS switches.
        const float gx = 0.5f * (qx0 + qx1) + 0.040f;
        for (int i = 0; i < s.throttles; ++i) {
            const float y = qy + (s.throttles > 1 ? -0.019f + 0.038f * static_cast<float>(i) : 0.0f);
            const float cant = 0.025f; // inboard (+y on the left console)
            const P3 bot{gx - 0.012f, y, qz - 0.026f}, top{gx + 0.006f, y + cant, qz - 0.118f};
            b.beam(bot, top, 0.0165f, 0.020f, s.dark);
            // Switch caps on the top and the inboard face.
            b.box(top[0] + 0.006f, top[1], top[2] - 0.004f, 0.007f, 0.007f, 0.004f, i == 0 ? s.key : s.switch_metal);
            b.box(top[0] - 0.010f, top[1] + 0.008f, top[2] + 0.004f, 0.004f, 0.004f, 0.003f, i == 0 ? s.guard_red : s.handle_yellow);
            b.box(0.5f * (bot[0] + top[0]) + 0.019f, 0.5f * (bot[1] + top[1]), 0.5f * (bot[2] + top[2]) - 0.02f, 0.003f, 0.007f, 0.010f, s.key);
        }
        b.box(gx - 0.002f, qy - 0.037f, qz - 0.075f, 0.012f, 0.0025f, 0.006f, s.key); // speed brake slider, outboard
    }
    b.box(qx0 - 0.07f, -0.30f, console_top(qx0 - 0.07f) - 0.012f, 0.025f, 0.03f, 0.01f, s.guard_red); // jettison guard

    // Control stick: on the right console (side-stick, forearm on the
    // armrest) or between the knees, rising from its boot on the floor.
    if (s.side_stick) {
        const float sx = pan_x0 + 0.30f, sy = 0.305f, sz = console_top(sx) - 0.004f;
        b.box(sx - 0.14f, sy, sz - 0.03f, 0.10f, 0.045f, 0.012f, s.cushion); // armrest
        b.box(sx, sy, sz - 0.010f, 0.040f, 0.040f, 0.010f, s.dark);
        b.beam({sx, sy, sz - 0.02f}, {sx + 0.012f, sy, sz - 0.135f}, 0.020f, 0.024f, s.dark);
        b.box(sx + 0.018f, sy, sz - 0.145f, 0.012f, 0.014f, 0.008f, s.key); // hat / pickle
    } else {
        // Centre stick: boot on the floor, column, then a contoured grip
        // canted back toward the pilot with its trigger, weapon release,
        // trim hat and the paddle switch on its front.
        const float sx = pan_x1 + 0.06f;
        b.tube({sx + 0.01f, 0.0f, s.floor_z}, {sx + 0.01f, 0.0f, s.floor_z - 0.08f}, 0.045f, 10, s.dark); // boot
        b.tube({sx + 0.01f, 0.0f, s.floor_z - 0.08f}, {sx - 0.005f, 0.0f, -0.115f}, 0.014f, 8, s.dark);
        const P3 g0{sx - 0.005f, 0.0f, -0.115f}, g1{sx - 0.030f, 0.0f, -0.255f};
        b.beam(g0, g1, 0.019f, 0.024f, s.dark);
        b.box(g1[0] - 0.004f, 0.0f, g1[2] - 0.008f, 0.024f, 0.021f, 0.010f, s.dark);           // grip head
        b.box(g1[0] - 0.012f, -0.010f, g1[2] - 0.019f, 0.007f, 0.007f, 0.003f, s.guard_red);   // weapon release
        b.box(g1[0] + 0.004f, 0.008f, g1[2] - 0.019f, 0.006f, 0.006f, 0.003f, s.switch_metal); // trim hat
        b.box(sx + 0.012f, 0.0f, -0.205f, 0.005f, 0.008f, 0.014f, s.switch_metal);            // trigger
        b.box(sx + 0.010f, 0.0f, -0.135f, 0.004f, 0.020f, 0.006f, s.switch_metal);            // paddle
    }
}

/// @brief HUD: projector body under the combiner, combiner posts and glass.
/// The defaults put the glass 0.67-0.73 m ahead of the eye, spanning about
/// 1.5 degrees below the horizon to 16 above and 9-10 degrees either side,
/// like the F-16C's: the symbology's field fits inside it.
struct HudSpec {
    float x_base = 2.47f, z_base = -0.634f; ///< Combiner lower edge
    float x_top = 2.53f, z_top = -0.86f;    ///< Combiner upper edge
    float half_w = 0.12f;
    float frame = 0.006f;                   ///< Post half-thickness
    bool top_bar = false;                   ///< Frame across the top (older HUDs)
    /// Side posts up to the combiner's top. Without them (a wide-angle
    /// single combiner on minimal structure) the glass is held at its foot
    /// only, by a slim rail and two brackets.
    bool posts = true;
    float half_w_top = 0.0f;                ///< Upper edge half-width (0: same as half_w)
    float corner_r = 0.0f;                  ///< Radius of the upper corners [m]
    float body_depth = 0.07f;               ///< Projector housing half-length
    float body_half_w = 0.0f;               ///< Housing half-width (0: 80% of the glass)
    float tint_alpha = 0.15f;               ///< Glass tint opacity
    /// Up-front face on the housing: symbol and video brightness rotaries,
    /// mode keys and the HUD camera window.
    bool controls = false;
    float body_bottom = 0.0f;               ///< Housing's lower face Z (0: just under the panel's top edge)
    Rgb body{0.016f, 0.016f, 0.018f};       ///< Housing and mount
};

inline void hud(Builder& b, CockpitLayout& lay, const TubSpec& t, const HudSpec& h) {
    const Rgb& body = h.body;
    // Projector housing between the glare shield and the combiner. Its top
    // stays below the combiner's lower edge, which sits just under the eye
    // line: anything higher would mask the HUD symbols on the horizon.
    const float top = h.z_base + 0.004f;
    const float bottom = h.body_bottom != 0.0f ? h.body_bottom : t.panel.z_top - 0.03f;
    const float body_hw = h.body_half_w > 0.0f ? h.body_half_w : h.half_w * 0.80f;
    b.box(h.x_base + h.body_depth - 0.01f, 0.0f, 0.5f * (top + bottom), h.body_depth, body_hw, 0.5f * (bottom - top), body);
    if (h.controls) {
        // The face plate, inset from the housing's edges so its outline
        // catches the light, then its controls standing proud of it.
        const float xf = h.x_base - 0.0115f;
        const float zt = top + 0.008f, zb = bottom - 0.006f;
        const Rgb plate{body.r * 1.35f, body.g * 1.35f, body.b * 1.35f};
        b.quad({xf, -body_hw + 0.008f, zt}, {xf, body_hw - 0.008f, zt}, {xf, body_hw - 0.008f, zb},
               {xf, -body_hw + 0.008f, zb}, plate);
        const float zm = 0.5f * (zt + zb);
        // Symbol (left) and video (right) brightness: a ribbed skirt, the
        // knob, and a white index line across its face.
        for (float side : {-1.0f, 1.0f}) {
            const float y = side * (body_hw - 0.040f);
            b.tube({xf, y, zm}, {xf - 0.003f, y, zm}, 0.0145f, 20, t.key);
            b.tube({xf - 0.003f, y, zm}, {xf - 0.016f, y, zm}, 0.0095f, 16, t.knob);
            b.box(xf - 0.0163f, y + side * 0.0035f, zm - 0.0035f, 0.0004f, 0.0009f, 0.0045f, t.legend);
            // Detent marks around the skirt, upper half.
            for (int k = -2; k <= 2; ++k) {
                const float a = 0.55f * static_cast<float>(k);
                b.box(xf - 0.0004f, y + 0.0185f * std::sin(a), zm - 0.0185f * std::cos(a), 0.0004f, 0.0008f, 0.0008f,
                      t.legend);
            }
        }
        // HUD camera window, top centre.
        const float zc = zt + 0.016f;
        b.box(xf - 0.002f, 0.0f, zc, 0.002f, 0.024f, 0.010f, t.dark);
        b.tube({xf - 0.002f, 0.0f, zc}, {xf - 0.009f, 0.0f, zc}, 0.0065f, 16, t.knob);
        b.tube({xf - 0.009f, 0.0f, zc}, {xf - 0.0095f, 0.0f, zc}, 0.0045f, 16, {0.05f, 0.07f, 0.10f});
        // Mode keys along the bottom, each with its legend above it.
        for (int k = 0; k < 4; ++k) {
            const float y = -0.063f + 0.042f * static_cast<float>(k);
            b.box(xf - 0.004f, y, zb - 0.014f, 0.004f, 0.014f, 0.0075f, t.key);
            legend_words(y - 0.010f, y + 0.010f,
                         [&](float c, float hl) { b.box(xf - 0.0002f, c, zb - 0.030f, 0.0004f, hl, 0.0011f, t.legend); });
        }
    }
    const float hw_top = h.half_w_top > 0.0f ? h.half_w_top : h.half_w;
    if (h.posts) {
        for (float side : {-1.0f, 1.0f}) {
            b.beam({h.x_base + 0.01f, side * (h.half_w + h.frame), h.z_base + 0.01f},
                   {h.x_top, side * (hw_top + h.frame), h.z_top}, h.frame, h.frame, body);
        }
    } else {
        // Foot rail along the glass's lower edge and a bracket at each end.
        b.beam({h.x_base, -h.half_w, h.z_base + 0.004f}, {h.x_base, h.half_w, h.z_base + 0.004f}, h.frame, h.frame, body);
        for (float side : {-1.0f, 1.0f}) {
            b.beam({h.x_base + 0.03f, side * (h.half_w - 0.02f), top + 0.006f}, {h.x_base, side * (h.half_w - 0.005f), h.z_base},
                   0.006f, 0.008f, body);
        }
    }
    if (h.top_bar) {
        b.beam({h.x_top, -hw_top - h.frame, h.z_top - h.frame}, {h.x_top, hw_top + h.frame, h.z_top - h.frame},
               h.frame, h.frame, body);
    }

    // The glass, faintly tinted, normal back toward the pilot. Its outline
    // in the combiner's plane: across (y) and up the glass (s, 0 at the foot).
    const float tint[4] = {0.10f, 0.35f, 0.30f, h.tint_alpha};
    const float n[3] = {-0.96f, 0.0f, 0.26f};
    const float dx = h.x_top - h.x_base, dz = h.z_top - h.z_base;
    const float len = std::sqrt(dx * dx + dz * dz);
    auto at = [&](float y, float s) {
        const float f = s / len;
        return P3{h.x_base + dx * f, y, h.z_base + dz * f};
    };
    std::vector<std::array<float, 2>> outline;
    outline.push_back({-h.half_w, 0.0f});
    outline.push_back({h.half_w, 0.0f});
    const float r = std::min(h.corner_r, 0.5f * std::min(hw_top, len));
    if (r > 0.0f) {
        constexpr int kArc = 8;
        for (int side : {1, -1}) {
            for (int i = 0; i <= kArc; ++i) {
                const float a = 1.5707963f * static_cast<float>(side > 0 ? i : kArc - i) / kArc;
                const float cy = static_cast<float>(side) * (hw_top - r), cs = len - r;
                outline.push_back({cy + static_cast<float>(side) * r * std::cos(a), cs + r * std::sin(a)});
            }
        }
    } else {
        outline.push_back({hw_top, len});
        outline.push_back({-hw_top, len});
    }
    auto emit = [&](const P3& p) {
        lay.combiner.push_back(Vertex3D{{p[0], p[1], p[2]}, {n[0], n[1], n[2]}, {0.0f, 0.0f},
                                        {tint[0], tint[1], tint[2], tint[3]}});
    };
    // Fan from the first corner: the outline is convex.
    for (size_t i = 1; i + 1 < outline.size(); ++i) {
        emit(at(outline[0][0], outline[0][1]));
        emit(at(outline[i][0], outline[i][1]));
        emit(at(outline[i + 1][0], outline[i + 1][1]));
    }
}

/// @brief A canopy frame across the canopy at @p dx from the eye, just
/// inside the glass, from sill to sill.
inline void canopy_hoop(Builder& b, const CanopyFit& fit, float dx, float thickness, const Rgb& col,
                        float max_angle = 1.62f) {
    constexpr int SEG = 14;
    // Where the canopy slopes (the aft fairing) a square-section frame's
    // fore and aft faces reach toward the glass: inset it by the rise over
    // its half-thickness too.
    const float slope = std::fabs(canopy_station(fit, dx + 0.01f).crown_dz - canopy_station(fit, dx - 0.01f).crown_dz) / 0.02f;
    const float inset = thickness * (1.0f + slope) + 0.004f;
    for (int i = 0; i < SEG; ++i) {
        const float a0 = -max_angle + 2.0f * max_angle * static_cast<float>(i) / SEG;
        const float a1 = -max_angle + 2.0f * max_angle * static_cast<float>(i + 1) / SEG;
        // Each segment overlaps its neighbours so the joints close.
        auto p0 = canopy_point(fit, EYE_X, EYE_Z, dx, a0, inset);
        auto p1 = canopy_point(fit, EYE_X, EYE_Z, dx, a1, inset);
        float d[3] = {p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]};
        const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const float ext = (thickness * 0.6f + 0.002f) / std::max(len, 1e-6f);
        for (int k = 0; k < 3; ++k) {
            p0[static_cast<size_t>(k)] -= d[k] * ext;
            p1[static_cast<size_t>(k)] += d[k] * ext;
        }
        b.beam(p0, p1, thickness, thickness, col);
    }
}

/// @brief Windscreen side frames from the coaming up to the arch, lying on the glass.
inline void windscreen_frames(Builder& b, const CanopyFit& fit, float angle, float thickness, const Rgb& col) {
    constexpr int SEG = 6;
    for (float side : {-1.0f, 1.0f}) {
        for (int i = 0; i < SEG; ++i) {
            const float t0 = static_cast<float>(i) / SEG, t1 = static_cast<float>(i + 1) / SEG;
            const float dx0 = fit.ws_dx + t0 * (fit.bow_dx - fit.ws_dx);
            const float dx1 = fit.ws_dx + t1 * (fit.bow_dx - fit.ws_dx);
            // The windscreen curves under a square-section frame: inset it
            // by more than its half-thickness so its corners stay inside.
            b.beam(canopy_point(fit, EYE_X, EYE_Z, dx0, side * angle, thickness * 1.6f + 0.004f),
                   canopy_point(fit, EYE_X, EYE_Z, dx1, side * angle, thickness * 1.6f + 0.004f), thickness, thickness, col);
        }
    }
}

// ---------------------------------------------------------------------------
// The airframes
// ---------------------------------------------------------------------------

/**
 * F-15EX Eagle II (front seat). Dominated by the 10x19 in Large Area
 * Display, split into two windows; the up-front controller sits under the
 * HUD with its own small display. Standby flight display and threat
 * display upper left, fuel gauge and clock upper right. Centre stick, two
 * throttles. Framed windscreen and a canopy frame behind the seat.
 */
inline CockpitLayout f15ex() {
    CockpitLayout lay;
    Builder b(lay.shell);
    const CanopyFit fit = airframe_info(aircraft::AircraftType::F15EX_EAGLE_II).canopy;
    TubSpec t;
    t.half_w = 0.45f;
    t.seat_recline = 0.30f;
    tub(b, t, fit);
    const Plane& pl = t.panel;

    // Large Area Display: one glass, two windows side by side.
    b.panel_box(pl, 0.0f, -0.355f, 0.240f, 0.125f, 0.004f, t.rail);
    b.panel_box(pl, 0.0f, -0.355f, 0.218f, 0.106f, 0.0055f, t.dark);
    lay.screens.push_back(panel_screen(pl, -0.107f, -0.355f, 0.102f, 0.102f, LEFT_PAGE));
    lay.screens.push_back(panel_screen(pl, 0.107f, -0.355f, 0.102f, 0.102f, RIGHT_PAGE));
    // Bezel keys along the bottom edge.
    b.key_row(pl, -0.21f, 0.21f, 12, -0.240f, 0.008f, 0.005f, t.key);

    // Up-front controller with its display, keys either side.
    b.panel_box(pl, 0.0f, -0.502f, 0.135f, 0.020f, 0.004f, t.rail);
    lay.gauges.push_back(panel_gauge(pl, Gauge::DED, 0.0f, -0.502f, 0.050f, 0.0172f));
    for (float y : {-0.11f, -0.08f, 0.08f, 0.11f}) b.key_column(pl, y, -0.510f, -0.494f, 2, 0.012f, 0.006f, t.key);

    // Upper left: threat display (TEWS), standby flight display; gear below.
    dial(b, lay, pl, Gauge::RWR, -0.350f, -0.470f, 0.036f, t.rail);
    dial(b, lay, pl, Gauge::ADI, -0.350f, -0.385f, 0.034f, t.rail);
    dial(b, lay, pl, Gauge::ASI, -0.412f, -0.310f, 0.026f, t.rail);
    dial(b, lay, pl, Gauge::ALT, -0.350f, -0.310f, 0.026f, t.rail);
    b.panel_box(pl, -0.375f, -0.250f, 0.030f, 0.018f, 0.004f, t.rail);
    b.panel_box(pl, -0.375f, -0.250f, 0.010f, 0.012f, 0.016f, t.key); // gear handle
    for (int i = 0; i < 3; ++i) {
        lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.330f + 0.018f * static_cast<float>(i), -0.250f, 0.007f, 0.007f));
    }

    // Upper right: fuel quantity, clock, master caution lights.
    dial(b, lay, pl, Gauge::FUEL_QTY, 0.350f, -0.460f, 0.040f, t.rail);
    dial(b, lay, pl, Gauge::CLOCK, 0.350f, -0.370f, 0.028f, t.rail);
    const Gauge warn[5] = {Gauge::WARN_1, Gauge::WARN_2, Gauge::WARN_3, Gauge::WARN_4, Gauge::WARN_5};
    for (int i = 0; i < 5; ++i) {
        const float z = -0.500f + 0.033f * static_cast<float>(i);
        b.panel_box(pl, 0.420f, z, 0.016f, 0.014f, 0.003f, t.rail);
        lay.gauges.push_back(panel_gauge(pl, warn[i], 0.420f, z, 0.013f, 0.011f));
    }

    // Combiner set back under the windscreen (its top stays inside the glass).
    HudSpec h;
    h.x_base = 2.43f; h.x_top = 2.48f;
    h.half_w = 0.125f;
    hud(b, lay, t, h);

    // Framed windscreen: side bows on the glass, the arch, and the canopy's
    // aft frame behind the seat.
    const Rgb frame{0.030f, 0.031f, 0.034f};
    windscreen_frames(b, fit, 1.05f, 0.012f, frame);
    canopy_hoop(b, fit, fit.bow_dx, 0.014f, frame);
    canopy_hoop(b, fit, fit.full_dx - 0.30f, 0.016f, frame);
    return lay;
}

/**
 * Eurofighter Typhoon. Three 6.25 in full-colour multifunction head-down
 * displays in a row, each with five soft keys a side. Either side of the
 * wide-angle HUD the glareshield carries a raised panel: on the left the
 * Manual Data Entry Facility (subsystem keys, its display and the keypad,
 * under the master caution); on the right the Dedicated Warnings Panel
 * (master warning and the caption block), with the LED Get-U-Home
 * reversionary display beneath it. Left forward panel: jettison switches,
 * emergency gear selector, gear lever with its annunciators, landing and
 * taxi light selector. Centre stick, two throttles. One-piece windscreen
 * with no side posts, a heavy arch carrying two mirrors, and a single
 * canopy transparency framed only at its rear.
 */
inline CockpitLayout typhoon() {
    CockpitLayout lay;
    Builder b(lay.shell);
    const CanopyFit fit = airframe_info(aircraft::AircraftType::EUROFIGHTER_TYPHOON).canopy;
    TubSpec t;
    t.half_w = 0.43f;
    t.seat_recline = 0.30f;
    // Dark grey satin panels, the anti-glare a shade darker, so the
    // shapes still read in the shade under the canopy.
    t.glare = {0.020f, 0.021f, 0.023f};
    t.panel_col = {0.040f, 0.042f, 0.046f};
    t.console = {0.036f, 0.038f, 0.041f};
    t.rail = {0.062f, 0.064f, 0.068f};
    t.key = {0.080f, 0.082f, 0.086f};
    t.dark = {0.016f, 0.016f, 0.018f};
    t.knob = {0.018f, 0.018f, 0.020f};
    t.seat = {0.022f, 0.023f, 0.022f};     // Mk16A: dark grey structure
    t.cushion = {0.034f, 0.036f, 0.027f};  // olive-drab cushions
    tub(b, t, fit);
    const Plane& pl = t.panel;

    // The three 6.25 in MHDDs, five soft keys along each side of each.
    const float mz = -0.365f, mh = 0.0794f;
    const AtlasCell pages[3] = {LEFT_PAGE, CENTRE_PAGE, RIGHT_PAGE};
    for (int i = 0; i < 3; ++i) {
        const float y = 0.212f * static_cast<float>(i - 1);
        display_bezel(b, pl, y, mz, mh, mh, t.rail, t.dark, t.key, 5, 5, 0.64f);
        lay.screens.push_back(panel_screen(pl, y, mz, mh, mh, pages[i]));
    }

    // Glareshield panels either side of the HUD: a raised box whose aft
    // face stands nearly upright, its top a little below the eye line and
    // stepping down outboard to stay clear of the windscreen.
    const Plane pod{2.452f, -0.600f, 2.466f, -0.474f};
    for (float side : {-1.0f, 1.0f}) {
        const float y_in = side * 0.185f, y_out = side * 0.425f;
        const float z_in = -0.600f, z_out = -0.585f;
        const float x_in = pod.x(z_in, 0.0f), x_out = pod.x(z_out, 0.0f);
        const float x_fwd = 2.535f;
        b.quad({x_in, y_in, z_in}, {x_out, y_out, z_out}, {pod.x_bot, y_out, pod.z_bot}, {pod.x_bot, y_in, pod.z_bot}, t.glare);
        b.quad({x_in, y_in, z_in}, {x_fwd, y_in, z_in + 0.012f}, {x_fwd, y_out, z_out + 0.012f}, {x_out, y_out, z_out}, t.glare);
        b.quad({x_fwd, y_in, z_in + 0.012f}, {x_fwd, y_out, z_out + 0.012f}, {x_fwd + 0.03f, y_out, -0.520f},
               {x_fwd + 0.03f, y_in, -0.520f}, t.glare);
        for (float y : {y_in, y_out}) {
            const float zt = y == y_in ? z_in : z_out;
            b.quad({pod.x(zt, 0.0f), y, zt}, {x_fwd, y, zt + 0.012f}, {x_fwd + 0.03f, y, -0.520f}, {pod.x_bot, y, pod.z_bot}, t.glare);
        }
    }
    // MDEF, left: its face fills the panel.
    b.panel_box(pod, -0.305f, -0.533f, 0.111f, 0.057f, 0.003f, t.rail);
    lay.gauges.push_back(panel_gauge(pod, Gauge::MDEF, -0.305f, -0.533f, 0.107f, 0.0535f));
    // DWP, right.
    b.panel_box(pod, 0.305f, -0.533f, 0.111f, 0.057f, 0.003f, t.rail);
    lay.gauges.push_back(panel_gauge(pod, Gauge::DWP, 0.305f, -0.533f, 0.107f, 0.0535f));

    // Get-U-Home reversionary display under the right glareshield panel.
    b.panel_box(pl, 0.376f, -0.400f, 0.047f, 0.047f, 0.004f, t.rail);
    b.panel_box(pl, 0.376f, -0.400f, 0.0415f, 0.0415f, 0.0055f, t.dark);
    lay.gauges.push_back(panel_gauge(pl, Gauge::GUH, 0.376f, -0.400f, 0.040f, 0.040f));
    // Below it: wheel brake / hook switches and a guarded switch.
    b.panel_box(pl, 0.376f, -0.300f, 0.045f, 0.030f, 0.003f, t.panel_col);
    panel_legend(b, pl, 0.376f, -0.318f, 0.036f, t.legend);
    for (float y : {0.356f, 0.396f}) {
        b.panel_box(pl, y, -0.296f, 0.0055f, 0.0055f, 0.006f, t.switch_metal);
        b.beam({pl.x(-0.296f, 0.012f), y, -0.296f}, {pl.x(-0.296f, 0.026f) , y, -0.309f}, 0.0022f, 0.0022f, t.switch_metal);
    }
    b.panel_box(pl, 0.376f, -0.245f, 0.020f, 0.012f, 0.012f, t.guard_red);

    // Left forward panel. Jettison switches under hinged red guards at the
    // top, each on a black base with a yellow-and-black warning surround.
    b.panel_box(pl, -0.376f, -0.452f, 0.046f, 0.030f, 0.003f, t.panel_col);
    panel_legend(b, pl, -0.376f, -0.474f, 0.036f, t.legend);
    for (float y : {-0.400f, -0.352f}) {
        for (int k = 0; k < 6; ++k) { // striped surround: a ring of short bars
            const float f = -0.0125f + 0.005f * static_cast<float>(k);
            const Rgb& c = k % 2 == 0 ? t.handle_yellow : t.dark;
            b.panel_box(pl, y + f, -0.4615f, 0.0025f, 0.0015f, 0.0045f, c);
            b.panel_box(pl, y + f, -0.4345f, 0.0025f, 0.0015f, 0.0045f, c);
        }
        b.panel_box(pl, y, -0.448f, 0.012f, 0.0115f, 0.005f, t.dark);
        // The guard: a red cover hinged at its top, its lower edge raised.
        const float zt = -0.4575f, zb = -0.4385f;
        const P3 a{pl.x(zt, 0.007f), y - 0.010f, zt}, bq{pl.x(zt, 0.007f), y + 0.010f, zt};
        const P3 c{pl.x(zb, 0.016f), y + 0.010f, zb}, d{pl.x(zb, 0.016f), y - 0.010f, zb};
        b.quad(a, bq, c, d, t.guard_red);
        const Rgb red_side{t.guard_red.r * 0.6f, t.guard_red.g * 0.6f, t.guard_red.b * 0.6f};
        b.quad(d, c, {pl.x(zb, 0.005f), y + 0.010f, zb}, {pl.x(zb, 0.005f), y - 0.010f, zb}, red_side);
        b.tri(a, d, {pl.x(zb, 0.005f), y - 0.010f, zb}, red_side);
        b.tri(bq, c, {pl.x(zb, 0.005f), y + 0.010f, zb}, red_side);
    }
    // Emergency gear selector: a yellow-and-black striped T-handle on two
    // short stems over a black recess.
    b.panel_box(pl, -0.376f, -0.395f, 0.040f, 0.011f, 0.004f, t.dark);
    {
        const float zh = -0.395f;
        for (float y : {-0.400f, -0.352f}) {
            b.beam({pl.x(zh, 0.004f), y, zh}, {pl.x(zh, 0.016f), y, zh}, 0.0025f, 0.0025f, t.switch_metal);
        }
        constexpr int kStripes = 10;
        for (int k = 0; k < kStripes; ++k) {
            const float y0 = -0.408f + 0.064f * static_cast<float>(k) / kStripes;
            const float y1 = -0.408f + 0.064f * static_cast<float>(k + 1) / kStripes;
            b.tube({pl.x(zh, 0.020f), y0, zh}, {pl.x(zh, 0.020f), y1, zh}, 0.0065f, 12,
                   k % 2 == 0 ? t.handle_yellow : t.dark);
        }
    }
    // Gear lever with its wheel-shaped knob, up, and the three-greens.
    b.panel_box(pl, -0.396f, -0.320f, 0.016f, 0.045f, 0.003f, t.rail);
    b.panel_box(pl, -0.396f, -0.320f, 0.004f, 0.036f, 0.0035f, t.dark); // slot
    {
        const float zk = -0.352f;
        b.beam({pl.x(zk, 0.0f), -0.396f, zk}, {pl.x(zk, 0.05f), -0.396f, zk - 0.004f}, 0.0035f, 0.0035f, t.switch_metal);
        // Wheel-shaped knob facing the pilot: a white tyre round a darker hub.
        const Rgb tyre{0.36f, 0.36f, 0.35f};
        b.tube({pl.x(zk, 0.047f), -0.396f, zk - 0.004f}, {pl.x(zk, 0.059f), -0.396f, zk - 0.004f}, 0.0165f, 24, tyre);
        b.tube({pl.x(zk, 0.058f), -0.396f, zk - 0.004f}, {pl.x(zk, 0.0605f), -0.396f, zk - 0.004f}, 0.009f, 16,
               t.switch_metal);
    }
    b.panel_box(pl, -0.350f, -0.320f, 0.022f, 0.030f, 0.003f, t.rail);
    lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.350f, -0.338f, 0.0075f, 0.0075f));
    lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.362f, -0.310f, 0.0075f, 0.0075f));
    lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.338f, -0.310f, 0.0075f, 0.0075f));
    // Landing / taxi light selector.
    b.panel_box(pl, -0.376f, -0.250f, 0.046f, 0.024f, 0.003f, t.panel_col);
    panel_legend(b, pl, -0.376f, -0.266f, 0.036f, t.legend);
    b.panel_box(pl, -0.376f, -0.246f, 0.0055f, 0.0055f, 0.006f, t.switch_metal);
    b.beam({pl.x(-0.246f, 0.012f), -0.376f, -0.246f}, {pl.x(-0.246f, 0.028f), -0.376f, -0.240f}, 0.0024f, 0.0024f,
           t.switch_metal);

    // Wide-angle HUD: one holographic combiner on a minimal mount, as wide
    // as the windscreen allows (about 31 x 22 degrees).
    HudSpec h;
    // Its foot about 5.7 deg under the eye line so a flight path marker
    // driven down by angle of attack stays on the glass; top at ~16 deg.
    h.x_base = 2.405f; h.z_base = -0.590f;
    h.x_top = 2.445f;  h.z_top = -0.835f;
    h.half_w = 0.168f; h.half_w_top = 0.176f;
    h.corner_r = 0.035f;
    h.posts = false;
    h.frame = 0.005f;
    h.body_depth = 0.075f;
    h.body_half_w = 0.165f;
    h.tint_alpha = 0.07f; // holographic: barely tinted
    h.controls = true;
    h.body_bottom = -0.495f; // down to the centre MHDD's bezel
    h.body = {0.030f, 0.031f, 0.034f};
    hud(b, lay, t, h);

    // Windscreen arch (the windscreen itself is one piece, no side posts),
    // its two rear-view mirrors, and the canopy's rear frame.
    const Rgb frame{0.034f, 0.035f, 0.038f};
    canopy_hoop(b, fit, fit.bow_dx, 0.020f, frame);
    for (float side : {-1.0f, 1.0f}) {
        const auto m = canopy_point(fit, EYE_X, EYE_Z, fit.bow_dx - 0.035f, side * 0.42f, 0.060f);
        const auto root = canopy_point(fit, EYE_X, EYE_Z, fit.bow_dx - 0.005f, side * 0.42f, 0.030f);
        b.beam(root, m, 0.005f, 0.005f, frame);
        b.box(m[0], m[1], m[2], 0.010f, 0.050f, 0.024f, frame);
        // Mirror glass, on the housing's aft face.
        b.quad({m[0] - 0.0105f, m[1] - 0.046f, m[2] - 0.020f}, {m[0] - 0.0105f, m[1] + 0.046f, m[2] - 0.020f},
               {m[0] - 0.0105f, m[1] + 0.046f, m[2] + 0.020f}, {m[0] - 0.0105f, m[1] - 0.046f, m[2] + 0.020f},
               {0.32f, 0.38f, 0.46f});
    }
    canopy_hoop(b, fit, fit.aft_dx + 0.30f, 0.018f, frame);
    return lay;
}

/**
 * F-22A Raptor. An all-glass panel: the 8x8 in primary display in the
 * middle, 6.25 in secondary displays either side and one below it, and the
 * integrated control panel under the HUD between two up-front displays.
 * Side-stick on the right, two throttles. The canopy is frameless; only
 * its aft frame shows, over the shoulder.
 */
inline CockpitLayout f22() {
    CockpitLayout lay;
    Builder b(lay.shell);
    const CanopyFit fit = airframe_info(aircraft::AircraftType::F22_RAPTOR).canopy;
    TubSpec t;
    // A deep panel: it carries a display between the knees.
    t.panel = {2.50f, -0.53f, 2.36f, -0.10f};
    t.half_w = 0.45f;
    t.seat_recline = 0.26f;
    t.glare = {0.010f, 0.010f, 0.011f};
    t.panel_col = {0.016f, 0.016f, 0.018f};
    t.console = {0.018f, 0.018f, 0.020f};
    t.rail = {0.032f, 0.032f, 0.035f};
    t.key = {0.045f, 0.046f, 0.048f};
    t.side_stick = true;
    tub(b, t, fit);
    const Plane& pl = t.panel;

    // Primary MFD (tactical situation) and the secondary MFDs.
    display_bezel(b, pl, 0.0f, -0.355f, 0.100f, 0.100f, t.rail, t.dark, t.key, 7, 0);
    lay.screens.push_back(panel_screen(pl, 0.0f, -0.355f, 0.100f, 0.100f, CENTRE_PAGE));
    display_bezel(b, pl, -0.262f, -0.380f, 0.078f, 0.078f, t.rail, t.dark, t.key, 5, 4);
    lay.screens.push_back(panel_screen(pl, -0.262f, -0.380f, 0.078f, 0.078f, LEFT_PAGE));
    display_bezel(b, pl, 0.262f, -0.380f, 0.078f, 0.078f, t.rail, t.dark, t.key, 5, 4);
    lay.screens.push_back(panel_screen(pl, 0.262f, -0.380f, 0.078f, 0.078f, RIGHT_PAGE));
    // Lower secondary display, between the knees (navigation).
    b.panel_box(pl, 0.0f, -0.180f, 0.090f, 0.070f, 0.004f, t.rail);
    lay.gauges.push_back(panel_gauge(pl, Gauge::HSI, 0.0f, -0.180f, 0.062f, 0.062f));

    // Integrated control panel and the two up-front displays.
    b.panel_box(pl, 0.0f, -0.503f, 0.250f, 0.022f, 0.004f, t.rail);
    for (int r = 0; r < 2; ++r) {
        b.key_row(pl, -0.060f, 0.060f, 5, -0.508f + 0.018f * static_cast<float>(r), 0.008f, 0.006f, t.key);
    }
    b.panel_box(pl, -0.165f, -0.505f, 0.070f, 0.019f, 0.0055f, t.dark);
    lay.gauges.push_back(panel_gauge(pl, Gauge::DED, -0.165f, -0.505f, 0.052f, 0.0178f));
    b.panel_box(pl, 0.165f, -0.505f, 0.070f, 0.019f, 0.0055f, t.dark);
    const Gauge warn[3] = {Gauge::WARN_1, Gauge::WARN_2, Gauge::WARN_3};
    for (int i = 0; i < 3; ++i) {
        lay.gauges.push_back(panel_gauge(pl, warn[i], 0.120f + 0.045f * static_cast<float>(i), -0.505f, 0.017f, 0.014f));
    }

    // Standby flight display and gear, far left; clock far right.
    dial(b, lay, pl, Gauge::ADI, -0.405f, -0.470f, 0.030f, t.rail);
    b.panel_box(pl, -0.405f, -0.260f, 0.024f, 0.018f, 0.004f, t.rail);
    b.panel_box(pl, -0.405f, -0.260f, 0.009f, 0.011f, 0.016f, t.key);
    for (int i = 0; i < 3; ++i) {
        lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.425f + 0.020f * static_cast<float>(i), -0.290f, 0.007f, 0.007f));
    }
    dial(b, lay, pl, Gauge::CLOCK, 0.405f, -0.470f, 0.026f, t.rail);

    HudSpec h;
    h.half_w = 0.115f;
    h.z_top = -0.84f;
    hud(b, lay, t, h);

    // Frameless canopy: only its aft frame, behind the seat.
    canopy_hoop(b, fit, fit.full_dx - 0.45f, 0.020f, {0.020f, 0.020f, 0.022f});
    return lay;
}

/**
 * A-10C Thunderbolt II. Two 5x5 in multifunction colour displays either
 * side of a big centre ADI over the HSI; flight instruments outboard left;
 * the twin-engine gauge stack (fan, ITT, core, oil, fuel flow, one column
 * per engine) outboard right. Up-front controller under the old wide HUD,
 * whose glass has a heavy frame. The control display unit lies on the right
 * console. Centre stick, two throttles. Heavily framed armoured windscreen:
 * a flat front panel and two side panels.
 */
inline CockpitLayout a10c() {
    CockpitLayout lay;
    Builder b(lay.shell);
    const CanopyFit fit = airframe_info(aircraft::AircraftType::A10_THUNDERBOLT).canopy;
    TubSpec t;
    t.half_w = 0.44f;
    t.seat_recline = 0.24f;
    t.glare = {0.016f, 0.016f, 0.017f};
    t.panel_col = {0.030f, 0.031f, 0.033f};
    t.rail = {0.050f, 0.051f, 0.054f};
    tub(b, t, fit);
    const Plane& pl = t.panel;

    // The two MFCDs.
    for (float side : {-1.0f, 1.0f}) {
        display_bezel(b, pl, side * 0.215f, -0.405f, 0.064f, 0.064f, t.rail, t.dark, t.key, 5, 5);
        lay.screens.push_back(panel_screen(pl, side * 0.215f, -0.405f, 0.064f, 0.064f, side < 0.0f ? LEFT_PAGE : RIGHT_PAGE));
    }

    // Centre: the big ADI over the HSI.
    dial(b, lay, pl, Gauge::ADI, 0.0f, -0.410f, 0.060f, t.rail);
    dial(b, lay, pl, Gauge::HSI, 0.0f, -0.280f, 0.055f, t.rail);

    // Outboard left: threat warning, airspeed, altimeter, VVI and AOA.
    dial(b, lay, pl, Gauge::RWR, -0.385f, -0.485f, 0.026f, t.rail);
    dial(b, lay, pl, Gauge::ASI, -0.385f, -0.415f, 0.029f, t.rail);
    dial(b, lay, pl, Gauge::ALT, -0.385f, -0.343f, 0.029f, t.rail);
    b.panel_box(pl, -0.385f, -0.268f, 0.040f, 0.038f, 0.003f, t.rail);
    lay.gauges.push_back(panel_gauge(pl, Gauge::AOA, -0.405f, -0.268f, 0.0145f, 0.034f));
    lay.gauges.push_back(panel_gauge(pl, Gauge::VVI, -0.365f, -0.268f, 0.0145f, 0.034f));
    // Gear handle and lamps, lower left.
    b.panel_box(pl, -0.250f, -0.275f, 0.026f, 0.020f, 0.004f, t.rail);
    b.panel_box(pl, -0.250f, -0.275f, 0.010f, 0.012f, 0.016f, t.key);
    for (int i = 0; i < 3; ++i) {
        lay.gauges.push_back(panel_gauge(pl, Gauge::GEAR_LAMP, -0.270f + 0.020f * static_cast<float>(i), -0.305f, 0.007f, 0.007f));
    }

    // Outboard right: the engine instruments, left engine then right.
    const Gauge column[4] = {Gauge::RPM, Gauge::FTIT, Gauge::NOZ, Gauge::OIL};
    for (int e = 0; e < 2; ++e) {
        const float y = 0.355f + 0.052f * static_cast<float>(e);
        for (int r = 0; r < 4; ++r) dial(b, lay, pl, column[r], y, -0.490f + 0.050f * static_cast<float>(r), 0.019f, t.rail);
        b.panel_box(pl, y, -0.292f, 0.024f, 0.019f, 0.003f, t.rail);
        lay.gauges.push_back(panel_gauge(pl, Gauge::FUEL_FLOW, y, -0.292f, 0.021f, 0.016f));
    }
    // Fuel quantity, low centre right.
    dial(b, lay, pl, Gauge::FUEL_QTY, 0.215f, -0.270f, 0.034f, t.rail);

    // Up-front controller: keypad on the coaming under the HUD.
    b.panel_box(pl, 0.0f, -0.505f, 0.120f, 0.016f, 0.004f, t.rail);
    for (int r = 0; r < 2; ++r) b.key_row(pl, -0.105f, 0.105f, 9, -0.512f + 0.014f * static_cast<float>(r), 0.008f, 0.0055f, t.key);

    // Control display unit, a raised box on the right console.
    const float cz = t.console_z - 0.02f;
    b.box(2.12f, 0.31f, cz + 0.005f, 0.06f, 0.075f, 0.015f, t.rail);
    lay.gauges.push_back(CockpitGauges::screen(
        Gauge::DED, {{{2.160f, 0.258f, cz - 0.0115f}, {2.160f, 0.362f, cz - 0.0115f}, {2.125f, 0.362f, cz - 0.0115f},
                      {2.125f, 0.258f, cz - 0.0115f}}}));

    HudSpec h;
    h.x_base = 2.41f; h.x_top = 2.44f; h.z_top = -0.79f; // under the armoured front panel
    h.half_w = 0.112f;
    h.frame = 0.012f;
    h.top_bar = true;
    hud(b, lay, t, h);

    // Armoured windscreen: the flat front panel between two heavy bars,
    // side panels back to the arch, and the canopy's aft frame.
    const Rgb frame{0.034f, 0.035f, 0.038f};
    const float bar_y = 0.245f;
    for (float side : {-1.0f, 1.0f}) {
        const float a_ws = std::asin(std::clamp(bar_y / canopy_station(fit, fit.ws_dx).span, -1.0f, 1.0f));
        const float a_bow = std::asin(std::clamp(bar_y / canopy_station(fit, fit.bow_dx).span, -1.0f, 1.0f));
        b.beam(canopy_point(fit, EYE_X, EYE_Z, fit.ws_dx, side * a_ws, 0.022f),
               canopy_point(fit, EYE_X, EYE_Z, fit.bow_dx, side * a_bow, 0.022f), 0.018f, 0.018f, frame);
    }
    windscreen_frames(b, fit, 1.25f, 0.016f, frame);
    canopy_hoop(b, fit, fit.bow_dx, 0.020f, frame);
    canopy_hoop(b, fit, fit.full_dx - 0.20f, 0.022f, frame);
    return lay;
}

/**
 * F-35B Lightning II. No HUD: the flight symbology is on the Gen III
 * helmet, so the glareshield is low and bare. The panel is the 20 x 8 in
 * Panoramic Cockpit Display, one touch-screen glass split into two 10 x 8 in
 * halves with no bezel keys; below it, centred, the integrated standby
 * flight display, the gear handle to its left with the STOVL/HOOK button,
 * and the master arm and jettison to its right. Side-stick on the right
 * console, the single throttle on the left, the Martin-Baker US16E seat.
 * The one-piece canopy has no windscreen bow; its internal arch crosses
 * behind the headrest.
 */
inline CockpitLayout f35b() {
    CockpitLayout lay;
    lay.helmet_display = true;
    Builder b(lay.shell);
    const CanopyFit fit = airframe_info(aircraft::AircraftType::F35B_LIGHTNING_II).canopy;
    TubSpec t;
    t.half_w = 0.33f;
    t.seat_recline = 0.30f;
    t.side_stick = true;
    t.throttles = 1;
    // Mid grey, the anti-glare a shade darker, as in the jet.
    t.glare = {0.018f, 0.019f, 0.021f};
    t.panel_col = {0.034f, 0.036f, 0.040f};
    t.console = {0.032f, 0.034f, 0.037f};
    t.rail = {0.050f, 0.052f, 0.056f};
    t.key = {0.070f, 0.072f, 0.076f};
    t.seat = {0.020f, 0.021f, 0.022f};
    t.cushion = {0.030f, 0.031f, 0.026f};
    tub(b, t, fit);
    const Plane& pl = t.panel;

    // The PCD: one glass in a slim frame, its two halves side by side.
    constexpr float kHalfW = 0.127f, kHalfH = 0.1015f, kZ = -0.415f;
    b.panel_box(pl, 0.0f, kZ, 2.0f * kHalfW + 0.012f, kHalfH + 0.012f, 0.004f, t.rail);
    b.panel_box(pl, 0.0f, kZ, 2.0f * kHalfW + 0.002f, kHalfH + 0.002f, 0.0055f, t.dark);
    lay.screens.push_back(panel_screen(pl, -kHalfW, kZ, kHalfW, kHalfH, PCD_LEFT));
    lay.screens.push_back(panel_screen(pl, kHalfW, kZ, kHalfW, kHalfH, PCD_RIGHT));

    // Integrated standby flight display, centred under the PCD.
    b.panel_box(pl, 0.0f, -0.255f, 0.042f, 0.042f, 0.004f, t.rail);
    b.panel_box(pl, 0.0f, -0.255f, 0.0365f, 0.0365f, 0.0055f, t.dark);
    lay.gauges.push_back(panel_gauge(pl, Gauge::ISFD, 0.0f, -0.255f, 0.035f, 0.035f));

    // Gear handle (wheel-shaped knob, down) left of it, the STOVL/HOOK
    // button beside it.
    b.panel_box(pl, -0.215f, -0.262f, 0.016f, 0.042f, 0.003f, t.rail);
    b.panel_box(pl, -0.215f, -0.262f, 0.004f, 0.034f, 0.0035f, t.dark);
    {
        const float zk = -0.236f;
        b.beam({pl.x(zk, 0.0f), -0.215f, zk}, {pl.x(zk, 0.045f), -0.215f, zk - 0.004f}, 0.0035f, 0.0035f, t.switch_metal);
        const Rgb tyre{0.36f, 0.36f, 0.35f};
        b.tube({pl.x(zk, 0.042f), -0.215f, zk - 0.004f}, {pl.x(zk, 0.054f), -0.215f, zk - 0.004f}, 0.015f, 24, tyre);
        b.tube({pl.x(zk, 0.053f), -0.215f, zk - 0.004f}, {pl.x(zk, 0.0555f), -0.215f, zk - 0.004f}, 0.008f, 16,
               t.guard_red);
    }
    b.panel_box(pl, -0.140f, -0.262f, 0.026f, 0.016f, 0.004f, t.rail);
    b.panel_box(pl, -0.140f, -0.262f, 0.021f, 0.012f, 0.009f, t.key); // STOVL/HOOK
    panel_legend(b, pl, -0.140f, -0.285f, 0.020f, t.legend);
    // Master arm toggle and the emergency jettison button.
    b.panel_box(pl, 0.150f, -0.262f, 0.030f, 0.020f, 0.003f, t.panel_col);
    panel_legend(b, pl, 0.150f, -0.278f, 0.024f, t.legend);
    b.panel_box(pl, 0.150f, -0.258f, 0.0055f, 0.0055f, 0.006f, t.switch_metal);
    b.beam({pl.x(-0.258f, 0.012f), 0.150f, -0.258f}, {pl.x(-0.258f, 0.027f), 0.150f, -0.266f}, 0.0022f, 0.0022f,
           t.switch_metal);
    b.panel_box(pl, 0.220f, -0.262f, 0.020f, 0.020f, 0.004f, t.dark);
    b.panel_box(pl, 0.220f, -0.262f, 0.015f, 0.015f, 0.012f, t.guard_red);

    // The canopy's internal arch, behind the headrest.
    canopy_hoop(b, fit, fit.full_dx - 0.35f, 0.022f, {0.022f, 0.023f, 0.025f});
    return lay;
}

} // namespace cockpit_build

/// @brief The procedural cockpit for @p type. The F-16C flies from its
/// airframe asset's own cockpit and is not built here.
[[nodiscard]] inline CockpitLayout build_jet_cockpit(aircraft::AircraftType type) {
    using aircraft::AircraftType;
    switch (type) {
        case AircraftType::F15EX_EAGLE_II:      return cockpit_build::f15ex();
        case AircraftType::EUROFIGHTER_TYPHOON: return cockpit_build::typhoon();
        case AircraftType::F22_RAPTOR:          return cockpit_build::f22();
        case AircraftType::A10_THUNDERBOLT:     return cockpit_build::a10c();
        case AircraftType::F35B_LIGHTNING_II:   return cockpit_build::f35b();
        case AircraftType::F16_FIGHTING_FALCON: break;
    }
    return {};
}

} // namespace fastjet::graphics
