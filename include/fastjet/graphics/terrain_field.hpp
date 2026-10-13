#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fastjet::graphics {

/// @brief Deterministic procedural terrain heightfield.
///
/// Fractional Brownian motion over a hash-based value-noise lattice. The field
/// is a pure function of position, so it needs no storage, is identical on every
/// machine and every run, and can be evaluated by the mesh builder and by any
/// future ground-collision query without the two drifting apart.
///
/// The airfield sits in a deliberately flat basin around the runway so that
/// takeoff and landing stay usable; relief grows with distance from the field.
///
/// East of the field a sandstone mesa is cut by a meandering canyon (see
/// Canyon), deep and wide enough to fly through: about 36 km of river gorge,
/// 700 m deep, with a narrow slot in the middle third.
class TerrainField {
public:
    // Runway occupies X = [0, 3000] m, Y = [-30, +30] m in NED. The basin is
    // centred on the midpoint and blends out over BASIN_FADE metres.
    static constexpr float FIELD_CENTER_X = 1500.0f;
    static constexpr float FIELD_CENTER_Y = 0.0f;
    static constexpr float BASIN_RADIUS   = 2600.0f; // [m] fully flat inside
    static constexpr float BASIN_FADE     = 5200.0f; // [m] blend to full relief

    // Relief shaping.
    static constexpr float BASE_FREQ   = 1.0f / 9000.0f; // [1/m] largest landform
    static constexpr float BASE_AMP    = 620.0f;         // [m] peak-to-trough scale
    static constexpr int   OCTAVES     = 5;
    static constexpr float LACUNARITY  = 2.13f; // non-integer avoids axis-aligned repeats
    static constexpr float GAIN        = 0.47f;
    // Fraction of the ridged range treated as low ground (sea-level plain).
    static constexpr float PLAIN_BIAS  = 0.45f;
    // Domain warp: bends ridge lines into the sweeping, branching ranges real
    // mountains have, instead of the lattice-aligned blobs plain fBm makes.
    static constexpr float WARP_FREQ   = 0.55f; // relative to BASE_FREQ
    static constexpr float WARP_AMP    = 0.65f; // in base-noise cells (~5.8 km)

    // The northern range: an elongated massif 22-42 km north of the field.
    static constexpr float NORTH_RANGE_X  = 32000.0f;
    static constexpr float NORTH_RANGE_Y  = -3000.0f;
    static constexpr float NORTH_RANGE_RX = 12000.0f;
    static constexpr float NORTH_RANGE_RY = 24000.0f;

    /// Highest point anywhere on the map [m]: the canyon mesa top.
    static constexpr float MAX_HEIGHT  = 900.0f;

    /**
     * @brief The canyon: a river gorge through a mesa, laid along a gently
     * meandering centreline.
     *
     * Coordinates: s runs along the canyon axis from its western mouth, t
     * across it. The centreline sits at t = centre(s). Rows of constant s are
     * straight cross-lines, so the canyon mesh can be a sheared grid in
     * (s, t) with no folds however the river bends.
     *
     * Every dimension is set by what a fighter can fly: the tightest bend has a
     * radius near 900 m (2.6 G at 300 kt, 7 G at 500 kt), the open reaches are
     * 600-900 m rim to rim, and the gorge narrows to 130 m on the floor.
     */
    struct Canyon {
        // Western mouth and axis heading (060: east-north-east from the field).
        static constexpr float ORIGIN_X = 3000.0f;
        static constexpr float ORIGIN_Y = 9000.0f;
        static constexpr float AXIS_X   = 0.5f;        // cos 60 deg
        static constexpr float AXIS_Y   = 0.8660254f;  // sin 60 deg
        static constexpr float LENGTH   = 36000.0f;    // [m] mouth to mouth

        // Meanders: two sine trains. Their summed curvature sets the tightest bend.
        static constexpr float MEANDER_A1 = 1100.0f, MEANDER_L1 = 9500.0f;
        static constexpr float MEANDER_A2 = 280.0f,  MEANDER_L2 = 4300.0f, MEANDER_P2 = 1.3f;
        /// Past each mouth the meanders straighten over this distance [m].
        static constexpr float MEANDER_FADE = 4500.0f;

        // The gorge: the slot section in the middle third.
        static constexpr float GORGE_START = 16500.0f;
        static constexpr float GORGE_END   = 21500.0f;
        static constexpr float GORGE_BLEND = 1500.0f;

        // Mesa the canyon cuts.
        static constexpr float PLATEAU_TOP  = 790.0f;  // [m] mean tableland height
        static constexpr float PLATEAU_HALF = 3900.0f; // [m] full height this far either side (before erosion)
        static constexpr float PLATEAU_FADE = 1500.0f; // [m] escarpment down to the plains
        static constexpr float MOUTH_RAMP_IN  = -600.0f; // [m] along s: the escarpment at each mouth
        static constexpr float MOUTH_RAMP_OUT = 500.0f;

        /// River width either side of the centreline [m].
        static constexpr float RIVER_HALF = 22.0f;

        /// The high-resolution mesh strip: covers everything the canyon
        /// shapes, |d| <= STRIP_HALF and s in [-STRIP_EXTEND, LENGTH + STRIP_EXTEND].
        static constexpr float STRIP_HALF   = 3000.0f;
        static constexpr float STRIP_EXTEND = 3000.0f;

        /// @brief Centreline offset across the axis at @p s [m].
        [[nodiscard]] static float centre(float s) noexcept {
            constexpr float k1 = 6.2831853f / MEANDER_L1;
            constexpr float k2 = 6.2831853f / MEANDER_L2;
            // Meanders die out beyond the mouths, so the approach valleys run straight.
            return meander_weight(s) * (MEANDER_A1 * std::sin(k1 * s) + MEANDER_A2 * std::sin(k2 * s + MEANDER_P2));
        }

        /// @brief d(centre)/ds, for the perpendicular-distance scale.
        [[nodiscard]] static float centre_slope(float s) noexcept {
            constexpr float k1 = 6.2831853f / MEANDER_L1;
            constexpr float k2 = 6.2831853f / MEANDER_L2;
            const float w = meander_weight(s);
            const float c = MEANDER_A1 * std::sin(k1 * s) + MEANDER_A2 * std::sin(k2 * s + MEANDER_P2);
            const float dc = MEANDER_A1 * k1 * std::cos(k1 * s) + MEANDER_A2 * k2 * std::cos(k2 * s + MEANDER_P2);
            return w * dc + meander_weight_slope(s) * c;
        }

        /// @brief Along (s) and across (t) coordinates of a world point.
        static void to_axis(float x, float y, float& s, float& t) noexcept {
            const float dx = x - ORIGIN_X;
            const float dy = y - ORIGIN_Y;
            s = dx * AXIS_X + dy * AXIS_Y;
            t = -dx * AXIS_Y + dy * AXIS_X;
        }

        /// @brief World point of axis coordinates (s, t).
        static void from_axis(float s, float t, float& x, float& y) noexcept {
            x = ORIGIN_X + s * AXIS_X - t * AXIS_Y;
            y = ORIGIN_Y + s * AXIS_Y + t * AXIS_X;
        }

        /// @brief Signed distance from the centreline, measured square to it [m].
        [[nodiscard]] static float distance(float s, float t) noexcept {
            const float slope = centre_slope(s);
            return (t - centre(s)) / std::sqrt(1.0f + slope * slope);
        }

        /// @brief Across offset t of the point a distance @p d from the centreline on row @p s.
        [[nodiscard]] static float across_at(float s, float d) noexcept {
            const float slope = centre_slope(s);
            return centre(s) + d * std::sqrt(1.0f + slope * slope);
        }

        /// @brief Half-width of the flat floor at @p s [m].
        [[nodiscard]] static float floor_half_width(float s) noexcept {
            float w = 150.0f + 40.0f * std::sin(s / 2900.0f);
            w = mix(w, 65.0f, gorge_weight(s));
            return w * mouth_widening(s);
        }

        /// @brief Half-width from the centreline to the rim at @p s [m].
        [[nodiscard]] static float rim_half_width(float s) noexcept {
            float wall = 300.0f + 60.0f * std::sin(s / 3700.0f + 2.0f);
            wall = mix(wall, 170.0f, gorge_weight(s));
            return floor_half_width(s) + wall * mouth_widening(s);
        }

        /// @brief River and floor elevation [m]: falls gently along the canyon.
        [[nodiscard]] static float floor_elevation(float s) noexcept {
            const float f = std::clamp(s / LENGTH, 0.0f, 1.0f);
            return 120.0f - 50.0f * f;
        }

        /// @brief Weight of the slot-gorge narrowing at @p s.
        [[nodiscard]] static float gorge_weight(float s) noexcept {
            return smoothstep(std::clamp((s - (GORGE_START - GORGE_BLEND)) / GORGE_BLEND, 0.0f, 1.0f)) *
                   (1.0f - smoothstep(std::clamp((s - GORGE_END) / GORGE_BLEND, 0.0f, 1.0f)));
        }

        /// @brief True if (x, y) is covered by the canyon's own mesh strip,
        /// with @p margin metres to spare on every side.
        [[nodiscard]] static bool in_strip(float x, float y, float margin) noexcept {
            float s, t;
            to_axis(x, y, s, t);
            if (s < -STRIP_EXTEND + margin || s > LENGTH + STRIP_EXTEND - margin) return false;
            return std::fabs(distance(s, t)) <= STRIP_HALF - margin;
        }

    private:
        friend class TerrainField;

        [[nodiscard]] static constexpr float mix(float a, float b, float t) noexcept { return a + (b - a) * t; }
        [[nodiscard]] static constexpr float smoothstep(float t) noexcept { return t * t * (3.0f - 2.0f * t); }

        /// Meanders are full inside the canyon and straighten out past the mouths.
        [[nodiscard]] static float meander_weight(float s) noexcept {
            constexpr float R = MEANDER_FADE;
            const float a = std::clamp((s + R) / R, 0.0f, 1.0f);
            const float b = std::clamp((LENGTH + R - s) / R, 0.0f, 1.0f);
            return smoothstep(a) * smoothstep(b);
        }
        [[nodiscard]] static float meander_weight_slope(float s) noexcept {
            constexpr float R = MEANDER_FADE;
            const float a = std::clamp((s + R) / R, 0.0f, 1.0f);
            const float b = std::clamp((LENGTH + R - s) / R, 0.0f, 1.0f);
            auto ds = [](float u) { return 6.0f * u * (1.0f - u); };
            const float da = (a > 0.0f && a < 1.0f) ? ds(a) / R : 0.0f;
            const float db = (b > 0.0f && b < 1.0f) ? -ds(b) / R : 0.0f;
            return da * smoothstep(b) + smoothstep(a) * db;
        }

        /// Past each mouth the canyon opens out into a broad valley.
        [[nodiscard]] static float mouth_widening(float s) noexcept {
            const float out = std::max(-s, s - LENGTH); // metres beyond the nearer mouth
            const float u = std::clamp((out + 1000.0f) / 3000.0f, 0.0f, 1.0f);
            return 1.0f + 3.0f * smoothstep(u);
        }
    };

    /// @brief Terrain elevation above mean sea level [m], as a function of
    ///        horizontal NED position. Always >= 0 near the airfield.
    [[nodiscard]] static float height(float x, float y) noexcept {
        const float basin = basin_weight(x, y);
        if (basin <= 0.0f) return 0.0f; // the airfield pad is exactly flat
        return shape_canyon(x, y, relief(x, y) * basin, basin);
    }

    /// @brief Analytic-ish surface normal in NED (points up, so z is negative).
    /// Central differences over a fixed span; cheap and stable at mesh scale.
    static void normal_ned(float x, float y, float span, float& nx, float& ny, float& nz) noexcept {
        const float hl = height(x - span, y);
        const float hr = height(x + span, y);
        const float hd = height(x, y - span);
        const float hu = height(x, y + span);

        // Gradient of elevation. NED z is down, so "up" carries a negative z.
        const float dzdx = (hr - hl) / (2.0f * span);
        const float dzdy = (hu - hd) / (2.0f * span);

        nx = -dzdx;
        ny = -dzdy;
        nz = -1.0f;

        const float inv_len = 1.0f / std::sqrt(nx * nx + ny * ny + 1.0f);
        nx *= inv_len;
        ny *= inv_len;
        nz *= inv_len;
    }

    /// @brief Weight in [0, 1] scaling relief away from the airfield basin.
    [[nodiscard]] static float basin_weight(float x, float y) noexcept {
        const float dx = x - FIELD_CENTER_X;
        const float dy = y - FIELD_CENTER_Y;
        const float dist = std::sqrt(dx * dx + dy * dy);

        if (dist <= BASIN_RADIUS) return 0.0f;
        if (dist >= BASIN_RADIUS + BASIN_FADE) return 1.0f;

        const float t = (dist - BASIN_RADIUS) / BASIN_FADE;
        return smoothstep(t);
    }

private:
    [[nodiscard]] static constexpr float smoothstep(float t) noexcept {
        return t * t * (3.0f - 2.0f * t);
    }

    [[nodiscard]] static float smoothstep(float e0, float e1, float x) noexcept {
        return smoothstep(std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f));
    }

    /// @brief Mountains, foothills and lowland before the basin is cut, [0, BASE_AMP].
    ///
    /// Real country is not uniformly rough: ranges stand in belts tens of
    /// kilometres long with plains between them. A very low-frequency mask
    /// places the ranges; inside them a ridged multifractal raises sharp crests
    /// over smooth, branching valleys (each octave is weighted by the one
    /// before it, so detail gathers on the ridges, not on the valley floors);
    /// outside them the ground rolls gently, with flat plains for farmland and
    /// lakes.
    [[nodiscard]] static float relief(float x, float y) noexcept {
        float px = x * BASE_FREQ;
        float py = y * BASE_FREQ;

        // Domain warp: bends ranges and ridge lines into sweeping arcs.
        const float wx = fbm(px * WARP_FREQ + 3.1f, py * WARP_FREQ + 8.7f, 2);
        const float wy = fbm(px * WARP_FREQ - 5.2f, py * WARP_FREQ + 1.9f, 2);
        px += WARP_AMP * wx;
        py += WARP_AMP * wy;

        // Where the ranges are: belts about 30-60 km across, plus the
        // northern range that closes the valley the airfield sits in.
        const float belt = fbm(px * 0.21f + 17.0f, py * 0.21f - 23.0f, 2);
        const float nx = (x - NORTH_RANGE_X) / NORTH_RANGE_RX;
        const float ny = (y - NORTH_RANGE_Y) / NORTH_RANGE_RY;
        const float north = 1.0f - smoothstep(0.55f, 1.0f, std::sqrt(nx * nx + ny * ny));
        const float range = std::max(smoothstep(-0.30f, 0.10f, belt), north);

        // Ridged multifractal (Musgrave).
        float sum = 0.0f;
        float weight = 1.0f;
        float amp = 1.0f;
        float norm = 0.0f;
        // Each octave is turned 34 deg against the last: ridges traced on a
        // value-noise lattice otherwise run along its axes in boxy grids.
        constexpr float ROT_C = 0.8290376f, ROT_S = 0.5591929f;
        float qx = px * 2.1f, qy = py * 2.1f;
        for (int i = 0; i < OCTAVES; ++i) {
            float signal = 0.92f - std::fabs(value_noise(qx, qy));
            signal *= signal;
            signal *= weight;
            weight = std::clamp(signal * 1.9f, 0.0f, 1.0f);
            sum += signal * amp;
            norm += amp;
            amp *= GAIN;
            const float rx = ROT_C * qx - ROT_S * qy;
            const float ry = ROT_S * qx + ROT_C * qy;
            qx = rx * LACUNARITY + 71.3f;
            qy = ry * LACUNARITY + ula_offset(i);
        }
        // Normalised so only the highest crests approach the cap; squaring
        // keeps the valley floors low and broad.
        const float r = std::clamp(sum / norm / 0.62f, 0.0f, 1.0f);
        const float ridges = r * (0.35f + 0.65f * r);
        // Foothills step the ranges down into the plain.
        const float mountains = ridges * (0.30f + 0.70f * range) * range;

        // Lowland: gentle swells with flat plains in the hollows.
        const float swell = fbm(px * 1.4f - 9.0f, py * 1.4f + 3.0f, 3);
        const float hills = std::max(0.0f, swell - PLAIN_BIAS * 0.2f) * 0.11f;

        return std::min(1.0f, mountains + hills * (1.0f - range)) * BASE_AMP;
    }

    /// @brief Lays the mesa and cuts the canyon into the base relief.
    [[nodiscard]] static float shape_canyon(float x, float y, float base, float basin) noexcept {
        using C = Canyon;
        float s, t;
        C::to_axis(x, y, s, t);
        // Nothing the canyon shapes reaches this far (mesa edge, its erosion
        // noise and the meander offset included): skip the work.
        constexpr float REACH_T = C::PLATEAU_HALF + C::PLATEAU_FADE + 0.5f * (C::MEANDER_A1 + C::MEANDER_A2) + 2300.0f;
        if (s < -9000.0f || s > C::LENGTH + 9000.0f || std::fabs(t) > REACH_T) return base;
        const float d = C::distance(s, t);
        const float ad = std::fabs(d);

        const float floor_h = C::floor_elevation(s);

        // Approach valleys: past each mouth the plain is levelled toward the
        // river, so the canyon opens onto low ground rather than into a ridge.
        const float beyond = std::max(-s, s - C::LENGTH);
        const float approach = (1.0f - smoothstep(1200.0f, 3800.0f, ad)) * (1.0f - smoothstep(3500.0f, 7500.0f, beyond)) *
                               smoothstep(-2500.0f, 0.0f, beyond) * basin;
        float h = base + (floor_h + 8.0f * value_noise(x / 900.0f, y / 900.0f) - base) * approach;

        // The mesa: a tableland with gentle swells, ending in escarpments
        // either side and at each mouth. Its edges are measured from the
        // straight axis (not the winding river) and eaten back by erosion,
        // so they wander like a real mesa rim instead of echoing the bends.
        const float edge_noise = 650.0f * fbm(s / 2300.0f + 5.0f, t / 2300.0f - 2.0f, 3);
        // The mesa itself wanders and swells along its length.
        const float sway = 650.0f * std::sin(s / 5200.0f + 1.0f);
        const float swell = 500.0f * std::sin(s / 3900.0f + (t > sway ? 2.4f : 0.3f));
        const float across = std::fabs(t - sway) - (C::MEANDER_A1 + C::MEANDER_A2) * 0.5f + edge_noise - swell;
        const float across_w = 1.0f - smoothstep(C::PLATEAU_HALF, C::PLATEAU_HALF + C::PLATEAU_FADE, across);
        const float s_ragged = s + 900.0f * fbm(t / 2600.0f + 3.0f, s / 2600.0f, 3);
        const float along_w = smoothstep(C::MOUTH_RAMP_IN, C::MOUTH_RAMP_OUT, s_ragged) *
                              (1.0f - smoothstep(C::LENGTH - C::MOUTH_RAMP_OUT, C::LENGTH - C::MOUTH_RAMP_IN, s_ragged));
        const float plateau_w = across_w * along_w;
        if (plateau_w > 0.0f) {
            float top = C::PLATEAU_TOP + 45.0f * std::sin(s / 6100.0f + 0.7f) +
                        28.0f * fbm(x / 2600.0f + 11.0f, y / 2600.0f - 4.0f, 3);
            // Side gullies draining into the canyon: ridged-noise channels,
            // deepest near the rim and dying out back on the tableland.
            const float rim = C::rim_half_width(s);
            const float near_rim = 1.0f - smoothstep(rim + 200.0f, rim + 1800.0f, ad);
            if (near_rim > 0.0f) {
                const float g = 1.0f - std::fabs(fbm(x / 1500.0f - 7.0f, y / 1500.0f + 13.0f, 2));
                const float g2 = g * g;
                top -= 210.0f * near_rim * g2 * g2 * g2;
            }
            // Escarpments are steep: a sharpened weight gives a cliff band
            // rather than a ramp.
            const float cliff = smoothstep(std::clamp(plateau_w * 1.6f - 0.3f, 0.0f, 1.0f));
            h = std::max(h, h + (top - h) * cliff);
        }

        // The canyon itself, cut down to the floor through whatever is above it.
        const float carve_w = 1.0f - smoothstep(2500.0f, 4500.0f, beyond);
        if (carve_w <= 0.0f || h <= floor_h) return std::max(h, 0.0f);
        const float w_floor = C::floor_half_width(s);
        const float w_rim = C::rim_half_width(s);
        // Ragged rims and buttresses: erosion eats the walls back unevenly.
        const float jag = 38.0f * value_noise(s / 210.0f, d / 160.0f + 31.0f) +
                          14.0f * value_noise(s / 70.0f + 7.0f, d / 55.0f);
        const float ad_j = std::max(0.0f, ad + jag * smoothstep(w_floor * 0.6f, w_floor, ad));
        if (ad_j >= w_rim) return std::max(h, 0.0f);

        float wall_h;
        if (ad_j <= w_floor) {
            // Floor: flat, with the river channel a few metres down the middle.
            const float r = ad_j / C::RIVER_HALF;
            wall_h = r < 1.0f ? -4.0f * (1.0f - r * r) : 1.5f * (ad_j - C::RIVER_HALF) / w_floor;
            wall_h = floor_h + wall_h;
        } else {
            // Walls: four sandstone cliff bands, each over a talus bench.
            const float u = (ad_j - w_floor) / (w_rim - w_floor);
            const float bands = 4.0f;
            const float b = u * bands;
            const float k = std::floor(b);
            const float f = b - k;
            const float stepped = (k + smoothstep(std::clamp(f / 0.62f, 0.0f, 1.0f))) / bands;
            const float profile = 0.25f * u + 0.75f * stepped;
            wall_h = floor_h + 1.5f + profile * (h - floor_h - 1.5f);
        }
        h = h + (wall_h - h) * carve_w;
        return std::max(h, 0.0f);
    }

    /// @brief Integer hash to [-1, 1]. Deterministic across platforms because it
    /// works entirely in uint32 with defined wrapping.
    [[nodiscard]] static float hash2(int32_t ix, int32_t iy) noexcept {
        uint32_t h = static_cast<uint32_t>(ix) * 0x8DA6B343u ^
                     static_cast<uint32_t>(iy) * 0xD8163841u;
        h ^= h >> 15;
        h *= 0x2C1B3C6Du;
        h ^= h >> 12;
        h *= 0x297A2D39u;
        h ^= h >> 15;
        // Map to [-1, 1]
        return static_cast<float>(h & 0x00FFFFFFu) / 8388608.0f - 1.0f;
    }

    /// @brief Smooth value noise on the integer lattice.
    [[nodiscard]] static float value_noise(float x, float y) noexcept {
        const float fx = std::floor(x);
        const float fy = std::floor(y);
        const auto ix = static_cast<int32_t>(fx);
        const auto iy = static_cast<int32_t>(fy);

        const float tx = smoothstep(x - fx);
        const float ty = smoothstep(y - fy);

        const float c00 = hash2(ix,     iy);
        const float c10 = hash2(ix + 1, iy);
        const float c01 = hash2(ix,     iy + 1);
        const float c11 = hash2(ix + 1, iy + 1);

        const float a = c00 + (c10 - c00) * tx;
        const float b = c01 + (c11 - c01) * tx;
        return a + (b - a) * ty;
    }

    /// @brief Fractional Brownian motion: octaves of value noise, normalised to
    /// roughly [-1, 1] regardless of octave count.
    [[nodiscard]] static float fbm(float x, float y, int octaves) noexcept {
        float sum = 0.0f;
        float amp = 1.0f;
        float norm = 0.0f;
        float px = x;
        float py = y;

        for (int i = 0; i < octaves; ++i) {
            sum += amp * value_noise(px, py);
            norm += amp;
            amp *= GAIN;
            px *= LACUNARITY;
            py *= LACUNARITY;
            // Offset each octave so lattices do not align at the origin.
            px += 71.3f;
            py += ula_offset(i);
        }

        return sum / norm;
    }

    [[nodiscard]] static constexpr float ula_offset(int i) noexcept {
        return 13.7f + static_cast<float>(i) * 29.1f;
    }
};

} // namespace fastjet::graphics
