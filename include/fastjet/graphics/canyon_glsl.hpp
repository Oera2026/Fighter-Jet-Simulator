#pragma once

#include "fastjet/graphics/terrain_field.hpp"
#include <cstdio>
#include <string>

namespace fastjet::graphics {

/// @brief GLSL for the canyon's geometry and rock, generated from the same
/// constants as TerrainField::Canyon so the shaders and the heightfield
/// cannot drift apart.
///
/// Needs glsl::COMMON (vnoise) and a GroundSample struct. Provides:
///  - canyon_sd(xy): (along s, signed across distance d) of a world point
///  - canyon_zone(sd, h): 0-1 weight of the mesa country
///  - canyon_rock(h, xy): banded sandstone colour at elevation h
///  - canyon_shade(g, sd, world, h, level, crisp): repaints a ground sample
/// @param material Include the rock and ground painting (needs GroundSample);
///        false gives the geometry only, for vertex shaders.
inline std::string canyon_glsl(bool material = true) {
    using C = TerrainField::Canyon;
    char consts[1024];
    std::snprintf(consts, sizeof(consts),
                  "const vec2 CY_O = vec2(%.3f, %.3f);\n"
                  "const vec2 CY_U = vec2(%.7f, %.7f);\n"
                  "const float CY_L = %.3f;\n"
                  "const float CY_A1 = %.3f, CY_K1 = %.9f;\n"
                  "const float CY_A2 = %.3f, CY_K2 = %.9f, CY_P2 = %.4f;\n"
                  "const float CY_RIVER = %.3f;\n"
                  "const float CY_FADE = %.3f;\n",
                  C::ORIGIN_X, C::ORIGIN_Y, C::AXIS_X, C::AXIS_Y, C::LENGTH, C::MEANDER_A1,
                  6.2831853 / C::MEANDER_L1, C::MEANDER_A2, 6.2831853 / C::MEANDER_L2, C::MEANDER_P2,
                  C::RIVER_HALF, C::MEANDER_FADE);
    const char* geometry = R"(
float cy_weight(float s) {
    float a = clamp((s + CY_FADE) / CY_FADE, 0.0, 1.0);
    float b = clamp((CY_L + CY_FADE - s) / CY_FADE, 0.0, 1.0);
    return a * a * (3.0 - 2.0 * a) * b * b * (3.0 - 2.0 * b);
}
float cy_centre(float s) {
    return cy_weight(s) * (CY_A1 * sin(CY_K1 * s) + CY_A2 * sin(CY_K2 * s + CY_P2));
}
vec2 canyon_sd(vec2 xy) {
    vec2 p = xy - CY_O;
    float s = dot(p, CY_U);
    float t = dot(p, vec2(-CY_U.y, CY_U.x));
    float c = cy_centre(s);
    float slope = cy_centre(s + 1.0) - c;
    return vec2(s, (t - c) * inversesqrt(1.0 + slope * slope));
}
float canyon_floor(float s) { return 120.0 - 50.0 * clamp(s / CY_L, 0.0, 1.0); }
)";
    if (!material) return std::string(consts) + geometry;
    return std::string(consts) + geometry + R"(

/// Mesa country: near the canyon and up on the tableland.
float canyon_zone(vec2 sd, float h) {
    float along = smoothstep(-3500.0, -1500.0, sd.x) * (1.0 - smoothstep(CY_L + 1500.0, CY_L + 3500.0, sd.x));
    float across = 1.0 - smoothstep(5000.0, 7000.0, abs(sd.y));
    float inside = 1.0 - smoothstep(1200.0, 2200.0, abs(sd.y));
    // The tableland by height, the canyon walls by position.
    return along * across * max(smoothstep(250.0, 500.0, h), inside);
}

/// Layered sandstone, limestone and shale, as a canyon exposes them from the
/// river up: dark basement rock, red cliffs, pale shale benches, a buff
/// cliff band under the cap rock. Bedding planes wander a few metres.
vec3 canyon_rock(float h, vec2 xy) {
    float z = h + (vnoise(xy * 0.0035) - 0.5) * 22.0 + (vnoise(xy * 0.021) - 0.5) * 6.0;
    const int N = 8;
    float tops[N] = float[N](175.0, 255.0, 300.0, 415.0, 470.0, 600.0, 690.0, 2000.0);
    vec3 cols[N] = vec3[N](
        vec3(0.110, 0.075, 0.065),  // basement schist, inner gorge
        vec3(0.230, 0.125, 0.075),  // red-brown sandstone
        vec3(0.215, 0.180, 0.130),  // pale shale bench
        vec3(0.275, 0.115, 0.060),  // red-stained limestone cliff
        vec3(0.170, 0.105, 0.075),  // shale slope
        vec3(0.300, 0.165, 0.085),  // orange sandstone
        vec3(0.300, 0.250, 0.180),  // buff cliff band
        vec3(0.250, 0.205, 0.145)); // cap rock
    vec3 c = cols[0];
    float lo = -1000.0;
    for (int i = 0; i < N; ++i) {
        c = mix(c, cols[i], smoothstep(lo - 3.0, lo + 3.0, z));
        lo = tops[i];
    }
    // Fine bedding lines within each layer.
    c *= 0.90 + 0.10 * sin(z * 0.85) + 0.04 * sin(z * 2.7);
    return c;
}

/// Repaints a ground sample in the mesa country: desert scrub on the
/// tableland, banded rock on the walls, a green riverbank and the river.
/// `crisp` (0-1) adds the detail only the per-pixel pass can resolve.
void canyon_shade(inout GroundSample g, vec2 sd, vec3 world, float h, float level, float crisp) {
    float zone = canyon_zone(sd, h);
    if (zone <= 0.0) return;
    vec2 xy = world.xy;
    float slope = 1.0 - level;
    float rock_w = smoothstep(0.10, 0.35, slope);
    float in_canyon = 1.0 - smoothstep(700.0, 1100.0, abs(sd.y));
    float bench_w = in_canyon * smoothstep(0.85, 0.6, level) * 0.6;
    // Each layer's noise is only looked up where that layer shows.
    vec3 col = vec3(0.0);
    if (rock_w < 1.0) {
        // Tableland: pale soil and slickrock with dark juniper scrub.
        float scrub_n = vnoise(xy * 0.03) * 0.6 + vnoise(xy * 0.11) * 0.4;
        vec3 soil = mix(vec3(0.150, 0.118, 0.082), vec3(0.190, 0.150, 0.102), vnoise(xy * 0.004));
        col = mix(soil, vec3(0.055, 0.070, 0.040), smoothstep(0.62, 0.78, scrub_n) * 0.75);
    }
    if (rock_w > 0.0 || bench_w > 0.0) {
        // Rock wherever the ground steepens, banded by elevation; talus and
        // benches inside the canyon take the colour of the rock above.
        vec3 rock = canyon_rock(h, xy);
        col = mix(col, rock, rock_w);
        col = mix(col, rock * 0.85 + vec3(0.02, 0.015, 0.01), bench_w);
    }
    // The floor: riverbank vegetation, sand bars, the river.
    float floor_h = canyon_floor(sd.x);
    float on_floor = (1.0 - smoothstep(floor_h + 3.0, floor_h + 9.0, h)) * in_canyon;
    if (on_floor <= 0.0) {
        g.albedo = mix(g.albedo, col, zone);
        g.roughness = mix(g.roughness, 0.85, zone);
        g.forest *= 1.0 - zone;
        return;
    }
    // Sand bars at the water's edge, then a green riparian belt, then dry
    // sage and grass across the rest of the flood plain.
    float bank_d = abs(sd.y) - CY_RIVER + (vnoise(xy * 0.05) - 0.5) * 30.0;
    vec3 sage = mix(vec3(0.075, 0.078, 0.048), vec3(0.110, 0.098, 0.064), vnoise(xy * 0.02));
    vec3 bank = mix(vec3(0.040, 0.072, 0.028), sage, smoothstep(25.0, 80.0, bank_d));
    bank = mix(vec3(0.165, 0.142, 0.100), bank, smoothstep(2.0, 9.0, bank_d));
    col = mix(col, bank, on_floor);
    float river = (1.0 - smoothstep(CY_RIVER - 2.0, CY_RIVER + 2.0, abs(sd.y))) * on_floor * crisp;

    g.albedo = mix(g.albedo, col, zone);
    g.roughness = mix(g.roughness, 0.85, zone);
    g.forest *= 1.0 - zone;
    g.albedo = mix(g.albedo, vec3(0.018, 0.028, 0.022), river);
    g.roughness = mix(g.roughness, 0.06, river);
    g.water = max(g.water, step(0.5, river));
}
)";
}

} // namespace fastjet::graphics
