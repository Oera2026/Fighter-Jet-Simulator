#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/shader_library.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace fastjet::graphics {

/// @brief Engine exhaust: hot nozzle glow and afterburner plume.
///
/// The plume is a tapered tube behind the nozzle drawn additively into the
/// HDR target. Its fragment shader treats the tube as a glowing gas column:
/// brightness follows the path length through the column (bright down the
/// middle, soft at the edges), cools from white-yellow at the nozzle to
/// orange, and carries a train of shock diamonds. Being HDR, the core blooms.
class ExhaustPlume {
public:
    /// Nozzle exit radius of the F-16's F110 [m].
    static constexpr float kNozzleRadius = 0.40f;
    /// Visible afterburner flame length at full reheat [m].
    static constexpr float kPlumeLength = 6.5f;

    ExhaustPlume() = default;
    ~ExhaustPlume() { destroy(); }
    ExhaustPlume(const ExhaustPlume&) = delete;
    ExhaustPlume& operator=(const ExhaustPlume&) = delete;

    bool init() {
        if (vao_) return true;
        if (!init_shader()) return false;
        build_mesh();
        return true;
    }

    void destroy() noexcept {
        if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
        if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
        shader_.destroy();
        vertex_count_ = 0;
    }

    /// @brief Draws the exhaust.
    /// @param view_proj_rot Projection times view rotation (eye-relative frame).
    /// @param model Nozzle frame -> eye-relative frame: +X aft along the jet,
    ///        unit length = one nozzle radius across, kPlumeLength along.
    /// @param afterburner Reheat fraction in [0, 1].
    /// @param core_glow Nozzle glow in [0, 1] from engine temperature.
    /// @param time_s Render time, for flicker.
    void render(const Mat4& view_proj_rot, const Mat4& model, float afterburner, float core_glow, float time_s) {
        if (!vao_ || (afterburner <= 0.0f && core_glow <= 0.0f)) return;
        shader_.use();
        shader_.set_int("uHdrOutput", 1);
        shader_.set_int("uMode", 0);
        shader_.set_mat4("uViewProj", view_proj_rot);
        shader_.set_mat4("uModel", model);
        shader_.set_float("uAfterburner", afterburner);
        shader_.set_float("uGlow", core_glow);
        shader_.set_float("uTime", time_s);
        draw();
    }

    /// @brief Draws a dry lift jet (F-35B in STOVL): no flame, only the
    /// faint shimmer of a fast column of air, spreading as it goes. The
    /// LiftFan's is cold and grey-blue; the 3BSM's dry core warm.
    /// @param model As render(), with the column's own radius and length.
    /// @param strength Thrust relative to the post's rating, ~[0, 1.5].
    /// @param warmth 0 cold (fan, roll posts) .. 1 hot (engine core).
    void render_jet(const Mat4& view_proj_rot, const Mat4& model, float strength, float warmth, float time_s) {
        if (!vao_ || strength <= 0.01f) return;
        shader_.use();
        shader_.set_int("uHdrOutput", 1);
        shader_.set_int("uMode", 1);
        shader_.set_mat4("uViewProj", view_proj_rot);
        shader_.set_mat4("uModel", model);
        shader_.set_float("uAfterburner", strength);
        shader_.set_float("uGlow", warmth);
        shader_.set_float("uTime", time_s);
        draw();
    }

private:
    void draw() {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_TRUE);
    }

    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLsizei vertex_count_ = 0;
    ShaderProgram shader_;

    static constexpr int kSegments = 24; ///< Around the tube
    static constexpr int kRings = 28;    ///< Along the tube

    /// Plume radius (in nozzle radii) along its normalised length.
    static float radius_at(float t) noexcept {
        // Slight expansion past the lip, then a long taper to a point.
        return (1.0f + 0.25f * std::sin(t * 3.14159265f * 0.7f)) * (1.0f - 0.85f * std::pow(t, 1.4f));
    }

    void build_mesh() {
        // Vertex: position(3) in nozzle frame (x along, y/z radial), t(1).
        std::vector<float> v;
        auto ring_point = [&](int ring, int seg) {
            const float t = static_cast<float>(ring) / kRings;
            const float a = 6.2831853f * static_cast<float>(seg) / kSegments;
            const float r = radius_at(t);
            v.insert(v.end(), {t, r * std::cos(a), r * std::sin(a), t});
        };
        for (int ring = 0; ring < kRings; ++ring) {
            for (int seg = 0; seg < kSegments; ++seg) {
                ring_point(ring, seg);
                ring_point(ring + 1, seg);
                ring_point(ring + 1, seg + 1);
                ring_point(ring, seg);
                ring_point(ring + 1, seg + 1);
                ring_point(ring, seg + 1);
            }
        }
        // Nozzle glow disc at t = 0 (fan), marked with t = -1.
        for (int seg = 0; seg < kSegments; ++seg) {
            const float a0 = 6.2831853f * static_cast<float>(seg) / kSegments;
            const float a1 = 6.2831853f * static_cast<float>(seg + 1) / kSegments;
            v.insert(v.end(), {0.0f, 0.0f, 0.0f, -1.0f});
            v.insert(v.end(), {0.0f, 0.95f * std::cos(a0), 0.95f * std::sin(a0), -1.0f});
            v.insert(v.end(), {0.0f, 0.95f * std::cos(a1), 0.95f * std::sin(a1), -1.0f});
        }
        vertex_count_ = static_cast<GLsizei>(v.size() / 4);

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    bool init_shader() {
        const char* vert = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in float aT;
            uniform mat4 uViewProj;
            uniform mat4 uModel;
            uniform int uMode;
            out float vT;
            out vec3 vPos;      // eye-relative
            out vec3 vAxisPos;  // nozzle frame
            void main() {
                vT = aT;
                vec3 q = aPos;
                // A dry jet spreads instead of tapering like a flame.
                if (uMode == 1) {
                    float r = length(q.yz);
                    q.yz = r > 1e-4 && aT >= 0.0 ? q.yz / r * (1.0 + 1.3 * aT) : vec2(0.0);
                }
                vAxisPos = q;
                vec4 p = uModel * vec4(q, 1.0);
                vPos = p.xyz;
                gl_Position = uViewProj * p;
            }
        )";
        const std::string frag = std::string(R"(
            #version 330 core
            in float vT;
            in vec3 vPos;
            in vec3 vAxisPos;
            out vec4 FragColor;
            uniform mat4  uModel;
            uniform float uAfterburner;
            uniform float uGlow;
            uniform float uTime;
            uniform int uMode;
        )") + glsl::COMMON + glsl::OUTPUT + R"(
            void main() {
                if (uMode == 1) {
                    // Dry lift jet (uAfterburner: strength, uGlow: warmth).
                    if (vT < 0.0) discard;
                    vec3 axis = normalize(mat3(uModel) * vec3(1.0, 0.0, 0.0));
                    vec3 v = normalize(-vPos);
                    vec3 radial = normalize(mat3(uModel) * vec3(0.0, vAxisPos.yz));
                    vec3 side = normalize(radial - axis * dot(radial, axis));
                    float column = pow(abs(dot(side, normalize(v - axis * dot(v, axis)))), 1.5);
                    float fade = pow(1.0 - vT, 1.3) * smoothstep(0.0, 0.06, vT);
                    float turb = 0.55 + 0.45 * vnoise(vec2(vT * 7.0 - uTime * 11.0, vAxisPos.y * 2.5 + uTime * 4.0));
                    vec3 c = mix(vec3(0.78, 0.82, 0.88), vec3(1.0, 0.82, 0.66), uGlow);
                    FragColor = scene_output(c * (0.08 + 0.03 * uGlow) * uAfterburner * column * fade * turb, 1.0);
                    return;
                }
                if (vT < 0.0) {
                    // Nozzle interior glow: turbine-face heat, dull red to orange.
                    float r = length(vAxisPos.yz);
                    // Dull red at military power (barely visible by day),
                    // bright orange in reheat.
                    vec3 c = mix(vec3(1.0, 0.22, 0.04), vec3(1.0, 0.50, 0.16), uAfterburner) * (1.0 - r * 0.6);
                    FragColor = scene_output(c * (0.12 * uGlow * uGlow + 1.8 * uAfterburner), 1.0);
                    return;
                }
                if (uAfterburner <= 0.0) discard;

                // How squarely the view looks through the column here: a
                // stand-in for path length through the glowing gas.
                vec3 axis = normalize(mat3(uModel) * vec3(1.0, 0.0, 0.0));
                vec3 v = normalize(-vPos);
                vec3 radial = normalize(mat3(uModel) * vec3(0.0, vAxisPos.yz));
                vec3 side = normalize(radial - axis * dot(radial, axis));
                float facing = abs(dot(side, normalize(v - axis * dot(v, axis))));
                float column = pow(facing, 2.2);

                float t = vT;
                float length_fade = pow(1.0 - t, 1.6) * smoothstep(0.0, 0.04, t);
                // Shock diamonds: a train of bright nodes, closer together
                // near the nozzle, fading downstream.
                float node = 0.5 + 0.5 * cos(t * 48.0 - t * t * 12.0);
                float diamonds = pow(node, 6.0) * (1.0 - smoothstep(0.1, 0.65, t));
                float flicker = 0.85 + 0.15 * vnoise(vec2(t * 9.0 - uTime * 24.0, vAxisPos.y * 3.0 + uTime * 5.0));

                // Reheat is faint against daylight: an orange core with
                // bluish shock diamonds, not a white bar.
                vec3 hot  = vec3(1.00, 0.62, 0.30);
                vec3 warm = vec3(1.00, 0.36, 0.10);
                vec3 blue = vec3(0.50, 0.55, 1.00);
                vec3 c = mix(hot, warm, smoothstep(0.05, 0.6, t));
                c = mix(c, blue, diamonds * 0.45);
                float intensity = (0.45 + 1.6 * diamonds) * column * length_fade * flicker * uAfterburner;
                FragColor = scene_output(c * intensity, 1.0);
            }
        )";
        return shader_.init_from_source(vert, frag.c_str());
    }
};

} // namespace fastjet::graphics
