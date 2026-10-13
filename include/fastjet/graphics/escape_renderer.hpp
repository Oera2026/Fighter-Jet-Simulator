#pragma once

#include "fastjet/graphics/frame_context.hpp"
#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/shader_library.hpp"
#include "fastjet/math/quaternion.hpp"
#include "fastjet/math/vector3.hpp"
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace fastjet::graphics {

/// @brief One ejected pilot to draw: where he is, how he hangs, and what is
/// deployed above him.
struct EscapeView {
    math::Vector3 pilot_pos{};         ///< Harness point, NED [m]
    math::Quaternion attitude{};       ///< Body axes: X forward, Z toward the feet
    bool seat_attached = false;        ///< Still strapped to the seat
    bool drogue = false;               ///< Drogue streaming above the seat
    float chute = 0.0f;                ///< Main canopy: 0 packed, 1 fully open
    bool seat_free = false;            ///< The empty seat, falling on its own
    math::Vector3 seat_pos{};
    math::Quaternion seat_attitude{};
};

/**
 * @brief Draws ejection seats, pilots and parachutes in the world pass.
 *
 * Everything is procedural and built once: a 24-gore canopy in alternating
 * international orange and white (the high-visibility C-9 pattern), its
 * suspension lines and risers, a small ribbon drogue, the pilot in a sage
 * flight suit and grey helmet, and the seat itself with its yellow and black
 * striped handle. The canopy is stretched into a streamer while it inflates.
 * Drawn eye-relative, lit by the frame's sun and sky, with aerial haze.
 */
class EscapeRenderer {
public:
    static constexpr int kMaxViews = 8;

    bool init() {
        if (initialized_) return true;
        const char* vert = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            layout (location = 2) in vec3 aColor;
            uniform mat4 uViewProj;  // projection x view rotation (eye-relative)
            uniform mat4 uModel;     // model -> eye-relative NED
            out vec3 vRel;
            out vec3 vNormal;
            out vec3 vColor;
            void main() {
                vec4 p = uModel * vec4(aPos, 1.0);
                vRel = p.xyz;
                vNormal = mat3(uModel) * aNormal;
                vColor = aColor;
                gl_Position = uViewProj * p;
            }
        )";
        const std::string frag = std::string(R"(
            #version 330 core
            in vec3 vRel;
            in vec3 vNormal;
            in vec3 vColor;
            out vec4 FragColor;
            uniform vec3 uEyeNed;
        )") + glsl::COMMON + glsl::OUTPUT + glsl::ATMOSPHERE + glsl::LIGHTING + R"(
            void main() {
                vec3 n = normalize(vNormal);
                vec3 v = normalize(-vRel);
                // Fabric and gear are seen from both sides.
                if (dot(n, v) < 0.0) n = -n;
                vec3 lit = sun_brdf(vColor, 0.0, 0.8, n, v, uSunDir) + vColor * hemi_ambient(n);
                // Thin nylon glows with the sun behind it.
                lit += vColor * sun_radiance() * max(dot(-n, uSunDir), 0.0) * 0.25;
                vec3 world = vRel + uEyeNed;
                FragColor = scene_output(apply_haze(lit, world, uEyeNed, uSunDir), 1.0);
            }
        )";
        if (!shader_.init_from_source(vert, frag.c_str())) return false;
        build();
        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        for (Mesh* m : {&canopy_, &lines_, &drogue_, &pilot_, &seat_}) m->destroy();
        shader_.destroy();
        initialized_ = false;
    }

    void clear() noexcept { count_ = 0; }
    void add(const EscapeView& v) noexcept {
        if (count_ < kMaxViews) views_[static_cast<size_t>(count_++)] = v;
    }
    [[nodiscard]] int count() const noexcept { return count_; }

    /// @param view_proj_rot Projection times the view rotation (eye-relative drawing).
    void render(const Mat4& view_proj_rot, const FrameContext& ctx) {
        if (!initialized_ || count_ == 0) return;
        shader_.use();
        ctx.upload(shader_);
        shader_.set_mat4("uViewProj", view_proj_rot);
        shader_.set_vec3("uEyeNed", static_cast<float>(ctx.eye_ned.x), static_cast<float>(ctx.eye_ned.y),
                         static_cast<float>(ctx.eye_ned.z));
        glDisable(GL_CULL_FACE);
        for (int i = 0; i < count_; ++i) {
            const EscapeView& e = views_[static_cast<size_t>(i)];
            const Mat4 base = place(e.pilot_pos, e.attitude, ctx.eye_ned);
            draw(pilot_, base);
            if (e.seat_attached) draw(seat_, base);
            if (e.drogue) draw(drogue_, base);
            if (e.chute > 0.0f) {
                // Inflating: a long thin streamer that opens out into the dome.
                const float f = std::clamp(e.chute, 0.0f, 1.0f);
                const float radial = 0.12f + 0.88f * f * f;
                const float stretch = 1.0f + 1.6f * (1.0f - f);
                // Lines run from the risers (z = -1.6) up; scale about there.
                const Mat4 m = base * Mat4::translate(0.0f, 0.0f, -kRiserZ) *
                               Mat4::scale(radial, radial, stretch) * Mat4::translate(0.0f, 0.0f, kRiserZ);
                draw(canopy_, m);
                draw(lines_, m, GL_LINES);
            }
            if (e.seat_free) draw(seat_, place(e.seat_pos, e.seat_attitude, ctx.eye_ned));
        }
        glBindVertexArray(0);
    }

private:
    struct Vertex {
        float p[3];
        float n[3];
        float c[3];
    };
    struct Mesh {
        GLuint vao = 0, vbo = 0;
        GLsizei count = 0;
        void upload(const std::vector<Vertex>& v) {
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
            glBindVertexArray(vao);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(Vertex)), v.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(3 * sizeof(float)));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(6 * sizeof(float)));
            glEnableVertexAttribArray(2);
            glBindVertexArray(0);
            count = static_cast<GLsizei>(v.size());
        }
        void destroy() noexcept {
            if (vao) glDeleteVertexArrays(1, &vao);
            if (vbo) glDeleteBuffers(1, &vbo);
            vao = vbo = 0;
            count = 0;
        }
    };

    // Canopy geometry in the pilot's frame (Z toward the feet, so up is -Z).
    static constexpr int kGores = 24;
    static constexpr float kSkirtZ = -6.4f;  ///< Canopy skirt above the harness
    static constexpr float kApexZ = -8.9f;
    static constexpr float kRadius = 3.6f;   ///< Inflated radius at the equator
    static constexpr float kRiserZ = -1.6f;  ///< Where the lines meet the risers

    ShaderProgram shader_;
    Mesh canopy_, lines_, drogue_, pilot_, seat_;
    std::array<EscapeView, kMaxViews> views_{};
    int count_ = 0;
    bool initialized_ = false;

    void draw(const Mesh& m, const Mat4& model, GLenum mode = GL_TRIANGLES) {
        shader_.set_mat4("uModel", model);
        glBindVertexArray(m.vao);
        glDrawArrays(mode, 0, m.count);
    }

    /// @brief Model matrix: body axes at @p pos, relative to the eye.
    [[nodiscard]] static Mat4 place(const math::Vector3& pos, const math::Quaternion& q, const math::Vector3& eye) {
        const math::Matrix3x3 c = q.to_dcm_body_to_ned();
        Mat4 m = Mat4::identity();
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) m(r, k) = static_cast<float>(c(r, k));
        }
        const math::Vector3 d = pos - eye;
        m(0, 3) = static_cast<float>(d.x);
        m(1, 3) = static_cast<float>(d.y);
        m(2, 3) = static_cast<float>(d.z);
        return m;
    }

    static void tri(std::vector<Vertex>& v, const float* a, const float* b, const float* c, const float* col) {
        float u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        float w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        float n[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
        const float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (l > 0.0f) { n[0] /= l; n[1] /= l; n[2] /= l; }
        for (const float* p : {a, b, c}) v.push_back({{p[0], p[1], p[2]}, {n[0], n[1], n[2]}, {col[0], col[1], col[2]}});
    }

    /// @brief Axis-aligned box centred at (cx, cy, cz).
    static void box(std::vector<Vertex>& v, float cx, float cy, float cz, float hx, float hy, float hz, const float* col) {
        const float x0 = cx - hx, x1 = cx + hx, y0 = cy - hy, y1 = cy + hy, z0 = cz - hz, z1 = cz + hz;
        const float P[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                               {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
        const int F[6][4] = {{0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3}, {3, 7, 4, 0}};
        for (const auto& f : F) {
            tri(v, P[f[0]], P[f[1]], P[f[2]], col);
            tri(v, P[f[0]], P[f[2]], P[f[3]], col);
        }
    }

    void build() {
        // ---- Main canopy: gored dome with a vent, and its suspension lines.
        {
            std::vector<Vertex> v, lines;
            const float orange[3] = {0.80f, 0.20f, 0.03f};
            const float white[3] = {0.78f, 0.78f, 0.74f};
            const float line_col[3] = {0.55f, 0.55f, 0.50f};
            constexpr int RINGS = 7;
            auto pt = [](float az, float u, float* out) {
                // u = 0 skirt (pulled in a little), 1 apex vent edge.
                const float lat = u * 1.45f;
                const float r = kRadius * std::cos(lat) * (u < 0.15f ? 0.9f + 0.1f * u / 0.15f : 1.0f);
                out[0] = r * std::cos(az);
                out[1] = r * std::sin(az);
                out[2] = kSkirtZ + (kApexZ - kSkirtZ) * std::sin(lat) / std::sin(1.45f);
            };
            for (int g = 0; g < kGores; ++g) {
                const float a0 = 6.2831853f * static_cast<float>(g) / kGores;
                const float a1 = 6.2831853f * static_cast<float>(g + 1) / kGores;
                const float* col = (g % 2 == 0) ? orange : white;
                for (int r = 0; r < RINGS; ++r) {
                    const float u0 = static_cast<float>(r) / RINGS, u1 = static_cast<float>(r + 1) / RINGS;
                    float p00[3], p01[3], p10[3], p11[3];
                    pt(a0, u0, p00); pt(a1, u0, p01); pt(a0, u1, p10); pt(a1, u1, p11);
                    // Each gore bulges between its seams: push the panel middle out.
                    tri(v, p00, p01, p11, col);
                    tri(v, p00, p11, p10, col);
                }
                // A line from each seam on the skirt down to the riser confluence.
                float s[3];
                pt(a0, 0.0f, s);
                const float riser[3] = {0.0f, (g < kGores / 2 ? 0.18f : -0.18f), kRiserZ};
                lines.push_back({{s[0], s[1], s[2]}, {0, 0, -1}, {line_col[0], line_col[1], line_col[2]}});
                lines.push_back({{riser[0], riser[1], riser[2]}, {0, 0, -1}, {line_col[0], line_col[1], line_col[2]}});
            }
            // Risers down to the harness.
            for (float side : {-0.18f, 0.18f}) {
                lines.push_back({{0.0f, side, kRiserZ}, {0, 0, -1}, {0.15f, 0.15f, 0.12f}});
                lines.push_back({{0.0f, side * 0.8f, -0.55f}, {0, 0, -1}, {0.15f, 0.15f, 0.12f}});
            }
            canopy_.upload(v);
            lines_.upload(lines);
        }
        // ---- Drogue: a small cone streaming 4.5 m above the seat, on a bridle.
        {
            std::vector<Vertex> v;
            const float col[3] = {0.75f, 0.70f, 0.62f};
            const float dark[3] = {0.12f, 0.12f, 0.10f};
            const float apex[3] = {0.0f, 0.0f, -5.4f};
            for (int i = 0; i < 12; ++i) {
                const float a0 = 6.2831853f * static_cast<float>(i) / 12.0f;
                const float a1 = 6.2831853f * static_cast<float>(i + 1) / 12.0f;
                const float p0[3] = {0.55f * std::cos(a0), 0.55f * std::sin(a0), -4.6f};
                const float p1[3] = {0.55f * std::cos(a1), 0.55f * std::sin(a1), -4.6f};
                tri(v, p0, p1, apex, col);
            }
            box(v, 0.0f, 0.0f, -2.6f, 0.02f, 0.02f, 2.0f, dark); // bridle
            drogue_.upload(v);
        }
        // ---- Pilot: sage flight suit, harness, grey helmet, dark visor and boots.
        {
            std::vector<Vertex> v;
            const float suit[3] = {0.16f, 0.17f, 0.11f};
            const float harness[3] = {0.05f, 0.05f, 0.05f};
            const float helmet[3] = {0.55f, 0.56f, 0.55f};
            const float visor[3] = {0.02f, 0.02f, 0.03f};
            const float boot[3] = {0.03f, 0.03f, 0.03f};
            const float vest[3] = {0.10f, 0.11f, 0.07f};
            box(v, 0.0f, 0.0f, -0.32f, 0.13f, 0.20f, 0.30f, suit);      // torso
            box(v, 0.03f, 0.0f, -0.30f, 0.12f, 0.21f, 0.22f, vest);     // survival vest
            box(v, 0.0f, 0.0f, -0.05f, 0.14f, 0.21f, 0.06f, harness);   // harness and leg straps
            box(v, 0.0f, 0.0f, -0.78f, 0.13f, 0.12f, 0.13f, helmet);    // helmet
            box(v, 0.12f, 0.0f, -0.76f, 0.03f, 0.10f, 0.06f, visor);
            for (float side : {-1.0f, 1.0f}) {
                box(v, 0.0f, side * 0.25f, -0.36f, 0.06f, 0.05f, 0.26f, suit);   // arms up on the risers
                box(v, 0.04f, side * 0.10f, 0.30f, 0.08f, 0.08f, 0.32f, suit);   // legs
                box(v, 0.08f, side * 0.10f, 0.66f, 0.12f, 0.06f, 0.05f, boot);
            }
            pilot_.upload(v);
        }
        // ---- Seat: frame, seat bucket, headrest and the striped handle.
        {
            std::vector<Vertex> v;
            const float grey[3] = {0.07f, 0.075f, 0.08f};
            const float cushion[3] = {0.12f, 0.10f, 0.07f};
            const float yellow[3] = {0.75f, 0.60f, 0.05f};
            const float black[3] = {0.02f, 0.02f, 0.02f};
            box(v, -0.24f, 0.0f, -0.40f, 0.08f, 0.27f, 0.55f, grey);     // seat back and rails
            box(v, -0.20f, 0.0f, -1.00f, 0.10f, 0.20f, 0.10f, grey);     // headrest / drogue container
            box(v, 0.00f, 0.0f, 0.10f, 0.26f, 0.27f, 0.07f, grey);       // seat bucket / survival kit
            box(v, -0.14f, 0.0f, -0.40f, 0.02f, 0.22f, 0.45f, cushion);
            for (float side : {-1.0f, 1.0f}) {
                box(v, 0.18f, side * 0.24f, 0.0f, 0.04f, 0.025f, 0.04f, yellow);   // side handles
                box(v, 0.18f, side * 0.24f, 0.04f, 0.041f, 0.026f, 0.01f, black);
            }
            seat_.upload(v);
        }
    }
};

} // namespace fastjet::graphics
