#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/frame_context.hpp"
#include "fastjet/math/vector3.hpp"
#include "fastjet/sim/missile.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

namespace fastjet::graphics {

/// @brief Procedural 3D Combat Visual Effects:
/// - 3D In-flight missile bodies with supersonic cruciform fins
/// - Blinding rocket motor flame plume (HDR emissive core)
/// - Persistent volumetric rocket smoke ribbon trails expanding and dissipating with wind
/// - Pyrotechnic flares with intense blinding HDR glare and smoke puffs
/// - Shimmering reflective chaff dipole clouds
/// - Detonation shockwave fireballs and rolling black smoke bursts
/// - Aircraft battle damage smoke and fire streams
class CombatFxRenderer {
public:
    struct ExplosionEffect {
        math::Vector3 pos_ned{};
        float radius = 2.0f;
        float max_radius = 24.0f;
        float age = 0.0f;
        float duration = 2.2f;
        bool active = false;
    };

    struct SmokeNode {
        math::Vector3 pos_ned{};
        float radius = 0.45f;
        float age = 0.0f;
        float max_age = 8.0f;
        float density = 1.0f;
        bool is_fire = false;
    };

    struct FlareParticle {
        math::Vector3 pos_ned{};
        float intensity = 1.0f;
        float age = 0.0f;
        float max_age = 5.0f;
        bool active = false;
    };

    struct ChaffParticle {
        math::Vector3 pos_ned{};
        float age = 0.0f;
        float max_age = 12.0f;
        float radius = 3.0f;
        bool active = false;
    };

    static constexpr int kMaxExplosions = 16;
    static constexpr int kMaxFlares = 64;
    static constexpr int kMaxChaffClouds = 32;
    static constexpr int kMaxSmokeNodes = 1024;

    CombatFxRenderer() = default;
    ~CombatFxRenderer() { destroy(); }

    bool init() {
        if (initialized_) return true;

        // 1. Particle / Ribbon Shader (Camera-facing billboard quads for smoke, fireballs, flares, chaff)
        static constexpr const char* kParticleVert = R"(
            #version 330 core
            layout (location = 0) in vec3 aCenterRel;
            layout (location = 1) in vec2 aCornerOffset;
            layout (location = 2) in vec4 aColor;
            layout (location = 3) in float aSize;

            uniform mat4 uViewProj;
            uniform vec3 uCamRight;
            uniform vec3 uCamUp;

            out vec2 vUV;
            out vec4 vColor;

            void main() {
                vUV = aCornerOffset * 0.5 + 0.5;
                vColor = aColor;
                vec3 worldPos = aCenterRel + (uCamRight * aCornerOffset.x + uCamUp * aCornerOffset.y) * aSize;
                gl_Position = uViewProj * vec4(worldPos, 1.0);
            }
        )";

        static constexpr const char* kParticleFrag = R"(
            #version 330 core
            in vec2 vUV;
            in vec4 vColor;
            out vec4 FragColor;

            void main() {
                vec2 centerDist = vUV - vec2(0.5);
                float distSq = dot(centerDist, centerDist);
                if (distSq > 0.25) discard;

                // Soft circular radial falloff
                float radial = 1.0 - smoothstep(0.04, 0.25, distSq);
                vec4 col = vColor;
                col.a *= radial;
                FragColor = col;
            }
        )";

        if (!particle_shader_.init_from_source(kParticleVert, kParticleFrag)) {
            return false;
        }

        // 2. Missile 3D Mesh Shader (Lit cylindrical body and aerodynamic fins)
        static constexpr const char* kMissileVert = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            layout (location = 2) in vec3 aColor;

            uniform mat4 uViewProj;
            uniform mat4 uModel;

            out vec3 vNormal;
            out vec3 vColor;
            out vec3 vPos;

            void main() {
                vec4 p = uModel * vec4(aPos, 1.0);
                vPos = p.xyz;
                vNormal = mat3(uModel) * aNormal;
                vColor = aColor;
                gl_Position = uViewProj * p;
            }
        )";

        static constexpr const char* kMissileFrag = R"(
            #version 330 core
            in vec3 vNormal;
            in vec3 vColor;
            in vec3 vPos;

            uniform vec3 uSunDir;
            out vec4 FragColor;

            void main() {
                vec3 n = normalize(vNormal);
                float diff = max(dot(n, normalize(uSunDir)), 0.15);
                vec3 ambient = vec3(0.35, 0.4, 0.45);
                vec3 lit = vColor * (ambient + diff * vec3(1.2, 1.15, 1.0));
                FragColor = vec4(lit, 1.0);
            }
        )";

        if (!missile_shader_.init_from_source(kMissileVert, kMissileFrag)) {
            particle_shader_.destroy();
            return false;
        }

        build_missile_mesh();

        glGenVertexArrays(1, &particle_vao_);
        glGenBuffers(1, &particle_vbo_);
        glBindVertexArray(particle_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);

        // Layout: vec3 center, vec2 corner, vec4 color, float size = 10 floats per vertex
        static constexpr GLsizei kStride = 10 * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kStride, (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, kStride, (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, kStride, (void*)(5 * sizeof(float)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, kStride, (void*)(9 * sizeof(float)));

        glBindVertexArray(0);

        smoke_pool_.reserve(kMaxSmokeNodes);
        particle_buffer_.reserve(kMaxSmokeNodes * 6 * 10);

        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        if (particle_vao_) { glDeleteVertexArrays(1, &particle_vao_); particle_vao_ = 0; }
        if (particle_vbo_) { glDeleteBuffers(1, &particle_vbo_); particle_vbo_ = 0; }
        if (missile_vao_) { glDeleteVertexArrays(1, &missile_vao_); missile_vao_ = 0; }
        if (missile_vbo_) { glDeleteBuffers(1, &missile_vbo_); missile_vbo_ = 0; }
        particle_shader_.destroy();
        missile_shader_.destroy();
        initialized_ = false;
    }

    /// @brief Trigger an explosion fireball and shockwave at world NED coordinates
    void spawn_explosion(const math::Vector3& pos_ned, float max_radius = 28.0f) {
        for (auto& exp : explosions_) {
            if (!exp.active) {
                exp.pos_ned = pos_ned;
                exp.radius = 2.0f;
                exp.max_radius = max_radius;
                exp.age = 0.0f;
                exp.duration = 2.2f;
                exp.active = true;
                break;
            }
        }
        // Spawn initial burst of dense smoke puffs
        for (int i = 0; i < 8; ++i) {
            if (smoke_pool_.size() >= kMaxSmokeNodes) smoke_pool_.erase(smoke_pool_.begin());
            SmokeNode puff;
            puff.pos_ned = pos_ned + math::Vector3((i % 3 - 1) * 3.0, ((i / 3) % 3 - 1) * 3.0, (i * 0.5 - 2.0));
            puff.radius = 4.0f;
            puff.age = 0.0f;
            puff.max_age = 9.0f;
            puff.density = 1.0f;
            puff.is_fire = (i < 4);
            smoke_pool_.push_back(puff);
        }
    }

    /// @brief Drop a smoke puff behind a missile or damaged engine
    void emit_smoke(const math::Vector3& pos_ned, float radius = 0.6f, bool is_fire = false, float max_age = 7.0f) {
        if (smoke_pool_.size() >= kMaxSmokeNodes) {
            smoke_pool_.erase(smoke_pool_.begin());
        }
        SmokeNode node;
        node.pos_ned = pos_ned;
        node.radius = radius;
        node.age = 0.0f;
        node.max_age = max_age;
        node.density = 1.0f;
        node.is_fire = is_fire;
        smoke_pool_.push_back(node);
    }

    /// @brief Update particles, explosions, and smoke aging
    void update(double dt) {
        // 1. Advance explosions
        for (auto& exp : explosions_) {
            if (!exp.active) continue;
            exp.age += static_cast<float>(dt);
            const float t = exp.age / exp.duration;
            if (t >= 1.0f) {
                exp.active = false;
            } else {
                exp.radius = 2.0f + exp.max_radius * std::sin(t * 1.57079f);
            }
        }

        // 2. Advance smoke nodes
        for (auto it = smoke_pool_.begin(); it != smoke_pool_.end();) {
            it->age += static_cast<float>(dt);
            if (it->age >= it->max_age) {
                it = smoke_pool_.erase(it);
            } else {
                // Expanding radius as smoke disperses in atmosphere
                it->radius += static_cast<float>(dt * 1.8);
                // Slight upward thermal buoyancy
                it->pos_ned.z -= dt * 1.2;
                ++it;
            }
        }
    }

    /// @brief Render combat visuals: in-flight missiles, smoke ribbon trails, flares, chaff, and explosion bursts
    void render(const Mat4& proj_world_rot,
                const math::Vector3& eye_ned,
                const Mat4& view_matrix,
                const FrameContext& ctx,
                const sim::Missile* missiles,
                size_t missile_count,
                const sim::FlareDecoy* flares,
                size_t flare_count,
                const sim::ChaffCloud* chaff,
                size_t chaff_count) {
        if (!initialized_) return;

        // Camera basis vectors in world NED
        const math::Vector3 cam_right(view_matrix(0, 0), view_matrix(0, 1), view_matrix(0, 2));
        const math::Vector3 cam_up(view_matrix(1, 0), view_matrix(1, 1), view_matrix(1, 2));

        // ---------------------------------------------------------------------
        // 1. Render 3D In-Flight Missiles
        // ---------------------------------------------------------------------
        if (missiles && missile_count > 0) {
            missile_shader_.use();
            missile_shader_.set_mat4("uViewProj", proj_world_rot);
            missile_shader_.set_vec3("uSunDir", ctx.lighting.sun_dir[0], ctx.lighting.sun_dir[1], ctx.lighting.sun_dir[2]);
            glBindVertexArray(missile_vao_);

            for (size_t i = 0; i < missile_count; ++i) {
                const auto& m = missiles[i];
                if (!m.active) continue;

                // Eye-relative position
                const math::Vector3 rel = m.body.pos - eye_ned;
                const math::Vector3 fwd = m.body.vel.normalized();
                math::Vector3 up(0.0, 0.0, -1.0);
                if (std::abs(fwd.dot(up)) > 0.95) up = math::Vector3(0.0, 1.0, 0.0);
                const math::Vector3 right = fwd.cross(up).normalized();
                up = right.cross(fwd).normalized();

                // Model matrix orienting missile along its velocity vector
                Mat4 model = Mat4::identity();
                model(0, 0) = static_cast<float>(right.x); model(0, 1) = static_cast<float>(up.x); model(0, 2) = static_cast<float>(fwd.x); model(0, 3) = static_cast<float>(rel.x);
                model(1, 0) = static_cast<float>(right.y); model(1, 1) = static_cast<float>(up.y); model(1, 2) = static_cast<float>(fwd.y); model(1, 3) = static_cast<float>(rel.y);
                model(2, 0) = static_cast<float>(right.z); model(2, 1) = static_cast<float>(up.z); model(2, 2) = static_cast<float>(fwd.z); model(2, 3) = static_cast<float>(rel.z);

                missile_shader_.set_mat4("uModel", model);
                glDrawArrays(GL_TRIANGLES, 0, missile_vert_count_);

                // Continuous smoke emission while motor burns
                if (m.body.motor_burning(m.spec)) {
                    emit_smoke(m.body.pos - fwd * 1.8, 0.75f, false, 8.0f);
                }
            }
            glBindVertexArray(0);
        }

        // ---------------------------------------------------------------------
        // 2. Build Billboard Quad Vertices for Smoke, Flares, Chaff, and Explosions
        // ---------------------------------------------------------------------
        particle_buffer_.clear();

        // (a) Volumetric smoke trail nodes
        for (const auto& node : smoke_pool_) {
            const math::Vector3 rel = node.pos_ned - eye_ned;
            const float life = 1.0f - (node.age / node.max_age);
            float r, g, b, a;
            if (node.is_fire) {
                // Fiery licking flame cooling to soot
                r = 3.5f * life;
                g = 1.6f * life * life;
                b = 0.3f * life * life * life;
                a = std::clamp(life * 0.9f, 0.0f, 1.0f);
            } else {
                // Expanding white-grey smoke ribbon
                const float grey = 0.85f * (0.4f + 0.6f * life);
                r = grey; g = grey; b = grey;
                a = std::clamp(life * 0.65f, 0.0f, 0.75f);
            }
            append_billboard_quad(rel, node.radius, r, g, b, a);
        }

        // (b) Pyrotechnic Flares
        if (flares && flare_count > 0) {
            for (size_t i = 0; i < flare_count; ++i) {
                const auto& f = flares[i];
                if (!f.active) continue;
                const math::Vector3 rel = f.pos - eye_ned;
                // Blinding HDR emission: blooms heavily
                const float heat = static_cast<float>(f.intensity());
                const float r = 8.0f * heat;
                const float g = 6.5f * heat;
                const float b = 3.2f * heat;
                append_billboard_quad(rel, 1.8f + 1.2f * heat, r, g, b, 1.0f);
            }
        }

        // (c) Radar Chaff Dipole Clouds
        if (chaff && chaff_count > 0) {
            for (size_t i = 0; i < chaff_count; ++i) {
                const auto& c = chaff[i];
                if (!c.active) continue;
                const math::Vector3 rel = c.pos - eye_ned;
                const float life = std::clamp(1.0f - static_cast<float>(c.age / sim::ChaffCloud::LIFE_S), 0.0f, 1.0f);
                // Shimmering silver-white glitter cloud
                const float r = 0.85f;
                const float g = 0.90f;
                const float b = 0.98f;
                const float a = life * 0.35f;
                append_billboard_quad(rel, 5.0f + 12.0f * (1.0f - life), r, g, b, a);
            }
        }

        // (d) Active Detonation Fireballs
        for (const auto& exp : explosions_) {
            if (!exp.active) continue;
            const math::Vector3 rel = exp.pos_ned - eye_ned;
            const float t = exp.age / exp.duration;
            const float life = 1.0f - t;
            // Hot inner plasma burst transitioning into sooty rolling fire
            const float r = 12.0f * life * life;
            const float g = 5.0f * life * life * life;
            const float b = 1.5f * life * life * life * life;
            const float a = std::clamp(life * 1.5f, 0.0f, 1.0f);
            append_billboard_quad(rel, exp.radius, r, g, b, a);
        }

        // ---------------------------------------------------------------------
        // 3. Render Particles with Depth-Tested Blending
        // ---------------------------------------------------------------------
        if (!particle_buffer_.empty()) {
            particle_shader_.use();
            particle_shader_.set_mat4("uViewProj", proj_world_rot);
            particle_shader_.set_vec3("uCamRight", cam_right.x, cam_right.y, cam_right.z);
            particle_shader_.set_vec3("uCamUp", cam_up.x, cam_up.y, cam_up.z);

            glBindVertexArray(particle_vao_);
            glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(particle_buffer_.size() * sizeof(float)),
                         particle_buffer_.data(),
                         GL_STREAM_DRAW);

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive/Translucent HDR bloom blend
            glDepthMask(GL_FALSE);

            glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(particle_buffer_.size() / 10));

            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
            glBindVertexArray(0);
        }
    }

private:
    void append_billboard_quad(const math::Vector3& center_rel, float size, float r, float g, float b, float a) {
        // Two triangles forming a camera-facing quad: 6 vertices
        static constexpr float kOffsets[6][2] = {
            {-1.0f, -1.0f}, { 1.0f, -1.0f}, { 1.0f,  1.0f},
            {-1.0f, -1.0f}, { 1.0f,  1.0f}, {-1.0f,  1.0f}
        };

        const float cx = static_cast<float>(center_rel.x);
        const float cy = static_cast<float>(center_rel.y);
        const float cz = static_cast<float>(center_rel.z);

        for (int i = 0; i < 6; ++i) {
            particle_buffer_.push_back(cx);
            particle_buffer_.push_back(cy);
            particle_buffer_.push_back(cz);
            particle_buffer_.push_back(kOffsets[i][0]);
            particle_buffer_.push_back(kOffsets[i][1]);
            particle_buffer_.push_back(r);
            particle_buffer_.push_back(g);
            particle_buffer_.push_back(b);
            particle_buffer_.push_back(a);
            particle_buffer_.push_back(size);
        }
    }

    void build_missile_mesh() {
        // Procedural supersonic missile: 12-sided cylinder + conical radome nose + 4 aft fins
        std::vector<float> verts;
        constexpr int kSides = 12;
        constexpr float kRadius = 0.085f;
        constexpr float kLength = 3.65f;
        constexpr float kNoseLen = 0.65f;

        // Cylinder body: [0 to kLength - kNoseLen] along +Z
        for (int i = 0; i < kSides; ++i) {
            const float a0 = static_cast<float>(i) * 2.0f * 3.14159265f / kSides;
            const float a1 = static_cast<float>(i + 1) * 2.0f * 3.14159265f / kSides;
            const float c0 = std::cos(a0), s0 = std::sin(a0);
            const float c1 = std::cos(a1), s1 = std::sin(a1);

            // Two triangles for body quad
            auto add_v = [&](float x, float y, float z, float nx, float ny, float nz, float cr, float cg, float cb) {
                verts.insert(verts.end(), {x, y, z, nx, ny, nz, cr, cg, cb});
            };

            // Quad 1
            add_v(c0 * kRadius, s0 * kRadius, -kLength * 0.5f, c0, s0, 0.0f, 0.85f, 0.87f, 0.90f);
            add_v(c1 * kRadius, s1 * kRadius, -kLength * 0.5f, c1, s1, 0.0f, 0.85f, 0.87f, 0.90f);
            add_v(c1 * kRadius, s1 * kRadius,  kLength * 0.5f - kNoseLen, c1, s1, 0.0f, 0.85f, 0.87f, 0.90f);

            // Quad 2
            add_v(c0 * kRadius, s0 * kRadius, -kLength * 0.5f, c0, s0, 0.0f, 0.85f, 0.87f, 0.90f);
            add_v(c1 * kRadius, s1 * kRadius,  kLength * 0.5f - kNoseLen, c1, s1, 0.0f, 0.85f, 0.87f, 0.90f);
            add_v(c0 * kRadius, s0 * kRadius,  kLength * 0.5f - kNoseLen, c0, s0, 0.0f, 0.85f, 0.87f, 0.90f);

            // Conical Nose Cone
            add_v(c0 * kRadius, s0 * kRadius, kLength * 0.5f - kNoseLen, c0, s0, 0.5f, 0.35f, 0.38f, 0.42f);
            add_v(c1 * kRadius, s1 * kRadius, kLength * 0.5f - kNoseLen, c1, s1, 0.5f, 0.35f, 0.38f, 0.42f);
            add_v(0.0f, 0.0f, kLength * 0.5f, 0.0f, 0.0f, 1.0f, 0.25f, 0.26f, 0.30f);
        }

        // 4 Cruciform delta fins at tail
        for (int f = 0; f < 4; ++f) {
            const float ang = static_cast<float>(f) * 1.5707963f;
            const float ca = std::cos(ang);
            const float sa = std::sin(ang);
            const float span = 0.32f;

            auto add_fin = [&](float x, float y, float z) {
                verts.insert(verts.end(), {x, y, z, -sa, ca, 0.0f, 0.3f, 0.32f, 0.35f});
            };

            add_fin(ca * kRadius, sa * kRadius, -kLength * 0.5f);
            add_fin(ca * (kRadius + span), sa * (kRadius + span), -kLength * 0.5f);
            add_fin(ca * kRadius, sa * kRadius, -kLength * 0.5f + 0.45f);

            add_fin(ca * kRadius, sa * kRadius, -kLength * 0.5f);
            add_fin(ca * kRadius, sa * kRadius, -kLength * 0.5f + 0.45f);
            add_fin(ca * (kRadius + span), sa * (kRadius + span), -kLength * 0.5f);
        }

        missile_vert_count_ = static_cast<GLsizei>(verts.size() / 9);

        glGenVertexArrays(1, &missile_vao_);
        glGenBuffers(1, &missile_vbo_);
        glBindVertexArray(missile_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, missile_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)), verts.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
        glBindVertexArray(0);
    }

    ShaderProgram particle_shader_{};
    ShaderProgram missile_shader_{};
    GLuint particle_vao_ = 0;
    GLuint particle_vbo_ = 0;
    GLuint missile_vao_ = 0;
    GLuint missile_vbo_ = 0;
    GLsizei missile_vert_count_ = 0;
    bool initialized_ = false;

    std::array<ExplosionEffect, kMaxExplosions> explosions_{};
    std::vector<SmokeNode> smoke_pool_;
    std::vector<float> particle_buffer_;
};

} // namespace fastjet::graphics
