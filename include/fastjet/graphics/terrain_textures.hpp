#pragma once

#include "fastjet/core/thread_pool.hpp"
#include "fastjet/graphics/canyon_glsl.hpp"
#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/shader_library.hpp"
#include "fastjet/graphics/terrain_field.hpp"
#include <array>
#include <string>
#include <vector>

namespace fastjet::graphics {

/// @brief Ground material baked into nested world-space textures.
///
/// glsl::TERRAIN_MATERIAL (farmland parcels, hedgerows, woods, lakes, rock,
/// snow) costs about a hundred noise evaluations per pixel: far too much to
/// run for every terrain fragment on an integrated GPU. The world is static,
/// so it is evaluated once at start-up into three nested textures centred on
/// the airfield, each covering four times the area of the one inside it, like
/// a clipmap. The per-pixel shader then samples the level whose texel size
/// matches the viewing distance and adds only cheap close-range detail.
///
/// Texel sizes: 7.8 m (inner 16 km), 31 m (64 km), 94 m (192 km).
class TerrainTextures {
public:
    static constexpr int kLevels = 3;
    static constexpr int kAlbedoResolution = 2048;
    static constexpr int kHeightResolution = 1024;
    /// Edge length of each level [m], all centred on the airfield.
    static constexpr std::array<float, kLevels> kLevelSize{16000.0f, 64000.0f, 192000.0f};
    static constexpr float kAnisotropy = 16.0f;
    /// First of the kLevels texture units the levels are bound to; above
    /// the material, shadow and cloud units.
    static constexpr int kFirstUnit = 7;

    TerrainTextures() = default;
    ~TerrainTextures() { destroy(); }
    TerrainTextures(const TerrainTextures&) = delete;
    TerrainTextures& operator=(const TerrainTextures&) = delete;

    /// @brief Samples the heightfield and bakes every level.
    bool init() {
        if (initialized_) return true;
        if (!init_shader()) return false;

        GLint prev_viewport[4];
        glGetIntegerv(GL_VIEWPORT, prev_viewport);
        GLuint vao = 0;
        glGenVertexArrays(1, &vao);
        GLuint fbo = 0;
        glGenFramebuffers(1, &fbo);

        bool ok = true;
        for (int i = 0; i < kLevels && ok; ++i) ok = bake_level(i, vao, fbo);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fbo);
        glDeleteVertexArrays(1, &vao);
        glViewport(prev_viewport[0], prev_viewport[1], prev_viewport[2], prev_viewport[3]);
        bake_shader_.destroy();
        if (!ok) {
            destroy();
            return false;
        }
        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        for (GLuint& t : albedo_) {
            if (t) glDeleteTextures(1, &t);
            t = 0;
        }
        bake_shader_.destroy();
        initialized_ = false;
    }

    /// @brief Binds the levels to consecutive units from `first_unit` and sets
    /// the glsl::GROUND_BAKED sampler and rectangle uniforms.
    void bind(const ShaderProgram& s, int first_unit) const noexcept {
        static constexpr std::array<const char*, kLevels> kSampler{"uGround0", "uGround1", "uGround2"};
        for (int i = 0; i < kLevels; ++i) {
            glActiveTexture(GL_TEXTURE0 + static_cast<GLenum>(first_unit + i));
            glBindTexture(GL_TEXTURE_2D, albedo_[static_cast<size_t>(i)]);
            s.set_int(kSampler[static_cast<size_t>(i)], first_unit + i);
        }
        glActiveTexture(GL_TEXTURE0);
        const float ox = TerrainField::FIELD_CENTER_X;
        const float oy = TerrainField::FIELD_CENTER_Y;
        s.set_vec4("uGroundLevels", 1.0f / kLevelSize[0], 1.0f / kLevelSize[1], 1.0f / kLevelSize[2], 0.0f);
        s.set_vec2("uGroundCenter", ox, oy);
    }

    [[nodiscard]] bool is_initialized() const noexcept { return initialized_; }

private:
    std::array<GLuint, kLevels> albedo_{};
    ShaderProgram bake_shader_;
    bool initialized_ = false;

    bool init_shader() {
        const char* vert = R"(
            #version 330 core
            out vec2 vUV;
            void main() {
                vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
                vUV = p;
                gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
            }
        )";
        const std::string frag = std::string(R"(
            #version 330 core
            in vec2 vUV;
            out vec4 FragColor;
            uniform sampler2D uHeight;
            uniform vec2  uOrigin;     // world XY of the level's corner [m]
            uniform float uSize;       // level edge [m]
            uniform float uHeightTexel; // one height texel in UV
        )") + glsl::COMMON + glsl::TERRAIN_MATERIAL + canyon_glsl() + R"(
            void main() {
                vec2 xy = uOrigin + vUV * uSize;
                float h = texture(uHeight, vUV).r;
                // Slope from the heightfield, as the mesh normals are built.
                float span = uHeightTexel * uSize;
                float hl = texture(uHeight, vUV - vec2(uHeightTexel, 0.0)).r;
                float hr = texture(uHeight, vUV + vec2(uHeightTexel, 0.0)).r;
                float hd = texture(uHeight, vUV - vec2(0.0, uHeightTexel)).r;
                float hu = texture(uHeight, vUV + vec2(0.0, uHeightTexel)).r;
                vec3 n = normalize(vec3(-(hr - hl) / (2.0 * span), -(hu - hd) / (2.0 * span), -1.0));
                // A large distance disables the close-range detail: it is
                // added per pixel at run time instead.
                GroundSample g = ground_material(vec3(xy, -h), h, -n.z, 1.0e9);
                // The mesa country's own palette; the river is added per pixel.
                canyon_shade(g, canyon_sd(xy), vec3(xy, -h), h, -n.z, 0.0);
                // Square-root encoding keeps precision in the dark albedos
                // that grass and soil live in.
                float cover = g.water > 0.5 ? 1.0 : g.forest * 0.95;
                FragColor = vec4(sqrt(clamp(g.albedo, 0.0, 1.0)), cover);
            }
        )";
        if (!bake_shader_.init_from_source(vert, frag.c_str())) return false;
        return true;
    }

    bool bake_level(int level, GLuint vao, GLuint fbo) {
        const float size = kLevelSize[static_cast<size_t>(level)];
        const float ox = TerrainField::FIELD_CENTER_X - size * 0.5f;
        const float oy = TerrainField::FIELD_CENTER_Y - size * 0.5f;

        // Heights at texel centres, sampled from the same field the mesh uses.
        std::vector<float> heights(static_cast<size_t>(kHeightResolution) * kHeightResolution);
        const float step = size / static_cast<float>(kHeightResolution);
        // A million independent samples per level: spread over the pool.
        core::ThreadPool::shared().parallel_for(0, kHeightResolution, [&](int j) {
            const float y = oy + (static_cast<float>(j) + 0.5f) * step;
            for (int i = 0; i < kHeightResolution; ++i) {
                const float x = ox + (static_cast<float>(i) + 0.5f) * step;
                heights[static_cast<size_t>(j) * kHeightResolution + static_cast<size_t>(i)] = TerrainField::height(x, y);
            }
        });
        GLuint height_tex = 0;
        glGenTextures(1, &height_tex);
        glBindTexture(GL_TEXTURE_2D, height_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, kHeightResolution, kHeightResolution, 0, GL_RED, GL_FLOAT, heights.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        GLuint& albedo = albedo_[static_cast<size_t>(level)];
        glGenTextures(1, &albedo);
        glBindTexture(GL_TEXTURE_2D, albedo);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kAlbedoResolution, kAlbedoResolution, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // The ground is almost always seen at a grazing angle.
        apply_texture_anisotropy(kAnisotropy);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, albedo, 0);
        const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        if (complete) {
            glViewport(0, 0, kAlbedoResolution, kAlbedoResolution);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            glDisable(GL_CULL_FACE);
            glDisable(GL_STENCIL_TEST);
            bake_shader_.use();
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, height_tex);
            bake_shader_.set_int("uHeight", 0);
            bake_shader_.set_vec2("uOrigin", ox, oy);
            bake_shader_.set_float("uSize", size);
            bake_shader_.set_float("uHeightTexel", 1.0f / static_cast<float>(kHeightResolution));
            glBindVertexArray(vao);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            glBindVertexArray(0);
            glUseProgram(0);
        }
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glBindTexture(GL_TEXTURE_2D, albedo);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
        glDeleteTextures(1, &height_tex);
        return complete;
    }
};

} // namespace fastjet::graphics
