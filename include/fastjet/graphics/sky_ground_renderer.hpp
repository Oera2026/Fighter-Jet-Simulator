#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/shader_library.hpp"
#include "fastjet/graphics/frame_context.hpp"
#include "fastjet/graphics/terrain_field.hpp"
#include "fastjet/graphics/terrain_mesh.hpp"
#include "fastjet/graphics/terrain_textures.hpp"
#include "fastjet/graphics/canyon_glsl.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace fastjet::graphics {

/// @brief Visual environment: scattering sky dome with cirrus, procedural
/// terrain, and the airfield surface with its markings.
///
/// The sky is an analytic single-scattering approximation (Rayleigh + Mie)
/// rather than a fixed gradient, so the horizon reddens near the sun, the zenith
/// deepens with altitude, and terrain haze matches the sky it fades into. The
/// same sun direction and haze constants feed the terrain, airframe and cockpit
/// shaders, which is what keeps the scene one coherent lighting environment.
///
/// Every draw takes an optional FrameContext. Without one (standalone use and
/// the render tests) the shaders fall back to calibrated default lighting, no
/// clouds or shadows, and encode straight to the display.
class SkyGroundRenderer {
public:
    /// @brief Sun direction in NED, pointing *toward* the sun. Late-morning,
    /// high and slightly forward-left of the runway heading.
    static constexpr float SUN_NED_X = 0.42f;
    static constexpr float SUN_NED_Y = -0.34f;
    static constexpr float SUN_NED_Z = -0.84f; // negative z = upward

    static void sun_dir_ned(float& x, float& y, float& z) noexcept {
        const float inv = 1.0f / std::sqrt(SUN_NED_X * SUN_NED_X +
                                           SUN_NED_Y * SUN_NED_Y +
                                           SUN_NED_Z * SUN_NED_Z);
        x = SUN_NED_X * inv;
        y = SUN_NED_Y * inv;
        z = SUN_NED_Z * inv;
    }

    [[nodiscard]] static std::array<float, 3> sun_dir_ned() noexcept {
        std::array<float, 3> d{};
        sun_dir_ned(d[0], d[1], d[2]);
        return d;
    }

    /// @brief Surface types of the airfield decals, shaded procedurally.
    enum class Surface : int {
        GRASS = 0,    ///< Infield: the terrain material, so it meets the terrain seamlessly
        ASPHALT = 1,  ///< Runway: rubber deposits, patching and aggregate
        SHOULDER = 2, ///< Graded gravel margins
        CONCRETE = 3, ///< Dispersal ramp: jointed slabs
        PAINT = 4,    ///< Markings: vertex colour, worn and rubbered
        LIGHT = 5,    ///< Edge, threshold and end lights: emissive
        TAXIWAY = 6,  ///< Older asphalt, lighter, no touchdown rubber
    };

    /// Emitted radiance of the airfield lights (HDR, relative to their colour).
    static constexpr float LIGHT_RADIANCE = 4.0f;

    /// Cirrus pattern texture: resolution and noise cells across one tile.
    static constexpr int kCirrusResolution = 1024;
    static constexpr int kCirrusCells = 8;
    /// Texture unit of the cirrus pattern in the sky shader.
    static constexpr int kCirrusUnit = 1;

private:
    GLuint sky_vao_ = 0;
    GLuint sky_vbo_ = 0;
    GLuint cirrus_tex_ = 0;
    ShaderProgram sky_shader_;

    GLuint ground_vao_ = 0;
    GLuint ground_vbo_ = 0;
    GLsizei ground_vertex_count_ = 0;
    ShaderProgram ground_shader_;

    TerrainMesh terrain_;
    TerrainTextures ground_textures_;
    ShaderProgram terrain_shader_;

    bool initialized_ = false;

    void build_sky_quad() {
        // Fullscreen quad in NDC [-1, 1]
        const float quad_verts[] = {
            -1.0f, -1.0f,
             1.0f, -1.0f,
             1.0f,  1.0f,
            -1.0f, -1.0f,
             1.0f,  1.0f,
            -1.0f,  1.0f
        };

        glGenVertexArrays(1, &sky_vao_);
        glGenBuffers(1, &sky_vbo_);

        glBindVertexArray(sky_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, sky_vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad_verts), quad_verts, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

    /// @brief Airfield surface: apron, runway, and markings. These sit on the
    /// flat basin the terrain field guarantees around the field, so they need no
    /// height lookup, only a small bias to stay clear of the terrain mesh.
    ///
    /// Vertex layout: pos(3), colour(4), surface(1).
    void build_airfield() {
        std::vector<float> verts;

        auto add_cell = [&](float x0, float y0, float x1, float y1, float z,
                            const Color4& col, float m) {
            // Reversed winding, matching TerrainMesh: NED is left-handed with
            // respect to the screen convention, so listing the corners in their
            // natural order yields a clockwise (back) face seen from above.
            const float v[6][8] = {
                {x0, y0, z, col.r, col.g, col.b, col.a, m},
                {x1, y1, z, col.r, col.g, col.b, col.a, m},
                {x1, y0, z, col.r, col.g, col.b, col.a, m},
                {x0, y0, z, col.r, col.g, col.b, col.a, m},
                {x0, y1, z, col.r, col.g, col.b, col.a, m},
                {x1, y1, z, col.r, col.g, col.b, col.a, m}
            };
            for (const auto& vert : v) verts.insert(verts.end(), vert, vert + 8);
        };
        // Large surfaces are split into cells no bigger than kMaxCell, so the
        // per-vertex haze interpolates accurately across them.
        constexpr float kMaxCell = 80.0f;
        auto add_quad = [&](float x0, float y0, float x1, float y1, float z,
                            const Color4& col, Surface surf) {
            const int nx = (std::max)(1, static_cast<int>(std::ceil((x1 - x0) / kMaxCell)));
            const int ny = (std::max)(1, static_cast<int>(std::ceil((y1 - y0) / kMaxCell)));
            const float dx = (x1 - x0) / static_cast<float>(nx);
            const float dy = (y1 - y0) / static_cast<float>(ny);
            for (int i = 0; i < nx; ++i) {
                for (int j = 0; j < ny; ++j) {
                    const float cx0 = x0 + dx * static_cast<float>(i);
                    const float cy0 = y0 + dy * static_cast<float>(j);
                    add_cell(cx0, cy0, i + 1 == nx ? x1 : cx0 + dx, j + 1 == ny ? y1 : cy0 + dy, z, col,
                             static_cast<float>(surf));
                }
            }
        };

        // Decal heights: each layer sits a few centimetres above the one it
        // covers; polygon offset in render_ground() does the rest.
        constexpr float Z_APRON = -0.05f;
        constexpr float Z_SHOULDER = -0.12f;
        constexpr float Z_RAMP = -0.17f;
        constexpr float Z_TAXI = -0.18f;
        constexpr float Z_CONNECTOR = -0.19f;
        constexpr float Z_RUNWAY = -0.20f;
        constexpr float Z_PAINT = -0.28f;
        constexpr float Z_RUNWAY_PAINT = -0.30f;

        const Color4 none{1.0f, 1.0f, 1.0f, 1.0f};
        const Color4 white{0.62f, 0.62f, 0.60f, 1.0f};
        const Color4 yellow{0.90f, 0.76f, 0.10f, 1.0f};

        // 1. Infield grass. TerrainMesh cuts a hole here, snapped out to its
        // 125 m cell grid, so the apron covers the snapped bounds
        // (-750..3750, -1000..1000) or the seam would show through to the sky.
        add_quad(-750.0f, -1000.0f, 3750.0f, 1000.0f, Z_APRON, none, Surface::GRASS);

        // 2. Runway surface (3,000 m x 60 m) and graded shoulders.
        add_quad(0.0f, -30.0f, 3000.0f, 30.0f, Z_RUNWAY, none, Surface::ASPHALT);
        add_quad(0.0f, -46.0f, 3000.0f, -30.0f, Z_SHOULDER, none, Surface::SHOULDER);
        add_quad(0.0f, 30.0f, 3000.0f, 46.0f, Z_SHOULDER, none, Surface::SHOULDER);

        // 3. Centreline stripes (30 m dash, 20 m gap, width 0.9 m)
        for (float x = 120.0f; x < 2880.0f; x += 50.0f) {
            add_quad(x, -0.45f, x + 30.0f, 0.45f, Z_RUNWAY_PAINT, white, Surface::PAINT);
        }

        // 4. Runway edge lines
        add_quad(0.0f, -29.4f, 3000.0f, -28.5f, Z_RUNWAY_PAINT, white, Surface::PAINT);
        add_quad(0.0f, 28.5f, 3000.0f, 29.4f, Z_RUNWAY_PAINT, white, Surface::PAINT);

        // 5. Threshold piano keys at both runway heads
        for (float y = -24.0f; y <= 24.0f; y += 4.0f) {
            add_quad(30.0f, y - 1.2f, 80.0f, y + 1.2f, Z_RUNWAY_PAINT, white, Surface::PAINT);
            add_quad(2920.0f, y - 1.2f, 2970.0f, y + 1.2f, Z_RUNWAY_PAINT, white, Surface::PAINT);
        }

        // 6. Touchdown zone markers, 150 m past each threshold
        for (float side : {-1.0f, 1.0f}) {
            for (float y0 : {8.0f, 14.0f}) {
                const float ya = side * y0;
                const float yb = side * (y0 + 2.4f);
                add_quad(230.0f, (std::min)(ya, yb), 275.0f, (std::max)(ya, yb), Z_RUNWAY_PAINT, white, Surface::PAINT);
                add_quad(2725.0f, (std::min)(ya, yb), 2770.0f, (std::max)(ya, yb), Z_RUNWAY_PAINT, white, Surface::PAINT);
            }
        }

        // 7. Aiming point blocks
        for (float side : {-1.0f, 1.0f}) {
            const float ya = side * 8.0f;
            const float yb = side * 14.0f;
            add_quad(400.0f, (std::min)(ya, yb), 460.0f, (std::max)(ya, yb), Z_RUNWAY_PAINT, white, Surface::PAINT);
            add_quad(2540.0f, (std::min)(ya, yb), 2600.0f, (std::max)(ya, yb), Z_RUNWAY_PAINT, white, Surface::PAINT);
        }

        // 8. Runway designators: "09" at the west end, "27" at the east end.
        auto paint = [&](float rx0, float ry0, float rx1, float ry1) {
            add_quad(rx0, ry0, rx1, ry1, Z_RUNWAY_PAINT, white, Surface::PAINT);
        };
        // Digit '0'
        paint(110.0f, -5.5f, 114.0f, -1.5f);
        paint(120.0f, -5.5f, 124.0f, -1.5f);
        paint(114.0f, -5.5f, 120.0f, -4.5f);
        paint(114.0f, -2.5f, 120.0f, -1.5f);
        // Digit '9'
        paint(110.0f, 1.5f, 124.0f, 2.5f);
        paint(110.0f, 2.5f, 117.0f, 5.5f);
        paint(110.0f, 4.5f, 124.0f, 5.5f);
        paint(117.0f, 1.5f, 124.0f, 5.5f);
        // Digit '2'
        paint(2865.0f, -5.5f, 2879.0f, -4.5f);
        paint(2875.0f, -4.5f, 2879.0f, -3.2f);
        paint(2865.0f, -3.8f, 2879.0f, -3.0f);
        paint(2865.0f, -3.0f, 2869.0f, -1.8f);
        paint(2865.0f, -2.2f, 2879.0f, -1.5f);
        // Digit '7'
        paint(2865.0f, 1.5f, 2879.0f, 2.5f);
        paint(2875.0f, 1.5f, 2879.0f, 5.5f);

        // 9. Parallel taxiway (X = -300..3300 m, Y = -110..-85 m)
        add_quad(-300.0f, -110.0f, 3300.0f, -85.0f, Z_TAXI, none, Surface::TAXIWAY);
        add_quad(-300.0f, -118.0f, 3300.0f, -110.0f, Z_SHOULDER, none, Surface::SHOULDER);
        add_quad(-300.0f, -85.0f, 3300.0f, -77.0f, Z_SHOULDER, none, Surface::SHOULDER);

        // 10. Connectors linking the runway to the parallel taxiway: west
        // turnoff, mid-west rapid exit, midfield, mid-east rapid exit, east.
        struct Connector { float x0, x1; };
        for (const Connector c : {Connector{-50.0f, 50.0f}, Connector{850.0f, 970.0f}, Connector{1450.0f, 1550.0f},
                                  Connector{2030.0f, 2150.0f}, Connector{2950.0f, 3050.0f}}) {
            add_quad(c.x0, -85.0f, c.x1, -30.0f, Z_CONNECTOR, none, Surface::TAXIWAY);
            add_quad(c.x0 - 10.0f, -85.0f, c.x0, -30.0f, Z_SHOULDER, none, Surface::SHOULDER);
            add_quad(c.x1, -85.0f, c.x1 + 10.0f, -30.0f, Z_SHOULDER, none, Surface::SHOULDER);
        }

        // 11. Aviation yellow taxiway centrelines and holding points.
        add_quad(-290.0f, -98.0f, 3290.0f, -97.0f, Z_PAINT, yellow, Surface::PAINT);
        for (float cx : {0.0f, 910.0f, 1500.0f, 2090.0f, 3000.0f}) {
            add_quad(cx - 1.0f, -97.0f, cx + 1.0f, -30.0f, Z_PAINT, yellow, Surface::PAINT);
            add_quad(cx - 18.0f, -44.0f, cx + 18.0f, -43.2f, Z_PAINT, yellow, Surface::PAINT);
            add_quad(cx - 18.0f, -42.4f, cx + 18.0f, -41.6f, Z_PAINT, yellow, Surface::PAINT);
        }

        // 12. Dispersal ramp with six hardstands and lead-in lines.
        add_quad(400.0f, -240.0f, 2600.0f, -110.0f, Z_RAMP, none, Surface::CONCRETE);
        for (float pad_x : {600.0f, 950.0f, 1300.0f, 1650.0f, 2000.0f, 2350.0f}) {
            add_quad(pad_x - 35.0f, -220.0f, pad_x + 35.0f, -130.0f, Z_CONNECTOR, none, Surface::TAXIWAY);
            add_quad(pad_x - 0.5f, -210.0f, pad_x + 0.5f, -98.0f, Z_PAINT, yellow, Surface::PAINT);
            add_quad(pad_x - 6.0f, -170.0f, pad_x + 6.0f, -169.2f, Z_PAINT, yellow, Surface::PAINT);
        }

        // 13. Runway lighting: edge lights every 60 m, green threshold bars,
        // red end bars. Emissive, so they read at distance and glow at dusk.
        const Color4 edge_light{0.96f, 0.94f, 0.70f, 1.0f};
        for (float lx = 0.0f; lx <= 3000.0f; lx += 60.0f) {
            add_quad(lx - 0.4f, -29.8f, lx + 0.4f, -29.0f, Z_PAINT, edge_light, Surface::LIGHT);
            add_quad(lx - 0.4f, 29.0f, lx + 0.4f, 29.8f, Z_PAINT, edge_light, Surface::LIGHT);
        }
        const Color4 green_thresh{0.15f, 0.95f, 0.35f, 1.0f};
        for (float ty = -28.0f; ty <= 28.0f; ty += 2.5f) {
            add_quad(0.0f, ty - 0.4f, 2.0f, ty + 0.4f, Z_PAINT, green_thresh, Surface::LIGHT);
            add_quad(2998.0f, ty - 0.4f, 3000.0f, ty + 0.4f, Z_PAINT, green_thresh, Surface::LIGHT);
        }
        const Color4 red_end{0.95f, 0.15f, 0.15f, 1.0f};
        for (float ty = -28.0f; ty <= 28.0f; ty += 3.0f) {
            add_quad(-4.0f, ty - 0.5f, -2.0f, ty + 0.5f, Z_PAINT, red_end, Surface::LIGHT);
            add_quad(3002.0f, ty - 0.5f, 3004.0f, ty + 0.5f, Z_PAINT, red_end, Surface::LIGHT);
        }

        // Draw the top layer first (paint, then pavement, then grass): each
        // lower layer is then rejected by the early depth test wherever it is
        // covered instead of being shaded and overdrawn, and where depth
        // precision runs out at range the upper layer still wins the tie.
        {
            constexpr size_t kFloatsPerQuad = 6 * 8;
            const size_t quads = verts.size() / kFloatsPerQuad;
            std::vector<size_t> order(quads);
            for (size_t q = 0; q < quads; ++q) order[q] = q;
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
                return verts[a * kFloatsPerQuad + 2] < verts[b * kFloatsPerQuad + 2]; // NED z: up is negative
            });
            std::vector<float> sorted;
            sorted.reserve(verts.size());
            for (size_t q : order) {
                sorted.insert(sorted.end(), verts.begin() + static_cast<std::ptrdiff_t>(q * kFloatsPerQuad),
                              verts.begin() + static_cast<std::ptrdiff_t>((q + 1) * kFloatsPerQuad));
            }
            verts.swap(sorted);
        }

        ground_vertex_count_ = static_cast<GLsizei>(verts.size() / 8);

        glGenVertexArrays(1, &ground_vao_);
        glGenBuffers(1, &ground_vbo_);
        glBindVertexArray(ground_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, ground_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);
        constexpr GLsizei stride = 8 * sizeof(float);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(7 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glBindVertexArray(0);
    }

    /// @brief Bakes the cirrus streak pattern into a tiling texture.
    ///
    /// Six octaves of warped noise per sky pixel cost ~2 ms at 1080p on an
    /// integrated GPU; the pattern never changes, so it is evaluated once on
    /// a periodic lattice (so it tiles) and sampled with one lookup.
    bool bake_cirrus() {
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
            uniform int uCells;
        )") + glsl::COMMON + R"(
            // Value noise on a lattice that wraps every `period` cells.
            float vnoise_periodic(vec2 p, int period) {
                vec2 i = floor(p);
                vec2 f = p - i;
                vec2 u = f * f * (3.0 - 2.0 * f);
                ivec2 c0 = ivec2(i) % period;
                c0 += ivec2(lessThan(c0, ivec2(0))) * period;
                ivec2 c1 = (c0 + 1) % period;
                float a = hash21(c0), b = hash21(ivec2(c1.x, c0.y));
                float d = hash21(ivec2(c0.x, c1.y)), e = hash21(c1);
                return mix(mix(a, b, u.x), mix(d, e, u.x), u.y);
            }
            float fbm_periodic(vec2 p, int period, int octaves) {
                float sum = 0.0, amp = 0.5, norm = 0.0;
                for (int k = 0; k < octaves; ++k) {
                    sum += amp * vnoise_periodic(p, period);
                    norm += amp;
                    p = p * 2.0 + vec2(float(k + 3));
                    period *= 2;
                    amp *= 0.5;
                }
                return sum / norm;
            }
            void main() {
                vec2 q = vUV * float(uCells);
                // Warp by a periodic field so the result still tiles.
                float warp = fbm_periodic(q * 2.0, uCells * 2, 3);
                FragColor = vec4(fbm_periodic(q + vec2(warp * 0.8, 0.0), uCells, 6), 0.0, 0.0, 1.0);
            }
        )";
        ShaderProgram bake;
        if (!bake.init_from_source(vert, frag.c_str())) return false;

        glGenTextures(1, &cirrus_tex_);
        glBindTexture(GL_TEXTURE_2D, cirrus_tex_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, kCirrusResolution, kCirrusResolution, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        apply_texture_anisotropy(TerrainTextures::kAnisotropy);

        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        GLuint fbo = 0, vao = 0;
        glGenFramebuffers(1, &fbo);
        glGenVertexArrays(1, &vao);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, cirrus_tex_, 0);
        const bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
        if (ok) {
            glViewport(0, 0, kCirrusResolution, kCirrusResolution);
            glDisable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            bake.use();
            bake.set_int("uCells", kCirrusCells);
            glBindVertexArray(vao);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            glBindVertexArray(0);
            glUseProgram(0);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fbo);
        glDeleteVertexArrays(1, &vao);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glBindTexture(GL_TEXTURE_2D, cirrus_tex_);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
        return ok;
    }

    /// Uniforms common to every environment shader for one draw.
    void upload_frame(const ShaderProgram& s, const FrameContext* ctx, const math::Vector3& eye) const {
        ground_textures_.bind(s, TerrainTextures::kFirstUnit);
        if (ctx) {
            ctx->upload(s);
            ctx->bind_shadow(s, ctx->ground_shadow);
        } else {
            // Standalone: default lighting, display output, no clouds/shadow.
            float sx, sy, sz;
            sun_dir_ned(sx, sy, sz);
            s.set_vec3("uSunDir", sx, sy, sz);
            s.set_int("uHdrOutput", 0);
            s.set_float("uCloudCoverage", 0.0f);
            s.set_int("uShadowEnabled", 0);
        }
        s.set_vec3("uEyeNed", static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z));
        s.set_float("uAltitude", static_cast<float>(-eye.z));
    }

public:
    SkyGroundRenderer() = default;

    ~SkyGroundRenderer() {
        destroy();
    }

    /// @brief The shared atmosphere GLSL (see glsl::ATMOSPHERE).
    static constexpr const char* ATMOSPHERE_GLSL = glsl::ATMOSPHERE;

    bool init() {
        if (initialized_) return true;

        // -------------------------------------------------------------------
        // Sky: analytic single-scattering dome with a cirrus deck
        // -------------------------------------------------------------------
        const char* sky_vert = R"(
            #version 330 core
            layout (location = 0) in vec2 aPos;
            out vec2 vPos;
            void main() {
                vPos = aPos;
                gl_Position = vec4(aPos, 0.9999, 1.0); // Render at far plane
            }
        )";

        const std::string sky_frag = std::string(R"(
            #version 330 core
            in vec2 vPos;
            out vec4 FragColor;

            uniform mat4  uInvViewProj;
            uniform float uAltitude;  // [m] above sea level
            uniform vec3  uEyeNed;
            uniform float uCirrus;    // cirrus cover, 0 = none
            const float CIRRUS_CELLS = )" + std::to_string(kCirrusCells) + R"(;
        )") + glsl::COMMON + glsl::OUTPUT + glsl::ATMOSPHERE + glsl::LIGHTING + glsl::CLOUDS + R"(
            const float CIRRUS_ALT = 9500.0; // [m]
            uniform sampler2D uCirrusTex;    // tiling streak pattern, CIRRUS_CELLS cells across

            /// Thin, wind-stretched ice cloud, lit mostly by forward scatter.
            vec3 add_cirrus(vec3 sky, vec3 ray) {
                if (uCirrus <= 0.0 || ray.z > -0.005 || uAltitude >= CIRRUS_ALT) return sky;
                float t = (CIRRUS_ALT - uAltitude) / -ray.z;
                vec2 p = uEyeNed.xy + ray.xy * t + uCloudOffset * 0.6;
                const mat2 STRETCH = mat2(0.94, 0.34, -0.34, 0.94);
                vec2 q = STRETCH * p * vec2(1.0 / 14000.0, 1.0 / 3200.0);
                float streak = texture(uCirrusTex, q / CIRRUS_CELLS).r;
                float cover = smoothstep(1.0 - uCirrus * 0.55, 1.0 - uCirrus * 0.55 + 0.25, streak);
                // Thin toward the horizon, where the deck is far and hazy.
                cover *= exp(-t / 90000.0) * 0.55;
                float mu = dot(ray, uSunDir);
                float g = 0.7;
                float hg = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5) / (4.0 * PI);
                vec3 lit = sun_radiance() * (0.18 + hg * 1.4) + sky_ambient() * 0.8;
                return mix(sky, lit, cover);
            }

            void main() {
                vec4 near_world = uInvViewProj * vec4(vPos, -1.0, 1.0);
                vec4 far_world  = uInvViewProj * vec4(vPos,  1.0, 1.0);
                vec3 ray = normalize(far_world.xyz / far_world.w - near_world.xyz / near_world.w);

                vec3 sky = sky_radiance(ray, uSunDir, uAltitude, true);
                sky = add_cirrus(sky, ray);
                FragColor = scene_output(sky, 1.0);
            }
        )";

        if (!sky_shader_.init_from_source(sky_vert, sky_frag.c_str())) {
            return false;
        }

        // -------------------------------------------------------------------
        // Terrain: procedural ground material, sun, sky, clouds, aerial haze
        // -------------------------------------------------------------------
        // Aerial perspective varies smoothly with range, so it is evaluated
        // per vertex (the airfield decals are subdivided for this) and
        // interpolated: the scattering integral is the costliest part of
        // shading a ground pixel otherwise.
        const std::string haze_vert_common = std::string(glsl::COMMON) + glsl::ATMOSPHERE + R"(
            uniform vec3  uSunDir;
            uniform float uAltitude;
            out vec3  vHaze;
            out float vFog;
            void compute_haze(vec3 world, vec3 eye) {
                float dist = length(world - eye);
                vec3 ray = (world - eye) / max(dist, 1e-3);
                vHaze = haze_radiance(ray, uSunDir, uAltitude, -world.z, dist);
                vFog = haze_fraction(uAltitude, -world.z, dist);
            }
        )";

        const std::string terr_vert = std::string(R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            layout (location = 2) in float aHeight;

            uniform mat4 uViewProj;
            uniform vec3 uEyeNed;

            out vec3  vNormal;
            out float vHeight;
            out float vDist;
            out vec3  vWorld;
            out vec2  vCanyon;
        )") + haze_vert_common + canyon_glsl(/*material=*/false) + R"(
            void main() {
                vNormal = aNormal;
                vHeight = aHeight;
                vWorld  = aPos;
                // Canyon coordinates vary smoothly: per vertex is enough.
                vCanyon = canyon_sd(aPos.xy);
                vDist   = length(aPos - uEyeNed);
                compute_haze(aPos, uEyeNed);
                gl_Position = uViewProj * vec4(aPos, 1.0);
            }
        )";

        // Shared lighting for everything on the ground, including water.
        const std::string ground_common = std::string(glsl::COMMON) + glsl::OUTPUT + glsl::ATMOSPHERE +
                                          glsl::LIGHTING + glsl::SHADOW + glsl::CLOUDS +
                                          glsl::GROUND_BAKED + R"(
            uniform vec3  uEyeNed;
            uniform float uAltitude;
            in vec3  vHaze;
            in float vFog;

            /// Detail relief: perturbs the normal with small-scale bumps that
            /// fade out before they could alias.
            vec3 detail_normal(vec3 n, vec2 xy, float dist, float amount) {
                float fade = 1.0 - smoothstep(600.0, 4000.0, dist);
                if (fade <= 0.0 || amount <= 0.0) return n;
                vec2 p = xy * 0.09;
                vec2 grad = vnoise_d(p).yz + 0.5 * 3.1 * vnoise_d(p * 3.1).yz;
                vec3 bump = vec3(grad * (0.3 * fade * amount), 0.0);
                return normalize(n + bump);
            }

            /// Lights a ground sample and applies aerial perspective.
            vec3 shade_ground(GroundSample g, vec3 world, vec3 n, float sky_occ) {
                vec3 v = normalize(uEyeNed - world);
                float vis = sun_shadow(world, n, gl_FragCoord.xy) * cloud_shadow(world, uSunDir);
                vec3 lit;
                if (g.water > 0.0) {
                    // Wind ripples, a dark body and the reflected sky, sun glint included.
                    vec2 p = world.xy * 0.35;
                    vec3 wn = normalize(vec3((vnoise(p) - 0.5) * 0.12, (vnoise(p + 7.7) - 0.5) * 0.12, -1.0));
                    float ndv = max(dot(wn, v), 1e-3);
                    float F = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);
                    vec3 r = reflect(-v, wn);
                    r.z = min(r.z, -0.002);
                    vec3 refl = sky_radiance(r, uSunDir, max(-world.z, 0.0), true);
                    refl = mix(sky_radiance(r, uSunDir, max(-world.z, 0.0), false), refl, vis);
                    vec3 body = g.albedo * (sun_radiance() * max(-uSunDir.z, 0.0) * vis + sky_ambient());
                    vec3 wet = mix(body, refl, F);
                    vec3 dry = g.albedo * (sun_radiance() * max(dot(n, uSunDir), 0.0) * vis + hemi_ambient(n) * sky_occ);
                    lit = mix(dry, wet, g.water);
                } else {
                    lit = sun_brdf(g.albedo, 0.0, g.roughness, n, v, uSunDir) * vis
                        + g.albedo * hemi_ambient(n) * sky_occ * (1.0 - 0.35 * g.forest);
                }
                return mix(lit, vHaze, vFog);
            }
        )";

        const std::string terr_frag = std::string(R"(
            #version 330 core
            in vec3  vNormal;
            in float vHeight;
            in float vDist;
            in vec3  vWorld;
            in vec2  vCanyon;
            out vec4 FragColor;
        )") + ground_common + canyon_glsl() + R"(
            void main() {
                vec3 n = normalize(vNormal);
                float level = clamp(-n.z, 0.0, 1.0);
                GroundSample g = ground_baked(vWorld, vDist);
                // Canyon rock bands and the river are finer than the bake's
                // texels: repaint them per pixel while they can be resolved.
                float crisp = 1.0 - smoothstep(5000.0, 14000.0, vDist);
                if (crisp > 0.0) canyon_shade(g, vCanyon, vWorld, vHeight, level, crisp);
                n = detail_normal(n, vWorld.xy, vDist, 1.0 - g.water);
                float sky_occ = 0.55 + 0.45 * level;
                FragColor = scene_output(shade_ground(g, vWorld, n, sky_occ), 1.0);
            }
        )";

        if (!terrain_shader_.init_from_source(terr_vert.c_str(), terr_frag.c_str())) {
            return false;
        }

        // -------------------------------------------------------------------
        // Airfield: procedural pavement, paint and lights on the flat basin
        // -------------------------------------------------------------------
        const std::string gnd_vert = std::string(R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec4 aColor;
            layout (location = 2) in float aSurface;
            uniform mat4 uViewProj;
            uniform vec3 uEyeNed;
            out vec4  vColor;
            out float vDist;
            out vec3  vWorld;
            flat out int vSurface;
        )") + haze_vert_common + R"(
            void main() {
                vColor = aColor;
                vWorld = aPos;
                vDist  = length(aPos - uEyeNed);
                vSurface = int(aSurface + 0.5);
                compute_haze(aPos, uEyeNed);
                gl_Position = uViewProj * vec4(aPos, 1.0);
            }
        )";

        const std::string gnd_frag = std::string(R"(
            #version 330 core
            in vec4  vColor;
            in float vDist;
            in vec3  vWorld;
            flat in int vSurface;
            out vec4 FragColor;
        )") + ground_common + R"(
            const int GRASS = 0, ASPHALT = 1, SHOULDER = 2, CONCRETE = 3, PAINT = 4, LIGHT = 5, TAXIWAY = 6;
            const float LIGHT_RADIANCE = )" + std::to_string(LIGHT_RADIANCE) + R"(;

            /// Rubber laid down by tyres in the touchdown zones, streaked along
            /// the runway and concentrated either side of the centreline.
            float rubber(vec2 xy) {
                float tdz = smoothstep(150.0, 320.0, xy.x) * smoothstep(1000.0, 650.0, xy.x)
                          + smoothstep(2000.0, 2350.0, xy.x) * smoothstep(2850.0, 2680.0, xy.x);
                float lane = smoothstep(15.0, 5.0, abs(xy.y)) * (0.55 + 0.45 * smoothstep(0.5, 3.5, abs(xy.y)));
                float zone = tdz * lane;
                if (zone <= 0.0) return 0.0; // most of the runway: no noise to evaluate
                float streak = vnoise(vec2(xy.x * 0.015, xy.y * 1.7)) * 0.6 + vnoise(vec2(xy.x * 0.06, xy.y * 4.1)) * 0.4;
                return zone * smoothstep(0.25, 0.75, streak);
            }

            vec3 asphalt(vec2 xy, float near, bool runway) {
                // Broad tone (laying runs, weathering) and darker repair patches.
                float tone = vnoise(xy * vec2(0.004, 0.03));
                float patches = smoothstep(0.62, 0.66, vnoise(xy * 0.045 + 11.0));
                // Weathered asphalt: dark, faintly cool (bitumen binder over
                // grey aggregate).
                vec3 c = vec3(0.026, 0.027, 0.030) * (0.82 + 0.32 * tone);
                c = mix(c, vec3(0.022, 0.022, 0.024), patches * 0.7);
                // Aggregate grain, only where it can resolve.
                if (near > 0.0) c *= mix(1.0, 0.86 + 0.28 * vnoise(xy * 9.0), near);
                if (runway) c = mix(c, vec3(0.010, 0.010, 0.011), rubber(xy) * 0.85);
                else c *= 1.18; // older, oxidised taxiway surface
                return c;
            }

            vec3 concrete(vec2 xy, float near) {
                // 5 m slabs with sealed joints and per-slab tone.
                vec2 cell = floor(xy / 5.0);
                vec2 f = fract(xy / 5.0);
                float joint = 1.0 - smoothstep(0.0, 0.012, min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y)));
                float slab = hash21(ivec2(cell));
                float stain = vnoise(xy * 0.35);
                vec3 c = vec3(0.080, 0.080, 0.075) * (0.88 + 0.2 * slab) * (0.85 + 0.25 * stain);
                return mix(c, vec3(0.030), joint * near);
            }

            void main() {
                vec2 xy = vWorld.xy;
                float near = 1.0 - smoothstep(300.0, 2500.0, vDist);
                const vec3 up = vec3(0.0, 0.0, -1.0);
                GroundSample g;
                g.roughness = 0.85;
                g.water = 0.0;
                g.forest = 0.0;
                vec3 emit = vec3(0.0);

                if (vSurface == GRASS) {
                    g = ground_baked(vWorld, vDist);
                } else if (vSurface == ASPHALT || vSurface == TAXIWAY) {
                    g.albedo = asphalt(xy, near, vSurface == ASPHALT);
                    g.roughness = 0.95; // weathered, open-textured surface
                } else if (vSurface == SHOULDER) {
                    float grit = vnoise(xy * 3.0) * 0.6 + vnoise(xy * 0.3) * 0.4;
                    g.albedo = vec3(0.060, 0.056, 0.047) * (0.8 + 0.4 * grit);
                    g.roughness = 0.95;
                } else if (vSurface == CONCRETE) {
                    g.albedo = concrete(xy, near);
                    g.roughness = 0.9;
                } else if (vSurface == PAINT) {
                    // Paint wears through to the surface below and picks up
                    // rubber where it lies in the touchdown zones.
                    float wear = near > 0.0 ? smoothstep(0.55, 0.85, vnoise(xy * 2.3)) * 0.35 * near : 0.0;
                    vec3 under = asphalt(xy, near, false);
                    g.albedo = mix(vColor.rgb, under, wear);
                    g.albedo = mix(g.albedo, vec3(0.012), rubber(xy) * 0.6);
                    g.roughness = 0.6;
                } else {
                    g.albedo = vColor.rgb * 0.3;
                    emit = vColor.rgb * LIGHT_RADIANCE;
                }

                // Paved surfaces are smooth at the scale the bumps work at.
                vec3 n = vSurface == GRASS ? detail_normal(up, xy, vDist, 1.0) : up;
                vec3 lit = shade_ground(g, vWorld, n, 1.0) + emit;
                FragColor = scene_output(lit, 1.0);
            }
        )";

        if (!ground_shader_.init_from_source(gnd_vert.c_str(), gnd_frag.c_str())) {
            return false;
        }

        // Samplers on their reserved units, so types never alias.
        for (const ShaderProgram* s : {&terrain_shader_, &ground_shader_}) {
            s->use();
            s->set_int("uShadowMap", FrameContext::kShadowUnit);
            s->set_int("uCloudMap", CloudSettings::kMapUnit);
        }
        glUseProgram(0);

        build_sky_quad();
        build_airfield();
        terrain_.init();
        if (!ground_textures_.init() || !bake_cirrus()) {
            return false;
        }
        sky_shader_.use();
        sky_shader_.set_int("uCirrusTex", kCirrusUnit);
        sky_shader_.set_int("uCloudMap", CloudSettings::kMapUnit);
        glUseProgram(0);

        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        if (sky_vao_) { glDeleteVertexArrays(1, &sky_vao_); sky_vao_ = 0; }
        if (sky_vbo_) { glDeleteBuffers(1, &sky_vbo_); sky_vbo_ = 0; }
        if (cirrus_tex_) { glDeleteTextures(1, &cirrus_tex_); cirrus_tex_ = 0; }
        if (ground_vao_) { glDeleteVertexArrays(1, &ground_vao_); ground_vao_ = 0; }
        if (ground_vbo_) { glDeleteBuffers(1, &ground_vbo_); ground_vbo_ = 0; }
        terrain_.destroy();
        ground_textures_.destroy();
        sky_shader_.destroy();
        ground_shader_.destroy();
        terrain_shader_.destroy();
        initialized_ = false;
    }

    /// @brief Render the scattering sky dome.
    /// @param inv_view_proj Inverse of the infinity view-projection matrix
    /// @param altitude_m Eye altitude above sea level [m]
    /// @param ctx Frame lighting, or null for standalone defaults
    /// @param cirrus Cirrus cover in [0, 1]
    void render_sky(const Mat4& inv_view_proj, float altitude_m = 0.0f,
                    const FrameContext* ctx = nullptr, float cirrus = 0.0f) {
        if (!initialized_) return;

        glDepthMask(GL_FALSE); // Don't write depth for sky
        sky_shader_.use();
        const math::Vector3 eye = ctx ? ctx->eye_ned : math::Vector3(0.0, 0.0, -altitude_m);
        if (ctx) {
            ctx->upload(sky_shader_);
        } else {
            float sx, sy, sz;
            sun_dir_ned(sx, sy, sz);
            sky_shader_.set_vec3("uSunDir", sx, sy, sz);
            sky_shader_.set_int("uHdrOutput", 0);
        }
        sky_shader_.set_mat4("uInvViewProj", inv_view_proj);
        sky_shader_.set_float("uAltitude", altitude_m);
        sky_shader_.set_vec3("uEyeNed", static_cast<float>(eye.x), static_cast<float>(eye.y),
                             static_cast<float>(eye.z));
        sky_shader_.set_float("uCirrus", cirrus);
        glActiveTexture(GL_TEXTURE0 + kCirrusUnit);
        glBindTexture(GL_TEXTURE_2D, cirrus_tex_);
        glActiveTexture(GL_TEXTURE0);

        glBindVertexArray(sky_vao_);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
        glActiveTexture(GL_TEXTURE0 + kCirrusUnit);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        glDepthMask(GL_TRUE);
    }

    /// @brief Render procedural terrain, then the airfield surface on top of it.
    /// @param view_proj World view-projection matrix
    /// @param eye_ned Eye position in NED [m], for range-based haze
    /// @param ctx Frame lighting, or null for standalone defaults
    void render_ground(const Mat4& view_proj, const math::Vector3& eye_ned, const FrameContext* ctx = nullptr) {
        if (!initialized_) return;

        // Terrain first, at true depth.
        terrain_shader_.use();
        upload_frame(terrain_shader_, ctx, eye_ned);
        terrain_shader_.set_mat4("uViewProj", view_proj);
        terrain_.draw();

        // Airfield markings are coplanar decals on the flat basin: bias them
        // toward the viewer so they never z-fight with the terrain mesh.
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-2.0f, -2.0f);

        ground_shader_.use();
        upload_frame(ground_shader_, ctx, eye_ned);
        ground_shader_.set_mat4("uViewProj", view_proj);

        glBindVertexArray(ground_vao_);
        glDrawArrays(GL_TRIANGLES, 0, ground_vertex_count_);
        glBindVertexArray(0);

        glDisable(GL_POLYGON_OFFSET_FILL);
        for (int unit : {FrameContext::kShadowUnit, CloudSettings::kMapUnit, TerrainTextures::kFirstUnit,
                         TerrainTextures::kFirstUnit + 1, TerrainTextures::kFirstUnit + 2}) {
            glActiveTexture(GL_TEXTURE0 + static_cast<GLenum>(unit));
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glActiveTexture(GL_TEXTURE0);
    }

    [[nodiscard]] GLsizei terrain_vertex_count() const noexcept {
        return terrain_.vertex_count();
    }

    bool is_initialized() const noexcept { return initialized_; }
};

} // namespace fastjet::graphics
