#pragma once

#include "fastjet/graphics/gl_common.hpp"
#include "fastjet/graphics/shader.hpp"
#include "fastjet/graphics/shader_library.hpp"
#include "fastjet/graphics/frame_context.hpp"
#include "fastjet/graphics/cockpit_geometry.hpp"
#include "fastjet/graphics/cockpit_gauges.hpp"
#include "fastjet/fdm/flight_state.hpp"
#include "fastjet/core/thread_pool.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <future>
#include <optional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "fastjet/graphics/cgltf.h"
#include "fastjet/graphics/stb_image.h"

namespace fastjet::graphics {

/// @brief Submesh classification within the F-16 airframe hierarchy
enum class F16PartType {
    AIRFRAME,
    CANOPY,
    COCKPIT,
    HUD,
    GEAR_UP,
    GEAR_DOWN,
    LIGHTS,
    RAILS,
    OTHER
};

/// @brief Where the model is placed relative to the flight model's CG.
///
/// The asset and the flight model disagree by about half a metre: seated on
/// the FDM's gear contact points, the model's own cockpit sits below the
/// pilot's design eye point. Each view therefore aligns the model to what it
/// can see: the chase view puts the wheels on the runway, the cockpit view
/// puts the canopy and HUD around the eye.
enum class ModelAlignment {
    WORLD,   ///< Wheels on the FDM contact points (chase view, shadows)
    COCKPIT  ///< Canopy and HUD around the design eye point
};

/// @brief Single drawable submesh primitive with OpenGL vertex & index buffers
struct GLBSubmesh {
    std::string name;
    F16PartType part_type = F16PartType::OTHER;

    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    GLsizei index_count = 0;
    GLenum index_type = GL_UNSIGNED_INT;

    // Metallic-roughness material (glTF 2.0)
    int base_texture = -1;      ///< Index into textures_, sRGB
    int mr_texture = -1;        ///< G = roughness, B = metallic, linear
    int normal_texture = -1;    ///< Tangent-space normal map, linear
    Color4 base_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
    float metallic_factor = 1.0f;
    float roughness_factor = 1.0f;
    std::array<float, 3> emissive{0.0f, 0.0f, 0.0f};
    bool is_transparent = false;
    int joint = -1;             ///< Articulated part: index of its joint transform (-1: rigid)
    int shown_by = -1;          ///< Drawn only while this joint has moved (-1: always)
    int hidden_by = -1;         ///< Hidden while this joint has moved (-1: never)

    /// Bounds in body axes, model units (before scale and placement).
    std::array<float, 3> bounds_min{0.0f, 0.0f, 0.0f};
    std::array<float, 3> bounds_max{0.0f, 0.0f, 0.0f};
};

/// @brief Axis-aligned bounds in body axes [m].
struct BodyBounds {
    math::Vector3 min{0.0, 0.0, 0.0};
    math::Vector3 max{0.0, 0.0, 0.0};
    bool valid = false;
};

/// @brief Where a model's own axes sit in the body frame, per alignment.
/// The defaults are the F-16C asset's (model units of 0.1 m).
struct ModelPlacement {
    float scale = 0.1f;          ///< Model units to metres
    float world_x = -3.50f;      ///< WORLD: X of the model origin [m]
    float world_z = -0.33f;      ///< WORLD: Z of the model origin [m]
    float cockpit_x = -4.035f;   ///< COCKPIT: X of the model origin [m]
    float cockpit_z = 0.175f;    ///< COCKPIT: Z of the model origin [m]
};

/// @brief One part of a procedurally built airframe: a triangle soup of
/// positions and normals (6 floats per vertex, body axes, metres) and its
/// glTF metallic-roughness material.
struct ProceduralPart {
    F16PartType part_type = F16PartType::AIRFRAME;
    std::vector<float> vertices;   ///< px py pz nx ny nz per vertex
    Color4 base{0.5f, 0.5f, 0.5f, 1.0f};
    float metallic = 0.0f;
    float roughness = 0.6f;
    bool transparent = false;
    std::array<float, 3> emissive{0.0f, 0.0f, 0.0f};
    /// Moving part (door, swivelling nozzle segment): drawn through joint
    /// transform `joint` of the model, set each frame (ModelGLB::set_joints).
    /// -1 for the rigid airframe.
    int joint = -1;
    /// Drawn only while this joint has moved (-1: always): a door laid over
    /// an asset's closed skin shows only once it opens.
    int shown_by = -1;
};

/// @brief A round region of an asset's skin, in body axes (world alignment)
/// [m]: within `radius` of the X-parallel axis through (y, z), aft of
/// `x_max`. Its triangles load as a part of their own, hidden while joint
/// `hidden_by` has moved: a fixed nozzle that a swivelling one replaces.
struct AssetCutaway {
    float x_max = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float radius = 0.0f;
    int hidden_by = -1;
};

/// @brief Per-frame transforms of an airframe's moving parts, in the
/// airframe's own axes (each maps the part from where it was built to where
/// it is now). Unused entries are identity.
struct JointPose {
    static constexpr int kMax = 16;
    std::array<Mat4, kMax> m{};
    int count = 0;
};

/// @brief glTF/GLB airframe loader and physically based renderer.
///
/// Shading is the glTF metallic-roughness model: base colour (sRGB), packed
/// roughness/metallic and tangent-space normal maps, a GGX sun term with the
/// sun shadow map, and an image-based term that reflects the actual sky and
/// ground. Canopy glass uses dual-source blending, so it can both reflect
/// (added light) and tint what is behind it (per-channel transmittance).
class ModelGLB {
public:
    struct Prepared;

private:
    std::vector<GLBSubmesh> submeshes_;
    std::vector<GLuint> textures_;
    ShaderProgram shader_;
    ShaderProgram depth_shader_;

    bool initialized_ = false;
    bool loaded_ = false;
    bool cockpit_interior_ = true;
    /// Moving parts' transforms this draw, and the uniforms they are
    /// composed with (the model and normal matrices of the whole airframe).
    JointPose joints_{};
    std::array<bool, JointPose::kMax> moved_{};  ///< Joint i is away from its rest pose
    Mat4 base_model_ = Mat4::identity();
    Mat4 base_rot_ = Mat4::identity();
    bool depth_pass_ = false;          ///< Drawing with the depth-only shader (uLightMVP)
    int posed_ = -1;                   ///< Joint the bound model matrix carries (-1: rigid)
    ModelPlacement placement_{};  ///< F-16C asset unless a load says otherwise
    std::optional<AssetCutaway> cutaway_;  ///< Skin region split off at upload
    std::future<std::unique_ptr<Prepared>> prefetch_;  ///< Background prepare()
    std::string prefetch_path_;

    /// Scale from GLB model units to metres (10 units = 1 metre)
    static constexpr float MODEL_SCALE = 0.1f;

    /// Body-frame placement of the model origin per alignment [m]. X centres
    /// the airframe on the CG; Z seats it (see ModelAlignment).
    static constexpr float WORLD_OFFSET_X   = -3.50f;
    static constexpr float WORLD_OFFSET_Z   = -0.33f; // wheels (1.89 m) onto FDM contact (1.56 m)
    static constexpr float COCKPIT_OFFSET_X = -4.035f; // model HUD onto the cockpit combiner
    static constexpr float COCKPIT_OFFSET_Z = 0.175f;

    /// Landing light output when the gear is down (HDR, blooms at night and dusk).
    static constexpr float LANDING_LIGHT_RADIANCE = 12.0f;

    /// Gold-coated canopy: a faint golden reflection and a warm tint.
    static constexpr std::array<float, 3> CANOPY_F0{0.090f, 0.078f, 0.045f};

    /// Vertex stream: position(3), normal(3), uv(2), tangent(4).
    static constexpr int kFloatsPerVertex = 12;

public:
    /// @brief The part a node belongs to, from its name and its parent's.
    /// Any case: the series' assets spell them "F-16-landingOn_6",
    /// "LandingOn_6", "LandingLight_7", "WeaponRails_8"...
    [[nodiscard]] static F16PartType classify_node(std::string name) {
        for (char& ch : name) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        auto has = [&](const char* s) { return name.find(s) != std::string::npos; };
        if (has("canopy")) return F16PartType::CANOPY;
        if (has("cockpit")) return F16PartType::COCKPIT;
        if (has("hud")) return F16PartType::HUD;
        if (has("landingoff")) return F16PartType::GEAR_UP;
        if (has("landingonlight") || has("landinglight")) return F16PartType::LIGHTS;
        if (has("landingon")) return F16PartType::GEAR_DOWN;
        if (has("rails")) return F16PartType::RAILS;
        return F16PartType::AIRFRAME;
    }

private:
    /// @brief Model axes to body axes.
    ///
    /// The asset is authored X forward, Y up, Z along the span; the body frame
    /// is X forward, Y right, Z down. Without this rotation the airframe drew
    /// rolled 90 degrees: fin to one side, gear out to the other.
    static void model_to_body(float& /*x: forward in both*/, float& y, float& z) noexcept {
        const float my = y;
        const float mz = z;
        y = mz;
        z = -my;
    }

    bool init_shaders() {
        const char* vert_src = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec3 aNormal;
            layout (location = 2) in vec2 aUV;
            layout (location = 3) in vec4 aTangent;

            uniform mat4 uModel;         // model -> shading frame (eye-relative or body)
            uniform mat4 uViewProj;      // shading frame -> clip
            uniform mat3 uNormalMatrix;

            out vec3 vPos;
            out vec3 vNormal;
            out vec4 vTangent;
            out vec2 vUV;

            void main() {
                vec4 p = uModel * vec4(aPos, 1.0);
                vPos = p.xyz;
                vNormal = uNormalMatrix * aNormal;
                vTangent = vec4(uNormalMatrix * aTangent.xyz, aTangent.w);
                vUV = aUV;
                gl_Position = uViewProj * p;
            }
        )";

        const std::string frag_src = std::string(R"(
            #version 330 core
            in vec3 vPos;
            in vec3 vNormal;
            in vec4 vTangent;
            in vec2 vUV;
            layout (location = 0, index = 0) out vec4 FragColor;
            layout (location = 0, index = 1) out vec4 FragTransmit;

            uniform sampler2D uBaseTex;
            uniform sampler2D uMrTex;
            uniform sampler2D uNormalTex;
            uniform int   uHasBase;
            uniform int   uHasMr;
            uniform int   uHasNormal;
            uniform vec4  uBaseFactor;
            uniform float uMetallic;
            uniform float uRoughness;
            uniform vec3  uEmissive;
            uniform int   uGlass;          // 1: dual-source glass
            uniform vec3  uGlassF0;
            uniform vec3  uGlassTint;
            uniform vec3  uFrameOrigin;    // world position of the shading frame origin
            uniform float uEyeAltitude;    // [m]
            uniform int   uApplyHaze;
        )") + glsl::COMMON + glsl::OUTPUT + glsl::ATMOSPHERE + glsl::LIGHTING + glsl::SHADOW + glsl::CLOUDS + R"(
            void main() {
                vec4 base = uBaseFactor;
                if (uHasBase == 1) base *= texture(uBaseTex, vUV);
                float metal = uMetallic;
                float rough = uRoughness;
                if (uHasMr == 1) {
                    vec4 mr = texture(uMrTex, vUV);
                    rough *= mr.g;
                    metal *= mr.b;
                }
                rough = clamp(rough, 0.04, 1.0);

                // The eye is at the origin of the eye-relative frame; in the
                // body frame (cockpit) the view direction comes from the eye
                // uniform instead.
                vec3 eye = uFrameIsBody == 1 ? uFrameOrigin : vec3(0.0);
                vec3 v = normalize(eye - vPos);

                // Double-sided materials: shade the side of the triangle that
                // faces the camera, whatever the winding.
                vec3 n = normalize(vNormal);
                vec3 fn = normalize(cross(dFdx(vPos), dFdy(vPos)));
                if (dot(fn, v) < 0.0) fn = -fn;
                if (dot(n, fn) < 0.0) n = -n;

                if (uHasNormal == 1 && dot(vTangent.xyz, vTangent.xyz) > 1e-8) {
                    vec3 t = normalize(vTangent.xyz - n * dot(n, vTangent.xyz));
                    vec3 b = cross(n, t) * (vTangent.w < 0.0 ? -1.0 : 1.0);
                    vec3 tn = texture(uNormalTex, vUV).xyz * 2.0 - 1.0;
                    n = normalize(t * tn.x + b * tn.y + n * max(tn.z, 0.05));
                }

                vec3 world = uFrameIsBody == 1 ? vec3(0.0) : vPos + uFrameOrigin;
                float sun_vis = sun_shadow(vPos, n, gl_FragCoord.xy);
                if (uFrameIsBody == 0) sun_vis *= cloud_shadow(world, uSunDir);

                if (uGlass == 1) {
                    float ndv = max(dot(n, v), 1e-3);
                    vec3 F = f_schlick(uGlassF0, ndv);
                    vec3 r = reflect(-v, n);
                    vec3 refl = F * environment_radiance(r, 0.03, uEyeAltitude);
                    // Sun glint off the canopy.
                    vec3 h = normalize(v + uSunDir);
                    float a = 0.03 * 0.03;
                    float ndl = max(dot(n, uSunDir), 0.0);
                    refl += f_schlick(uGlassF0, max(dot(v, h), 0.0)) * d_ggx(max(dot(n, h), 0.0), a)
                          * v_smith_ggx(ndv, ndl, a) * PI * sun_radiance() * ndl * sun_vis;
                    FragColor = scene_output(refl, 1.0);
                    FragTransmit = vec4((1.0 - F) * uGlassTint, 1.0);
                    return;
                }

                vec3 lit = sun_brdf(base.rgb, metal, rough, n, v, uSunDir) * sun_vis
                         + ambient_brdf(base.rgb, metal, rough, n, v, uEyeAltitude, 1.0)
                         + uEmissive;
                if (uApplyHaze == 1) lit = apply_haze(lit, world, uFrameOrigin, uSunDir);
                FragColor = scene_output(lit, base.a);
                FragTransmit = vec4(0.0);
            }
        )";

        if (!shader_.init_from_source(vert_src, frag_src.c_str())) return false;

        const char* depth_vert = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            uniform mat4 uLightMVP;
            void main() { gl_Position = uLightMVP * vec4(aPos, 1.0); }
        )";
        const char* depth_frag = R"(
            #version 330 core
            void main() {}
        )";
        if (!depth_shader_.init_from_source(depth_vert, depth_frag)) return false;

        // Fixed texture units: material maps on 0-2, shadow map on its own unit
        // so the sampler types never alias.
        shader_.use();
        shader_.set_int("uBaseTex", 0);
        shader_.set_int("uMrTex", 1);
        shader_.set_int("uNormalTex", 2);
        shader_.set_int("uShadowMap", FrameContext::kShadowUnit);
        shader_.set_int("uCloudMap", CloudSettings::kMapUnit);
        glUseProgram(0);
        return true;
    }

public:
    ModelGLB() = default;
    ~ModelGLB() { destroy(); }

    ModelGLB(const ModelGLB&) = delete;
    ModelGLB& operator=(const ModelGLB&) = delete;

    /// @brief The CPU half of a load: the parsed file and its decoded images.
    /// Built without touching GL, so it can be prepared on a worker thread.
    struct Prepared {
        struct Image {
            unsigned char* pixels = nullptr;  ///< RGBA8, owned (stbi)
            int width = 0;
            int height = 0;
        };
        cgltf_data* data = nullptr;
        std::string path;                     ///< As requested
        std::vector<Image> images;

        Prepared() = default;
        Prepared(const Prepared&) = delete;
        Prepared& operator=(const Prepared&) = delete;
        ~Prepared() {
            for (Image& im : images) {
                if (im.pixels) stbi_image_free(im.pixels);
            }
            if (data) cgltf_free(data);
        }
    };

    /// @brief Parses a glTF file and decodes its images, all images at once
    /// across the thread pool. Makes no GL calls; safe on any thread.
    /// @return nullptr if the file cannot be read
    [[nodiscard]] static std::unique_ptr<Prepared> prepare(const std::string& filepath) {
        cgltf_options options = {};
        cgltf_data* data = nullptr;

        const std::vector<std::string> candidates = {
            filepath,
            "../" + filepath,
            "../../" + filepath
        };
        std::string actual_path;
        cgltf_result result = cgltf_result_file_not_found;

        for (const auto& p : candidates) {
            result = cgltf_parse_file(&options, p.c_str(), &data);
            if (result == cgltf_result_success) {
                actual_path = p;
                break;
            }
        }

        if (result != cgltf_result_success) {
            std::cerr << "[ModelGLB] Failed to parse " << filepath << " (error " << result << ")\n";
            return nullptr;
        }

        result = cgltf_load_buffers(&options, data, actual_path.c_str());
        if (result != cgltf_result_success) {
            std::cerr << "[ModelGLB] Failed to load buffers for " << actual_path << "\n";
            cgltf_free(data);
            return nullptr;
        }

        auto prepared = std::make_unique<Prepared>();
        prepared->data = data;
        prepared->path = filepath;
        prepared->images.resize(data->images_count);
        // PNG decoding dominates the load (three 4096 x 4096 maps), and each
        // image decodes independently.
        core::ThreadPool::shared().parallel_for(0, static_cast<int>(data->images_count), [&](int i) {
            const cgltf_image& img = data->images[i];
            if (!img.buffer_view) return;
            const uint8_t* raw = reinterpret_cast<const uint8_t*>(img.buffer_view->buffer->data) + img.buffer_view->offset;
            Prepared::Image& out = prepared->images[static_cast<size_t>(i)];
            int channels = 0;
            out.pixels = stbi_load_from_memory(raw, static_cast<int>(img.buffer_view->size), &out.width, &out.height,
                                               &channels, 4);
            if (!out.pixels) {
                std::cerr << "[ModelGLB] Failed to decode image " << i << ": " << stbi_failure_reason() << "\n";
            }
        });
        return prepared;
    }

    /// @brief Starts preparing `filepath` in the background; a later load()
    /// of the same path picks up the result instead of reading the file.
    void prefetch(const std::string& filepath) {
        prefetch_path_ = filepath;
        prefetch_ = core::ThreadPool::shared().submit([filepath] { return prepare(filepath); });
    }

    /// @brief True once a prefetch() has finished preparing, so a load()
    /// of its path only has the GPU upload left to do.
    [[nodiscard]] bool prefetch_ready() const {
        return prefetch_.valid() && prefetch_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    /// @brief Loads an airframe asset other than the F-16C's: @p placement
    /// puts it on the CG, and @p cutaway (if any) splits a region of its
    /// skin off as a part of its own.
    bool load(const std::string& filepath, const ModelPlacement& placement,
              const std::optional<AssetCutaway>& cutaway = std::nullopt) {
        placement_ = placement;
        cutaway_ = cutaway;
        return load(filepath);
    }

    /// @brief Load binary glTF model from disk and initialize OpenGL resources
    bool load(const std::string& filepath) {
        std::unique_ptr<Prepared> prepared;
        if (prefetch_.valid() && prefetch_path_ == filepath) {
            prepared = prefetch_.get();
        } else {
            prepared = prepare(filepath);
        }
        if (!prepared) return false;
        return upload(*prepared);
    }

    /// @brief Builds the model from procedural parts instead of a file.
    /// The parts are authored in metres in their own axes; @p placement
    /// puts those axes in the body frame for each alignment.
    bool load_procedural(const std::vector<ProceduralPart>& parts, const ModelPlacement& placement) {
        placement_ = placement;
        append_parts(parts, 1.0f, 0.0f, 0.0f);
        loaded_ = true;
        return true;
    }

    /// @brief Adds procedural parts, authored in body axes (world alignment,
    /// metres), to a loaded asset: the moving parts its own mesh lacks.
    void add_body_parts(const std::vector<ProceduralPart>& parts) {
        const ModelPlacement& p = placement_;
        append_parts(parts, 1.0f / p.scale, p.world_x, p.world_z);
    }

private:
    /// @brief Procedural parts into submeshes, each position taken from body
    /// axes into this model's own: (p - (offset_x, 0, offset_z)) * k.
    void append_parts(const std::vector<ProceduralPart>& parts, float k, float offset_x, float offset_z) {
        for (const ProceduralPart& part : parts) {
            const size_t n = part.vertices.size() / 6;
            if (n < 3) continue;
            std::array<float, 3> bmin{1e30f, 1e30f, 1e30f};
            std::array<float, 3> bmax{-1e30f, -1e30f, -1e30f};
            std::vector<float> vertex_data;
            vertex_data.reserve(n * kFloatsPerVertex);
            std::vector<uint32_t> indices(n);
            for (size_t v = 0; v < n; ++v) {
                const float* src = &part.vertices[v * 6];
                const float pos[3] = {(src[0] - offset_x) * k, src[1] * k, (src[2] - offset_z) * k};
                for (size_t c = 0; c < 3; ++c) {
                    bmin[c] = (std::min)(bmin[c], pos[c]);
                    bmax[c] = (std::max)(bmax[c], pos[c]);
                }
                const float vert[kFloatsPerVertex] = {pos[0], pos[1], pos[2], src[3], src[4], src[5],
                                                      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   1.0f};
                vertex_data.insert(vertex_data.end(), vert, vert + kFloatsPerVertex);
                indices[v] = static_cast<uint32_t>(v);
            }
            GLBSubmesh sm = make_submesh(vertex_data, indices);
            sm.name = "procedural";
            sm.part_type = part.part_type;
            sm.bounds_min = bmin;
            sm.bounds_max = bmax;
            sm.base_color_factor = part.base;
            sm.metallic_factor = part.metallic;
            sm.roughness_factor = part.roughness;
            sm.emissive = part.emissive;
            sm.is_transparent = part.transparent;
            sm.joint = part.joint < JointPose::kMax ? part.joint : -1;
            sm.shown_by = part.shown_by < JointPose::kMax ? part.shown_by : -1;
            submeshes_.push_back(sm);
        }
    }

    /// @brief True if the triangle (body-axis vertices, model units) lies
    /// wholly inside the cutaway region.
    [[nodiscard]] bool in_cutaway(const float* a, const float* b, const float* c) const noexcept {
        if (!cutaway_) return false;
        const AssetCutaway& r = *cutaway_;
        const ModelPlacement& p = placement_;
        for (const float* v : {a, b, c}) {
            const float x = v[0] * p.scale + p.world_x;
            const float y = v[1] * p.scale - r.y;
            const float z = v[2] * p.scale + p.world_z - r.z;
            if (x > r.x_max || y * y + z * z > r.radius * r.radius) return false;
        }
        return true;
    }
    /// @brief Vertex array, vertex and index buffers for one submesh
    /// (kFloatsPerVertex floats per vertex).
    static GLBSubmesh make_submesh(const std::vector<float>& vertex_data, const std::vector<uint32_t>& indices) {
        GLuint vao = 0, vbo = 0, ebo = 0;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_data.size() * sizeof(float)),
                     vertex_data.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
                     indices.data(), GL_STATIC_DRAW);

        constexpr GLsizei stride = kFloatsPerVertex * sizeof(float);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(8 * sizeof(float)));
        glBindVertexArray(0);

        GLBSubmesh sm;
        sm.vao = vao;
        sm.vbo = vbo;
        sm.ebo = ebo;
        sm.index_count = static_cast<GLsizei>(indices.size());
        sm.index_type = GL_UNSIGNED_INT;
        return sm;
    }

    /// @brief The GL half of a load: textures, buffers and submeshes.
    bool upload(const Prepared& prepared) {
        cgltf_data* data = prepared.data;

        auto image_index = [&](const cgltf_texture_view& view) -> int {
            if (!view.texture || !view.texture->image) return -1;
            return static_cast<int>(view.texture->image - data->images);
        };

        // 1. Colour textures are sRGB-encoded and must be linearised when
        //    sampled; data textures (roughness/metal, normals) must not be.
        std::vector<bool> is_color(data->images_count, false);
        for (cgltf_size m = 0; m < data->materials_count; ++m) {
            const cgltf_material& mat = data->materials[m];
            if (mat.has_pbr_metallic_roughness) {
                const int bi = image_index(mat.pbr_metallic_roughness.base_color_texture);
                if (bi >= 0) is_color[static_cast<size_t>(bi)] = true;
            }
            const int ei = image_index(mat.emissive_texture);
            if (ei >= 0) is_color[static_cast<size_t>(ei)] = true;
        }

        // 4x keeps the livery sharp at grazing angles without the texture
        // bandwidth of 8x on 4K maps.
        constexpr float kAnisotropy = 4.0f;

        textures_.resize(data->images_count, 0);
        for (cgltf_size i = 0; i < data->images_count; ++i) {
            const Prepared::Image& decoded = prepared.images[i];
            if (!decoded.pixels) continue;
            const int width = decoded.width;
            const int height = decoded.height;
            const unsigned char* pixels = decoded.pixels;

            GLuint tex = 0;
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            apply_texture_anisotropy(kAnisotropy);
            const GLint internal = is_color[i] ? GL_SRGB8_ALPHA8 : GL_RGBA8;
            glTexImage2D(GL_TEXTURE_2D, 0, internal, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
            glGenerateMipmap(GL_TEXTURE_2D);

            textures_[i] = tex;
        }
        glBindTexture(GL_TEXTURE_2D, 0);

        // 2. Iterate nodes and extract mesh primitives
        for (cgltf_size ni = 0; ni < data->nodes_count; ++ni) {
            const cgltf_node& node = data->nodes[ni];
            if (!node.mesh) continue;

            // Determine part classification from node or parent node name
            std::string part_name = node.name ? node.name : "";
            if (node.parent && node.parent->name) {
                part_name += " " + std::string(node.parent->name);
            }
            const F16PartType ptype = classify_node(part_name);

            // Composite world transform of this node
            float m[16];
            cgltf_node_transform_world(&node, m);

            for (cgltf_size pi = 0; pi < node.mesh->primitives_count; ++pi) {
                const cgltf_primitive& prim = node.mesh->primitives[pi];
                if (prim.type != cgltf_primitive_type_triangles) continue;

                const cgltf_accessor* pos_acc = nullptr;
                const cgltf_accessor* norm_acc = nullptr;
                const cgltf_accessor* uv_acc = nullptr;
                const cgltf_accessor* tan_acc = nullptr;
                for (cgltf_size ai = 0; ai < prim.attributes_count; ++ai) {
                    const cgltf_attribute& attr = prim.attributes[ai];
                    if (attr.type == cgltf_attribute_type_position) pos_acc = attr.data;
                    else if (attr.type == cgltf_attribute_type_normal) norm_acc = attr.data;
                    else if (attr.type == cgltf_attribute_type_tangent) tan_acc = attr.data;
                    else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uv_acc = attr.data;
                }
                if (!pos_acc) continue;
                const cgltf_size num_verts = pos_acc->count;

                std::vector<float> positions(num_verts * 3);
                cgltf_accessor_unpack_floats(pos_acc, positions.data(), positions.size());
                std::vector<float> normals;
                if (norm_acc) {
                    normals.resize(num_verts * 3);
                    cgltf_accessor_unpack_floats(norm_acc, normals.data(), normals.size());
                }
                std::vector<float> uvs;
                if (uv_acc) {
                    uvs.resize(num_verts * 2);
                    cgltf_accessor_unpack_floats(uv_acc, uvs.data(), uvs.size());
                }
                std::vector<float> tangents;
                if (tan_acc && tan_acc->count == num_verts) {
                    tangents.resize(num_verts * 4);
                    cgltf_accessor_unpack_floats(tan_acc, tangents.data(), tangents.size());
                }

                auto xform_dir = [&](float x, float y, float z, float& ox, float& oy, float& oz) {
                    ox = m[0] * x + m[4] * y + m[8] * z;
                    oy = m[1] * x + m[5] * y + m[9] * z;
                    oz = m[2] * x + m[6] * y + m[10] * z;
                    const float len = std::sqrt(ox * ox + oy * oy + oz * oz);
                    if (len > 1e-6f) { ox /= len; oy /= len; oz /= len; }
                    model_to_body(ox, oy, oz);
                };

                std::array<float, 3> bmin{1e30f, 1e30f, 1e30f};
                std::array<float, 3> bmax{-1e30f, -1e30f, -1e30f};
                std::vector<float> vertex_data;
                vertex_data.reserve(num_verts * kFloatsPerVertex);
                for (cgltf_size v = 0; v < num_verts; ++v) {
                    const float px = positions[v * 3 + 0];
                    const float py = positions[v * 3 + 1];
                    const float pz = positions[v * 3 + 2];
                    float tx = m[0] * px + m[4] * py + m[8] * pz + m[12];
                    float ty = m[1] * px + m[5] * py + m[9] * pz + m[13];
                    float tz = m[2] * px + m[6] * py + m[10] * pz + m[14];
                    model_to_body(tx, ty, tz);
                    const float p[3] = {tx, ty, tz};
                    for (int k = 0; k < 3; ++k) {
                        bmin[static_cast<size_t>(k)] = (std::min)(bmin[static_cast<size_t>(k)], p[k]);
                        bmax[static_cast<size_t>(k)] = (std::max)(bmax[static_cast<size_t>(k)], p[k]);
                    }

                    float nx = 0.0f, ny = 0.0f, nz = -1.0f;
                    if (!normals.empty()) xform_dir(normals[v * 3], normals[v * 3 + 1], normals[v * 3 + 2], nx, ny, nz);

                    float gx = 0.0f, gy = 0.0f, gz = 0.0f, gw = 1.0f;
                    if (!tangents.empty()) {
                        xform_dir(tangents[v * 4], tangents[v * 4 + 1], tangents[v * 4 + 2], gx, gy, gz);
                        gw = tangents[v * 4 + 3];
                    }

                    const float u = uvs.empty() ? 0.0f : uvs[v * 2 + 0];
                    const float w = uvs.empty() ? 0.0f : uvs[v * 2 + 1];
                    const float vert[kFloatsPerVertex] = {tx, ty, tz, nx, ny, nz, u, w, gx, gy, gz, gw};
                    vertex_data.insert(vertex_data.end(), vert, vert + kFloatsPerVertex);
                }

                std::vector<uint32_t> indices;
                if (prim.indices) {
                    indices.resize(prim.indices->count);
                    cgltf_accessor_unpack_indices(prim.indices, indices.data(), sizeof(uint32_t), indices.size());
                } else {
                    indices.resize(num_verts);
                    for (uint32_t vi = 0; vi < num_verts; ++vi) indices[vi] = vi;
                }

                // The cutaway's triangles become a part of their own.
                std::vector<uint32_t> cut;
                if (cutaway_ && ptype == F16PartType::AIRFRAME) {
                    std::vector<uint32_t> kept;
                    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
                        const float* a = &vertex_data[indices[t] * kFloatsPerVertex];
                        const float* b = &vertex_data[indices[t + 1] * kFloatsPerVertex];
                        const float* c = &vertex_data[indices[t + 2] * kFloatsPerVertex];
                        auto& to = in_cutaway(a, b, c) ? cut : kept;
                        to.insert(to.end(), {indices[t], indices[t + 1], indices[t + 2]});
                    }
                    indices = std::move(kept);
                }

                GLBSubmesh sm = make_submesh(vertex_data, indices);
                sm.name = part_name;
                sm.part_type = ptype;
                sm.bounds_min = bmin;
                sm.bounds_max = bmax;

                if (const cgltf_material* mat = prim.material) {
                    if (mat->has_pbr_metallic_roughness) {
                        const auto& pbr = mat->pbr_metallic_roughness;
                        sm.base_texture = image_index(pbr.base_color_texture);
                        sm.mr_texture = image_index(pbr.metallic_roughness_texture);
                        sm.base_color_factor = Color4{pbr.base_color_factor[0], pbr.base_color_factor[1],
                                                      pbr.base_color_factor[2], pbr.base_color_factor[3]};
                        sm.metallic_factor = pbr.metallic_factor;
                        sm.roughness_factor = pbr.roughness_factor;
                    }
                    sm.normal_texture = image_index(mat->normal_texture);
                    sm.emissive = {mat->emissive_factor[0], mat->emissive_factor[1], mat->emissive_factor[2]};
                    if (mat->alpha_mode == cgltf_alpha_mode_blend) sm.is_transparent = true;
                }
                // The canopy is glass whatever its material says, and the
                // asset's HUD plate is the combiner glass, not a green panel.
                if (ptype == F16PartType::CANOPY || ptype == F16PartType::HUD) sm.is_transparent = true;

                if (!cut.empty()) {
                    GLBSubmesh part = make_submesh(vertex_data, cut);
                    part.name = part_name + " cutaway";
                    part.part_type = ptype;
                    part.bounds_min = bmin;
                    part.bounds_max = bmax;
                    part.base_texture = sm.base_texture;
                    part.mr_texture = sm.mr_texture;
                    part.normal_texture = sm.normal_texture;
                    part.base_color_factor = sm.base_color_factor;
                    part.metallic_factor = sm.metallic_factor;
                    part.roughness_factor = sm.roughness_factor;
                    part.emissive = sm.emissive;
                    part.is_transparent = sm.is_transparent;
                    part.hidden_by = cutaway_->hidden_by < JointPose::kMax ? cutaway_->hidden_by : -1;
                    submeshes_.push_back(part);
                }
                submeshes_.push_back(sm);
            }
        }

        loaded_ = true;
        std::cout << "[ModelGLB] Loaded " << submeshes_.size() << " submeshes and "
                  << textures_.size() << " textures successfully from " << prepared.path << "\n";
        return true;
    }

public:
    bool init() {
        if (initialized_) return true;
        if (!init_shaders()) return false;
        initialized_ = true;
        return true;
    }

    void destroy() noexcept {
        for (auto& sm : submeshes_) {
            if (sm.vao) glDeleteVertexArrays(1, &sm.vao);
            if (sm.vbo) glDeleteBuffers(1, &sm.vbo);
            if (sm.ebo) glDeleteBuffers(1, &sm.ebo);
            sm.vao = sm.vbo = sm.ebo = 0;
        }
        submeshes_.clear();

        for (auto tex : textures_) {
            if (tex) glDeleteTextures(1, &tex);
        }
        textures_.clear();

        shader_.destroy();
        depth_shader_.destroy();
        initialized_ = false;
        loaded_ = false;
    }

    bool is_loaded() const noexcept { return loaded_; }

    /// @brief Poses the moving parts for the next draws (doors, nozzle). The
    /// model is shared by every jet of its type: set before each one.
    void set_joints(const JointPose& pose) noexcept {
        // Joints move parts in body axes (world alignment). An asset's own
        // axes are those scaled and shifted, so each joint is carried into
        // them; on a procedural model (placed at scale 1, no offset) both
        // matrices are the identity.
        const ModelPlacement& p = placement_;
        const Mat4 to_body = placed_matrix(ModelAlignment::WORLD);
        const Mat4 to_model = Mat4::scale(1.0f / p.scale, 1.0f / p.scale, 1.0f / p.scale) *
                              Mat4::translate(-p.world_x, 0.0f, -p.world_z);
        joints_.count = pose.count;
        for (int i = 0; i < JointPose::kMax; ++i) {
            const auto k = static_cast<size_t>(i);
            if (i >= pose.count) {
                joints_.m[k] = Mat4::identity();
                moved_[k] = false;
                continue;
            }
            joints_.m[k] = to_model * pose.m[k] * to_body;
            float off = 0.0f;
            const Mat4 id = Mat4::identity();
            for (size_t e = 0; e < 16; ++e) off = (std::max)(off, std::fabs(pose.m[k].m[e] - id.m[e]));
            moved_[k] = off > 1e-5f;
        }
    }
    [[nodiscard]] size_t submesh_count() const noexcept { return submeshes_.size(); }

    /// @brief Body-frame model matrix (model units -> metres, placed on the CG).
    [[nodiscard]] static Mat4 body_matrix(ModelAlignment align) noexcept {
        const bool world = align == ModelAlignment::WORLD;
        return Mat4::translate(world ? WORLD_OFFSET_X : COCKPIT_OFFSET_X, 0.0f,
                               world ? WORLD_OFFSET_Z : COCKPIT_OFFSET_Z) *
               Mat4::scale(MODEL_SCALE, MODEL_SCALE, MODEL_SCALE);
    }

    /// @brief This model's body-frame matrix (its own placement; the F-16C
    /// asset's is body_matrix()).
    [[nodiscard]] Mat4 placed_matrix(ModelAlignment align) const noexcept {
        const bool world = align == ModelAlignment::WORLD;
        const ModelPlacement& p = placement_;
        return Mat4::translate(world ? p.world_x : p.cockpit_x, 0.0f, world ? p.world_z : p.cockpit_z) *
               Mat4::scale(p.scale, p.scale, p.scale);
    }

    /// @brief Body-to-NED rotation of the aircraft as a Mat4.
    [[nodiscard]] static Mat4 attitude_matrix(const fdm::FlightState& state) noexcept {
        const math::Matrix3x3 c_nb = state.q_att.to_dcm_body_to_ned();
        Mat4 r = Mat4::identity();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) r(row, col) = static_cast<float>(c_nb(row, col));
        }
        return r;
    }

    /// @brief Body axes into a frame whose origin is `frame_origin` (NED).
    /// The CG offset is computed in double precision before narrowing.
    [[nodiscard]] static Mat4 relative_matrix_body(const fdm::FlightState& state,
                                                   const math::Vector3& frame_origin) noexcept {
        const math::Vector3 d = state.pos_ned - frame_origin;
        return Mat4::translate(static_cast<float>(d.x), static_cast<float>(d.y), static_cast<float>(d.z)) *
               attitude_matrix(state);
    }

    /// @brief Model matrix into a frame whose origin is `frame_origin` (NED).
    [[nodiscard]] static Mat4 relative_matrix(const fdm::FlightState& state, const math::Vector3& frame_origin,
                                              ModelAlignment align) noexcept {
        return relative_matrix_body(state, frame_origin) * body_matrix(align);
    }

    /// @brief The asset cockpit's two MFD screens in body axes (cockpit
    /// alignment), with the instrument-atlas cells to show on them.
    ///
    /// Measured by mapping the screens' texture rectangles back through the
    /// cockpit mesh. The asset's painted displays are static, so the live
    /// pages are laid 2 mm proud of them.
    /// @param row_v0 Atlas V of the MFD row's lower edge (its top is V = 1).
    [[nodiscard]] static std::vector<ScreenQuad> cockpit_screens(float row_v0 = 0.5f) {
        constexpr float kProud = 0.002f;
        // Screen face normal (toward the pilot): the panel leans back ~13 deg.
        constexpr float kNx = -0.974f;
        constexpr float kNz = -0.225f;
        auto quad = [&](float y_left, float y_right, float u0, float u1) {
            ScreenQuad q;
            const float x_top = 2.4966f, z_top = -0.4585f; // upper edge
            const float x_bot = 2.4736f, z_bot = -0.3595f; // lower edge
            q.corners = {{{x_top + kNx * kProud, y_left, z_top + kNz * kProud},
                          {x_top + kNx * kProud, y_right, z_top + kNz * kProud},
                          {x_bot + kNx * kProud, y_right, z_bot + kNz * kProud},
                          {x_bot + kNx * kProud, y_left, z_bot + kNz * kProud}}};
            q.u0 = u0;
            q.u1 = u1;
            q.v0 = row_v0;
            q.v1 = 1.0f;
            return q;
        };
        return {quad(-0.2214f, -0.1207f, 0.0f, 0.5f), quad(0.1187f, 0.2235f, 0.5f, 1.0f)};
    }

    /// @brief Live faces for the asset cockpit's painted instruments, in body
    /// axes (cockpit alignment), each showing its CockpitGauges atlas cell.
    ///
    /// Every entry was measured like cockpit_screens(): the painted dial's
    /// texture rectangle mapped back through the cockpit mesh, then turned so
    /// the face reads upright from the seat. Quads sit 1.5 mm proud of the
    /// paint; the round dials' corners are transparent, so the painted
    /// bezels and screws around them still show.
    [[nodiscard]] static std::vector<ScreenQuad> cockpit_gauges() {
        using Corners = std::array<std::array<float, 3>, 4>;
        static const std::pair<Gauge, Corners> kPlacements[] = {
            {Gauge::ASI, {{{2.4507f, -0.0805f, -0.4130f}, {2.4507f, -0.0050f, -0.4130f}, {2.4188f, -0.0050f, -0.3447f}, {2.4188f, -0.0805f, -0.3447f}}}},
            {Gauge::ALT, {{{2.4505f, 0.0045f, -0.4126f}, {2.4505f, 0.0800f, -0.4126f}, {2.4186f, 0.0800f, -0.3442f}, {2.4186f, 0.0045f, -0.3442f}}}},
            {Gauge::ADI, {{{2.4132f, -0.0278f, -0.3325f}, {2.4132f, 0.0338f, -0.3325f}, {2.3872f, 0.0338f, -0.2767f}, {2.3872f, -0.0278f, -0.2767f}}}},
            {Gauge::HSI, {{{2.3752f, -0.0324f, -0.2573f}, {2.3752f, 0.0392f, -0.2573f}, {2.3276f, 0.0388f, -0.2024f}, {2.3276f, -0.0328f, -0.2024f}}}},
            {Gauge::VVI, {{{2.4096f, 0.0537f, -0.3249f}, {2.4096f, 0.0756f, -0.3249f}, {2.3882f, 0.0756f, -0.2790f}, {2.3882f, 0.0537f, -0.2790f}}}},
            {Gauge::AOA, {{{2.4096f, -0.0795f, -0.3249f}, {2.4096f, -0.0577f, -0.3249f}, {2.3882f, -0.0577f, -0.2790f}, {2.3882f, -0.0795f, -0.2790f}}}},
            {Gauge::FUEL_QTY, {{{2.5126f, 0.1264f, -0.5348f}, {2.5126f, 0.1681f, -0.5348f}, {2.5032f, 0.1682f, -0.4938f}, {2.5032f, 0.1264f, -0.4938f}}}},
            {Gauge::FUEL_FLOW, {{{2.5088f, 0.1882f, -0.5181f}, {2.5089f, 0.2191f, -0.5185f}, {2.5034f, 0.2192f, -0.4950f}, {2.5034f, 0.1883f, -0.4947f}}}},
            {Gauge::OIL, {{{2.4848f, 0.2515f, -0.4710f}, {2.4847f, 0.2785f, -0.4709f}, {2.4712f, 0.2785f, -0.4474f}, {2.4712f, 0.2515f, -0.4476f}}}},
            {Gauge::NOZ, {{{2.4679f, 0.2514f, -0.4418f}, {2.4678f, 0.2864f, -0.4416f}, {2.4502f, 0.2864f, -0.4112f}, {2.4503f, 0.2514f, -0.4114f}}}},
            {Gauge::RPM, {{{2.4447f, 0.2490f, -0.4017f}, {2.4445f, 0.2950f, -0.4013f}, {2.4214f, 0.2949f, -0.3613f}, {2.4216f, 0.2490f, -0.3617f}}}},
            {Gauge::FTIT, {{{2.4163f, 0.2605f, -0.3524f}, {2.4161f, 0.3054f, -0.3520f}, {2.3936f, 0.3052f, -0.3131f}, {2.3938f, 0.2603f, -0.3135f}}}},
            {Gauge::CABIN, {{{2.2786f, 0.3781f, -0.1749f}, {2.2786f, 0.4336f, -0.1827f}, {2.2363f, 0.4338f, -0.1469f}, {2.2363f, 0.3782f, -0.1391f}}}},
            {Gauge::HYD_A, {{{2.3043f, 0.3967f, -0.1993f}, {2.3043f, 0.4237f, -0.2031f}, {2.2837f, 0.4238f, -0.1857f}, {2.2837f, 0.3968f, -0.1819f}}}},
            {Gauge::HYD_B, {{{2.3254f, 0.3764f, -0.2144f}, {2.3254f, 0.4079f, -0.2188f}, {2.3014f, 0.4080f, -0.1985f}, {2.3014f, 0.3765f, -0.1941f}}}},
            {Gauge::CLOCK, {{{2.3876f, 0.3356f, -0.2615f}, {2.3876f, 0.3911f, -0.2693f}, {2.3454f, 0.3913f, -0.2335f}, {2.3454f, 0.3358f, -0.2257f}}}},
            {Gauge::DED, {{{2.3807f, 0.0982f, -0.6016f}, {2.3807f, 0.1798f, -0.6016f}, {2.3744f, 0.1798f, -0.5741f}, {2.3744f, 0.0982f, -0.5741f}}}},
            {Gauge::WARN_1, {{{2.3784f, 0.1994f, -0.5670f}, {2.3808f, 0.2119f, -0.5771f}, {2.3779f, 0.2225f, -0.5645f}, {2.3755f, 0.2100f, -0.5545f}}}},
            {Gauge::WARN_2, {{{2.3753f, 0.2110f, -0.5533f}, {2.3776f, 0.2235f, -0.5633f}, {2.3747f, 0.2341f, -0.5508f}, {2.3724f, 0.2216f, -0.5408f}}}},
            {Gauge::WARN_3, {{{2.3720f, 0.2229f, -0.5392f}, {2.3743f, 0.2354f, -0.5492f}, {2.3714f, 0.2461f, -0.5367f}, {2.3691f, 0.2336f, -0.5266f}}}},
            {Gauge::WARN_4, {{{2.3688f, 0.2346f, -0.5255f}, {2.3712f, 0.2470f, -0.5355f}, {2.3683f, 0.2577f, -0.5229f}, {2.3659f, 0.2452f, -0.5129f}}}},
            {Gauge::WARN_5, {{{2.3656f, 0.2465f, -0.5113f}, {2.3679f, 0.2590f, -0.5214f}, {2.3650f, 0.2696f, -0.5088f}, {2.3627f, 0.2571f, -0.4988f}}}},
            {Gauge::GEAR_LAMP, {{{2.3768f, -0.3296f, -0.2819f}, {2.3775f, -0.3175f, -0.2802f}, {2.3695f, -0.3201f, -0.2729f}, {2.3688f, -0.3321f, -0.2746f}}}},
            {Gauge::GEAR_LAMP, {{{2.3788f, -0.3051f, -0.2791f}, {2.3795f, -0.2931f, -0.2774f}, {2.3715f, -0.2956f, -0.2701f}, {2.3708f, -0.3076f, -0.2717f}}}},
            {Gauge::GEAR_LAMP, {{{2.3898f, -0.3179f, -0.2923f}, {2.3905f, -0.3062f, -0.2907f}, {2.3826f, -0.3062f, -0.2830f}, {2.3819f, -0.3179f, -0.2846f}}}},
            // Threat warning azimuth scope, upper left. Located by projecting the
            // painted scope's screen position from the design eye point onto the
            // plane of the DED's panel, 3 mm proud of it.
            {Gauge::RWR, {{{2.3727f, -0.1388f, -0.5813f}, {2.3727f, -0.0848f, -0.5813f}, {2.3607f, -0.0848f, -0.5287f}, {2.3607f, -0.1388f, -0.5287f}}}},
        };
        std::vector<ScreenQuad> quads;
        for (const auto& [g, corners] : kPlacements) quads.push_back(CockpitGauges::screen(g, corners));
        return quads;
    }

    /// @brief Draws the asset's HUD combiner glass into the bound mask
    /// (stencil) with colour writes left to the caller.
    void render_hud_mask(const Mat4& projection, const Mat4& view_cockpit) {
        if (!loaded_ || !initialized_) return;
        depth_shader_.use();
        depth_shader_.set_mat4("uLightMVP", projection * view_cockpit * body_matrix(ModelAlignment::COCKPIT));
        for (const auto& sm : submeshes_) {
            if (sm.part_type != F16PartType::HUD) continue;
            glBindVertexArray(sm.vao);
            glDrawElements(GL_TRIANGLES, sm.index_count, sm.index_type, nullptr);
        }
        glBindVertexArray(0);
    }

    /// @brief Writes the opaque cockpit and hull into the bound depth buffer
    /// at the near plane, so world geometry they hide is rejected by the
    /// early depth test instead of being shaded and then painted over.
    void render_cockpit_occluder(const Mat4& projection, const Mat4& view_cockpit) {
        if (!loaded_ || !initialized_) return;
        depth_shader_.use();
        depth_shader_.set_mat4("uLightMVP", projection * view_cockpit * body_matrix(ModelAlignment::COCKPIT));
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDepthRange(0.0, 0.0);
        glDepthFunc(GL_ALWAYS);
        cockpit_interior_ = true;
        for (const auto& sm : submeshes_) {
            if (sm.is_transparent || !visible_from_cockpit(sm)) continue;
            glBindVertexArray(sm.vao);
            glDrawElements(GL_TRIANGLES, sm.index_count, sm.index_type, nullptr);
        }
        glBindVertexArray(0);
        glDepthFunc(GL_LESS);
        glDepthRange(0.0, 1.0);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }

    /// @brief Bounds of all submeshes of one part, in body axes [m], as
    /// placed for a given alignment.
    [[nodiscard]] BodyBounds part_bounds(F16PartType type, ModelAlignment align) const noexcept {
        BodyBounds b;
        const bool world = align == ModelAlignment::WORLD;
        const ModelPlacement& p = placement_;
        const double off[3] = {world ? p.world_x : p.cockpit_x, 0.0, world ? p.world_z : p.cockpit_z};
        const double k = p.scale;
        for (const auto& sm : submeshes_) {
            if (sm.part_type != type) continue;
            math::Vector3 lo(sm.bounds_min[0] * k + off[0], sm.bounds_min[1] * k + off[1],
                             sm.bounds_min[2] * k + off[2]);
            math::Vector3 hi(sm.bounds_max[0] * k + off[0], sm.bounds_max[1] * k + off[1],
                             sm.bounds_max[2] * k + off[2]);
            if (!b.valid) {
                b.min = lo;
                b.max = hi;
                b.valid = true;
            } else {
                b.min = math::Vector3((std::min)(b.min.x, lo.x), (std::min)(b.min.y, lo.y), (std::min)(b.min.z, lo.z));
                b.max = math::Vector3((std::max)(b.max.x, hi.x), (std::max)(b.max.y, hi.y), (std::max)(b.max.z, hi.z));
            }
        }
        return b;
    }

    /// @brief True if the asset provides a cockpit interior to fly from.
    [[nodiscard]] bool has_cockpit() const noexcept {
        if (!loaded_) return false;
        for (const auto& sm : submeshes_) {
            if (sm.part_type == F16PartType::COCKPIT) return true;
        }
        return false;
    }

    /// @brief Centre of the engine nozzle exit in body axes, world alignment [m].
    [[nodiscard]] static math::Vector3 nozzle_exit_body() noexcept {
        // Measured from the asset: the nozzle lip sits 4.39 m aft of the
        // model origin on the fuselage axis.
        constexpr double kNozzleModelX = -4.39;
        return math::Vector3(WORLD_OFFSET_X + kNozzleModelX, 0.0, WORLD_OFFSET_Z);
    }

    /// @brief Draws the airframe into the bound shadow map.
    /// @param with_interior False leaves out the cockpit interior (a separate
    ///        cockpit shell casts its own shadow).
    void render_shadow(const ShadowMap& shadow, const fdm::FlightState& state, bool gear_down,
                       ModelAlignment align, bool with_interior = true) {
        if (!loaded_ || !initialized_) return;
        depth_shader_.use();
        const Mat4 mvp = shadow.caster_matrix(state.pos_ned) * attitude_matrix(state) * placed_matrix(align);
        depth_shader_.set_mat4("uLightMVP", mvp);
        begin_joints(mvp, Mat4::identity(), true);
        for (const auto& sm : submeshes_) {
            if (sm.is_transparent || !visible(sm, gear_down)) continue;
            if (!with_interior && sm.part_type == F16PartType::COCKPIT) continue;
            pose(sm);
            glBindVertexArray(sm.vao);
            glDrawElements(GL_TRIANGLES, sm.index_count, sm.index_type, nullptr);
        }
        glBindVertexArray(0);
    }

    /// @brief Render the external airframe (chase view) into the scene target.
    /// @param view_proj_rot Projection times the view *rotation* only: the
    ///        model is drawn eye-relative so it keeps sub-millimetre precision
    ///        however far the jet is from the world origin.
    void render_world(const Mat4& view_proj_rot, const fdm::FlightState& state, bool gear_down,
                      const FrameContext& ctx) {
        if (!loaded_ || !initialized_) return;

        shader_.use();
        ctx.upload(shader_);
        shader_.set_mat4("uViewProj", view_proj_rot);
        const Mat4 model = relative_matrix_body(state, ctx.eye_ned) * placed_matrix(ModelAlignment::WORLD);
        shader_.set_mat4("uModel", model);
        set_normal_matrix(attitude_matrix(state));
        begin_joints(model, attitude_matrix(state), false);
        shader_.set_vec3("uFrameOrigin", static_cast<float>(ctx.eye_ned.x), static_cast<float>(ctx.eye_ned.y),
                         static_cast<float>(ctx.eye_ned.z));
        shader_.set_float("uEyeAltitude", static_cast<float>(-ctx.eye_ned.z));
        shader_.set_int("uApplyHaze", 1);
        ctx.bind_shadow(shader_, 1.0f, ctx.eye_ned);

        draw_pass(gear_down, /*cockpit=*/false);
    }

    /// @brief Render the cockpit and the airframe around the pilot, in body
    /// axes (cockpit view).
    ///
    /// Drawn straight to the display, so it encodes its own output. The sun
    /// comes through the canopy and is shadowed by the airframe itself.
    /// @param with_interior False draws only the hull and canopy, around a
    ///        separate cockpit shell.
    void render_cockpit(const Mat4& projection, const Mat4& view_cockpit, const fdm::FlightState& state,
                        const FrameContext& ctx, const math::Vector3& eye_body, bool with_interior = true) {
        if (!loaded_ || !initialized_) return;

        shader_.use();
        ctx.upload(shader_);
        shader_.set_int("uHdrOutput", 0);
        shader_.set_float("uCloudCoverage", 0.0f);
        shader_.set_int("uFrameIsBody", 1);

        // Light and sky arrive in body axes.
        const math::Vector3 sun_b = state.q_att.rotate_ned_to_body(
            math::Vector3(ctx.lighting.sun_dir[0], ctx.lighting.sun_dir[1], ctx.lighting.sun_dir[2]));
        shader_.set_vec3("uSunDir", static_cast<float>(sun_b.x), static_cast<float>(sun_b.y),
                         static_cast<float>(sun_b.z));
        const math::Matrix3x3 c_nb = state.q_att.to_dcm_body_to_ned();
        float body_to_ned[9];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) body_to_ned[c * 3 + r] = static_cast<float>(c_nb(r, c));
        }
        shader_.set_mat3("uBodyToNed", body_to_ned);

        shader_.set_mat4("uViewProj", projection * view_cockpit);
        shader_.set_mat4("uModel", placed_matrix(ModelAlignment::COCKPIT));
        set_normal_matrix(Mat4::identity());
        begin_joints(placed_matrix(ModelAlignment::COCKPIT), Mat4::identity(), false);
        shader_.set_vec3("uFrameOrigin", static_cast<float>(eye_body.x), static_cast<float>(eye_body.y),
                         static_cast<float>(eye_body.z));
        shader_.set_float("uEyeAltitude", static_cast<float>(-state.pos_ned.z));
        shader_.set_int("uApplyHaze", 0);

        // Body-frame receivers: shadow texture = bias * light * T(cg) * R_bn.
        if (ctx.shadow) {
            const Mat4 m = ctx.shadow->receiver_matrix(state.pos_ned) * attitude_matrix(state);
            ctx.shadow->bind_receiver(shader_, FrameContext::kShadowUnit, 1.0f, m);
        } else {
            ctx.bind_shadow(shader_, 0.0f);
        }

        cockpit_interior_ = with_interior;
        draw_pass(/*gear_down=*/false, /*cockpit=*/true);
    }

private:
    [[nodiscard]] bool visible(const GLBSubmesh& sm, bool gear_down) const noexcept {
        if (sm.shown_by >= 0 && !moved_[static_cast<size_t>(sm.shown_by)]) return false;
        if (sm.hidden_by >= 0 && moved_[static_cast<size_t>(sm.hidden_by)]) return false;
        switch (sm.part_type) {
            case F16PartType::GEAR_DOWN: return gear_down;
            case F16PartType::GEAR_UP: return !gear_down;
            case F16PartType::LIGHTS: return gear_down;
            default: return true;
        }
    }

    /// Parts seen from the pilot's seat: the hull, canopy and (unless a
    /// separate shell replaces it) the cockpit interior and HUD glass.
    [[nodiscard]] bool visible_from_cockpit(const GLBSubmesh& sm) const noexcept {
        switch (sm.part_type) {
            case F16PartType::AIRFRAME:
            case F16PartType::RAILS:
            case F16PartType::CANOPY: return true;
            case F16PartType::COCKPIT:
            case F16PartType::HUD: return cockpit_interior_;
            default: return false;
        }
    }

    /// Moving parts: the matrices their joint transforms compose with.
    void begin_joints(const Mat4& model, const Mat4& rot, bool depth) noexcept {
        base_model_ = model;
        base_rot_ = rot;
        depth_pass_ = depth;
        posed_ = -1;
    }

    /// Sets the model (and normal) matrix for submesh @p sm if its joint
    /// differs from the last one drawn: rigid parts cost nothing extra.
    void pose(const GLBSubmesh& sm) {
        if (sm.joint == posed_) return;
        posed_ = sm.joint;
        const Mat4 j = sm.joint >= 0 ? joints_.m[static_cast<size_t>(sm.joint)] : Mat4::identity();
        if (depth_pass_) {
            depth_shader_.set_mat4("uLightMVP", base_model_ * j);
            return;
        }
        shader_.set_mat4("uModel", base_model_ * j);
        set_normal_matrix(base_rot_ * j);
    }

    void set_normal_matrix(const Mat4& rot) {
        float n[9];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) n[c * 3 + r] = rot(r, c);
        }
        shader_.set_mat3("uNormalMatrix", n);
    }

    void draw_pass(bool gear_down, bool cockpit) {
        // Opaque first, then glass over it without writing depth. From the
        // seat the interior is drawn before the hull: it hides most of the
        // hull, whose fragments the depth test then rejects before shading.
        auto opaque = [&](bool interior_part) {
            for (const auto& sm : submeshes_) {
                if (sm.is_transparent || !visible(sm, gear_down)) continue;
                if (cockpit && !visible_from_cockpit(sm)) continue;
                const bool interior = sm.part_type == F16PartType::COCKPIT;
                if (cockpit && interior != interior_part) continue;
                draw_submesh(sm, gear_down);
            }
        };
        if (cockpit) opaque(true);
        opaque(false);

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_SRC1_COLOR);
        glDepthMask(GL_FALSE);
        for (const auto& sm : submeshes_) {
            if (!sm.is_transparent || !visible(sm, gear_down)) continue;
            if (cockpit && !visible_from_cockpit(sm)) continue;
            draw_submesh(sm, gear_down);
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        for (int unit = 0; unit < 3; ++unit) {
            glActiveTexture(GL_TEXTURE0 + static_cast<GLenum>(unit));
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(0);
    }

    void bind_map(int unit, int texture_index, const char* flag) {
        const bool has = texture_index >= 0 && texture_index < static_cast<int>(textures_.size()) &&
                         textures_[static_cast<size_t>(texture_index)] != 0;
        glActiveTexture(GL_TEXTURE0 + static_cast<GLenum>(unit));
        glBindTexture(GL_TEXTURE_2D, has ? textures_[static_cast<size_t>(texture_index)] : 0);
        shader_.set_int(flag, has ? 1 : 0);
    }

    void draw_submesh(const GLBSubmesh& sm, bool gear_down) {
        pose(sm);
        bind_map(0, sm.base_texture, "uHasBase");
        bind_map(1, sm.mr_texture, "uHasMr");
        bind_map(2, sm.normal_texture, "uHasNormal");
        glActiveTexture(GL_TEXTURE0);

        const Color4& b = sm.base_color_factor;
        shader_.set_vec4("uBaseFactor", b.r, b.g, b.b, b.a);
        shader_.set_float("uMetallic", sm.metallic_factor);
        shader_.set_float("uRoughness", sm.roughness_factor);

        const bool lit = sm.part_type == F16PartType::LIGHTS && gear_down;
        const float e = lit ? LANDING_LIGHT_RADIANCE : 0.0f;
        shader_.set_vec3("uEmissive", sm.emissive[0] * e, sm.emissive[1] * e, sm.emissive[2] * e);

        shader_.set_int("uGlass", sm.is_transparent ? 1 : 0);
        if (sm.is_transparent) {
            // Transmission tint: a softened version of the glass colour, so the
            // canopy warms the view without turning it amber. The HUD
            // combiner's coating is barely visible in transmission.
            const bool canopy = sm.part_type == F16PartType::CANOPY;
            const float kTintStrength = canopy ? 0.2f : 0.05f;
            // The combiner's anti-reflection coating keeps it nearly invisible.
            const std::array<float, 3> f0 = canopy ? CANOPY_F0 : std::array<float, 3>{0.012f, 0.014f, 0.012f};
            shader_.set_vec3("uGlassF0", f0[0], f0[1], f0[2]);
            shader_.set_vec3("uGlassTint", 1.0f + (b.r - 1.0f) * kTintStrength, 1.0f + (b.g - 1.0f) * kTintStrength,
                             1.0f + (b.b - 1.0f) * kTintStrength);
        }

        glBindVertexArray(sm.vao);
        glDrawElements(GL_TRIANGLES, sm.index_count, sm.index_type, nullptr);
    }
};

} // namespace fastjet::graphics
