#define CGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#include "fastjet/graphics/render_engine.hpp"

#include <SDL3/SDL.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using namespace fastjet;
using namespace fastjet::graphics;

namespace {

constexpr int W = 960;
constexpr int H = 540;

std::array<float, 3> sun() { return SkyGroundRenderer::sun_dir_ned(); }

/// Applies a column-major Mat4 to a point, returning clip coordinates.
std::array<float, 4> apply(const Mat4& m, double x, double y, double z) {
    const float v[4] = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 1.0f};
    std::array<float, 4> o{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) o[static_cast<size_t>(r)] += m(r, c) * v[c];
    }
    return o;
}

struct Frame {
    std::vector<unsigned char> rgb;
    [[nodiscard]] float luminance(size_t i) const {
        return 0.2126f * rgb[i * 3] + 0.7152f * rgb[i * 3 + 1] + 0.0722f * rgb[i * 3 + 2];
    }
};

Frame read_frame() {
    Frame f;
    f.rgb.resize(static_cast<size_t>(W) * H * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, f.rgb.data());
    return f;
}

} // namespace

// ---------------------------------------------------------------------------
// CPU: lighting model, shadow projection, quality presets
// ---------------------------------------------------------------------------

void test_scene_lighting() {
    std::cout << "[Test] Lighting: sun and sky follow the scattering model... " << std::flush;
    const SceneLighting sea = SceneLighting::compute(sun(), 0.0f, 1.35f);
    const SceneLighting high = SceneLighting::compute(sun(), 12000.0f, 1.35f);

    // Sea-level sun keeps the calibrated warm-white illuminance.
    assert(std::abs(sea.sun.r - 3.0f) < 0.05f);
    assert(sea.sun.r > sea.sun.g && sea.sun.g > sea.sun.b);
    // The sky is blue, and bluer than the sun.
    assert(sea.sky_ambient.b > sea.sky_ambient.g && sea.sky_ambient.g > sea.sky_ambient.r);
    assert(sea.sky_ambient.b / sea.sky_ambient.r > sea.sun.b / sea.sun.r);
    // Climbing: less air overhead scatters less (darker sky), and the sun
    // loses less on the way down (brighter, whiter).
    assert(high.sky_ambient.luminance() < sea.sky_ambient.luminance());
    assert(high.sun.luminance() > sea.sun.luminance());
    assert(high.sun.b / high.sun.r > sea.sun.b / sea.sun.r);
    // The ambient is a fraction of the direct light, as on a clear day.
    assert(sea.sky_ambient.luminance() < 0.3f * sea.sun.luminance());
    std::cout << "PASS\n";
}

void test_shadow_projection() {
    std::cout << "[Test] Shadow map: projection, depth ordering, texel snapping... " << std::flush;
    const auto s = sun();
    const math::Vector3 focus(1234.5, -87.25, -1.56);
    const Mat4 lvp = ShadowMap::compute_light_view_proj(s, focus, 2048);

    // The focus sits at the centre of the map.
    const auto c = apply(lvp, 0.0, 0.0, 0.0);
    const double texel_ndc = 2.0 / 2048.0;
    assert(std::abs(c[0]) <= texel_ndc && std::abs(c[1]) <= texel_ndc);

    // Along the sun ray, position in the map is unchanged but depth grows:
    // that is what makes ground under the jet shadowed rather than lit.
    const double t = 1.56 / -s[2];
    const auto g = apply(lvp, -s[0] * t, -s[1] * t, -s[2] * t);
    assert(std::abs(g[0] - c[0]) < 1e-4f && std::abs(g[1] - c[1]) < 1e-4f);
    assert(g[2] > c[2]);

    // Snapping: a fixed world point keeps its sub-texel position however the
    // focus moves, so the shadow edge does not crawl.
    auto subtexel = [&](const math::Vector3& f) {
        const Mat4 m = ShadowMap::compute_light_view_proj(s, f, 2048);
        const math::Vector3 world(1240.0, -80.0, 0.0);
        const math::Vector3 rel = world - f;
        const auto p = apply(m, rel.x, rel.y, rel.z);
        const double px = (p[0] * 0.5 + 0.5) * 2048.0;
        return px - std::floor(px);
    };
    const double a = subtexel(focus);
    const double b = subtexel(focus + math::Vector3(0.37, -0.11, 0.02));
    assert(std::abs(a - b) < 1e-2 || std::abs(std::abs(a - b) - 1.0) < 1e-2);

    // Ground shadows fade with height: crisp on the gear, gone when high.
    assert(ShadowMap::ground_strength(0.0) == 1.0f);
    assert(ShadowMap::ground_strength(100.0) < 1.0f && ShadowMap::ground_strength(100.0) > 0.0f);
    assert(ShadowMap::ground_strength(1000.0) == 0.0f);
    std::cout << "PASS\n";
}

void test_quality_presets() {
    std::cout << "[Test] Quality: presets scale monotonically... " << std::flush;
    RenderQuality prev = RenderQuality::from_preset(ui::QualityPreset::LOW);
    for (auto p : {ui::QualityPreset::MEDIUM, ui::QualityPreset::HIGH, ui::QualityPreset::ULTRA}) {
        const RenderQuality q = RenderQuality::from_preset(p);
        assert(q.scene_samples >= prev.scene_samples);
        assert(q.shadow_map_size >= prev.shadow_map_size);
        assert(q.bloom >= prev.bloom && q.cirrus >= prev.cirrus);
        assert(ui::msaa_samples(p) <= 4);
        prev = q;
    }
    // Every preset keeps sun shadows: they are what seats the jet on the runway.
    assert(RenderQuality::from_preset(ui::QualityPreset::LOW).shadow_map_size > 0);
    std::cout << "PASS\n";
}

// ---------------------------------------------------------------------------
// GL: asset orientation, cockpit displays, HDR pipeline, shadows, clouds
// ---------------------------------------------------------------------------

/// The airframe asset is authored Y-up; the body frame is Z-down. Loading it
/// without the axis change drew the jet rolled 90 degrees.
void test_model_orientation(const RenderEngine& engine) {
    std::cout << "[Test] Model: airframe upright in body axes, wheels on the gear... " << std::flush;
    const ModelGLB& m = engine.model();
    assert(m.is_loaded());
    const BodyBounds airframe = m.part_bounds(F16PartType::AIRFRAME, ModelAlignment::WORLD);
    const BodyBounds gear = m.part_bounds(F16PartType::GEAR_DOWN, ModelAlignment::WORLD);
    assert(airframe.valid && gear.valid);

    const double length = airframe.max.x - airframe.min.x;
    const double span = airframe.max.y - airframe.min.y;
    const double height = airframe.max.z - airframe.min.z;
    assert(length > 14.0 && length < 16.5); // F-16: 15.06 m
    assert(span > 9.0 && span < 10.5);      // 9.45 m with tip rails
    assert(height < 6.0 && height < span);  // not rolled on its side
    assert(std::abs(airframe.max.y + airframe.min.y) < 0.2); // symmetric in span

    // Gear hangs below (+Z) and the wheels meet the FDM contact plane.
    assert(gear.max.z > 1.4 && gear.max.z < 1.7);
    // Fin stands up (-Z).
    assert(airframe.min.z < -2.5);
    std::cout << "PASS (span " << span << " m, wheels at z=" << gear.max.z << " m)\n";
}

/// The live MFD pages must sit on the cockpit's screens, facing the pilot.
void test_cockpit_screens() {
    std::cout << "[Test] Cockpit: live MFD pages face the design eye point... " << std::flush;
    const auto screens = ModelGLB::cockpit_screens();
    assert(screens.size() == 2);
    const math::Vector3 eye(CameraRig::DEP_X, CameraRig::DEP_Y, CameraRig::DEP_Z);
    for (size_t i = 0; i < screens.size(); ++i) {
        const auto& c = screens[i].corners;
        const math::Vector3 tl(c[0][0], c[0][1], c[0][2]), tr(c[1][0], c[1][1], c[1][2]), bl(c[3][0], c[3][1], c[3][2]);
        const math::Vector3 centre = (tl + math::Vector3(c[2][0], c[2][1], c[2][2])) * 0.5;
        const double w = (tr - tl).norm();
        const double h = (bl - tl).norm();
        assert(w > 0.08 && w < 0.13 && h > 0.08 && h < 0.13); // 4-inch-class displays
        assert(tl.z < bl.z);                                   // top edge is up (-Z)
        const math::Vector3 n = (tr - tl).cross(bl - tl).normalized();
        const math::Vector3 to_eye = (eye - centre).normalized();
        assert(std::abs(n.dot(to_eye)) > 0.8);
        // Left page on the left, right page on the right.
        assert(i == 0 ? centre.y < 0.0 : centre.y > 0.0);
    }
    std::cout << "PASS\n";
}

/// Every painted instrument gets a live face lying on it, facing the pilot.
void test_cockpit_gauges() {
    std::cout << "[Test] Cockpit: live gauge faces cover the painted instruments... " << std::flush;
    const auto quads = ModelGLB::cockpit_gauges();
    std::vector<int> seen(static_cast<size_t>(Gauge::COUNT), 0);
    const math::Vector3 eye(CameraRig::DEP_X, CameraRig::DEP_Y, CameraRig::DEP_Z);
    for (const auto& q : quads) {
        const auto& c = q.corners;
        const math::Vector3 tl(c[0][0], c[0][1], c[0][2]), tr(c[1][0], c[1][1], c[1][2]);
        const math::Vector3 br(c[2][0], c[2][1], c[2][2]), bl(c[3][0], c[3][1], c[3][2]);
        const math::Vector3 centre = (tl + br) * 0.5;
        const double w = (tr - tl).norm();
        const double h = (bl - tl).norm();
        assert(w > 0.008 && w < 0.10 && h > 0.008 && h < 0.10);  // lamps to 3-inch dials
        // Inside the cockpit, forward of the pilot.
        assert(centre.x > 2.2 && centre.x < 2.6 && std::abs(centre.y) < 0.45);
        // Face turned toward the design eye point (corners run clockwise
        // as the pilot sees them).
        const math::Vector3 n = (bl - tl).cross(tr - tl).normalized();
        assert(n.dot((eye - centre).normalized()) > 0.5);
        // Reads upright: the top edge is higher (-Z) than the bottom.
        assert((tl + tr).z < (bl + br).z);
        // The cell's aspect matches the face it is laid on (no stretching
        // beyond the asset's own: its gear lamps are painted ~10% oval).
        const int gi = [&] {
            for (int i = 0; i < static_cast<int>(Gauge::COUNT); ++i) {
                const ScreenQuad ref = CockpitGauges::screen(static_cast<Gauge>(i), c);
                if (ref.u0 == q.u0 && ref.v0 == q.v0) return i;
            }
            return -1;
        }();
        assert(gi >= 0);
        ++seen[static_cast<size_t>(gi)];
        const auto cell = CockpitGauges::cell(static_cast<Gauge>(gi));
        const double cell_aspect = static_cast<double>(cell.w) / cell.h;
        assert(std::abs(w / h - cell_aspect) / cell_aspect < 0.12);
    }
    for (int i = 0; i < static_cast<int>(Gauge::COUNT); ++i) {
        // The Typhoon glareshield faces (MDEF onward) are not on the F-16C.
        const int want = static_cast<Gauge>(i) >= Gauge::MDEF ? 0 : (static_cast<Gauge>(i) == Gauge::GEAR_LAMP ? 3 : 1);
        assert(seen[static_cast<size_t>(i)] == want);
    }
    std::cout << "PASS (" << quads.size() << " faces)\n";
}

void test_hdr_frame(RenderEngine& engine) {
    std::cout << "[Test] HDR: scene target, bloom chain and clean frames... " << std::flush;
    fdm::FlightState st{};
    st.pos_ned = math::Vector3(150.0, 0.0, -1.56);
    st.q_att = math::Quaternion::identity();
    flcs::IMUData imu{};
    imu.Nz = 1.0;
    AvionicsTelemetry tel{};
    tel.throttle_input = 1.0; // reheat: exercises the exhaust pass
    tel.engine_ftit_deg_c = 980.0;
    engine.camera_rig().set_mode(CameraMode::CHASE);
    for (int i = 0; i < 5; ++i) engine.render_frame(0.016, st, imu, false, tel);
    assert(glGetError() == GL_NO_ERROR);
    assert(engine.hdr().width() == W && engine.hdr().height() == H);
    assert(engine.hdr().bloom_levels() >= 4);
    assert(engine.shadow_map().enabled());

    // Cockpit view too, with G-LOC greyout in progress.
    engine.camera_rig().set_mode(CameraMode::COCKPIT);
    tel.ofc_gloc_blackout_frac = 0.5;
    for (int i = 0; i < 3; ++i) engine.render_frame(0.016, st, imu, false, tel);
    assert(glGetError() == GL_NO_ERROR);
    const Frame f = read_frame();
    // The tunnel darkens the edges more than the centre.
    auto lum_at = [&](int x, int y) { return f.luminance(static_cast<size_t>(y) * W + static_cast<size_t>(x)); };
    assert(lum_at(4, H / 2) < 10.0f);
    std::cout << "PASS\n";
}

/// With the sun high and the jet parked, its shadow must darken the runway.
void test_aircraft_shadow(RenderEngine& engine) {
    std::cout << "[Test] Shadows: the parked jet darkens the runway... " << std::flush;
    fdm::FlightState st{};
    st.pos_ned = math::Vector3(150.0, 0.0, -1.56);
    st.q_att = math::Quaternion::identity();
    flcs::IMUData imu{};
    imu.Nz = 1.0;
    engine.camera_rig().set_mode(CameraMode::CHASE);
    engine.camera_rig().reset_head_look();
    // Rear quarter, looking down at the shadow side (sun is ahead-left).
    engine.camera_rig().add_head_look(30.0f * 3.14159265f / 180.0f, 12.0f * 3.14159265f / 180.0f);

    auto dark_pixels = [&](int shadow_size) {
        RenderQuality q = RenderQuality::from_preset(ui::QualityPreset::HIGH);
        q.shadow_map_size = shadow_size;
        q.clouds = false; // keep cloud shadows out of the comparison
        engine.set_quality(q);
        for (int i = 0; i < 3; ++i) engine.render_frame(0.016, st, imu);
        const Frame f = read_frame();
        int n = 0;
        for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i) n += f.luminance(i) < 30.0f;
        return n;
    };
    const int lit = dark_pixels(0);
    const int shadowed = dark_pixels(2048);
    engine.set_quality(RenderQuality::from_preset(ui::QualityPreset::ULTRA));
    engine.camera_rig().reset_head_look();
    assert(glGetError() == GL_NO_ERROR);
    assert(shadowed > lit + W * H / 50 && "Aircraft shadow must cover a visible patch of runway");
    std::printf("PASS (%d dark px without shadows, %d with)\n", lit, shadowed);
}

/// The coverage setting must mean what it says: the fraction of sky covered.
void test_cloud_coverage() {
    std::cout << "[Test] Clouds: coverage setting matches covered fraction... " << std::flush;
    const char* vert = R"(
        #version 330 core
        out vec2 vUV;
        void main() {
            vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
            vUV = p;
            gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
        }
    )";
    const std::string frag = std::string("#version 330 core\nin vec2 vUV;\nout vec4 FragColor;\n") +
                             glsl::COMMON + glsl::CLOUDS +
                             "void main() { FragColor = vec4(cloud_field(vUV * 200000.0, 6), 0.0, 0.0, 1.0); }";
    ShaderProgram prog;
    assert(prog.init_from_source(vert, frag.c_str()));
    GLuint vao = 0;
    glGenVertexArrays(1, &vao);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, W, H);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_STENCIL_TEST);
    prog.use();
    glBindVertexArray(vao);
    for (float coverage : {0.2f, 0.35f, 0.6f}) {
        prog.set_float("uCloudCoverage", coverage);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        const Frame f = read_frame();
        size_t covered = 0;
        for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i) covered += f.rgb[i * 3] > 10;
        const double fraction = static_cast<double>(covered) / (static_cast<double>(W) * H);
        assert(std::abs(fraction - coverage) < 0.08);
    }
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &vao);
    glEnable(GL_DEPTH_TEST);
    assert(glGetError() == GL_NO_ERROR);
    std::cout << "PASS\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Rendering Fidelity: Lighting, Shadows, HDR, Clouds\n";
    std::cout << "=========================================================\n";
    test_scene_lighting();
    test_shadow_projection();
    test_quality_presets();

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_Window* window = SDL_CreateWindow("Render Fidelity", W, H, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    assert(window != nullptr);
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    assert(ctx != nullptr);

    {
        RenderEngine engine;
        const bool ok = engine.init(W, H, 60.0f);
        assert(ok && "Render engine must initialise all passes");
        test_model_orientation(engine);
        test_cockpit_screens();
        test_cockpit_gauges();
        test_hdr_frame(engine);
        test_aircraft_shadow(engine);
        test_cloud_coverage();
        engine.destroy();
    }

    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::cout << "All rendering fidelity tests passed.\n";
    return 0;
}
